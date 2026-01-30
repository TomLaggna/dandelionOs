//! x86_64 Architecture Support for Unikraft Engine
//!
//! This module handles:
//! - 4-level page table setup (PML4 → PDPT → PD → PT)
//! - GDT with kernel and user segments
//! - TSS with interrupt stack table (IST)
//! - IDT with 33 interrupt gates (vectors 0-32)

use super::PAGE_SIZE;

// ============================================================================
// Error Types
// ============================================================================

/// Errors specific to x86_64 setup
#[derive(Debug, Clone, PartialEq)]
pub enum X86Error {
    /// Context size too small for required structures
    ContextTooSmall,
    /// Page table setup failed
    PageTableError,
    /// GDT/TSS/IDT setup failed
    InterruptTableError,
    /// Memory alignment error
    AlignmentError,
}

pub type X86Result<T> = Result<T, X86Error>;

// ============================================================================
// Constants - Page Table Flags
// ============================================================================

/// Page table entry flags (matching x86_64 specification)
pub mod pte_flags {
    pub const PRESENT: u64 = 1 << 0;
    pub const WRITABLE: u64 = 1 << 1;
    pub const USER: u64 = 1 << 2;
    pub const WRITE_THROUGH: u64 = 1 << 3;
    pub const CACHE_DISABLE: u64 = 1 << 4;
    pub const ACCESSED: u64 = 1 << 5;
    pub const DIRTY: u64 = 1 << 6;
    pub const HUGE_PAGE: u64 = 1 << 7; // PS bit for 2MB/1GB pages
    pub const GLOBAL: u64 = 1 << 8;
    pub const NO_EXECUTE: u64 = 1 << 63;

    /// Standard flags for user-accessible pages
    pub const USER_PAGE: u64 = PRESENT | WRITABLE | USER;

    /// Flags for page table entries (non-leaf)
    pub const TABLE_ENTRY: u64 = PRESENT | WRITABLE | USER;
}

// ============================================================================
// Constants - Segment Selectors
// ============================================================================

/// GDT segment selectors
pub mod selectors {
    pub const NULL: u16 = 0x00;
    pub const KERNEL_CODE_64: u16 = 0x08;
    pub const KERNEL_DATA: u16 = 0x10;
    pub const USER_CODE_64: u16 = 0x18;
    pub const USER_DATA: u16 = 0x20;
    pub const TSS: u16 = 0x28;

    /// User code selector with RPL=3 (for IRET)
    pub const USER_CODE_64_RPL3: u16 = USER_CODE_64 | 3;
    /// User data selector with RPL=3 (for IRET)
    pub const USER_DATA_RPL3: u16 = USER_DATA | 3;
}

// ============================================================================
// Constants - Structure Sizes
// ============================================================================

/// GDT entries (including TSS which spans 2 entries)
pub const GDT_ENTRIES: usize = 8;
pub const GDT_SIZE: usize = GDT_ENTRIES * 8;

/// TSS size: 104 bytes (x86_64 TSS structure)
pub const TSS_SIZE: usize = 104;

/// IDT entry size: 16 bytes per gate descriptor
pub const IDT_ENTRY_SIZE: usize = 16;

/// Number of IDT entries: vectors 0-32
pub const IDT_ENTRIES: usize = 33;

/// IDT total size
pub const IDT_SIZE: usize = IDT_ENTRIES * IDT_ENTRY_SIZE;

/// Interrupt stack size (pages)
pub const INTERRUPT_STACK_PAGES: usize = 1;

/// Interrupt stack total size
pub const INTERRUPT_STACK_SIZE: usize = INTERRUPT_STACK_PAGES * PAGE_SIZE;

// ============================================================================
// Page Table Setup
// ============================================================================

/// Set up 4-level page tables for user space
///
/// # Arguments
/// * `storage` - Guest memory buffer
/// * `stack_start` - Mutable reference tracking available memory (modified)
/// * `context_size` - Total size of the context
/// * `phys_base` - Physical base address of storage (for CR3)
/// * `use_large_pages` - Whether to use 2MB pages where possible
///
/// # Returns
/// Physical address of PML4 (for CR3)
pub fn set_page_table(
    storage: &mut [u8],
    stack_start: &mut usize,
    context_size: usize,
    phys_base: u64,
    use_large_pages: bool,
) -> X86Result<u64> {
    // Allocate PML4
    if *stack_start < PAGE_SIZE {
        return Err(X86Error::ContextTooSmall);
    }
    *stack_start -= PAGE_SIZE;
    let pml4_offset = *stack_start;
    let pml4_phys = phys_base + pml4_offset as u64;
    storage[pml4_offset..pml4_offset + PAGE_SIZE].fill(0);

    // Allocate PDPT
    if *stack_start < PAGE_SIZE {
        return Err(X86Error::ContextTooSmall);
    }
    *stack_start -= PAGE_SIZE;
    let pdpt_offset = *stack_start;
    let pdpt_phys = phys_base + pdpt_offset as u64;
    storage[pdpt_offset..pdpt_offset + PAGE_SIZE].fill(0);

    // Link PML4[0] → PDPT
    write_pte(storage, pml4_offset, 0, pdpt_phys | pte_flags::TABLE_ENTRY);

    // Calculate how many 1GB regions we need
    let num_gb_regions = (context_size + (1 << 30) - 1) / (1 << 30);
    let num_gb_regions = num_gb_regions.max(1).min(512);

    // For each 1GB region, allocate a PD
    for gb_idx in 0..num_gb_regions {
        if *stack_start < PAGE_SIZE {
            return Err(X86Error::ContextTooSmall);
        }
        *stack_start -= PAGE_SIZE;
        let pd_offset = *stack_start;
        let pd_phys = phys_base + pd_offset as u64;
        storage[pd_offset..pd_offset + PAGE_SIZE].fill(0);

        // Link PDPT[gb_idx] → PD
        write_pte(storage, pdpt_offset, gb_idx, pd_phys | pte_flags::TABLE_ENTRY);

        // Calculate how many 2MB regions in this GB
        let gb_start = gb_idx * (1 << 30);
        let gb_end = ((gb_idx + 1) * (1 << 30)).min(context_size);

        if gb_start >= context_size {
            break;
        }

        let num_2mb_regions = (gb_end - gb_start + (1 << 21) - 1) / (1 << 21);

        for mb_idx in 0..num_2mb_regions.min(512) {
            let vaddr = gb_start + mb_idx * (1 << 21);
            let paddr = phys_base + vaddr as u64;

            if use_large_pages {
                // Map as 2MB page
                write_pte(
                    storage,
                    pd_offset,
                    mb_idx,
                    paddr | pte_flags::USER_PAGE | pte_flags::HUGE_PAGE,
                );
            } else {
                // Allocate PT for 4KB pages
                if *stack_start < PAGE_SIZE {
                    return Err(X86Error::ContextTooSmall);
                }
                *stack_start -= PAGE_SIZE;
                let pt_offset = *stack_start;
                let pt_phys = phys_base + pt_offset as u64;
                storage[pt_offset..pt_offset + PAGE_SIZE].fill(0);

                write_pte(storage, pd_offset, mb_idx, pt_phys | pte_flags::TABLE_ENTRY);

                // Map 512 4KB pages in this PT
                for page_idx in 0..512 {
                    let page_vaddr = vaddr + page_idx * PAGE_SIZE;
                    if page_vaddr >= context_size {
                        break;
                    }
                    let page_paddr = phys_base + page_vaddr as u64;
                    write_pte(storage, pt_offset, page_idx, page_paddr | pte_flags::USER_PAGE);
                }
            }
        }
    }

    Ok(pml4_phys)
}

/// Write a page table entry
fn write_pte(storage: &mut [u8], table_offset: usize, index: usize, value: u64) {
    let offset = table_offset + index * 8;
    storage[offset..offset + 8].copy_from_slice(&value.to_le_bytes());
}

// ============================================================================
// Interrupt Table Setup (GDT, TSS, IDT)
// ============================================================================

/// Set up GDT, TSS, and IDT for user space
///
/// # Arguments
/// * `storage` - Guest memory buffer
/// * `stack_start` - Mutable reference tracking available memory
/// * `phys_base` - Physical base of storage
///
/// # Returns
/// Tuple of (gdt_offset, tss_offset, idt_offset, handler_offset, interrupt_stack_top)
pub fn set_interrupt_tables(
    storage: &mut [u8],
    stack_start: &mut usize,
    phys_base: u64,
) -> X86Result<(usize, usize, usize, usize, u64)> {
    // Allocate interrupt stack
    if *stack_start < INTERRUPT_STACK_SIZE {
        return Err(X86Error::ContextTooSmall);
    }
    *stack_start -= INTERRUPT_STACK_SIZE;
    let interrupt_stack_offset = *stack_start;
    let interrupt_stack_top = phys_base + interrupt_stack_offset as u64 + INTERRUPT_STACK_SIZE as u64;
    storage[interrupt_stack_offset..interrupt_stack_offset + INTERRUPT_STACK_SIZE].fill(0);

    // Allocate handler code region (page aligned)
    let handler_code = super::handlers::get_handler_code();
    let handler_size = handler_code.len();
    let handler_pages = (handler_size + PAGE_SIZE - 1) / PAGE_SIZE;
    let handler_alloc = handler_pages.max(1) * PAGE_SIZE;

    if *stack_start < handler_alloc {
        return Err(X86Error::ContextTooSmall);
    }
    *stack_start -= handler_alloc;
    let handler_offset = *stack_start;
    storage[handler_offset..handler_offset + handler_alloc].fill(0);
    storage[handler_offset..handler_offset + handler_size].copy_from_slice(handler_code);

    // Allocate GDT, TSS, IDT (8-byte aligned, contiguous)
    let structures_size = GDT_SIZE + TSS_SIZE + IDT_SIZE;
    let structures_size_aligned = (structures_size + 7) & !7;

    if *stack_start < structures_size_aligned {
        return Err(X86Error::ContextTooSmall);
    }
    *stack_start -= structures_size_aligned;
    let gdt_offset = *stack_start;
    let tss_offset = gdt_offset + GDT_SIZE;
    let idt_offset = tss_offset + TSS_SIZE;

    storage[gdt_offset..gdt_offset + structures_size_aligned].fill(0);

    // Set up GDT
    setup_gdt(storage, gdt_offset, tss_offset, phys_base)?;

    // Set up TSS
    setup_tss(storage, tss_offset, interrupt_stack_top)?;

    // Set up IDT
    let handler_base_va = phys_base + handler_offset as u64;
    setup_idt(storage, idt_offset, handler_base_va)?;

    Ok((
        gdt_offset,
        tss_offset,
        idt_offset,
        handler_offset,
        interrupt_stack_top,
    ))
}

/// Set up GDT with kernel and user segments
fn setup_gdt(
    storage: &mut [u8],
    gdt_offset: usize,
    tss_offset: usize,
    phys_base: u64,
) -> X86Result<()> {
    let tss_base = phys_base + tss_offset as u64;

    // Entry 0: Null descriptor
    write_gdt_entry(storage, gdt_offset, 0, 0);

    // Entry 1: Kernel Code 64-bit (DPL=0)
    write_gdt_entry(storage, gdt_offset, 1, 0x00AF_9A00_0000_FFFF);

    // Entry 2: Kernel Data (DPL=0)
    write_gdt_entry(storage, gdt_offset, 2, 0x00CF_9200_0000_FFFF);

    // Entry 3: User Code 64-bit (DPL=3)
    write_gdt_entry(storage, gdt_offset, 3, 0x00AF_FA00_0000_FFFF);

    // Entry 4: User Data (DPL=3)
    write_gdt_entry(storage, gdt_offset, 4, 0x00CF_F200_0000_FFFF);

    // Entry 5-6: TSS descriptor (16 bytes for 64-bit TSS)
    let tss_desc_low = 0x0000_8900_0000_0067_u64
        | ((tss_base & 0xFFFF) << 16)
        | ((tss_base & 0xFF_0000) << 16)
        | ((tss_base & 0xFF00_0000) << 32);
    let tss_desc_high = (tss_base >> 32) & 0xFFFF_FFFF;

    write_gdt_entry(storage, gdt_offset, 5, tss_desc_low);
    write_gdt_entry(storage, gdt_offset, 6, tss_desc_high);

    // Entry 7: Reserved
    write_gdt_entry(storage, gdt_offset, 7, 0);

    Ok(())
}

/// Write a GDT entry
fn write_gdt_entry(storage: &mut [u8], gdt_offset: usize, index: usize, value: u64) {
    let offset = gdt_offset + index * 8;
    storage[offset..offset + 8].copy_from_slice(&value.to_le_bytes());
}

/// Set up TSS with interrupt stack
fn setup_tss(
    storage: &mut [u8],
    tss_offset: usize,
    interrupt_stack_top: u64,
) -> X86Result<()> {
    // RSP0 - used when transitioning from Ring 3 to Ring 0
    write_tss_field(storage, tss_offset, 0x04, interrupt_stack_top);

    // IST1 - used by interrupt handlers
    write_tss_field(storage, tss_offset, 0x24, interrupt_stack_top);

    // I/O Map Base - set to TSS limit to indicate no I/O bitmap
    storage[tss_offset + 0x66] = 0x68;
    storage[tss_offset + 0x67] = 0x00;

    Ok(())
}

/// Write a 64-bit field to TSS
fn write_tss_field(storage: &mut [u8], tss_offset: usize, field_offset: usize, value: u64) {
    let offset = tss_offset + field_offset;
    storage[offset..offset + 8].copy_from_slice(&value.to_le_bytes());
}

/// Set up IDT with 33 interrupt gates
fn setup_idt(storage: &mut [u8], idt_offset: usize, handler_base_va: u64) -> X86Result<()> {
    let handler_offsets = super::handlers::get_handler_offsets();

    for vector in 0..IDT_ENTRIES {
        let handler_va = handler_base_va + handler_offsets[vector] as u64;
        setup_interrupt_gate(
            storage,
            idt_offset,
            vector,
            selectors::KERNEL_CODE_64,
            handler_va,
            3, // DPL=3 - allow user mode to trigger INT 32
            1, // IST=1 - use IST1 for stack
        );
    }

    Ok(())
}

/// Set up a single IDT interrupt gate
fn setup_interrupt_gate(
    storage: &mut [u8],
    idt_offset: usize,
    vector: usize,
    selector: u16,
    handler_addr: u64,
    dpl: u8,
    ist: u8,
) {
    let entry_offset = idt_offset + vector * IDT_ENTRY_SIZE;

    let offset_low = (handler_addr & 0xFFFF) as u16;
    let offset_mid = ((handler_addr >> 16) & 0xFFFF) as u16;
    let offset_high = ((handler_addr >> 32) & 0xFFFF_FFFF) as u32;

    // Type = 0xE (64-bit interrupt gate), P=1
    let type_attr = 0x8E | ((dpl & 0x3) << 5);

    storage[entry_offset..entry_offset + 2].copy_from_slice(&offset_low.to_le_bytes());
    storage[entry_offset + 2..entry_offset + 4].copy_from_slice(&selector.to_le_bytes());
    storage[entry_offset + 4] = ist & 0x7;
    storage[entry_offset + 5] = type_attr;
    storage[entry_offset + 6..entry_offset + 8].copy_from_slice(&offset_mid.to_le_bytes());
    storage[entry_offset + 8..entry_offset + 12].copy_from_slice(&offset_high.to_le_bytes());
    storage[entry_offset + 12..entry_offset + 16].fill(0);
}

// ============================================================================
// Descriptor Table Pointers
// ============================================================================

/// Create a GDT/IDT descriptor (10 bytes: 2-byte limit + 8-byte base)
pub fn create_descriptor(base: u64, limit: u16) -> [u8; 10] {
    let mut desc = [0u8; 10];
    desc[0..2].copy_from_slice(&limit.to_le_bytes());
    desc[2..10].copy_from_slice(&base.to_le_bytes());
    desc
}
