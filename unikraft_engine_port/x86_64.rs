//! x86_64 Architecture Support for Unikraft Engine
//!
//! This module handles:
//! - 4-level page table setup (PML4 → PDPT → PD → PT)
//! - GDT with kernel and user segments
//! - TSS with interrupt stack table (IST)
//! - IDT with 33 interrupt gates (vectors 0-32)
//!
//! Structure closely mirrors Dandelion's `kvm/x86_64.rs` for easy comparison.

use super::{UnikraftError, UnikraftResult, PAGE_SIZE};

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
///
/// GDT Layout (must match what we create in setup_gdt):
/// ```text
/// Index 0: Null descriptor
/// Index 1: Kernel Code 64-bit (0x08)
/// Index 2: Kernel Data (0x10)
/// Index 3: User Code 64-bit (0x18) - with DPL=3, selector with RPL=3 is 0x1B
/// Index 4: User Data (0x20) - with DPL=3, selector with RPL=3 is 0x23
/// Index 5-6: TSS (0x28, spans 2 entries)
/// ```
pub mod selectors {
    pub const NULL: u16 = 0x00;
    pub const KERNEL_CODE_64: u16 = 0x08;
    pub const KERNEL_DATA: u16 = 0x10;
    pub const USER_CODE_64: u16 = 0x18; // DPL=3 in descriptor
    pub const USER_DATA: u16 = 0x20; // DPL=3 in descriptor
    pub const TSS: u16 = 0x28;

    /// User code selector with RPL=3 (for IRET)
    pub const USER_CODE_64_RPL3: u16 = USER_CODE_64 | 3; // 0x1B
    /// User data selector with RPL=3 (for IRET)
    pub const USER_DATA_RPL3: u16 = USER_DATA | 3; // 0x23
}

// ============================================================================
// Constants - Structure Sizes
// ============================================================================

/// GDT size: 8 entries × 8 bytes = 64 bytes
/// (TSS spans 2 entries at index 5-6)
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

/// Trampoline region size (K2U + U2K code + data)
pub const TRAMPOLINE_K2U_SIZE: usize = PAGE_SIZE; // 1 page for K2U
pub const TRAMPOLINE_U2K_SIZE: usize = PAGE_SIZE; // 1 page for U2K
pub const TRAMPOLINE_REGION_SIZE: usize = TRAMPOLINE_K2U_SIZE + TRAMPOLINE_U2K_SIZE;

/// High virtual address for trampolines (mapped in both address spaces)
/// ~2TB mark, well above typical user space
pub const TRAMPOLINE_VA_BASE: u64 = 0x0000_0200_0000_0000;

// ============================================================================
// Page Table Sizing
// ============================================================================

/// Calculate number of page table pages needed for a given context size
///
/// For up to 512GB (full PDPT), we need:
/// - 1 PML4 page
/// - 1 PDPT page (512 entries × 1GB each = 512GB max)
/// - Up to 512 PD pages (one per PDPT entry used)
/// - Up to 512×512 PT pages (one per PD entry used for 4KB pages)
///
/// For simplicity, we pre-allocate based on context_size:
/// - 1 PML4
/// - 1 PDPT  
/// - ceil(context_size / 1GB) PD pages
/// - ceil(context_size / 2MB) PT pages (for 4KB mapping)
///
/// If using 2MB pages for most of the context, PT count is much lower.
pub fn calculate_page_table_pages(context_size: usize, use_large_pages: bool) -> usize {
    let pml4_pages = 1;
    let pdpt_pages = 1;

    // Number of 1GB regions needed
    let gb_regions = (context_size + (1 << 30) - 1) / (1 << 30);
    let pd_pages = gb_regions.max(1);

    if use_large_pages {
        // Using 2MB pages: no PT pages needed for main mapping
        // But we still need some for 4KB pages (trampolines, handlers, etc.)
        let pt_pages = 4; // Reserve a few for fine-grained mappings
        pml4_pages + pdpt_pages + pd_pages + pt_pages
    } else {
        // Using 4KB pages: need PT for each 2MB region
        let mb2_regions = (context_size + (1 << 21) - 1) / (1 << 21);
        let pt_pages = mb2_regions;
        pml4_pages + pdpt_pages + pd_pages + pt_pages
    }
}

/// Calculate total space needed for page tables
pub fn page_table_space_needed(context_size: usize, use_large_pages: bool) -> usize {
    calculate_page_table_pages(context_size, use_large_pages) * PAGE_SIZE
}

// ============================================================================
// Page Table Setup
// ============================================================================

/// Page table allocation state during setup
pub struct PageTableAllocator<'a> {
    storage: &'a mut [u8],
    /// Current allocation offset (grows downward from top)
    current_offset: usize,
    /// Physical base address of storage
    phys_base: u64,
}

impl<'a> PageTableAllocator<'a> {
    pub fn new(storage: &'a mut [u8], phys_base: u64) -> Self {
        Self {
            current_offset: storage.len(),
            storage,
            phys_base,
        }
    }

    /// Allocate a page-aligned page table page
    /// Returns (offset_in_storage, physical_address)
    pub fn alloc_page(&mut self) -> UnikraftResult<(usize, u64)> {
        if self.current_offset < PAGE_SIZE {
            return Err(UnikraftError::ContextTooSmall);
        }
        self.current_offset -= PAGE_SIZE;
        let offset = self.current_offset;

        // Zero the page
        self.storage[offset..offset + PAGE_SIZE].fill(0);

        let phys = self.phys_base + offset as u64;
        Ok((offset, phys))
    }

    /// Get current stack_start (top of used region)
    pub fn stack_start(&self) -> usize {
        self.current_offset
    }
}

/// Set up 4-level page tables for user space
///
/// This function mirrors `set_page_table()` in Dandelion's kvm/x86_64.rs.
///
/// # Arguments
/// * `storage` - Guest memory buffer
/// * `stack_start` - Mutable reference tracking available memory (modified)
/// * `context_size` - Total size of the context
/// * `phys_base` - Physical base address of storage (for CR3)
/// * `use_large_pages` - Whether to use 2MB pages where possible
///
/// # Returns
/// * Physical address of PML4 (for CR3)
pub fn set_page_table(
    storage: &mut [u8],
    stack_start: &mut usize,
    context_size: usize,
    phys_base: u64,
    use_large_pages: bool,
) -> UnikraftResult<u64> {
    let mut alloc = PageTableAllocator::new(storage, phys_base);

    // Allocate PML4
    let (pml4_offset, pml4_phys) = alloc.alloc_page()?;

    // Allocate PDPT
    let (pdpt_offset, pdpt_phys) = alloc.alloc_page()?;

    // Link PML4[0] → PDPT
    write_pte(
        &mut alloc.storage,
        pml4_offset,
        0,
        pdpt_phys | pte_flags::TABLE_ENTRY,
    );

    // Calculate how many 1GB regions we need
    let num_gb_regions = (context_size + (1 << 30) - 1) / (1 << 30);
    let num_gb_regions = num_gb_regions.max(1).min(512); // At least 1, at most 512

    // For each 1GB region, allocate a PD
    for gb_idx in 0..num_gb_regions {
        let (pd_offset, pd_phys) = alloc.alloc_page()?;

        // Link PDPT[gb_idx] → PD
        write_pte(
            &mut alloc.storage,
            pdpt_offset,
            gb_idx,
            pd_phys | pte_flags::TABLE_ENTRY,
        );

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
                    &mut alloc.storage,
                    pd_offset,
                    mb_idx,
                    paddr | pte_flags::USER_PAGE | pte_flags::HUGE_PAGE,
                );
            } else {
                // Allocate PT for 4KB pages
                let (pt_offset, pt_phys) = alloc.alloc_page()?;
                write_pte(
                    &mut alloc.storage,
                    pd_offset,
                    mb_idx,
                    pt_phys | pte_flags::TABLE_ENTRY,
                );

                // Map 512 4KB pages in this PT
                for page_idx in 0..512 {
                    let page_vaddr = vaddr + page_idx * PAGE_SIZE;
                    if page_vaddr >= context_size {
                        break;
                    }
                    let page_paddr = phys_base + page_vaddr as u64;
                    write_pte(
                        &mut alloc.storage,
                        pt_offset,
                        page_idx,
                        page_paddr | pte_flags::USER_PAGE,
                    );
                }
            }
        }
    }

    // Update stack_start to reflect allocated pages
    *stack_start = alloc.stack_start();

    Ok(pml4_phys)
}

/// Map additional pages at a specific virtual address
///
/// Used for mapping trampolines at high addresses, etc.
pub fn map_pages_at_va(
    storage: &mut [u8],
    pml4_offset: usize,
    phys_base: u64,
    va_start: u64,
    pa_start: u64,
    num_pages: usize,
    flags: u64,
    stack_start: &mut usize,
) -> UnikraftResult<()> {
    for i in 0..num_pages {
        let va = va_start + (i * PAGE_SIZE) as u64;
        let pa = pa_start + (i * PAGE_SIZE) as u64;
        map_single_4kb_page(storage, pml4_offset, phys_base, va, pa, flags, stack_start)?;
    }
    Ok(())
}

/// Map a single 4KB page, allocating intermediate tables as needed
fn map_single_4kb_page(
    storage: &mut [u8],
    pml4_offset: usize,
    phys_base: u64,
    va: u64,
    pa: u64,
    flags: u64,
    stack_start: &mut usize,
) -> UnikraftResult<()> {
    let pml4_idx = ((va >> 39) & 0x1FF) as usize;
    let pdpt_idx = ((va >> 30) & 0x1FF) as usize;
    let pd_idx = ((va >> 21) & 0x1FF) as usize;
    let pt_idx = ((va >> 12) & 0x1FF) as usize;

    // Get or create PDPT
    let pdpt_offset = get_or_alloc_table(storage, pml4_offset, pml4_idx, phys_base, stack_start)?;

    // Get or create PD
    let pd_offset = get_or_alloc_table(storage, pdpt_offset, pdpt_idx, phys_base, stack_start)?;

    // Get or create PT
    let pt_offset = get_or_alloc_table(storage, pd_offset, pd_idx, phys_base, stack_start)?;

    // Write the leaf entry
    write_pte(storage, pt_offset, pt_idx, pa | flags);

    Ok(())
}

/// Get existing table or allocate a new one
fn get_or_alloc_table(
    storage: &mut [u8],
    parent_offset: usize,
    index: usize,
    phys_base: u64,
    stack_start: &mut usize,
) -> UnikraftResult<usize> {
    let entry = read_pte(storage, parent_offset, index);

    if entry & pte_flags::PRESENT != 0 {
        // Table exists
        let table_phys = entry & 0x000F_FFFF_FFFF_F000;
        let table_offset = (table_phys - phys_base) as usize;
        Ok(table_offset)
    } else {
        // Allocate new table
        if *stack_start < PAGE_SIZE {
            return Err(UnikraftError::ContextTooSmall);
        }
        *stack_start -= PAGE_SIZE;
        let new_offset = *stack_start;
        storage[new_offset..new_offset + PAGE_SIZE].fill(0);

        let new_phys = phys_base + new_offset as u64;
        write_pte(
            storage,
            parent_offset,
            index,
            new_phys | pte_flags::TABLE_ENTRY,
        );

        Ok(new_offset)
    }
}

/// Write a page table entry
fn write_pte(storage: &mut [u8], table_offset: usize, index: usize, value: u64) {
    let offset = table_offset + index * 8;
    storage[offset..offset + 8].copy_from_slice(&value.to_le_bytes());
}

/// Read a page table entry
fn read_pte(storage: &[u8], table_offset: usize, index: usize) -> u64 {
    let offset = table_offset + index * 8;
    u64::from_le_bytes(storage[offset..offset + 8].try_into().unwrap())
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
pub fn set_interrupt_table(
    storage: &mut [u8],
    stack_start: &mut usize,
    phys_base: u64,
) -> UnikraftResult<(usize, usize, usize, usize, u64)> {
    // Allocate interrupt stack (6 pages)
    if *stack_start < INTERRUPT_STACK_SIZE {
        return Err(UnikraftError::ContextTooSmall);
    }
    *stack_start -= INTERRUPT_STACK_SIZE;
    let interrupt_stack_offset = *stack_start;
    let interrupt_stack_top =
        phys_base + interrupt_stack_offset as u64 + INTERRUPT_STACK_SIZE as u64;
    storage[interrupt_stack_offset..interrupt_stack_offset + INTERRUPT_STACK_SIZE].fill(0);

    // Allocate handler code region (page aligned)
    let handler_code = super::handlers::get_handler_code();
    let handler_size = handler_code.len();
    let handler_pages = (handler_size + PAGE_SIZE - 1) / PAGE_SIZE;
    let handler_alloc = handler_pages * PAGE_SIZE;

    if *stack_start < handler_alloc {
        return Err(UnikraftError::ContextTooSmall);
    }
    *stack_start -= handler_alloc;
    let handler_offset = *stack_start;
    storage[handler_offset..handler_offset + handler_alloc].fill(0);
    storage[handler_offset..handler_offset + handler_size].copy_from_slice(handler_code);

    // Allocate GDT, TSS, IDT (8-byte aligned, contiguous)
    let structures_size = GDT_SIZE + TSS_SIZE + IDT_SIZE;
    let structures_size_aligned = (structures_size + 7) & !7;

    if *stack_start < structures_size_aligned {
        return Err(UnikraftError::ContextTooSmall);
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
) -> UnikraftResult<()> {
    let tss_base = phys_base + tss_offset as u64;

    // Entry 0: Null descriptor
    write_gdt_entry(storage, gdt_offset, 0, 0);

    // Entry 1: Kernel Code 64-bit (DPL=0)
    // Flags: L=1 (long mode), P=1, S=1, Type=0xA (execute/read)
    write_gdt_entry(storage, gdt_offset, 1, 0x00AF_9A00_0000_FFFF);

    // Entry 2: Kernel Data (DPL=0)
    // Flags: P=1, S=1, Type=0x2 (read/write)
    write_gdt_entry(storage, gdt_offset, 2, 0x00CF_9200_0000_FFFF);

    // Entry 3: User Code 64-bit (DPL=3)
    // Flags: L=1 (long mode), P=1, DPL=3, S=1, Type=0xA (execute/read)
    write_gdt_entry(storage, gdt_offset, 3, 0x00AF_FA00_0000_FFFF);

    // Entry 4: User Data (DPL=3)
    // Flags: P=1, DPL=3, S=1, Type=0x2 (read/write)
    write_gdt_entry(storage, gdt_offset, 4, 0x00CF_F200_0000_FFFF);

    // Entry 5-6: TSS descriptor (16 bytes for 64-bit TSS)
    // TSS limit = 103 (0x67), Type = 0x9 (available 64-bit TSS)
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
) -> UnikraftResult<()> {
    // TSS structure (104 bytes for 64-bit):
    // Offset 0x00: Reserved (4 bytes)
    // Offset 0x04: RSP0 (8 bytes) - Ring 0 stack
    // Offset 0x0C: RSP1 (8 bytes)
    // Offset 0x14: RSP2 (8 bytes)
    // Offset 0x1C: Reserved (8 bytes)
    // Offset 0x24: IST1 (8 bytes) - Interrupt Stack Table entry 1
    // ... IST2-IST7 ...
    // Offset 0x64: Reserved (2 bytes)
    // Offset 0x66: I/O Map Base (2 bytes)

    // RSP0 - used when transitioning from Ring 3 to Ring 0
    write_tss_field(storage, tss_offset, 0x04, interrupt_stack_top);

    // IST1 - used by interrupt handlers (via IDT IST field)
    write_tss_field(storage, tss_offset, 0x24, interrupt_stack_top);

    // I/O Map Base - set to TSS limit to indicate no I/O bitmap
    storage[tss_offset + 0x66] = 0x68; // Just past TSS
    storage[tss_offset + 0x67] = 0x00;

    Ok(())
}

/// Write a 64-bit field to TSS
fn write_tss_field(storage: &mut [u8], tss_offset: usize, field_offset: usize, value: u64) {
    let offset = tss_offset + field_offset;
    storage[offset..offset + 8].copy_from_slice(&value.to_le_bytes());
}

/// Set up IDT with 33 interrupt gates
fn setup_idt(storage: &mut [u8], idt_offset: usize, handler_base_va: u64) -> UnikraftResult<()> {
    // Get handler offsets from handlers module
    let handler_offsets = super::handlers::get_handler_offsets();

    for vector in 0..IDT_ENTRIES {
        let handler_va = handler_base_va + handler_offsets[vector] as u64;
        setup_interrupt_gate(
            storage,
            idt_offset,
            vector,
            selectors::KERNEL_CODE_64, // Handler runs in kernel code segment
            handler_va,
            3, // DPL=3 - allow user mode to trigger (for INT 32)
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

    // IDT Gate Descriptor (16 bytes):
    // Bytes 0-1: Offset 15:0
    // Bytes 2-3: Segment Selector
    // Byte 4: IST (bits 0-2), Reserved (bits 3-7)
    // Byte 5: Type (bits 0-3), 0 (bit 4), DPL (bits 5-6), P (bit 7)
    // Bytes 6-7: Offset 31:16
    // Bytes 8-11: Offset 63:32
    // Bytes 12-15: Reserved

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
    storage[entry_offset + 12..entry_offset + 16].fill(0); // Reserved
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

// ============================================================================
// Page Table Walking Utilities
// ============================================================================

/// Unikraft directmap base
pub const DIRECTMAP_BASE: u64 = 0xffffff8000000000;

/// Mask for physical address in PTE
const PTE_ADDR_MASK: u64 = 0x000F_FFFF_FFFF_F000;

/// Walk page tables to translate virtual address to physical address
///
/// Uses the directmap to access page table entries.
///
/// # Safety
/// - `cr3` must be a valid page table root
/// - The virtual address must be mapped
pub unsafe fn virt_to_phys(cr3: u64, va: u64) -> Result<u64, &'static str> {
    let pml4_idx = ((va >> 39) & 0x1FF) as usize;
    let pdpt_idx = ((va >> 30) & 0x1FF) as usize;
    let pd_idx = ((va >> 21) & 0x1FF) as usize;
    let pt_idx = ((va >> 12) & 0x1FF) as usize;
    let offset = va & 0xFFF;

    // PML4
    let pml4_phys = cr3 & PTE_ADDR_MASK;
    let pml4 = (DIRECTMAP_BASE + pml4_phys) as *const u64;
    let pml4e = core::ptr::read_volatile(pml4.add(pml4_idx));
    if (pml4e & pte_flags::PRESENT) == 0 {
        return Err("PML4 entry not present");
    }

    // PDPT
    let pdpt_phys = pml4e & PTE_ADDR_MASK;
    let pdpt = (DIRECTMAP_BASE + pdpt_phys) as *const u64;
    let pdpte = core::ptr::read_volatile(pdpt.add(pdpt_idx));
    if (pdpte & pte_flags::PRESENT) == 0 {
        return Err("PDPT entry not present");
    }
    // Check for 1GB page
    if (pdpte & pte_flags::HUGE_PAGE) != 0 {
        let page_phys = pdpte & 0xFFFF_FFFF_C000_0000;
        return Ok(page_phys | (va & 0x3FFF_FFFF));
    }

    // PD
    let pd_phys = pdpte & PTE_ADDR_MASK;
    let pd = (DIRECTMAP_BASE + pd_phys) as *const u64;
    let pde = core::ptr::read_volatile(pd.add(pd_idx));
    if (pde & pte_flags::PRESENT) == 0 {
        return Err("PD entry not present");
    }
    // Check for 2MB page
    if (pde & pte_flags::HUGE_PAGE) != 0 {
        let page_phys = pde & 0xFFFF_FFFF_FFE0_0000;
        return Ok(page_phys | (va & 0x1F_FFFF));
    }

    // PT
    let pt_phys = pde & PTE_ADDR_MASK;
    let pt = (DIRECTMAP_BASE + pt_phys) as *const u64;
    let pte = core::ptr::read_volatile(pt.add(pt_idx));
    if (pte & pte_flags::PRESENT) == 0 {
        return Err("PT entry not present");
    }

    let page_phys = pte & PTE_ADDR_MASK;
    Ok(page_phys | offset)
}

/// Map a physical address at a virtual address in existing page tables
///
/// # Safety
/// - `cr3` must be a valid page table root
/// - Caller must ensure no conflicts with existing mappings
pub unsafe fn map_page(
    cr3: u64,
    va: u64,
    pa: u64,
    flags: u64,
    allocator: &mut impl FnMut() -> Option<u64>, // Returns physical address of new page
) -> Result<(), &'static str> {
    let pml4_idx = ((va >> 39) & 0x1FF) as usize;
    let pdpt_idx = ((va >> 30) & 0x1FF) as usize;
    let pd_idx = ((va >> 21) & 0x1FF) as usize;
    let pt_idx = ((va >> 12) & 0x1FF) as usize;

    // PML4
    let pml4_phys = cr3 & PTE_ADDR_MASK;
    let pml4 = (DIRECTMAP_BASE + pml4_phys) as *mut u64;
    let mut pml4e = core::ptr::read_volatile(pml4.add(pml4_idx));
    
    if (pml4e & pte_flags::PRESENT) == 0 {
        let new_page = allocator().ok_or("Failed to allocate PDPT")?;
        core::ptr::write_bytes((DIRECTMAP_BASE + new_page) as *mut u8, 0, PAGE_SIZE);
        pml4e = new_page | pte_flags::TABLE_ENTRY;
        core::ptr::write_volatile(pml4.add(pml4_idx), pml4e);
    }

    // PDPT
    let pdpt_phys = pml4e & PTE_ADDR_MASK;
    let pdpt = (DIRECTMAP_BASE + pdpt_phys) as *mut u64;
    let mut pdpte = core::ptr::read_volatile(pdpt.add(pdpt_idx));
    
    if (pdpte & pte_flags::PRESENT) == 0 {
        let new_page = allocator().ok_or("Failed to allocate PD")?;
        core::ptr::write_bytes((DIRECTMAP_BASE + new_page) as *mut u8, 0, PAGE_SIZE);
        pdpte = new_page | pte_flags::TABLE_ENTRY;
        core::ptr::write_volatile(pdpt.add(pdpt_idx), pdpte);
    }

    // PD
    let pd_phys = pdpte & PTE_ADDR_MASK;
    let pd = (DIRECTMAP_BASE + pd_phys) as *mut u64;
    let mut pde = core::ptr::read_volatile(pd.add(pd_idx));
    
    if (pde & pte_flags::PRESENT) == 0 {
        let new_page = allocator().ok_or("Failed to allocate PT")?;
        core::ptr::write_bytes((DIRECTMAP_BASE + new_page) as *mut u8, 0, PAGE_SIZE);
        pde = new_page | pte_flags::TABLE_ENTRY;
        core::ptr::write_volatile(pd.add(pd_idx), pde);
    }

    // PT
    let pt_phys = pde & PTE_ADDR_MASK;
    let pt = (DIRECTMAP_BASE + pt_phys) as *mut u64;
    core::ptr::write_volatile(pt.add(pt_idx), pa | flags);

    Ok(())
}

// ============================================================================
// TLS Initialization for APs
// ============================================================================

/// Initialize Thread-Local Storage for an AP
///
/// This allocates TLS area and sets up FS base register.
///
/// # Safety
/// - Must be called early in AP initialization
/// - TLS linker symbols must be valid
///
/// # Returns
/// The TLS base pointer on success
pub unsafe fn init_ap_tls() -> Result<*mut u8, &'static str> {
    // TLS symbols from linker
    extern "C" {
        static _tls_start: u8;
        static _etdata: u8;
        static _tls_end: u8;
    }

    let tls_start = &_tls_start as *const u8 as usize;
    let etdata = &_etdata as *const u8 as usize;
    let tls_end = &_tls_end as *const u8 as usize;

    let tdata_len = etdata - tls_start;
    let tbss_len = tls_end - etdata;
    let total_len = tdata_len + tbss_len;

    if total_len == 0 {
        return Ok(core::ptr::null_mut());
    }

    // Allocate TLS area (use page allocation for simplicity)
    let tls_size = (total_len + PAGE_SIZE - 1) & !(PAGE_SIZE - 1);
    let layout = core::alloc::Layout::from_size_align(tls_size, PAGE_SIZE)
        .map_err(|_| "Invalid TLS layout")?;
    
    // Note: This requires an allocator. In bare metal, use a static pool or boot allocator.
    #[cfg(feature = "std")]
    let tls_area = {
        extern crate alloc;
        alloc::alloc::alloc_zeroed(layout)
    };
    
    #[cfg(not(feature = "std"))]
    let tls_area: *mut u8 = return Err("TLS allocation requires allocator");

    if tls_area.is_null() {
        return Err("Failed to allocate TLS area");
    }

    // Copy .tdata section
    core::ptr::copy_nonoverlapping(
        &_tls_start as *const u8,
        tls_area,
        tdata_len,
    );

    // .tbss is already zeroed by alloc_zeroed

    // Set FS base (for TLS access)
    // Method 1: WRFSBASE (if FSGSBASE is enabled)
    // Method 2: WRMSR to FS_BASE (0xC0000100)
    const MSR_FS_BASE: u32 = 0xC000_0100;
    let tls_base = tls_area as u64;
    
    // Use WRMSR (works on all x86_64)
    core::arch::asm!(
        "wrmsr",
        in("ecx") MSR_FS_BASE,
        in("eax") tls_base as u32,
        in("edx") (tls_base >> 32) as u32,
        options(nomem, nostack)
    );

    Ok(tls_area)
}
