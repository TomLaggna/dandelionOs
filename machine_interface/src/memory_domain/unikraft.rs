//! Unikraft Memory Domain
//!
//! This module provides a memory domain for the Unikraft execution engine.
//! Unlike KVM which uses hardware virtualization, Unikraft runs user code
//! in Ring 3 with separate page tables within the same kernel address space.
//!
//! The UnikraftContext owns a contiguous memory buffer that contains:
//! - User code and data (ELF segments)
//! - Page tables (PML4, PDPT, PD, PT)
//! - GDT, TSS, IDT for user space
//! - Interrupt handlers
//! - Trampolines (K2U, U2K)
//! - User stack
//! - Dandelion system data

use crate::memory_domain::{Context, ContextTrait, ContextType, MemoryDomain, MemoryResource};
use dandelion_commons::{DandelionError, DandelionResult};
use std::alloc::{alloc_zeroed, dealloc, Layout};

// ============================================================================
// Constants
// ============================================================================

/// Page size for x86_64
pub const PAGE_SIZE: usize = 4096;

/// Default context size (64 MiB)
pub const DEFAULT_CONTEXT_SIZE: usize = 64 * 1024 * 1024;

/// Minimum context size (must fit page tables, GDT/TSS/IDT, handlers, etc.)
pub const MIN_CONTEXT_SIZE: usize = 2 * 1024 * 1024; // 2 MiB

/// GDT size: 8 entries × 8 bytes = 64 bytes
pub const GDT_SIZE: usize = 64;

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
pub const TRAMPOLINE_REGION_SIZE: usize = 2 * PAGE_SIZE;

/// High virtual address for trampolines (mapped in both address spaces)
pub const TRAMPOLINE_VA_BASE: u64 = 0x0000_0200_0000_0000;

// ============================================================================
// Memory Layout
// ============================================================================

/// Pre-computed memory layout offsets within the context
///
/// Memory layout (high to low addresses within storage):
/// ```text
/// [Page Tables: PML4, PDPT, PD, PT] <- storage.len() - PAGE_SIZE * N
/// [Trampoline Code]                  <- page_table_offset - TRAMPOLINE_REGION_SIZE
/// [Interrupt Stack]                  <- trampoline_offset - INTERRUPT_STACK_SIZE
/// [IDT: 33 entries × 16 bytes]      <- interrupt_stack_offset - IDT size
/// [TSS: 104 bytes]                  <- idt_offset - TSS size (aligned)
/// [GDT: 8 entries × 8 bytes]        <- tss_offset - GDT size (aligned)
/// [Interrupt Handlers]              <- gdt_offset - handler size (page aligned)
/// [... available for user code/data/stack ...]
/// [User code at p_vaddr from ELF]   <- 0x0 (or ELF-specified addresses)
/// ```
#[derive(Debug, Clone, Copy, Default)]
pub struct UnikraftLayout {
    /// Total size of the context
    pub total_size: usize,
    /// Offset where page tables start (PML4 at highest address)
    pub page_table_offset: usize,
    /// Physical address of PML4 (for CR3) - set after mapping to physical memory
    pub pml4_phys: u64,
    /// Offset where trampoline code starts
    pub trampoline_offset: usize,
    /// Offset where interrupt stack starts
    pub interrupt_stack_offset: usize,
    /// Offset where IDT starts
    pub idt_offset: usize,
    /// Offset where TSS starts
    pub tss_offset: usize,
    /// Offset where GDT starts
    pub gdt_offset: usize,
    /// Offset where interrupt handlers start
    pub handler_offset: usize,
    /// Top of user stack (grows down from here)
    pub user_stack_top: usize,
    /// Entry point for K2U trampoline
    pub k2u_entry_offset: usize,
    /// Entry point for U2K trampoline
    pub u2k_entry_offset: usize,
}

impl UnikraftLayout {
    /// Calculate layout for a given context size
    pub fn calculate(context_size: usize) -> DandelionResult<Self> {
        if context_size < MIN_CONTEXT_SIZE {
            return Err(DandelionError::MemoryAllocationError);
        }

        let mut layout = UnikraftLayout {
            total_size: context_size,
            ..Default::default()
        };

        // Work from the top of the storage downward
        let mut current = context_size;

        // Allocate page tables (estimate: 1 PML4 + 1 PDPT + up to 64 PD pages + PT pages)
        // For 64MB context with 2MB pages: ~68 pages
        // For 64MB context with 4KB pages: ~100+ pages
        // We'll allocate conservatively
        let page_table_pages = Self::estimate_page_table_pages(context_size);
        current -= page_table_pages * PAGE_SIZE;
        layout.page_table_offset = current;

        // Allocate trampoline region (2 pages)
        current -= TRAMPOLINE_REGION_SIZE;
        layout.trampoline_offset = current;
        layout.k2u_entry_offset = current;
        layout.u2k_entry_offset = current + PAGE_SIZE;

        // Allocate interrupt stack
        current -= INTERRUPT_STACK_SIZE;
        layout.interrupt_stack_offset = current;

        // Allocate IDT (align to 8 bytes)
        current = (current - IDT_SIZE) & !7;
        layout.idt_offset = current;

        // Allocate TSS (align to 4 bytes)
        current = (current - TSS_SIZE) & !3;
        layout.tss_offset = current;

        // Allocate GDT (align to 8 bytes)
        current = (current - GDT_SIZE) & !7;
        layout.gdt_offset = current;

        // Reserve space for interrupt handlers (page aligned)
        // Handlers are ~1KB, round up to page
        current = (current - PAGE_SIZE) & !(PAGE_SIZE - 1);
        layout.handler_offset = current;

        // User stack grows down from handler_offset
        layout.user_stack_top = layout.handler_offset;

        Ok(layout)
    }

    /// Estimate number of page table pages needed
    fn estimate_page_table_pages(context_size: usize) -> usize {
        // 1 PML4 + 1 PDPT + PD pages + PT pages
        let pml4_pages = 1;
        let pdpt_pages = 1;

        // Number of 1GB regions (each needs a PD)
        let gb_regions = (context_size + (1 << 30) - 1) / (1 << 30);
        let pd_pages = gb_regions.max(1);

        // For 2MB pages, we don't need PT pages for main mapping
        // Reserve some for fine-grained mappings (handlers, trampolines)
        let pt_pages = 4;

        pml4_pages + pdpt_pages + pd_pages + pt_pages
    }

    /// Get total size
    pub fn size(&self) -> usize {
        self.total_size
    }

    /// Get available space for user code/data (from 0 to handler_offset)
    pub fn user_space_size(&self) -> usize {
        self.handler_offset
    }
}

// ============================================================================
// Unikraft Context
// ============================================================================

/// Unikraft directmap base address (VA = directmap_base + PA)
/// This is the standard Unikraft directmap region
pub const DIRECTMAP_BASE: u64 = 0xffffff8000000000;

/// Unikraft-specific context data
///
/// This is analogous to `MallocContext` or `MmapContext` but includes
/// additional metadata needed for the Unikraft engine.
#[derive(Debug)]
pub struct UnikraftContext {
    /// Memory layout for alignment (stored for deallocation)
    pub alloc_layout: Layout,
    /// The guest memory buffer - all user space data lives here
    /// This is a raw pointer to allow manual memory management
    pub storage: *mut u8,
    /// Pre-computed memory layout offsets
    pub layout: UnikraftLayout,
    /// Whether the context has been initialized (page tables, GDT, etc.)
    pub initialized: bool,
    /// Cached physical base address (computed lazily)
    phys_base_cache: Option<u64>,
}

// Safety: UnikraftContext owns its memory and can be sent between threads
unsafe impl Send for UnikraftContext {}
unsafe impl Sync for UnikraftContext {}

impl UnikraftContext {
    /// Get the storage as a mutable slice
    ///
    /// # Safety
    /// Caller must ensure no concurrent access to the storage
    pub unsafe fn as_slice_mut(&mut self) -> &mut [u8] {
        std::slice::from_raw_parts_mut(self.storage, self.layout.total_size)
    }

    /// Get the storage as a slice
    pub fn as_slice(&self) -> &[u8] {
        unsafe { std::slice::from_raw_parts(self.storage, self.layout.total_size) }
    }

    /// Get the physical address of the storage
    ///
    /// On Unikraft, we walk the kernel page tables using the directmap region
    /// to translate our virtual address to a physical address.
    pub fn get_phys_base(&self) -> u64 {
        // If we have a cached value, return it
        if let Some(phys) = self.phys_base_cache {
            return phys;
        }

        // Walk page tables to get physical address
        let va = self.storage as u64;

        #[cfg(target_arch = "x86_64")]
        {
            // Use the ap_startup module's virt_to_phys if available
            // For now, compute using directmap assumption:
            // If VA is in directmap region, PA = VA - DIRECTMAP_BASE
            // Otherwise, walk page tables

            if va >= DIRECTMAP_BASE {
                // VA is in directmap - simple subtraction
                return va - DIRECTMAP_BASE;
            }

            // Walk kernel page tables
            if let Some(pa) = unsafe { walk_page_tables(va, DIRECTMAP_BASE) } {
                return pa;
            }
        }

        // Fallback: return VA (works for identity mapping in development)
        va
    }

    /// Set the cached physical base address
    pub fn set_phys_base(&mut self, phys_base: u64) {
        self.phys_base_cache = Some(phys_base);
    }
}

/// Walk kernel page tables to translate VA to PA
///
/// # Safety
/// - The virtual address must be mapped
/// - Called in a context where page tables won't change
#[cfg(target_arch = "x86_64")]
unsafe fn walk_page_tables(va: u64, directmap_base: u64) -> Option<u64> {
    use core::arch::asm;

    // Get CR3
    let cr3: u64;
    asm!("mov {}, cr3", out(reg) cr3, options(nomem, nostack));
    let cr3 = cr3 & !0xFFF; // Mask off flags

    // Helper to convert PA to VA via directmap
    let pa_to_va = |pa: u64| -> u64 { directmap_base.wrapping_add(pa) };

    // PML4 index (bits 47:39)
    let pml4_idx = ((va >> 39) & 0x1FF) as usize;
    let pml4_va = pa_to_va(cr3);
    let pml4_entry = core::ptr::read_volatile((pml4_va as *const u64).add(pml4_idx));

    if (pml4_entry & 1) == 0 {
        return None; // Not present
    }

    // PDPT index (bits 38:30)
    let pdpt_pa = pml4_entry & 0x000F_FFFF_FFFF_F000;
    let pdpt_idx = ((va >> 30) & 0x1FF) as usize;
    let pdpt_va = pa_to_va(pdpt_pa);
    let pdpt_entry = core::ptr::read_volatile((pdpt_va as *const u64).add(pdpt_idx));

    if (pdpt_entry & 1) == 0 {
        return None; // Not present
    }

    // Check for 1GB page (PS bit = bit 7)
    if (pdpt_entry & 0x80) != 0 {
        let page_pa = pdpt_entry & 0x000F_FFFF_C000_0000;
        return Some(page_pa | (va & 0x3FFF_FFFF));
    }

    // PD index (bits 29:21)
    let pd_pa = pdpt_entry & 0x000F_FFFF_FFFF_F000;
    let pd_idx = ((va >> 21) & 0x1FF) as usize;
    let pd_va = pa_to_va(pd_pa);
    let pd_entry = core::ptr::read_volatile((pd_va as *const u64).add(pd_idx));

    if (pd_entry & 1) == 0 {
        return None; // Not present
    }

    // Check for 2MB page (PS bit = bit 7)
    if (pd_entry & 0x80) != 0 {
        let page_pa = pd_entry & 0x000F_FFFF_FFE0_0000;
        return Some(page_pa | (va & 0x1F_FFFF));
    }

    // PT index (bits 20:12)
    let pt_pa = pd_entry & 0x000F_FFFF_FFFF_F000;
    let pt_idx = ((va >> 12) & 0x1FF) as usize;
    let pt_va = pa_to_va(pt_pa);
    let pt_entry = core::ptr::read_volatile((pt_va as *const u64).add(pt_idx));

    if (pt_entry & 1) == 0 {
        return None; // Not present
    }

    let page_pa = pt_entry & 0x000F_FFFF_FFFF_F000;
    Some(page_pa | (va & 0xFFF))
}

#[cfg(not(target_arch = "x86_64"))]
unsafe fn walk_page_tables(_va: u64, _directmap_base: u64) -> Option<u64> {
    None
}

impl Drop for UnikraftContext {
    fn drop(&mut self) {
        if !self.storage.is_null() {
            unsafe {
                dealloc(self.storage, self.alloc_layout);
            }
        }
    }
}

impl ContextTrait for UnikraftContext {
    fn write<T>(&mut self, offset: usize, data: &[T]) -> DandelionResult<()> {
        // Check alignment
        if offset % core::mem::align_of::<T>() != 0 {
            return Err(DandelionError::WriteMisaligned);
        }

        // Check bounds
        let write_length = data.len() * core::mem::size_of::<T>();
        let length = self.layout.total_size;
        if write_length + offset > length {
            return Err(DandelionError::InvalidWrite);
        }

        // Copy data
        let buffer =
            unsafe { core::slice::from_raw_parts(data.as_ptr() as *const u8, write_length) };
        let storage_slice = unsafe { std::slice::from_raw_parts_mut(self.storage, length) };
        let target = &mut storage_slice[offset..offset + write_length];
        target.copy_from_slice(buffer);

        Ok(())
    }

    fn read<T>(&self, offset: usize, read_buffer: &mut [T]) -> DandelionResult<()> {
        // Check alignment
        if offset % core::mem::align_of::<T>() != 0 {
            return Err(DandelionError::ReadMisaligned);
        }

        let length = self.layout.total_size;
        let read_size = core::mem::size_of::<T>() * read_buffer.len();
        if offset + read_size > length {
            return Err(DandelionError::InvalidRead);
        }

        let read_memory = unsafe {
            core::slice::from_raw_parts_mut(read_buffer.as_mut_ptr() as *mut u8, read_size)
        };
        let storage_slice = unsafe { std::slice::from_raw_parts(self.storage, length) };

        for index in 0..read_size {
            read_memory[index] = storage_slice[offset + index];
        }

        Ok(())
    }

    fn get_chunk_ref(&self, offset: usize, length: usize) -> DandelionResult<&[u8]> {
        if offset + length > self.layout.total_size {
            return Err(DandelionError::InvalidRead);
        }
        Ok(unsafe {
            &std::slice::from_raw_parts(self.storage, self.layout.total_size)
                [offset..offset + length]
        })
    }
}

// ============================================================================
// Unikraft Memory Domain
// ============================================================================

/// Memory domain for Unikraft execution engine
#[derive(Debug)]
pub struct UnikraftMemoryDomain {
    /// Default context size for this domain
    default_size: usize,
}

impl MemoryDomain for UnikraftMemoryDomain {
    fn init(config: MemoryResource) -> DandelionResult<Box<dyn MemoryDomain>> {
        let default_size = match config {
            MemoryResource::Anonymous { size } => {
                if size == 0 {
                    DEFAULT_CONTEXT_SIZE
                } else {
                    size
                }
            }
            MemoryResource::None => DEFAULT_CONTEXT_SIZE,
            _ => {
                return Err(DandelionError::DomainError(
                    dandelion_commons::DomainError::ConfigMissmatch,
                ))
            }
        };

        if default_size < MIN_CONTEXT_SIZE {
            return Err(DandelionError::MemoryAllocationError);
        }

        Ok(Box::new(UnikraftMemoryDomain { default_size }))
    }

    fn acquire_context(&self, size: usize) -> DandelionResult<Context> {
        let actual_size = if size == 0 { self.default_size } else { size };
        log::debug!("UnikraftMemoryDomain::acquire_context - requested size: {}, actual_size: {}, default_size: {}", 
            size, actual_size, self.default_size);

        if actual_size < MIN_CONTEXT_SIZE {
            log::error!(
                "UnikraftMemoryDomain::acquire_context - actual_size {} < MIN_CONTEXT_SIZE {}",
                actual_size,
                MIN_CONTEXT_SIZE
            );
            return Err(DandelionError::MemoryAllocationError);
        }

        // Calculate layout
        log::debug!("UnikraftMemoryDomain::acquire_context - calculating layout...");
        let layout = UnikraftLayout::calculate(actual_size)?;
        log::debug!("UnikraftMemoryDomain::acquire_context - layout calculated successfully");

        // Allocate page-aligned memory
        let alloc_layout = match Layout::from_size_align(actual_size, PAGE_SIZE) {
            Ok(l) => l,
            Err(e) => {
                log::error!(
                    "UnikraftMemoryDomain::acquire_context - Layout::from_size_align failed: {:?}",
                    e
                );
                return Err(DandelionError::MemoryAllocationError);
            }
        };

        log::debug!(
            "UnikraftMemoryDomain::acquire_context - allocating {} bytes...",
            actual_size
        );
        let storage = unsafe { alloc_zeroed(alloc_layout) };
        if storage.is_null() {
            log::error!("UnikraftMemoryDomain::acquire_context - alloc_zeroed returned null");
            return Err(DandelionError::MemoryAllocationError);
        }
        log::debug!(
            "UnikraftMemoryDomain::acquire_context - allocation successful at {:p}",
            storage
        );

        let context = Box::new(UnikraftContext {
            alloc_layout,
            storage,
            layout,
            initialized: false,
            phys_base_cache: None, // Will be computed on first access
        });

        log::info!(
            "UnikraftMemoryDomain::acquire_context - context created successfully (size: {})",
            actual_size
        );
        Ok(Context::new(ContextType::Unikraft(context), actual_size))
    }
}

// ============================================================================
// Transfer Functions
// ============================================================================

/// Transfer data between Unikraft contexts
pub fn unikraft_transfer(
    destination: &mut UnikraftContext,
    source: &UnikraftContext,
    destination_offset: usize,
    source_offset: usize,
    size: usize,
) -> DandelionResult<()> {
    // Bounds check
    if source.layout.total_size < source_offset + size {
        return Err(DandelionError::InvalidRead);
    }
    if destination.layout.total_size < destination_offset + size {
        return Err(DandelionError::InvalidWrite);
    }

    unsafe {
        let src_slice = std::slice::from_raw_parts(source.storage, source.layout.total_size);
        let dst_slice =
            std::slice::from_raw_parts_mut(destination.storage, destination.layout.total_size);
        dst_slice[destination_offset..destination_offset + size]
            .copy_from_slice(&src_slice[source_offset..source_offset + size]);
    }

    Ok(())
}

#[cfg(feature = "bytes_context")]
/// Transfer from BytesContext to UnikraftContext
pub fn bytes_to_unikraft_transfer(
    destination: &mut UnikraftContext,
    source: &crate::memory_domain::bytes_context::BytesContext,
    destination_offset: usize,
    source_offset: usize,
    size: usize,
) -> DandelionResult<()> {
    let source_data = source.get_chunk_ref(source_offset, size)?;

    if destination.layout.total_size < destination_offset + size {
        return Err(DandelionError::InvalidWrite);
    }

    unsafe {
        let dst_slice =
            std::slice::from_raw_parts_mut(destination.storage, destination.layout.total_size);
        dst_slice[destination_offset..destination_offset + size].copy_from_slice(source_data);
    }

    Ok(())
}
