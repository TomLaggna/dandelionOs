//! Unikraft Execution Engine for Dandelion
//!
//! This module provides user-space isolation within a Unikraft kernel using
//! CR3-switching trampolines. Unlike KVM which runs in a separate VM, this
//! engine runs user code in Ring 3 within the same physical address space
//! but with separate page tables.
//!
//! # Architecture
//!
//! ```text
//! ┌─────────────────────────────────────────────────────────────┐
//! │ Unikraft Kernel (Ring 0)                                    │
//! │  ┌────────────────────┐    ┌─────────────────────────────┐  │
//! │  │ UnikraftLoop       │    │ User Space (Ring 3)         │  │
//! │  │ - setup page tables│◄──►│ - Own CR3 (page tables)     │  │
//! │  │ - setup GDT/TSS/IDT│    │ - Own GDT/IDT/TSS          │  │
//! │  │ - K2U/U2K trampoline│   │ - User code execution       │  │
//! │  └────────────────────┘    └─────────────────────────────┘  │
//! └─────────────────────────────────────────────────────────────┘
//! ```
//!
//! # Differences from KVM Engine
//!
//! | Aspect | KVM | Unikraft |
//! |--------|-----|----------|
//! | Isolation | Separate VM | Same kernel, separate page tables |
//! | Ring transition | KVM handles | Manual via IRETQ/trampolines |
//! | CR3 management | KVM internal | Explicit CR3 switching |
//! | Interrupt handling | KVM vCPU | Custom IDT in user space |

pub mod ap_startup;
pub mod boot_trampoline;
pub mod handlers;
pub mod trampolines;
pub mod x86_64;

use core::ffi::c_int;

// ============================================================================
// Constants matching Dandelion's interface
// ============================================================================

/// Page size for x86_64
pub const PAGE_SIZE: usize = 4096;

/// Default context size (64 MiB)
pub const DEFAULT_CONTEXT_SIZE: usize = 64 * 1024 * 1024;

// ============================================================================
// Context Types (mirrors Dandelion's ContextType pattern)
// ============================================================================

/// Unikraft-specific context data
///
/// This is analogous to `KvmContext` in Dandelion's kvm.rs
#[derive(Debug)]
pub struct UnikraftContext {
    /// The guest memory buffer - all user space data lives here
    /// Analogous to KvmContext.storage
    pub storage: Box<[u8]>,

    /// Cached layout information (computed once during setup)
    pub layout: UnikraftLayout,
}

/// Pre-computed memory layout offsets within the context
///
/// Memory layout (high to low addresses):
/// ```text
/// [Page Tables: PML4, PDPT, PD, PT] <- storage.len() - PAGE_SIZE * 4
/// [Trampoline Tables: U2K, K2U]     <- page_table_offset - TRAMPOLINE_REGION_SIZE
/// [Interrupt Stack]                  <- trampoline_offset - INTERRUPT_STACK_SIZE
/// [IDT: 33 entries × 16 bytes]      <- interrupt_stack_offset - IDT size
/// [TSS: 104 bytes]                  <- idt_offset - TSS size (aligned)
/// [GDT: 10 entries × 8 bytes]       <- tss_offset - GDT size (aligned)
/// [Interrupt Handlers]              <- gdt_offset - handler size (page aligned)
/// [... gap ...]
/// [User heap grows up]
/// [User data]
/// [User code at p_vaddr from ELF]   <- 0x0 (or ELF-specified addresses)
/// ```
#[derive(Debug, Clone, Copy, Default)]
pub struct UnikraftLayout {
    /// Offset where page tables start (PML4 at highest address)
    pub page_table_offset: usize,
    /// Offset where trampoline code/data starts
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
    /// Physical address of PML4 (for CR3)
    pub pml4_phys: u64,
    /// Virtual address where trampolines are mapped (high VA, ~2TB)
    pub trampoline_va: u64,
    /// Top of user stack
    pub user_stack_top: u64,
    /// Entry point for K2U trampoline (VA in trampoline region)
    pub k2u_entry_va: u64,
    /// Entry point for U2K trampoline (VA in trampoline region)  
    pub u2k_entry_va: u64,
}

// ============================================================================
// ELF Configuration (matches Dandelion's ElfConfig)
// ============================================================================

/// Configuration extracted from ELF file
///
/// This matches Dandelion's `ElfConfig` struct in function_driver.rs
#[derive(Clone, Debug)]
pub struct ElfConfig {
    /// Offset of `__dandelion_system_data` symbol in context
    pub system_data_offset: usize,
    /// Entry point address from ELF header
    pub entry_point: usize,
    /// Memory protection flags per region (from ELF program headers)
    pub protection_flags: Vec<(usize, usize, u32)>, // (start, size, flags)
}

// ============================================================================
// DandelionSystemData (matches Dandelion's interface.rs)
// ============================================================================

/// System data structure shared between runtime and user code
///
/// This MUST match the layout in Dandelion's interface.rs exactly.
/// The user code accesses this via the `__dandelion_system_data` symbol.
#[repr(C)]
#[derive(Debug, Clone, Default)]
pub struct DandelionSystemData {
    pub exit_code: c_int,
    pub heap_begin: u64,
    pub heap_end: u64,
    pub input_sets_len: u64,
    pub input_sets: u64,
    pub output_sets_len: u64,
    pub output_sets: u64,
    pub input_bufs: u64,
    pub output_bufs: u64,
}

// ============================================================================
// Page Fault Metadata (matches KVM's PageFaultMetadata)
// ============================================================================

/// Metadata returned after page table setup
///
/// Analogous to KVM's `PageFaultMetadata` - tracks where the stack can start
/// after all kernel structures are placed.
#[derive(Debug, Clone, Copy)]
pub struct PageFaultMetadata {
    stack_start: usize,
}

impl PageFaultMetadata {
    pub fn new(stack_start: usize) -> Self {
        Self { stack_start }
    }

    pub fn get_stack_start(&self) -> usize {
        self.stack_start
    }
}

// ============================================================================
// Error Types
// ============================================================================

/// Errors specific to Unikraft engine
#[derive(Debug, Clone, PartialEq)]
pub enum UnikraftError {
    /// Context size too small for required structures
    ContextTooSmall,
    /// Page table setup failed
    PageTableError,
    /// GDT/TSS/IDT setup failed
    InterruptTableError,
    /// Trampoline setup failed
    TrampolineError,
    /// User code execution failed with given vector
    ExecutionFault(u8),
    /// Memory alignment error
    AlignmentError,
}

// ============================================================================
// Result Type
// ============================================================================

pub type UnikraftResult<T> = Result<T, UnikraftError>;
