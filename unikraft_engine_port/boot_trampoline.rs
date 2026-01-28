//! AP Boot Trampoline for Unikraft Engine
//!
//! This module contains the multi-stage boot trampoline that brings Application
//! Processors (APs) from 16-bit real mode to 64-bit long mode.
//!
//! # Why Fixed Physical Address?
//!
//! The SIPI (Startup Inter-Processor Interrupt) mechanism specifies a physical
//! address in the first 1MB of memory where the AP will begin execution. The
//! early boot code (16-bit real mode, 32-bit protected mode) cannot use
//! RIP-relative addressing, so it relies on known fixed addresses.
//!
//! # Memory Requirements
//!
//! 1. **Low Physical Memory**: The trampoline must be copied to a page-aligned
//!    address below 1MB (e.g., 0x8000). We use Unikraft's directmap region
//!    (0xffffff8000000000 + phys_addr) to access this memory.
//!
//! 2. **Identity Mapping**: The low physical memory must be identity-mapped
//!    in the page tables (VA == PA for addresses < 1MB). This allows
//!    instruction fetching to continue seamlessly when we set CR3.
//!
//! # Boot Sequence
//!
//! ```text
//! SIPI → 16-bit Real Mode → 32-bit Protected Mode → 64-bit Long Mode → Rust
//!        (0x8000)           (enable PE)              (enable PAE,LME,PG)  (ap_entry)
//! ```
//!
//! # Setup Steps
//!
//! 1. Allocate low memory for trampoline (e.g., 0x8000)
//! 2. Copy trampoline code via directmap
//! 3. Patch `x86_bpt_pml4_addr` with kernel CR3
//! 4. Patch `lcpus` address for per-CPU data
//! 5. Ensure identity mapping of low memory in kernel page tables
//! 6. Send INIT-SIPI-SIPI to target AP

use core::arch::global_asm;

// ============================================================================
// Constants
// ============================================================================

/// Default target address for boot trampoline in low memory
pub const DEFAULT_TRAMPOLINE_ADDR: u64 = 0x8000;

/// Size of per-CPU data structure (must match LCPU_SIZE in assembly)
pub const LCPU_SIZE: usize = 64;

/// Maximum number of CPUs supported
pub const LCPU_MAXCOUNT: usize = 16;

/// Unikraft directmap base address
pub const DIRECTMAP_BASE: u64 = 0xffffff8000000000;

// ============================================================================
// CPU States (must match assembly)
// ============================================================================

pub mod cpu_state {
    pub const OFFLINE: i32 = 0;
    pub const INIT: i32 = 1;
    pub const IDLE: i32 = 2;
    pub const BUSY: i32 = 3;
    pub const HALTED: i32 = i32::MIN;
}

// ============================================================================
// Boot Trampoline Assembly
// ============================================================================

// Note: We use .code16, .code32, .code64 directives to generate proper
// instructions for each CPU mode. The assembler handles the encoding.

global_asm!(
    r#"
// ============================================================================
// Constants (must match boot_defs.h)
// ============================================================================

// CR0 bits
.set X86_CR0_PE,        (1 << 0)
.set X86_CR0_MP,        (1 << 1)
.set X86_CR0_NE,        (1 << 5)
.set X86_CR0_WP,        (1 << 16)
.set X86_CR0_PG,        (1 << 31)

// CR4 bits
.set X86_CR4_PAE,       (1 << 5)
.set X86_CR4_OSFXSR,    (1 << 9)
.set X86_CR4_OSXMMEXCPT,(1 << 10)
.set X86_CR4_FSGSBASE,  (1 << 16)
.set X86_CR4_OSXSAVE,   (1 << 18)

// MSR numbers
.set X86_MSR_EFER,      0xc0000080
.set X86_EFER_LME,      (1 << 8)

// CPUID bits
.set X86_CPUID1_ECX_XSAVE, (1 << 26)
.set X86_CPUID1_ECX_AVX,   (1 << 28)
.set X86_CPUID7_EBX_FSGSBASE, (1 << 0)

// XCR0 bits
.set X86_XCR0_SSE,      (1 << 1)
.set X86_XCR0_AVX,      (1 << 2)

// GDT descriptor values (pre-computed)
.set GDT_DESC_CODE32_LO, 0x0000ffff
.set GDT_DESC_CODE32_HI, 0x00cf9a00
.set GDT_DESC_DATA32_LO, 0x0000ffff
.set GDT_DESC_DATA32_HI, 0x00cf9200
.set GDT_DESC_CODE64_LO, 0x0000ffff
.set GDT_DESC_CODE64_HI, 0x00af9a00
.set GDT_DESC_DATA64_LO, 0x0000ffff
.set GDT_DESC_DATA64_HI, 0x00af9200

// LCPU structure offsets
.set LCPU_STATE_OFFSET, 0x00
.set LCPU_ENTRY_OFFSET, 0x10
.set LCPU_STACKP_OFFSET, 0x18
.set LCPU_SIZE,         64
.set LCPU_MAXCOUNT,     16
.set LCPU_STATE_INIT,   1

// ============================================================================
// 16-BIT REAL MODE SECTION
// Entry point from SIPI (physical address must be < 1MB)
// ============================================================================

.section .text.port_boot16, "ax"
.align 16

.globl port_boot16_start
port_boot16_start:

.code16
.globl port_lcpu_start16_ap
port_lcpu_start16_ap:
    // Clear segment base pointers
    xorl    %edi, %edi
    xorl    %esi, %esi

    // Calculate address of lcpu_start16 accounting for CS base
    movw    $port_lcpu_start16, %ax
    movl    %cs, %ebx
    shll    $4, %ebx
    subl    %ebx, %eax
    jmp     *%eax

// 32-bit GDT for protected mode transition
.align 16
port_gdt32:
port_gdt32_null:
    .word   0x0000
port_gdt32_ptr:
    .word   (port_gdt32_end - port_gdt32 - 1)
    .long   port_gdt32
port_gdt32_cs:
    .long   GDT_DESC_CODE32_LO
    .long   GDT_DESC_CODE32_HI
port_gdt32_ds:
    .long   GDT_DESC_DATA32_LO
    .long   GDT_DESC_DATA32_HI
port_gdt32_end:

.code16
port_lcpu_start16:
    // Disable interrupts
    cli

    // Enable protected mode (PE bit in CR0)
    movl    $(X86_CR0_PE), %eax
    movl    %eax, %cr0

    // Load 32-bit GDT
    movw    $port_gdt32_ptr, %ax
    lgdt    (%eax)

    // Far jump to 32-bit code segment
    // Selector 0x08 = index 1 (code segment)
    ljmp    $0x08, $port_jump_to32

.globl port_boot16_end
port_boot16_end:

// ============================================================================
// 32-BIT PROTECTED MODE SECTION
// Transitions from protected mode to long mode
// ============================================================================

.section .text.port_boot32, "ax"
.align 16

// 64-bit GDT for long mode transition
port_gdt64:
port_gdt64_null:
    .quad   0x0000000000000000
port_gdt64_cs:
    .long   GDT_DESC_CODE64_LO
    .long   GDT_DESC_CODE64_HI
port_gdt64_ds:
    .long   GDT_DESC_DATA64_LO
    .long   GDT_DESC_DATA64_HI
port_gdt64_end:

port_gdt64_ptr:
    .word   port_gdt64_end - port_gdt64 - 1
    .quad   port_gdt64

.code32
port_jump_to32:
    // Set up data segments
    movl    $(port_gdt32_ds - port_gdt32), %eax
    movl    %eax, %es
    movl    %eax, %ss
    movl    %eax, %ds
    xorl    %eax, %eax
    movl    %eax, %fs
    movl    %eax, %gs

    // Fall through to lcpu_start32
    jmp     port_lcpu_start32

.globl port_lcpu_start32
port_lcpu_start32:
    // Enable PAE (Physical Address Extension)
    movl    $(X86_CR4_PAE), %eax
    movl    %eax, %cr4

    // Enable IA-32e mode (Long Mode) in EFER MSR
    xorl    %edx, %edx
    movl    $(X86_EFER_LME), %eax
    movl    $(X86_MSR_EFER), %ecx
    wrmsr

    // Load page table address into CR3
    // x86_bpt_pml4_addr is patched by BSP before sending SIPI
    movl    port_x86_bpt_pml4_addr, %eax
    movl    %eax, %cr3

    // Enable paging (PG), write-protect (WP), and keep PE
    movl    $(X86_CR0_PE | X86_CR0_WP | X86_CR0_PG), %eax
    movl    %eax, %cr0

    // Load 64-bit GDT
    movl    $port_gdt64_ptr, %eax
    lgdt    (%eax)

    // Far jump to 64-bit code segment
    // Selector 0x08 = index 1 (code segment)
    ljmp    $0x08, $port_jump_to64

.globl port_boot32_end
port_boot32_end:

// ============================================================================
// 64-BIT LONG MODE SECTION
// Final setup before jumping to Rust code
// ============================================================================

.section .text.port_boot64, "ax"
.align 16

.code64
port_jump_to64:
    // Set up 64-bit data segments
    movl    $(port_gdt64_ds - port_gdt64), %eax
    movl    %eax, %es
    movl    %eax, %ss
    movl    %eax, %ds
    xorl    %eax, %eax
    movl    %eax, %fs
    movl    %eax, %gs

    // Jump to main 64-bit entry using RIP-relative addressing
    // (now safe since we're in long mode)
    leaq    port_lcpu_start64(%rip), %rcx
    jmp     *%rcx

.globl port_lcpu_start64
port_lcpu_start64:
    // Get APIC ID from CPUID
    movl    $1, %eax
    cpuid
    shrl    $24, %ebx           // APIC ID in EBX

    // Calculate pointer to our CpuData struct
    // &lcpus[apic_id] = lcpus + (apic_id * LCPU_SIZE)
    movl    $LCPU_SIZE, %eax
    imul    %ebx, %eax
    leaq    port_lcpus(%rip), %rbp
    addq    %rax, %rbp

    // Mark CPU as initializing
    movl    $LCPU_STATE_INIT, LCPU_STATE_OFFSET(%rbp)

    // ================================================================
    // Enable FPU and SSE
    // ================================================================
    movq    %cr0, %rdi
    orl     $(X86_CR0_NE | X86_CR0_MP), %edi
    movq    %rdi, %cr0
    fninit

    // Skip over embedded data
    jmp     1f
port_mxcsr_default:
    .long   0x1f80              // Default MXCSR value
1:

    // Enable SSE in CR4
    movq    %cr4, %rdi
    orl     $(X86_CR4_OSFXSR | X86_CR4_OSXMMEXCPT), %edi
    movq    %rdi, %cr4

    // Load default MXCSR
    leaq    port_mxcsr_default(%rip), %rbx
    ldmxcsr (%rbx)

    // Save ECX (has CPUID.1 features) for later
    movq    %rcx, %r12

    // ================================================================
    // Enable XSAVE if available
    // ================================================================
    testl   $(X86_CPUID1_ECX_XSAVE), %r12d
    jz      2f
    movq    %cr4, %rdi
    orl     $(X86_CR4_OSXSAVE), %edi
    movq    %rdi, %cr4
2:

    // ================================================================
    // Enable AVX if available
    // ================================================================
    testl   $(X86_CPUID1_ECX_AVX), %r12d
    jz      3f
    xorl    %ecx, %ecx
    xgetbv
    orl     $(X86_XCR0_SSE | X86_XCR0_AVX), %eax
    xsetbv
3:

    // ================================================================
    // Enable FSGSBASE if available
    // ================================================================
    movl    $7, %eax
    xorl    %ecx, %ecx
    cpuid
    testl   $(X86_CPUID7_EBX_FSGSBASE), %ebx
    jz      4f
    movq    %cr4, %rdi
    orl     $(X86_CR4_FSGSBASE), %edi
    movq    %rdi, %cr4
4:

    // ================================================================
    // Load entry point and stack from CpuData
    // ================================================================
    movq    LCPU_ENTRY_OFFSET(%rbp), %rax
    movq    LCPU_STACKP_OFFSET(%rbp), %rsp

    // Save entry address
    movq    %rax, %r15

    // Pass CpuData pointer as first argument (System V ABI)
    movq    %rbp, %rdi

    // Align stack to 16-byte boundary
    andq    $~0xf, %rsp

    // Clear registers for clean state
    xorq    %rax, %rax
    xorq    %rbx, %rbx
    xorq    %rcx, %rcx
    xorq    %rdx, %rdx
    xorq    %rsi, %rsi
    xorq    %rbp, %rbp
    xorq    %r8, %r8
    xorq    %r9, %r9
    xorq    %r10, %r10
    xorq    %r11, %r11
    xorq    %r12, %r12
    xorq    %r13, %r13
    xorq    %r14, %r14

    // Jump to Rust entry point
    jmp     *%r15

// Halt loop for failures
port_boot_fail:
    cli
5:  hlt
    jmp     5b

.globl port_boot64_end
port_boot64_end:

// ============================================================================
// DATA SECTION - Patched by BSP before sending SIPI
// ============================================================================

.section .data.port_boot, "aw"
.align 4096

// Page table address - BSP must set this before starting APs
.globl port_x86_bpt_pml4_addr
port_x86_bpt_pml4_addr:
    .long   0

// Per-CPU data structures
.globl port_lcpus
.align 64
port_lcpus:
    .fill   (LCPU_SIZE * LCPU_MAXCOUNT), 1, 0

.globl port_boot_data_end
port_boot_data_end:
"#
);

// ============================================================================
// External Symbols from Assembly
// ============================================================================

extern "C" {
    // Section markers
    static port_boot16_start: u8;
    static port_boot16_end: u8;
    static port_boot32_end: u8;
    static port_boot64_end: u8;
    static port_boot_data_end: u8;

    // Entry points
    static port_lcpu_start16_ap: u8;
    static port_lcpu_start32: u8;
    static port_lcpu_start64: u8;

    // Patchable data
    static mut port_x86_bpt_pml4_addr: u32;
    static mut port_lcpus: [u8; LCPU_SIZE * LCPU_MAXCOUNT];
}

// ============================================================================
// Per-CPU Data Structure (must match assembly layout)
// ============================================================================

/// Per-CPU data structure used by boot trampoline
///
/// Layout must match LCPU_* offsets in assembly
#[repr(C, align(64))]
#[derive(Debug, Clone, Copy)]
pub struct CpuData {
    /// CPU state (see cpu_state module)
    pub state: i32,
    /// CPU index
    pub idx: u32,
    /// APIC ID
    pub id: u32,
    /// Padding
    _pad0: u32,
    /// Entry point address (Rust function to call)
    pub entry: u64,
    /// Stack pointer
    pub stackp: u64,
    /// Task info pointer (for AP work loop)
    pub task_info_ptr: u64,
    /// Reserved
    _reserved: [u64; 4],
}

impl CpuData {
    pub const fn new() -> Self {
        Self {
            state: cpu_state::OFFLINE,
            idx: 0,
            id: 0,
            _pad0: 0,
            entry: 0,
            stackp: 0,
            task_info_ptr: 0,
            _reserved: [0; 4],
        }
    }
}

// ============================================================================
// Public API
// ============================================================================

/// Get the complete boot trampoline code (16-bit + 32-bit + 64-bit sections)
///
/// This code must be copied to low physical memory before sending SIPIs.
pub fn get_trampoline_code() -> &'static [u8] {
    unsafe {
        let start = &port_boot16_start as *const u8;
        let end = &port_boot64_end as *const u8;
        let size = end as usize - start as usize;
        core::slice::from_raw_parts(start, size)
    }
}

/// Get just the 16-bit section (for calculating SIPI vector)
pub fn get_boot16_code() -> &'static [u8] {
    unsafe {
        let start = &port_boot16_start as *const u8;
        let end = &port_boot16_end as *const u8;
        let size = end as usize - start as usize;
        core::slice::from_raw_parts(start, size)
    }
}

/// Get the data section that needs to be patched
pub fn get_data_section() -> &'static [u8] {
    unsafe {
        let start = &port_x86_bpt_pml4_addr as *const u32 as *const u8;
        let end = &port_boot_data_end as *const u8;
        let size = end as usize - start as usize;
        core::slice::from_raw_parts(start, size)
    }
}

/// Calculate offset of lcpu_start16_ap from start of trampoline
pub fn get_entry_offset() -> usize {
    unsafe {
        let base = &port_boot16_start as *const u8 as usize;
        let entry = &port_lcpu_start16_ap as *const u8 as usize;
        entry - base
    }
}

/// Offset of PML4 address field from start of data section
pub fn get_pml4_addr_offset() -> usize {
    0 // It's at the start of data section
}

/// Offset of lcpus array from start of data section
pub fn get_lcpus_offset() -> usize {
    unsafe {
        let data_start = &port_x86_bpt_pml4_addr as *const u32 as usize;
        let lcpus_start = &port_lcpus as *const [u8; LCPU_SIZE * LCPU_MAXCOUNT] as usize;
        lcpus_start - data_start
    }
}

// ============================================================================
// Setup Functions
// ============================================================================

/// Configuration for boot trampoline setup
pub struct BootTrampolineConfig {
    /// Physical address where trampoline will be placed (must be < 1MB, page-aligned)
    pub target_phys_addr: u64,
    /// Physical address of kernel PML4 (for CR3)
    pub kernel_pml4_phys: u64,
    /// Directmap base address (typically 0xffffff8000000000)
    pub directmap_base: u64,
}

impl Default for BootTrampolineConfig {
    fn default() -> Self {
        Self {
            target_phys_addr: DEFAULT_TRAMPOLINE_ADDR,
            kernel_pml4_phys: 0,
            directmap_base: DIRECTMAP_BASE,
        }
    }
}

/// Copy boot trampoline to low physical memory using directmap
///
/// # Safety
/// - `config.target_phys_addr` must be valid physical memory < 1MB
/// - The target memory must be identity-mapped in kernel page tables
/// - `config.kernel_pml4_phys` must be a valid PML4 physical address
///
/// # Returns
/// The SIPI vector (target_phys_addr >> 12)
pub unsafe fn setup_boot_trampoline(config: &BootTrampolineConfig) -> u8 {
    // Calculate virtual address in directmap
    let target_va = config.directmap_base + config.target_phys_addr;
    let target_ptr = target_va as *mut u8;

    // Copy trampoline code
    let code = get_trampoline_code();
    core::ptr::copy_nonoverlapping(code.as_ptr(), target_ptr, code.len());

    // Copy and patch data section
    let data = get_data_section();
    let data_offset = code.len(); // Data follows code
    let data_ptr = target_ptr.add(data_offset);
    core::ptr::copy_nonoverlapping(data.as_ptr(), data_ptr, data.len());

    // Patch PML4 address
    let pml4_ptr = data_ptr.add(get_pml4_addr_offset()) as *mut u32;
    core::ptr::write_volatile(pml4_ptr, config.kernel_pml4_phys as u32);

    // Calculate SIPI vector (physical address >> 12, must fit in 8 bits)
    let sipi_vector = (config.target_phys_addr >> 12) as u8;
    sipi_vector
}

/// Get pointer to CpuData for a specific CPU (via directmap)
///
/// # Safety
/// Boot trampoline must be set up at the given address first
pub unsafe fn get_cpu_data(
    target_phys_addr: u64,
    directmap_base: u64,
    cpu_index: usize,
) -> *mut CpuData {
    let trampoline_code_len = get_trampoline_code().len();
    let data_offset = trampoline_code_len + get_lcpus_offset();
    let cpu_offset = data_offset + cpu_index * LCPU_SIZE;

    let target_va = directmap_base + target_phys_addr + cpu_offset as u64;
    target_va as *mut CpuData
}

/// Initialize CpuData for an AP
///
/// # Safety
/// Boot trampoline must be set up first
pub unsafe fn init_cpu_data(
    target_phys_addr: u64,
    directmap_base: u64,
    cpu_index: usize,
    entry_point: u64,
    stack_pointer: u64,
) {
    let cpu_data = get_cpu_data(target_phys_addr, directmap_base, cpu_index);

    (*cpu_data).idx = cpu_index as u32;
    (*cpu_data).entry = entry_point;
    (*cpu_data).stackp = stack_pointer;
    (*cpu_data).state = cpu_state::OFFLINE;
}
