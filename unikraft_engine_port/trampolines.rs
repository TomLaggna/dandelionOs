//! CR3-Switching Trampolines for Unikraft Engine
//!
//! These trampolines handle the transition between kernel and user contexts
//! by switching CR3 (page tables), loading appropriate GDT/IDT/TSS, and
//! managing the stack pointer.
//!
//! # Architecture
//!
//! ## Kernel-to-User (K2U) Trampoline
//! 1. Save kernel RSP to data section (for U2K to restore)
//! 2. Switch CR3 to user page tables
//! 3. Load user GDT/IDT/TSS
//! 4. Build IRET frame and IRET to Ring 3
//!
//! ## User-to-Kernel (U2K) Trampoline
//! 1. Switch CR3 back to kernel page tables
//! 2. Restore kernel RSP
//! 3. Restore kernel GDT/IDT
//! 4. Return to kernel code
//!
//! # Important: Both trampolines must be mapped at the same VA in both
//! kernel and user page tables so CR3 switch doesn't cause page fault.

use core::arch::global_asm;

// ============================================================================
// K2U Trampoline - Kernel to User
// ============================================================================

global_asm!(
    r#"
.section .text.port_k2u, "ax"
.align 16

// Entry point for Kernel-to-User transition
// Called from kernel code with kernel page tables active
//
// Data section fields (patched before call):
// - kernel_rsp_save: Where to save kernel RSP
// - user_cr3_value: CR3 value for user page tables
// - gdt_desc: 10-byte GDT descriptor (limit + base)
// - idt_desc: 10-byte IDT descriptor (limit + base)
// - tss_selector: TSS segment selector
// - user_stack_top: User mode RSP
// - user_entry_point: User code entry point (RIP for IRET)

.globl port_trampoline_k2u
port_trampoline_k2u:
    // Save registers we'll use
    push rax
    push rdx
    push rcx

    // ================================================================
    // SAVE KERNEL STATE
    // Save kernel GDT/IDT on stack so U2K can restore them
    // ================================================================

    // Save kernel IDT (push first = restore last)
    sub rsp, 16
    sidt [rsp]

    // Save kernel GDT
    sub rsp, 16
    sgdt [rsp]

    // Save kernel RSP to U2K data section
    // Both trampolines are mapped in both address spaces
    lea rax, [rip + port_u2k_kernel_rsp_restore]
    mov [rax], rsp

    // Also save locally for debugging
    lea rax, [rip + port_k2u_kernel_rsp_save]
    mov [rax], rsp

    // ================================================================
    // SWITCH TO USER ADDRESS SPACE
    // After this, kernel stack is unmapped!
    // ================================================================

    lea rax, [rip + port_k2u_user_cr3_value]
    mov rax, [rax]
    mov cr3, rax

    // ================================================================
    // LOAD USER GDT/IDT/TSS
    // These are now accessible via user page tables
    // ================================================================

    // Switch to user stack first (still Ring 0, but user mapping)
    lea rax, [rip + port_k2u_user_stack_top]
    mov rsp, [rax]

    // Load user GDT
    lea rax, [rip + port_k2u_gdt_desc]
    lgdt [rax]

    // Load user IDT
    lea rax, [rip + port_k2u_idt_desc]
    lidt [rax]

    // Load TSS
    lea rax, [rip + port_k2u_tss_selector]
    mov ax, [rax]
    ltr ax

    // ================================================================
    // BUILD IRET FRAME FOR RING 3 TRANSITION
    // Stack layout for IRET (bottom to top):
    //   SS, RSP, RFLAGS, CS, RIP
    // ================================================================

    // SS: User data selector with RPL=3 (0x23)
    push 0x23

    // RSP: User stack
    lea rax, [rip + port_k2u_user_stack_top]
    mov rax, [rax]
    push rax

    // RFLAGS: Enable interrupts (IF=1)
    pushfq
    pop rax
    or rax, 0x200      // Set IF
    and rax, ~0x100    // Clear TF (trap flag)
    push rax

    // CS: User code selector with RPL=3 (0x1B)
    push 0x1B

    // RIP: User entry point
    lea rax, [rip + port_k2u_user_entry_point]
    mov rax, [rax]
    push rax

    // ================================================================
    // TRANSITION TO RING 3
    // ================================================================
    iretq

// ================================================================
// K2U Data Section (patched at runtime)
// All fields 8-byte aligned
// ================================================================
.align 8
.globl port_k2u_data_start
port_k2u_data_start:

.globl port_k2u_kernel_rsp_save
port_k2u_kernel_rsp_save:
    .quad 0

.globl port_k2u_user_cr3_value
port_k2u_user_cr3_value:
    .quad 0

// GDT descriptor: 2-byte limit + 8-byte base
.globl port_k2u_gdt_desc
port_k2u_gdt_desc:
    .word 0          // limit
    .quad 0          // base
    .space 6         // padding to 16 bytes

// IDT descriptor: 2-byte limit + 8-byte base
.globl port_k2u_idt_desc
port_k2u_idt_desc:
    .word 0          // limit
    .quad 0          // base
    .space 6         // padding to 16 bytes

.globl port_k2u_tss_selector
port_k2u_tss_selector:
    .word 0
    .space 6         // padding

.globl port_k2u_user_stack_top
port_k2u_user_stack_top:
    .quad 0

.globl port_k2u_user_entry_point
port_k2u_user_entry_point:
    .quad 0

.globl port_k2u_data_end
port_k2u_data_end:

.globl port_trampoline_k2u_end
port_trampoline_k2u_end:
"#
);

// ============================================================================
// U2K Trampoline - User to Kernel
// ============================================================================

global_asm!(
    r#"
.section .text.port_u2k, "ax"
.align 16

// Entry point for User-to-Kernel transition
// Called from interrupt handler (INT 32) when user code wants to return
//
// At entry:
// - Running at Ring 0 (via interrupt gate)
// - CR3 still points to user page tables
// - RSP is interrupt stack (from TSS.RSP0)
//
// Data section fields (patched before user entry):
// - kernel_cr3_value: CR3 value for kernel page tables
// - kernel_rsp_restore: Saved kernel RSP from K2U

.globl port_trampoline_u2k
port_trampoline_u2k:
    // ================================================================
    // SWITCH BACK TO KERNEL ADDRESS SPACE
    // ================================================================

    lea rax, [rip + port_u2k_kernel_cr3_value]
    mov rax, [rax]
    mov cr3, rax

    // ================================================================
    // RESTORE KERNEL STATE
    // Kernel stack is now accessible again
    // ================================================================

    // Restore kernel RSP
    lea rax, [rip + port_u2k_kernel_rsp_restore]
    mov rsp, [rax]

    // Stack now has: [GDT desc 16 bytes][IDT desc 16 bytes][rcx][rdx][rax][ret addr]

    // Restore kernel GDT
    lgdt [rsp]
    add rsp, 16

    // Restore kernel IDT
    lidt [rsp]
    add rsp, 16

    // Restore saved registers
    pop rcx
    pop rdx
    pop rax

    // Return to K2U's caller (the kernel code that invoked the trampoline)
    ret

// ================================================================
// U2K Data Section (patched at runtime)
// ================================================================
.align 8
.globl port_u2k_data_start
port_u2k_data_start:

.globl port_u2k_kernel_cr3_value
port_u2k_kernel_cr3_value:
    .quad 0

.globl port_u2k_kernel_rsp_restore
port_u2k_kernel_rsp_restore:
    .quad 0

.globl port_u2k_data_end
port_u2k_data_end:

.globl port_trampoline_u2k_end
port_trampoline_u2k_end:
"#
);

// ============================================================================
// External Symbols from Assembly
// ============================================================================

extern "C" {
    // K2U trampoline
    static port_trampoline_k2u: u8;
    static port_trampoline_k2u_end: u8;
    static port_k2u_data_start: u8;
    static port_k2u_data_end: u8;

    // K2U data fields
    static mut port_k2u_kernel_rsp_save: u64;
    static mut port_k2u_user_cr3_value: u64;
    static mut port_k2u_gdt_desc: [u8; 16];
    static mut port_k2u_idt_desc: [u8; 16];
    static mut port_k2u_tss_selector: u16;
    static mut port_k2u_user_stack_top: u64;
    static mut port_k2u_user_entry_point: u64;

    // U2K trampoline
    static port_trampoline_u2k: u8;
    static port_trampoline_u2k_end: u8;
    static port_u2k_data_start: u8;
    static port_u2k_data_end: u8;

    // U2K data fields
    static mut port_u2k_kernel_cr3_value: u64;
    static mut port_u2k_kernel_rsp_restore: u64;
}

// ============================================================================
// Public API
// ============================================================================

/// Get K2U trampoline code as byte slice
pub fn get_k2u_code() -> &'static [u8] {
    unsafe {
        let start = &port_trampoline_k2u as *const u8;
        let end = &port_trampoline_k2u_end as *const u8;
        let size = end as usize - start as usize;
        core::slice::from_raw_parts(start, size)
    }
}

/// Get U2K trampoline code as byte slice
pub fn get_u2k_code() -> &'static [u8] {
    unsafe {
        let start = &port_trampoline_u2k as *const u8;
        let end = &port_trampoline_u2k_end as *const u8;
        let size = end as usize - start as usize;
        core::slice::from_raw_parts(start, size)
    }
}

/// Get size of K2U trampoline (code + data)
pub fn k2u_size() -> usize {
    get_k2u_code().len()
}

/// Get size of U2K trampoline (code + data)
pub fn u2k_size() -> usize {
    get_u2k_code().len()
}

/// Offset of data section in K2U trampoline
pub fn k2u_data_offset() -> usize {
    unsafe {
        let code_start = &port_trampoline_k2u as *const u8 as usize;
        let data_start = &port_k2u_data_start as *const u8 as usize;
        data_start - code_start
    }
}

/// Offset of data section in U2K trampoline
pub fn u2k_data_offset() -> usize {
    unsafe {
        let code_start = &port_trampoline_u2k as *const u8 as usize;
        let data_start = &port_u2k_data_start as *const u8 as usize;
        data_start - code_start
    }
}

// ============================================================================
// K2U Field Offsets (from start of K2U code)
// ============================================================================

pub mod k2u_offsets {
    use super::*;

    pub fn kernel_rsp_save() -> usize {
        unsafe {
            let start = &port_trampoline_k2u as *const u8 as usize;
            let field = &port_k2u_kernel_rsp_save as *const u64 as usize;
            field - start
        }
    }

    pub fn user_cr3_value() -> usize {
        unsafe {
            let start = &port_trampoline_k2u as *const u8 as usize;
            let field = &port_k2u_user_cr3_value as *const u64 as usize;
            field - start
        }
    }

    pub fn gdt_desc() -> usize {
        unsafe {
            let start = &port_trampoline_k2u as *const u8 as usize;
            let field = &port_k2u_gdt_desc as *const [u8; 16] as usize;
            field - start
        }
    }

    pub fn idt_desc() -> usize {
        unsafe {
            let start = &port_trampoline_k2u as *const u8 as usize;
            let field = &port_k2u_idt_desc as *const [u8; 16] as usize;
            field - start
        }
    }

    pub fn tss_selector() -> usize {
        unsafe {
            let start = &port_trampoline_k2u as *const u8 as usize;
            let field = &port_k2u_tss_selector as *const u16 as usize;
            field - start
        }
    }

    pub fn user_stack_top() -> usize {
        unsafe {
            let start = &port_trampoline_k2u as *const u8 as usize;
            let field = &port_k2u_user_stack_top as *const u64 as usize;
            field - start
        }
    }

    pub fn user_entry_point() -> usize {
        unsafe {
            let start = &port_trampoline_k2u as *const u8 as usize;
            let field = &port_k2u_user_entry_point as *const u64 as usize;
            field - start
        }
    }
}

// ============================================================================
// U2K Field Offsets (from start of U2K code)
// ============================================================================

pub mod u2k_offsets {
    use super::*;

    pub fn kernel_cr3_value() -> usize {
        unsafe {
            let start = &port_trampoline_u2k as *const u8 as usize;
            let field = &port_u2k_kernel_cr3_value as *const u64 as usize;
            field - start
        }
    }

    pub fn kernel_rsp_restore() -> usize {
        unsafe {
            let start = &port_trampoline_u2k as *const u8 as usize;
            let field = &port_u2k_kernel_rsp_restore as *const u64 as usize;
            field - start
        }
    }
}

// ============================================================================
// Setup Functions
// ============================================================================

/// Copy trampolines to guest memory and return entry addresses
///
/// # Arguments
/// * `storage` - Guest memory buffer
/// * `k2u_offset` - Offset in storage for K2U trampoline
/// * `u2k_offset` - Offset in storage for U2K trampoline
///
/// # Returns
/// (k2u_entry_offset, u2k_entry_offset) - Offsets of entry points in storage
pub fn copy_trampolines(
    storage: &mut [u8],
    k2u_offset: usize,
    u2k_offset: usize,
) -> (usize, usize) {
    let k2u_code = get_k2u_code();
    let u2k_code = get_u2k_code();

    storage[k2u_offset..k2u_offset + k2u_code.len()].copy_from_slice(k2u_code);
    storage[u2k_offset..u2k_offset + u2k_code.len()].copy_from_slice(u2k_code);

    // Entry points are at the start of each trampoline
    (k2u_offset, u2k_offset)
}

/// Patch K2U trampoline data section
///
/// # Arguments
/// * `storage` - Guest memory buffer
/// * `k2u_offset` - Offset of K2U trampoline in storage
/// * `user_cr3` - Physical address of user PML4
/// * `gdt_base` - Virtual address of GDT
/// * `gdt_limit` - GDT limit (size - 1)
/// * `idt_base` - Virtual address of IDT
/// * `idt_limit` - IDT limit (size - 1)
/// * `tss_selector` - TSS selector (typically 0x28)
/// * `user_stack_top` - User stack pointer
/// * `user_entry_point` - User code entry point
pub fn patch_k2u(
    storage: &mut [u8],
    k2u_offset: usize,
    user_cr3: u64,
    gdt_base: u64,
    gdt_limit: u16,
    idt_base: u64,
    idt_limit: u16,
    tss_selector: u16,
    user_stack_top: u64,
    user_entry_point: u64,
) {
    let data_base = k2u_offset + k2u_data_offset();

    // user_cr3_value is at offset 8 from data_start (after kernel_rsp_save)
    let cr3_offset = data_base + 8;
    storage[cr3_offset..cr3_offset + 8].copy_from_slice(&user_cr3.to_le_bytes());

    // gdt_desc at offset 16
    let gdt_offset = data_base + 16;
    storage[gdt_offset..gdt_offset + 2].copy_from_slice(&gdt_limit.to_le_bytes());
    storage[gdt_offset + 2..gdt_offset + 10].copy_from_slice(&gdt_base.to_le_bytes());

    // idt_desc at offset 32
    let idt_offset = data_base + 32;
    storage[idt_offset..idt_offset + 2].copy_from_slice(&idt_limit.to_le_bytes());
    storage[idt_offset + 2..idt_offset + 10].copy_from_slice(&idt_base.to_le_bytes());

    // tss_selector at offset 48
    let tss_offset = data_base + 48;
    storage[tss_offset..tss_offset + 2].copy_from_slice(&tss_selector.to_le_bytes());

    // user_stack_top at offset 56
    let stack_offset = data_base + 56;
    storage[stack_offset..stack_offset + 8].copy_from_slice(&user_stack_top.to_le_bytes());

    // user_entry_point at offset 64
    let entry_offset = data_base + 64;
    storage[entry_offset..entry_offset + 8].copy_from_slice(&user_entry_point.to_le_bytes());
}

/// Patch U2K trampoline data section
///
/// # Arguments
/// * `storage` - Guest memory buffer
/// * `u2k_offset` - Offset of U2K trampoline in storage
/// * `kernel_cr3` - Physical address of kernel PML4
pub fn patch_u2k(storage: &mut [u8], u2k_offset: usize, kernel_cr3: u64) {
    let data_base = u2k_offset + u2k_data_offset();

    // kernel_cr3_value at offset 0
    storage[data_base..data_base + 8].copy_from_slice(&kernel_cr3.to_le_bytes());

    // kernel_rsp_restore is filled by K2U at runtime
}
