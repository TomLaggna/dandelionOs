//! Interrupt Handlers for Unikraft Engine
//!
//! This module provides interrupt handlers for vectors 0-32.
//! Vector 32 (INT 32) is used as the "return to kernel" syscall,
//! which jumps to the U2K trampoline.
//!
//! # Handler Structure
//!
//! Each handler:
//! 1. Pushes a dummy error code (if CPU didn't push one)
//! 2. Pushes the vector number
//! 3. Jumps to the common handler
//!
//! The common handler then jumps to the U2K trampoline to return to kernel.
//!
//! # Vectors with Error Codes (pushed by CPU)
//! 8 (Double Fault), 10 (Invalid TSS), 11 (Segment Not Present),
//! 12 (Stack Fault), 13 (GP), 14 (Page Fault), 17 (Alignment Check),
//! 21 (Control Protection), 30 (Security Exception)

use core::arch::global_asm;

// ============================================================================
// Handler Assembly
// ============================================================================

global_asm!(
    r#"
.section .text.port_handlers, "ax"
.align 16

// ============================================================================
// Individual Exception Handlers
// Each handler is 16 bytes for easy offset calculation
// ============================================================================

.macro HANDLER_NO_ERROR vector
.align 16
.globl port_handler_\vector
port_handler_\vector:
    push 0                      // Dummy error code
    push \vector                // Vector number
    jmp port_common_handler
    // Pad to 16 bytes
    .align 16
.endm

.macro HANDLER_WITH_ERROR vector
.align 16
.globl port_handler_\vector
port_handler_\vector:
    // CPU already pushed error code
    push \vector                // Vector number
    jmp port_common_handler
    // Pad to 16 bytes
    .align 16
.endm

// Vector 0: Divide Error
HANDLER_NO_ERROR 0

// Vector 1: Debug
HANDLER_NO_ERROR 1

// Vector 2: NMI
HANDLER_NO_ERROR 2

// Vector 3: Breakpoint
HANDLER_NO_ERROR 3

// Vector 4: Overflow
HANDLER_NO_ERROR 4

// Vector 5: Bound Range
HANDLER_NO_ERROR 5

// Vector 6: Invalid Opcode
HANDLER_NO_ERROR 6

// Vector 7: Device Not Available
HANDLER_NO_ERROR 7

// Vector 8: Double Fault (has error code)
HANDLER_WITH_ERROR 8

// Vector 9: Coprocessor Segment
HANDLER_NO_ERROR 9

// Vector 10: Invalid TSS (has error code)
HANDLER_WITH_ERROR 10

// Vector 11: Segment Not Present (has error code)
HANDLER_WITH_ERROR 11

// Vector 12: Stack Fault (has error code)
HANDLER_WITH_ERROR 12

// Vector 13: General Protection (has error code)
HANDLER_WITH_ERROR 13

// Vector 14: Page Fault (has error code)
HANDLER_WITH_ERROR 14

// Vector 15: Reserved
HANDLER_NO_ERROR 15

// Vector 16: x87 FPU Error
HANDLER_NO_ERROR 16

// Vector 17: Alignment Check (has error code)
HANDLER_WITH_ERROR 17

// Vector 18: Machine Check
HANDLER_NO_ERROR 18

// Vector 19: SIMD Exception
HANDLER_NO_ERROR 19

// Vector 20: Virtualization
HANDLER_NO_ERROR 20

// Vector 21: Control Protection (has error code)
HANDLER_WITH_ERROR 21

// Vectors 22-28: Reserved
HANDLER_NO_ERROR 22
HANDLER_NO_ERROR 23
HANDLER_NO_ERROR 24
HANDLER_NO_ERROR 25
HANDLER_NO_ERROR 26
HANDLER_NO_ERROR 27
HANDLER_NO_ERROR 28

// Vector 29: VMM Communication
HANDLER_NO_ERROR 29

// Vector 30: Security Exception (has error code)
HANDLER_WITH_ERROR 30

// Vector 31: Reserved
HANDLER_NO_ERROR 31

// Vector 32: INT 32 - Return to kernel (software interrupt)
// This is the "syscall" to return from user space
.align 16
.globl port_handler_32
port_handler_32:
    push 0                      // Dummy error code
    push 32                     // Vector number
    jmp port_common_handler
    .align 16

// ============================================================================
// Common Handler
// Jumps to U2K trampoline to return to kernel
// ============================================================================

.align 16
.globl port_common_handler
port_common_handler:
    // At this point, stack contains:
    // [error_code] [vector] [RIP] [CS] [RFLAGS] [RSP] [SS]
    //
    // We could save more state here for debugging, but for now
    // just jump to U2K trampoline which will restore kernel context.
    //
    // The U2K address is stored at a known location patched at setup time.
    
    // Load U2K trampoline address and jump
    lea rax, [rip + port_handler_u2k_addr]
    mov rax, [rax]
    jmp rax

// ============================================================================
// Handler Data Section
// ============================================================================

.align 8
.globl port_handler_data_start
port_handler_data_start:

// Address of U2K trampoline (patched at setup time)
.globl port_handler_u2k_addr
port_handler_u2k_addr:
    .quad 0

.globl port_handler_data_end
port_handler_data_end:

.globl port_handlers_end
port_handlers_end:
"#
);

// ============================================================================
// External Symbols
// ============================================================================

extern "C" {
    static port_handler_0: u8;
    static port_handler_1: u8;
    static port_handler_2: u8;
    static port_handler_3: u8;
    static port_handler_4: u8;
    static port_handler_5: u8;
    static port_handler_6: u8;
    static port_handler_7: u8;
    static port_handler_8: u8;
    static port_handler_9: u8;
    static port_handler_10: u8;
    static port_handler_11: u8;
    static port_handler_12: u8;
    static port_handler_13: u8;
    static port_handler_14: u8;
    static port_handler_15: u8;
    static port_handler_16: u8;
    static port_handler_17: u8;
    static port_handler_18: u8;
    static port_handler_19: u8;
    static port_handler_20: u8;
    static port_handler_21: u8;
    static port_handler_22: u8;
    static port_handler_23: u8;
    static port_handler_24: u8;
    static port_handler_25: u8;
    static port_handler_26: u8;
    static port_handler_27: u8;
    static port_handler_28: u8;
    static port_handler_29: u8;
    static port_handler_30: u8;
    static port_handler_31: u8;
    static port_handler_32: u8;
    static port_common_handler: u8;
    static port_handler_data_start: u8;
    static port_handler_u2k_addr: u64;
    static port_handlers_end: u8;
}

// ============================================================================
// Public API
// ============================================================================

/// Get the complete handler code as a byte slice
pub fn get_handler_code() -> &'static [u8] {
    unsafe {
        let start = &port_handler_0 as *const u8;
        let end = &port_handlers_end as *const u8;
        let size = end as usize - start as usize;
        core::slice::from_raw_parts(start, size)
    }
}

/// Get offset of each handler within the handler code block
///
/// Returns array of 33 offsets (vectors 0-32)
pub fn get_handler_offsets() -> [usize; 33] {
    unsafe {
        let base = &port_handler_0 as *const u8 as usize;
        [
            (&port_handler_0 as *const u8 as usize) - base,
            (&port_handler_1 as *const u8 as usize) - base,
            (&port_handler_2 as *const u8 as usize) - base,
            (&port_handler_3 as *const u8 as usize) - base,
            (&port_handler_4 as *const u8 as usize) - base,
            (&port_handler_5 as *const u8 as usize) - base,
            (&port_handler_6 as *const u8 as usize) - base,
            (&port_handler_7 as *const u8 as usize) - base,
            (&port_handler_8 as *const u8 as usize) - base,
            (&port_handler_9 as *const u8 as usize) - base,
            (&port_handler_10 as *const u8 as usize) - base,
            (&port_handler_11 as *const u8 as usize) - base,
            (&port_handler_12 as *const u8 as usize) - base,
            (&port_handler_13 as *const u8 as usize) - base,
            (&port_handler_14 as *const u8 as usize) - base,
            (&port_handler_15 as *const u8 as usize) - base,
            (&port_handler_16 as *const u8 as usize) - base,
            (&port_handler_17 as *const u8 as usize) - base,
            (&port_handler_18 as *const u8 as usize) - base,
            (&port_handler_19 as *const u8 as usize) - base,
            (&port_handler_20 as *const u8 as usize) - base,
            (&port_handler_21 as *const u8 as usize) - base,
            (&port_handler_22 as *const u8 as usize) - base,
            (&port_handler_23 as *const u8 as usize) - base,
            (&port_handler_24 as *const u8 as usize) - base,
            (&port_handler_25 as *const u8 as usize) - base,
            (&port_handler_26 as *const u8 as usize) - base,
            (&port_handler_27 as *const u8 as usize) - base,
            (&port_handler_28 as *const u8 as usize) - base,
            (&port_handler_29 as *const u8 as usize) - base,
            (&port_handler_30 as *const u8 as usize) - base,
            (&port_handler_31 as *const u8 as usize) - base,
            (&port_handler_32 as *const u8 as usize) - base,
        ]
    }
}

/// Get offset where U2K address needs to be patched
pub fn get_u2k_patch_offset() -> usize {
    unsafe {
        let base = &port_handler_0 as *const u8 as usize;
        let u2k_addr = &port_handler_u2k_addr as *const u64 as usize;
        u2k_addr - base
    }
}

/// Patch the U2K trampoline address in the handler code
///
/// # Arguments
/// * `storage` - Guest memory buffer
/// * `handler_offset` - Offset of handler code in storage
/// * `u2k_va` - Virtual address of U2K trampoline
pub fn patch_handlers(storage: &mut [u8], handler_offset: usize, u2k_va: u64) {
    let patch_offset = handler_offset + get_u2k_patch_offset();
    storage[patch_offset..patch_offset + 8].copy_from_slice(&u2k_va.to_le_bytes());
}
