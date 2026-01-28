# Unikraft Engine Integration Guide for Dandelion

This document describes how to integrate the Unikraft-based execution engine into
Dandelion as a new `ContextType::Unikraft` variant, alongside existing KVM, MMU,
and CHERI engines.

## Overview

The Unikraft engine provides **Ring 3 isolation on shared address space hardware**
using x86-64 paging and privilege separation. Unlike KVM (which uses hardware
virtualization) or MMU (which uses MMAP), Unikraft shares the host kernel's
physical memory but isolates user code via:

1. **Separate page tables** - User code runs with its own CR3
2. **Ring 3 execution** - User code runs at CPL=3 (unprivileged)
3. **CR3-switching trampolines** - Assembly sequences that atomically switch
   between kernel and user address spaces

## File Structure

```
src/dandelion_port/
├── mod.rs           # Core types: UnikraftContext, UnikraftLayout, etc.
├── ap_startup.rs    # AP boot, core pool, task info for multi-core
├── x86_64.rs        # Page table and interrupt table setup
├── trampolines.rs   # K2U/U2K assembly trampolines (global_asm!)
└── handlers.rs      # Interrupt handlers (vectors 0-32, global_asm!)
```

### Target Integration Path in Dandelion

```
src/engines/
├── kvm/
│   ├── kvm.rs           # KvmDriver, KvmLoop, KvmContext
│   └── x86_64.rs        # KVM-specific x86_64 setup
├── mmu/
│   └── mmu.rs
└── unikraft/            # NEW: Copy dandelion_port/ here
    ├── mod.rs           # UnikraftDriver, UnikraftLoop
    ├── ap_startup.rs    # Multi-core support
    ├── x86_64.rs        # Page table and interrupt setup
    ├── trampolines.rs   # CR3-switching trampolines
    └── handlers.rs      # Interrupt handlers
```

---

## Key Components

### 1. UnikraftContext (mod.rs)

```rust
pub struct UnikraftContext {
    /// Guest memory storage
    pub storage: Box<[u8]>,
    /// Memory layout within storage
    pub layout: UnikraftLayout,
}
```

**Comparison with KVM:**
- `KvmContext` wraps a VM file descriptor and uses KVM ioctls
- `UnikraftContext` directly owns the memory buffer (no hypervisor)

### 2. x86_64.rs - Page Table Setup

The page table setup is **dynamic based on context size**, supporting up to 512GB:

```rust
/// Set up 4-level page tables for user space
pub fn set_page_table(
    storage: &mut [u8],
    stack_start: &mut usize,
    context_size: usize,
    phys_base: u64,
    use_large_pages: bool,
) -> UnikraftResult<u64>
```

**Key features:**
- Allocates PML4, PDPT, and PD/PT pages dynamically based on `context_size`
- Uses PDPT entries (1GB granularity) to cover up to 512GB
- Supports both 2MB large pages and 4KB pages
- Returns PML4 physical address for CR3

**Page table calculation:**
```rust
pub fn calculate_page_table_pages(context_size: usize, use_large_pages: bool) -> usize {
    let pml4_pages = 1;
    let pdpt_pages = 1;
    let gb_regions = (context_size + (1 << 30) - 1) / (1 << 30);
    let pd_pages = gb_regions.max(1);
    // ... PT pages if not using large pages
}
```

### 3. trampolines.rs - CR3 Switching (Assembly)

Uses `global_asm!` for readable assembly code:

**K2U Trampoline (Kernel → User):**
```asm
port_trampoline_k2u:
    // Save kernel state
    push rax, rdx, rcx
    sub rsp, 16; sidt [rsp]   // Save kernel IDT
    sub rsp, 16; sgdt [rsp]   // Save kernel GDT
    
    // Save kernel RSP for U2K to restore
    lea rax, [rip + port_u2k_kernel_rsp_restore]
    mov [rax], rsp
    
    // Switch to user address space
    mov rax, [rip + port_k2u_user_cr3_value]
    mov cr3, rax
    
    // Load user stack, GDT, IDT, TSS
    mov rsp, [rip + port_k2u_user_stack_top]
    lgdt [rip + port_k2u_gdt_desc]
    lidt [rip + port_k2u_idt_desc]
    ltr [rip + port_k2u_tss_selector]
    
    // Build IRET frame and transition to Ring 3
    push 0x23       // SS (user data, RPL=3)
    push rsp        // RSP
    pushfq; or ...; push  // RFLAGS with IF=1
    push 0x1B       // CS (user code, RPL=3)
    push [rip + port_k2u_user_entry_point]
    iretq
```

**U2K Trampoline (User → Kernel):**
```asm
port_trampoline_u2k:
    // Switch back to kernel address space
    mov rax, [rip + port_u2k_kernel_cr3_value]
    mov cr3, rax
    
    // Restore kernel RSP (kernel stack now accessible)
    mov rsp, [rip + port_u2k_kernel_rsp_restore]
    
    // Restore kernel GDT/IDT from stack
    lgdt [rsp]; add rsp, 16
    lidt [rsp]; add rsp, 16
    
    // Restore registers and return
    pop rcx, rdx, rax
    ret
```

### 4. handlers.rs - Interrupt Handlers (Assembly)

Uses `global_asm!` with macros for clean handler definitions:

```asm
.macro HANDLER_NO_ERROR vector
.align 16
port_handler_\vector:
    push 0              // Dummy error code
    push \vector        // Vector number
    jmp port_common_handler
.endm

.macro HANDLER_WITH_ERROR vector
.align 16
port_handler_\vector:
    // CPU already pushed error code
    push \vector
    jmp port_common_handler
.endm

// Generate all 33 handlers
HANDLER_NO_ERROR 0    // Divide Error
HANDLER_NO_ERROR 1    // Debug
// ...
HANDLER_WITH_ERROR 14 // Page Fault
// ...
HANDLER_NO_ERROR 32   // INT 32 - return to kernel

port_common_handler:
    // Jump to U2K trampoline
    mov rax, [rip + port_handler_u2k_addr]
    jmp rax
```

### 5. ap_startup.rs - Multi-Core Support

**This is unique to Unikraft** - KVM manages vCPUs through the hypervisor, but
Unikraft must manage Application Processors (APs) directly.

**Key structures:**

```rust
/// Per-CPU data for boot trampoline
pub struct CpuData {
    pub idx: u32,
    pub id: u32,       // APIC ID
    pub state: i32,
    pub entry: u64,
    pub stackp: u64,
    pub task_info_ptr: u64,
}

/// Task info passed from BSP to AP
pub struct ApTaskInfo {
    pub entry_point: AtomicU64,
    pub user_cr3: AtomicU64,
    pub kernel_cr3: AtomicU64,
    pub k2u_trampoline: AtomicU64,
    pub u2k_trampoline: AtomicU64,
    pub status: AtomicU8,
    // ... interrupt configuration
}

/// Core pool for parallel execution
pub struct CorePool {
    core_status: [AtomicU8; MAX_CPUS],
    task_info: [ApTaskInfo; MAX_CPUS],
}
```

**Integration pattern:**
```rust
// Acquire a free core
let core_id = core_pool.acquire_core()?;

// Set up task
let task_info = core_pool.get_task_info(core_id)?;
task_info.setup_task(entry, user_cr3, kernel_cr3, ...);

// Wake the AP (implementation depends on environment)
wake_ap(apic_id, sipi_vector);

// Wait for completion
let status = task_info.wait_for_completion();

// Release core
core_pool.release_core(core_id);
```

---

## Integration Steps

### Step 1: Add ContextType Variant

In `src/engines/mod.rs`:

```rust
pub enum ContextType {
    Kvm(KvmContext),
    Mmu(MmuContext),
    Cheri(CheriContext),
    Unikraft(UnikraftContext),  // NEW
}
```

### Step 2: Implement EngineLoop Trait

```rust
pub struct UnikraftLoop;

impl EngineLoop for UnikraftLoop {
    fn run_engine<S: DataStore>(
        domain: &UnikraftMemoryDomain,
        config: &ElfConfig,
        system_data: &DandelionSystemData,
    ) -> EngineResult<(...)> {
        // 1. Acquire a core from the pool
        // 2. Copy input data to data region
        // 3. Patch K2U trampoline with entry point and stack
        // 4. Call K2U trampoline (or wake AP)
        // 5. On return (via U2K), check exit reason
        // 6. Copy output data from data region
        // 7. Release core
    }
}
```

### Step 3: Memory Domain Setup

```rust
pub fn setup_unikraft_domain(context_size: usize) -> Result<UnikraftMemoryDomain> {
    let mut storage = vec![0u8; context_size];
    let phys_base = storage.as_ptr() as u64;
    let mut stack_start = context_size;
    
    // Setup page tables (dynamic based on context_size)
    let pml4_phys = x86_64::set_page_table(
        &mut storage,
        &mut stack_start,
        context_size,
        phys_base,
        true,  // use 2MB pages
    )?;
    
    // Setup interrupt infrastructure
    let (gdt_off, tss_off, idt_off, handler_off, int_stack_top) = 
        x86_64::set_interrupt_table(&mut storage, &mut stack_start, phys_base)?;
    
    // Copy trampolines
    let (k2u_off, u2k_off) = trampolines::copy_trampolines(
        &mut storage, k2u_offset, u2k_offset
    );
    
    // Patch U2K address in handlers
    handlers::patch_handlers(&mut storage, handler_off, u2k_va);
    
    // ...
}
```

---

## Missing Pieces (TODO in Dandelion)

### 1. Demand Paging / Page Fault Handler

The current implementation **pre-maps all memory**. For production use with large
contexts, you'll want demand paging:

```rust
// In page fault handler (vector 14):
fn handle_page_fault(fault_addr: u64, error_code: u64) -> Result<()> {
    // 1. Check if fault_addr is in valid user range
    // 2. Allocate a physical page
    // 3. Map it in the user page tables
    // 4. Return to retry the instruction
}
```

The current PoC's page fault handler simply jumps to U2K, treating any fault as
a termination condition. In Dandelion, you would:

1. Check the faulting address and error code
2. If it's a valid demand-page fault, allocate and map the page
3. Use IRET to return to user code and retry
4. If it's an invalid access, record the fault and return to kernel

### 2. AP Boot Trampoline Integration

The `ap_startup.rs` module provides structures and interfaces, but the actual
boot trampoline (`boot_trampoline.S`) needs to be integrated with the target
environment's memory management:

1. Copy 16-bit boot code to low memory (< 1MB)
2. Set up per-CPU stacks
3. Send INIT-SIPI-SIPI sequence
4. Wait for APs to reach idle state

### 3. Core Affinity and Scheduling

The `CorePool` provides basic acquire/release semantics. For production:

- Add NUMA awareness
- Add core affinity hints
- Add timeout/watchdog for stuck APs

---

## Memory Map

```
Virtual Address Space (User CR3):
┌─────────────────────────────────┐ 0xFFFF_FFFF_FFFF_FFFF
│ (Not mapped)                    │
├─────────────────────────────────┤ 0x0000_0200_0000_0000 (2TB)
│ K2U Trampoline (1 page)         │ ← Mapped in both kernel & user
│ U2K Trampoline (1 page)         │
├─────────────────────────────────┤
│ Page Tables                     │ ← Dynamic size based on context
│ (PML4, PDPT, PD, PT)           │
├─────────────────────────────────┤
│ Interrupt Stack (6 pages)       │
├─────────────────────────────────┤
│ Handlers, GDT, TSS, IDT         │
├─────────────────────────────────┤
│ (Available for user heap)       │
├─────────────────────────────────┤
│ User Stack ↓                    │
├─────────────────────────────────┤
│ Data Region (Dandelion I/O)     │
├─────────────────────────────────┤
│ User Code (ELF)                 │
└─────────────────────────────────┘ 0x0000_0000_0000_0000
```

---

## Critical Alignment Requirements

| Structure | Alignment | Size | Notes |
|-----------|-----------|------|-------|
| PML4 | 4096 | 4096 | Page-aligned |
| PDPT | 4096 | 4096 | Page-aligned |
| PD | 4096 | 4096 | Page-aligned |
| PT | 4096 | 4096 | Page-aligned |
| GDT | 8 | 64 | 8-byte aligned |
| TSS | 4 | 104 | 4-byte aligned |
| IDT | 8 | 528 | 8-byte aligned (33 × 16) |
| Trampolines | 16 | ~1 page | 16-byte aligned code |

---

## Page Table Entry Flags

```rust
pub mod pte_flags {
    pub const PRESENT: u64 = 1 << 0;
    pub const WRITABLE: u64 = 1 << 1;
    pub const USER: u64 = 1 << 2;
    pub const HUGE_PAGE: u64 = 1 << 7;  // 2MB/1GB pages
    pub const NO_EXECUTE: u64 = 1 << 63;
    
    pub const USER_PAGE: u64 = PRESENT | WRITABLE | USER;
    pub const TABLE_ENTRY: u64 = PRESENT | WRITABLE | USER;
}
```

**User code pages:** `PRESENT | WRITABLE | USER`
**Kernel-only pages:** `PRESENT | WRITABLE` (no USER bit)
**Trampoline pages:** `PRESENT | WRITABLE | USER` (accessible from both rings)

---

## GDT Layout

```
Index 0: Null descriptor           (0x00)
Index 1: Kernel Code 64-bit        (0x08) - DPL=0
Index 2: Kernel Data               (0x10) - DPL=0
Index 3: User Code 64-bit          (0x18) - DPL=3, selector 0x1B with RPL=3
Index 4: User Data                 (0x20) - DPL=3, selector 0x23 with RPL=3
Index 5-6: TSS (16 bytes)          (0x28)
Index 7: Reserved
```

---

## Differences from KVM Engine

| Aspect | KVM | Unikraft |
|--------|-----|----------|
| Isolation | Hardware VM (VT-x) | Ring 3 + Page Tables |
| Memory | Guest Physical (EPT) | Shared Virtual |
| Execution | KVM_RUN ioctl | Assembly trampoline |
| Multi-core | vCPU threads | Real APs via SIPI |
| Interrupts | VMCS injection | IDT + IST |
| Exit handling | VM-Exit reasons | Interrupt vector |
| Page tables | EPT/NPT (2-level) | Standard x86-64 |

---

## Debugging Tips

1. **Triple Fault on K2U**: Check GDT descriptor format and segment selectors
2. **Page Fault Loop**: Verify trampoline pages have USER bit in BOTH page tables
3. **GPF on IRET**: Ensure RPL in segment selectors matches DPL in GDT entries
4. **Wrong CR2**: Page table entry addresses must be physical, not virtual
5. **Stack Corruption**: Verify TSS.RSP0 points to valid Ring 0 stack
6. **AP won't wake**: Check boot trampoline is at correct SIPI vector address

---

## Testing Checklist

- [ ] Page tables map user code correctly (try different context sizes)
- [ ] GDT/TSS/IDT installed without triple fault
- [ ] K2U trampoline reaches user code entry point
- [ ] User code can execute basic instructions
- [ ] INT 32 triggers U2K trampoline correctly
- [ ] Return to kernel preserves registers
- [ ] Page faults captured with correct CR2
- [ ] DandelionSystemData read/write works
- [ ] Multiple invocations reuse context correctly
- [ ] Multi-core: AP wakes and executes task
- [ ] Multi-core: Core pool acquire/release works

---

## References

- Intel SDM Vol. 3A, Chapter 4: Paging
- Intel SDM Vol. 3A, Chapter 6: Interrupt and Exception Handling
- Intel SDM Vol. 3A, Chapter 7: Task Management
- Intel SDM Vol. 3A, Chapter 8: Multiple-Processor Management
- AMD APM Vol. 2, Chapter 5: Page Translation and Protection
