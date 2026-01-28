//! Application Processor (AP) Startup and Management
//!
//! This module handles multi-core support for the Unikraft execution engine:
//! - Boot trampoline for bringing APs from real mode to long mode
//! - Per-core state management
//! - Core allocation for parallel function execution
//!
//! # Architecture
//!
//! Unlike KVM which manages vCPUs through the hypervisor, Unikraft runs on bare
//! metal (or under a unikernel). We must manually:
//! 1. Wake APs via INIT-SIPI-SIPI sequence
//! 2. Provide 16-bit → 32-bit → 64-bit boot trampolines
//! 3. Set up per-core stacks, TLS, and page tables
//! 4. Manage core assignment for parallel execution
//!
//! # Integration Notes
//!
//! The boot trampoline assembly (boot_trampoline.S) must be:
//! 1. Copied to low memory (< 1MB) before sending SIPIs
//! 2. Linked with the kernel to resolve symbol references
//! 3. Patched with the kernel's CR3 and per-CPU struct addresses
//!
//! The AP entry point (ap_entry) receives a pointer to CpuData and should:
//! 1. Enable x2APIC
//! 2. Initialize TLS
//! 3. Enter the AP work loop waiting for tasks

use core::arch::asm;
use core::ptr;
use core::sync::atomic::{AtomicU8, AtomicU64, Ordering};

// ============================================================================
// Constants
// ============================================================================

/// Maximum number of CPUs supported
pub const MAX_CPUS: usize = 64;

/// Size of per-CPU data structure (must match boot_trampoline.S LCPU_SIZE)
pub const LCPU_SIZE: usize = 256;

/// CPU states (must match boot_trampoline.S LCPU_STATE_*)
pub mod cpu_state {
    pub const HALTED: i32 = 0;
    pub const INIT: i32 = 1;
    pub const IDLE: i32 = 2;
    pub const BUSY: i32 = 3;
}

// ============================================================================
// x2APIC Constants and Functions
// ============================================================================

/// MSR addresses for x2APIC
mod msr {
    pub const APIC_BASE: u32 = 0x1b;
    pub const APIC_SVR: u32 = 0x80f;
    pub const APIC_ICR: u32 = 0x830;
    pub const APIC_EOI: u32 = 0x80b;
}

/// APIC bits
mod apic_bits {
    pub const BASE_EN: u32 = 0x800;
    pub const BASE_EXTD: u32 = 0x400;
    pub const SVR_EN: u32 = 0x100;
    
    // ICR bits
    pub const ICR_TRIGGER_LEVEL: u32 = 1 << 15;
    pub const ICR_LEVEL_ASSERT: u32 = 1 << 14;
    pub const ICR_DESTMODE_PHYSICAL: u32 = 0;
    pub const ICR_DMODE_INIT: u32 = 5 << 8;
    pub const ICR_DMODE_SIPI: u32 = 6 << 8;
}

/// Read MSR
#[inline]
unsafe fn rdmsr(msr: u32) -> u64 {
    let low: u32;
    let high: u32;
    asm!(
        "rdmsr",
        in("ecx") msr,
        out("eax") low,
        out("edx") high,
        options(nomem, nostack)
    );
    ((high as u64) << 32) | (low as u64)
}

/// Write MSR
#[inline]
unsafe fn wrmsr(msr: u32, value: u64) {
    let low = value as u32;
    let high = (value >> 32) as u32;
    asm!(
        "wrmsr",
        in("ecx") msr,
        in("eax") low,
        in("edx") high,
        options(nomem, nostack)
    );
}

/// Check if x2APIC is supported via CPUID
pub fn x2apic_supported() -> bool {
    let ecx: u32;
    unsafe {
        asm!(
            "mov eax, 1",
            "cpuid",
            out("ecx") ecx,
            out("eax") _,
            out("ebx") _,
            out("edx") _,
            options(nomem, nostack)
        );
    }
    (ecx & (1 << 21)) != 0
}

/// Enable x2APIC mode
///
/// # Safety
/// Must be called only on CPUs that support x2APIC
pub unsafe fn x2apic_enable() -> Result<(), &'static str> {
    if !x2apic_supported() {
        return Err("x2APIC not supported");
    }
    
    let mut base = rdmsr(msr::APIC_BASE);
    
    // Check if APIC is enabled
    if (base & apic_bits::BASE_EN as u64) == 0 {
        return Err("APIC not enabled in firmware");
    }
    
    // Enable x2APIC mode
    base |= apic_bits::BASE_EXTD as u64;
    wrmsr(msr::APIC_BASE, base);
    
    // Enable software APIC
    let mut svr = rdmsr(msr::APIC_SVR);
    if (svr & apic_bits::SVR_EN as u64) == 0 {
        svr |= apic_bits::SVR_EN as u64;
        wrmsr(msr::APIC_SVR, svr);
    }
    
    Ok(())
}

/// Send INIT IPI to target CPU
///
/// # Safety
/// Caller must ensure destination APIC ID is valid
pub unsafe fn send_init_ipi(dest_apic_id: u32) {
    use apic_bits::*;
    let icr = ICR_TRIGGER_LEVEL | ICR_LEVEL_ASSERT | ICR_DESTMODE_PHYSICAL | ICR_DMODE_INIT;
    wrmsr(msr::APIC_ICR, ((dest_apic_id as u64) << 32) | (icr as u64));
}

/// Deassert INIT (required before SIPI)
///
/// # Safety
/// Caller must ensure destination APIC ID is valid
pub unsafe fn deassert_init(dest_apic_id: u32) {
    use apic_bits::*;
    let icr = ICR_TRIGGER_LEVEL | ICR_DESTMODE_PHYSICAL | ICR_DMODE_INIT;
    wrmsr(msr::APIC_ICR, ((dest_apic_id as u64) << 32) | (icr as u64));
}

/// Send Startup IPI (SIPI)
///
/// # Arguments
/// * `dest_apic_id` - Target APIC ID
/// * `vector` - Page number where boot code is located (address = vector << 12)
///
/// # Safety
/// - Boot trampoline must be at the specified vector address
/// - Caller must ensure destination APIC ID is valid
pub unsafe fn send_startup_ipi(dest_apic_id: u32, vector: u8) {
    use apic_bits::*;
    let icr = ICR_DESTMODE_PHYSICAL | ICR_DMODE_SIPI | (vector as u32);
    wrmsr(msr::APIC_ICR, ((dest_apic_id as u64) << 32) | (icr as u64));
}

/// Busy-wait delay (approximate microseconds)
pub fn delay_us(us: u32) {
    // ~1000 iterations per microsecond at 1GHz
    let iterations = us.saturating_mul(250);
    for _ in 0..iterations {
        unsafe { asm!("nop", options(nomem, nostack)); }
    }
}

/// Busy-wait delay in milliseconds
pub fn delay_ms(ms: u32) {
    delay_us(ms.saturating_mul(1000));
}

// ============================================================================
// Constants
// ============================================================================

/// Maximum number of CPUs supported
pub const MAX_CPUS: usize = 64;

/// Size of per-CPU data structure (must match boot_trampoline.S LCPU_SIZE)
pub const LCPU_SIZE: usize = 256;

/// CPU states (must match boot_trampoline.S LCPU_STATE_*)
pub mod cpu_state {
    pub const HALTED: i32 = 0;
    pub const INIT: i32 = 1;
    pub const IDLE: i32 = 2;
    pub const BUSY: i32 = 3;
}

// ============================================================================
// Per-CPU Data Structures
// ============================================================================

/// Per-CPU data structure
/// 
/// This structure is used by the boot trampoline to communicate with APs.
/// Layout must match boot_trampoline.S exactly.
#[repr(C, align(64))]
pub struct CpuData {
    /// CPU index (0 = BSP, 1+ = APs)
    pub idx: u32,
    /// APIC ID
    pub id: u32,
    /// Current state (see cpu_state module)
    pub state: i32,
    /// Padding
    _pad0: i32,
    /// Entry point address (set by BSP, read by AP)
    pub entry: u64,
    /// Stack pointer (set by BSP, read by AP)
    pub stackp: u64,
    /// Pointer to ApTaskInfo for this CPU's current task
    pub task_info_ptr: u64,
    /// Reserved for future use
    _reserved: [u64; 25],
}

impl CpuData {
    pub const fn new() -> Self {
        Self {
            idx: 0,
            id: 0,
            state: cpu_state::HALTED,
            _pad0: 0,
            entry: 0,
            stackp: 0,
            task_info_ptr: 0,
            _reserved: [0; 25],
        }
    }
}

// ============================================================================
// Task Info (BSP ↔ AP Communication)
// ============================================================================

/// Task information passed from BSP to AP for each execution
///
/// Uses volatile access patterns for cross-core visibility.
/// Access pattern:
/// - BSP writes all fields before waking AP
/// - AP reads fields and writes status updates
/// - BSP polls status to detect completion
#[repr(C, align(64))]
pub struct ApTaskInfo {
    /// User code entry point (from ELF)
    pub entry_point: AtomicU64,
    /// User page table physical address (for CR3)
    pub user_cr3: AtomicU64,
    /// Kernel page table physical address (for returning)
    pub kernel_cr3: AtomicU64,
    /// K→U trampoline virtual address
    pub k2u_trampoline: AtomicU64,
    /// U→K trampoline virtual address
    pub u2k_trampoline: AtomicU64,
    /// Task status: 0=idle, 1=running, 2=done, 3=error
    pub status: AtomicU8,
    _pad: [u8; 7],
    /// Interrupt configuration
    pub gdt_base: AtomicU64,
    pub gdt_limit: u16,
    _pad2: [u8; 6],
    pub idt_base: AtomicU64,
    pub idt_limit: u16,
    _pad3: [u8; 6],
    pub tss_base: AtomicU64,
    pub tss_selector: u16,
    _pad4: [u8; 6],
}

impl ApTaskInfo {
    pub const fn new() -> Self {
        Self {
            entry_point: AtomicU64::new(0),
            user_cr3: AtomicU64::new(0),
            kernel_cr3: AtomicU64::new(0),
            k2u_trampoline: AtomicU64::new(0),
            u2k_trampoline: AtomicU64::new(0),
            status: AtomicU8::new(0),
            _pad: [0; 7],
            gdt_base: AtomicU64::new(0),
            gdt_limit: 0,
            _pad2: [0; 6],
            idt_base: AtomicU64::new(0),
            idt_limit: 0,
            _pad3: [0; 6],
            tss_base: AtomicU64::new(0),
            tss_selector: 0,
            _pad4: [0; 6],
        }
    }

    /// BSP: Set all execution parameters before waking AP
    pub fn setup_task(
        &self,
        entry_point: u64,
        user_cr3: u64,
        kernel_cr3: u64,
        k2u_trampoline: u64,
        u2k_trampoline: u64,
        gdt_base: u64,
        gdt_limit: u16,
        idt_base: u64,
        idt_limit: u16,
        tss_base: u64,
        tss_selector: u16,
    ) {
        self.entry_point.store(entry_point, Ordering::Release);
        self.user_cr3.store(user_cr3, Ordering::Release);
        self.kernel_cr3.store(kernel_cr3, Ordering::Release);
        self.k2u_trampoline.store(k2u_trampoline, Ordering::Release);
        self.u2k_trampoline.store(u2k_trampoline, Ordering::Release);
        self.gdt_base.store(gdt_base, Ordering::Release);
        // Note: gdt_limit is not atomic, write via pointer
        unsafe {
            ptr::write_volatile(&self.gdt_limit as *const u16 as *mut u16, gdt_limit);
        }
        self.idt_base.store(idt_base, Ordering::Release);
        unsafe {
            ptr::write_volatile(&self.idt_limit as *const u16 as *mut u16, idt_limit);
        }
        self.tss_base.store(tss_base, Ordering::Release);
        unsafe {
            ptr::write_volatile(&self.tss_selector as *const u16 as *mut u16, tss_selector);
        }
        self.status.store(0, Ordering::Release);
    }

    /// AP: Read entry point
    pub fn read_entry_point(&self) -> u64 {
        self.entry_point.load(Ordering::Acquire)
    }

    /// AP: Read user CR3
    pub fn read_user_cr3(&self) -> u64 {
        self.user_cr3.load(Ordering::Acquire)
    }

    /// AP: Update status
    pub fn write_status(&self, status: u8) {
        self.status.store(status, Ordering::Release);
    }

    /// BSP: Poll status
    pub fn read_status(&self) -> u8 {
        self.status.load(Ordering::Acquire)
    }

    /// BSP: Wait for task completion
    pub fn wait_for_completion(&self) -> u8 {
        loop {
            let status = self.read_status();
            if status >= 2 {
                return status;
            }
            core::hint::spin_loop();
        }
    }
}

// ============================================================================
// Core Pool Management
// ============================================================================

/// Status values for cores in the pool
pub mod core_status {
    pub const OFFLINE: u8 = 0;
    pub const AVAILABLE: u8 = 1;
    pub const BUSY: u8 = 2;
}

/// Manages allocation of AP cores to execution tasks
///
/// # Integration with Dandelion
///
/// In Dandelion, each engine loop needs to acquire a core for execution.
/// The core pool provides thread-safe allocation:
///
/// ```ignore
/// let core_id = core_pool.acquire_core()?;
/// // Set up task info for this core
/// // Wake the AP
/// // Wait for completion
/// core_pool.release_core(core_id);
/// ```
pub struct CorePool {
    /// Status of each core (0=offline, 1=available, 2=busy)
    core_status: [AtomicU8; MAX_CPUS],
    /// Task info for each core
    task_info: [ApTaskInfo; MAX_CPUS],
    /// Number of online cores
    online_count: AtomicU8,
}

impl CorePool {
    pub const fn new() -> Self {
        const TASK_INFO_INIT: ApTaskInfo = ApTaskInfo::new();
        const STATUS_INIT: AtomicU8 = AtomicU8::new(core_status::OFFLINE);
        
        Self {
            core_status: [STATUS_INIT; MAX_CPUS],
            task_info: [TASK_INFO_INIT; MAX_CPUS],
            online_count: AtomicU8::new(0),
        }
    }

    /// Mark a core as online and available
    pub fn bring_core_online(&self, core_id: usize) {
        if core_id < MAX_CPUS {
            self.core_status[core_id].store(core_status::AVAILABLE, Ordering::Release);
            self.online_count.fetch_add(1, Ordering::Relaxed);
        }
    }

    /// Try to acquire an available core
    /// Returns core ID if successful, None if all cores busy
    pub fn acquire_core(&self) -> Option<usize> {
        for i in 1..MAX_CPUS {  // Skip core 0 (BSP)
            let status = &self.core_status[i];
            if status.compare_exchange(
                core_status::AVAILABLE,
                core_status::BUSY,
                Ordering::Acquire,
                Ordering::Relaxed,
            ).is_ok() {
                return Some(i);
            }
        }
        None
    }

    /// Release a core back to the pool
    pub fn release_core(&self, core_id: usize) {
        if core_id < MAX_CPUS {
            self.core_status[core_id].store(core_status::AVAILABLE, Ordering::Release);
        }
    }

    /// Get task info for a core
    pub fn get_task_info(&self, core_id: usize) -> Option<&ApTaskInfo> {
        if core_id < MAX_CPUS {
            Some(&self.task_info[core_id])
        } else {
            None
        }
    }

    /// Get number of online cores
    pub fn online_count(&self) -> u8 {
        self.online_count.load(Ordering::Relaxed)
    }
}

// ============================================================================
// Boot Trampoline Interface
// ============================================================================

/// Information needed to set up the boot trampoline
pub struct BootTrampolineConfig {
    /// Physical address where trampoline will be copied (must be < 1MB)
    pub target_addr: u64,
    /// Physical address of kernel PML4 (for AP CR3)
    pub kernel_cr3: u64,
    /// Physical address of per-CPU data array
    pub cpu_data_array: u64,
}

/// Copy boot trampoline to low memory and patch addresses
///
/// # Safety
/// - `config.target_addr` must be valid physical memory < 1MB
/// - `trampoline_code` must be the complete boot trampoline
/// - Must be called before sending any SIPIs
pub unsafe fn setup_boot_trampoline(
    config: &BootTrampolineConfig,
    trampoline_code: &[u8],
) {
    // This would copy the trampoline and patch:
    // - x86_bpt_pml4_addr with kernel_cr3
    // - lcpus address with cpu_data_array
    // - Relocation fixups for GDT pointers
    
    // Implementation depends on memory management primitives
    // available in the target environment
    todo!("Implement boot trampoline setup for target environment")
}

/// Send INIT-SIPI-SIPI sequence to wake an AP
///
/// This follows the Intel-specified sequence:
/// 1. Send INIT IPI
/// 2. Wait 10ms
/// 3. Send SIPI (twice, 200μs apart)
///
/// # Arguments
/// * `apic_id` - Target AP's APIC ID
/// * `sipi_vector` - Page number of boot trampoline (address = vector << 12)
///
/// # Safety
/// - Boot trampoline must be set up at the SIPI vector address
/// - Per-CPU data must be initialized for this APIC ID
pub unsafe fn wake_ap(apic_id: u32, sipi_vector: u8) {
    // Send INIT IPI
    send_init_ipi(apic_id);
    delay_ms(10);
    
    // Deassert INIT
    deassert_init(apic_id);
    delay_us(200);
    
    // Send SIPI twice (Intel specification)
    send_startup_ipi(apic_id, sipi_vector);
    delay_us(200);
    send_startup_ipi(apic_id, sipi_vector);
}

// ============================================================================
// AP Entry Point
// ============================================================================

/// AP work loop
///
/// This function is called by the AP after initialization.
/// It waits for tasks and executes them via the K→U trampoline.
///
/// # Arguments
/// * `cpu_data` - Pointer to this AP's CpuData structure
///
/// # Integration Notes
///
/// In Dandelion integration, this would:
/// 1. Read task info from cpu_data.task_info_ptr
/// 2. Set up the trampoline data section
/// 3. Call the K→U trampoline
/// 4. Handle return (via U→K trampoline)
/// 5. Update task status
/// 6. Loop waiting for next task
#[no_mangle]
pub extern "C" fn ap_work_loop(cpu_data: *const CpuData) -> ! {
    unsafe {
        let cpu = &*cpu_data;
        let _cpu_id = cpu.idx;

        // Mark as idle
        ptr::write_volatile(&cpu.state as *const i32 as *mut i32, cpu_state::IDLE);

        loop {
            // Wait for task assignment
            let task_ptr = cpu.task_info_ptr;
            if task_ptr == 0 {
                core::hint::spin_loop();
                continue;
            }

            let task_info = &*(task_ptr as *const ApTaskInfo);
            
            // Check if we have work
            let status = task_info.read_status();
            if status != 0 {
                core::hint::spin_loop();
                continue;
            }

            // Mark as running
            task_info.write_status(1);
            ptr::write_volatile(&cpu.state as *const i32 as *mut i32, cpu_state::BUSY);

            // Get trampoline entry
            let k2u_entry = task_info.k2u_trampoline.load(Ordering::Acquire);
            if k2u_entry == 0 {
                task_info.write_status(3); // Error
                continue;
            }

            // Call the K→U trampoline
            // This will switch to user CR3, load GDT/IDT/TSS,
            // and IRET to user code. Returns via U→K when user
            // code executes INT 32.
            let trampoline: extern "C" fn() = core::mem::transmute(k2u_entry);
            trampoline();

            // Returned from user space!
            task_info.write_status(2); // Done
            ptr::write_volatile(&cpu.state as *const i32 as *mut i32, cpu_state::IDLE);
        }
    }
}
