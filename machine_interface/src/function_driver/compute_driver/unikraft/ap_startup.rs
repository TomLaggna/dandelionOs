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

use core::sync::atomic::{AtomicU8, AtomicU64, Ordering};

#[cfg(target_arch = "x86_64")]
use core::arch::asm;

// ============================================================================
// Constants
// ============================================================================

/// Maximum number of CPUs supported
pub const MAX_CPUS: usize = 64;

/// Size of per-CPU data structure (must match boot_trampoline LCPU_SIZE)
pub const LCPU_SIZE: usize = 256;

/// Default target address for boot trampoline in low memory
pub const DEFAULT_TRAMPOLINE_ADDR: u64 = 0x8000;

/// Unikraft directmap base address (VA = directmap_base + PA)
pub const DIRECTMAP_BASE: u64 = 0xffffff8000000000;

/// CPU states
pub mod cpu_state {
    pub const OFFLINE: u8 = 0;
    pub const INIT: u8 = 1;
    pub const IDLE: u8 = 2;
    pub const BUSY: u8 = 3;
}

// ============================================================================
// x2APIC Constants and Functions
// ============================================================================

/// MSR addresses for x2APIC
mod msr {
    pub const APIC_BASE: u32 = 0x1b;
    pub const APIC_SVR: u32 = 0x80f;
    pub const APIC_ICR: u32 = 0x830;
}

/// APIC bits
mod apic_bits {
    pub const BASE_EN: u64 = 0x800;
    pub const BASE_EXTD: u64 = 0x400;
    pub const SVR_EN: u64 = 0x100;
    
    // ICR bits
    pub const ICR_TRIGGER_LEVEL: u64 = 1 << 15;
    pub const ICR_LEVEL_ASSERT: u64 = 1 << 14;
    pub const ICR_DESTMODE_PHYSICAL: u64 = 0;
    pub const ICR_DMODE_INIT: u64 = 5 << 8;
    pub const ICR_DMODE_SIPI: u64 = 6 << 8;
}

/// Read MSR
#[cfg(target_arch = "x86_64")]
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
#[cfg(target_arch = "x86_64")]
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
#[cfg(target_arch = "x86_64")]
pub fn x2apic_supported() -> bool {
    let ecx: u32;
    unsafe {
        // CPUID clobbers rbx which is reserved by LLVM, so we must preserve it
        asm!(
            "push rbx",
            "mov eax, 1",
            "cpuid",
            "pop rbx",
            out("ecx") ecx,
            out("eax") _,
            out("edx") _,
            options(nomem)
        );
    }
    (ecx & (1 << 21)) != 0
}

#[cfg(not(target_arch = "x86_64"))]
pub fn x2apic_supported() -> bool {
    false
}

/// Enable x2APIC mode
///
/// # Safety
/// Must be called only on CPUs that support x2APIC
#[cfg(target_arch = "x86_64")]
pub unsafe fn x2apic_enable() -> Result<(), &'static str> {
    if !x2apic_supported() {
        return Err("x2APIC not supported");
    }
    
    let mut base = rdmsr(msr::APIC_BASE);
    
    // Check if APIC is enabled
    if (base & apic_bits::BASE_EN) == 0 {
        return Err("APIC not enabled in firmware");
    }
    
    // Enable x2APIC mode
    base |= apic_bits::BASE_EXTD;
    wrmsr(msr::APIC_BASE, base);
    
    // Enable software APIC
    let mut svr = rdmsr(msr::APIC_SVR);
    if (svr & apic_bits::SVR_EN) == 0 {
        svr |= apic_bits::SVR_EN;
        wrmsr(msr::APIC_SVR, svr);
    }
    
    Ok(())
}

#[cfg(not(target_arch = "x86_64"))]
pub unsafe fn x2apic_enable() -> Result<(), &'static str> {
    Err("x2APIC only supported on x86_64")
}

/// Send INIT IPI to target CPU
///
/// # Safety
/// Caller must ensure destination APIC ID is valid
#[cfg(target_arch = "x86_64")]
pub unsafe fn send_init_ipi(dest_apic_id: u32) {
    use apic_bits::*;
    let icr = ICR_TRIGGER_LEVEL | ICR_LEVEL_ASSERT | ICR_DESTMODE_PHYSICAL | ICR_DMODE_INIT;
    wrmsr(msr::APIC_ICR, ((dest_apic_id as u64) << 32) | icr);
}

#[cfg(not(target_arch = "x86_64"))]
pub unsafe fn send_init_ipi(_dest_apic_id: u32) {}

/// Deassert INIT (required before SIPI)
///
/// # Safety
/// Caller must ensure destination APIC ID is valid
#[cfg(target_arch = "x86_64")]
pub unsafe fn deassert_init(dest_apic_id: u32) {
    use apic_bits::*;
    let icr = ICR_TRIGGER_LEVEL | ICR_DESTMODE_PHYSICAL | ICR_DMODE_INIT;
    wrmsr(msr::APIC_ICR, ((dest_apic_id as u64) << 32) | icr);
}

#[cfg(not(target_arch = "x86_64"))]
pub unsafe fn deassert_init(_dest_apic_id: u32) {}

/// Send Startup IPI (SIPI)
///
/// # Arguments
/// * `dest_apic_id` - Target APIC ID
/// * `vector` - Page number where boot code is located (address = vector << 12)
///
/// # Safety
/// - Boot trampoline must be at the specified vector address
/// - Caller must ensure destination APIC ID is valid
#[cfg(target_arch = "x86_64")]
pub unsafe fn send_startup_ipi(dest_apic_id: u32, vector: u8) {
    use apic_bits::*;
    let icr = ICR_DESTMODE_PHYSICAL | ICR_DMODE_SIPI | (vector as u64);
    wrmsr(msr::APIC_ICR, ((dest_apic_id as u64) << 32) | icr);
}

#[cfg(not(target_arch = "x86_64"))]
pub unsafe fn send_startup_ipi(_dest_apic_id: u32, _vector: u8) {}

/// Busy-wait delay (approximate microseconds)
pub fn delay_us(us: u32) {
    // ~250 iterations per microsecond at 1GHz
    let iterations = us.saturating_mul(250);
    for _ in 0..iterations {
        #[cfg(target_arch = "x86_64")]
        unsafe { asm!("nop", options(nomem, nostack)); }
        #[cfg(not(target_arch = "x86_64"))]
        core::hint::spin_loop();
    }
}

/// Busy-wait delay in milliseconds
pub fn delay_ms(ms: u32) {
    delay_us(ms.saturating_mul(1000));
}

// ============================================================================
// Per-CPU Data Structures
// ============================================================================

/// Per-CPU data structure
/// 
/// This structure is used by the boot trampoline to communicate with APs.
#[repr(C, align(64))]
pub struct CpuData {
    /// CPU index (0 = BSP, 1+ = APs)
    pub idx: u32,
    /// APIC ID
    pub id: u32,
    /// Current state (see cpu_state module)
    pub state: AtomicU8,
    /// Padding
    _pad0: [u8; 7],
    /// Entry point address (set by BSP, read by AP)
    pub entry: AtomicU64,
    /// Stack pointer (set by BSP, read by AP)
    pub stackp: AtomicU64,
    /// Pointer to ApTaskInfo for this CPU's current task
    pub task_info_ptr: AtomicU64,
    /// Reserved for future use
    _reserved: [u64; 25],
}

impl CpuData {
    pub const fn new() -> Self {
        Self {
            idx: 0,
            id: 0,
            state: AtomicU8::new(cpu_state::OFFLINE),
            _pad0: [0; 7],
            entry: AtomicU64::new(0),
            stackp: AtomicU64::new(0),
            task_info_ptr: AtomicU64::new(0),
            _reserved: [0; 25],
        }
    }
}

impl Default for CpuData {
    fn default() -> Self {
        Self::new()
    }
}

// ============================================================================
// Task Info (BSP ↔ AP Communication)
// ============================================================================

/// Task status values
pub mod task_status {
    pub const IDLE: u8 = 0;
    pub const RUNNING: u8 = 1;
    pub const DONE: u8 = 2;
    pub const ERROR: u8 = 3;
}

/// Task information passed from BSP to AP for each execution
///
/// Uses atomic access patterns for cross-core visibility.
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
    /// GDT base address
    pub gdt_base: AtomicU64,
    /// GDT limit
    pub gdt_limit: u16,
    _pad2: [u8; 6],
    /// IDT base address
    pub idt_base: AtomicU64,
    /// IDT limit
    pub idt_limit: u16,
    _pad3: [u8; 6],
    /// TSS base address
    pub tss_base: AtomicU64,
    /// TSS selector
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
            status: AtomicU8::new(task_status::IDLE),
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
    #[allow(clippy::too_many_arguments)]
    pub fn setup_task(
        &mut self,
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
        self.gdt_limit = gdt_limit;
        self.idt_base.store(idt_base, Ordering::Release);
        self.idt_limit = idt_limit;
        self.tss_base.store(tss_base, Ordering::Release);
        self.tss_selector = tss_selector;
        self.status.store(task_status::IDLE, Ordering::Release);
    }

    /// BSP: Poll status
    pub fn read_status(&self) -> u8 {
        self.status.load(Ordering::Acquire)
    }

    /// AP: Update status
    pub fn write_status(&self, status: u8) {
        self.status.store(status, Ordering::Release);
    }

    /// BSP: Wait for task completion
    pub fn wait_for_completion(&self) -> u8 {
        loop {
            let status = self.read_status();
            if status >= task_status::DONE {
                return status;
            }
            core::hint::spin_loop();
        }
    }
}

impl Default for ApTaskInfo {
    fn default() -> Self {
        Self::new()
    }
}

// ============================================================================
// Core Pool Management
// ============================================================================

/// Manages allocation of AP cores to execution tasks
pub struct CorePool {
    /// Status of each core (0=offline, 1=available, 2=busy)
    core_status: [AtomicU8; MAX_CPUS],
    /// Number of online cores
    online_count: AtomicU8,
}

impl CorePool {
    pub const fn new() -> Self {
        const STATUS_INIT: AtomicU8 = AtomicU8::new(cpu_state::OFFLINE);
        
        Self {
            core_status: [STATUS_INIT; MAX_CPUS],
            online_count: AtomicU8::new(0),
        }
    }

    /// Mark a core as online and available
    pub fn bring_core_online(&self, core_id: usize) {
        if core_id < MAX_CPUS {
            self.core_status[core_id].store(cpu_state::IDLE, Ordering::Release);
            self.online_count.fetch_add(1, Ordering::Relaxed);
        }
    }

    /// Try to acquire an available core
    /// Returns core ID if successful, None if all cores busy
    pub fn acquire_core(&self) -> Option<usize> {
        for i in 1..MAX_CPUS {  // Skip core 0 (BSP)
            let status = &self.core_status[i];
            if status.compare_exchange(
                cpu_state::IDLE,
                cpu_state::BUSY,
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
            self.core_status[core_id].store(cpu_state::IDLE, Ordering::Release);
        }
    }

    /// Get number of online cores
    pub fn online_count(&self) -> u8 {
        self.online_count.load(Ordering::Relaxed)
    }
}

impl Default for CorePool {
    fn default() -> Self {
        Self::new()
    }
}

// ============================================================================
// AP Wake Functions
// ============================================================================

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
// Physical Address Utilities
// ============================================================================

/// Convert virtual address to physical address using directmap
///
/// In Unikraft with directmap, VA = directmap_base + PA
/// So PA = VA - directmap_base
///
/// # Arguments
/// * `va` - Virtual address in the directmap region
/// * `directmap_base` - Base of the directmap region (typically 0xffffff8000000000)
///
/// # Safety
/// The address must be in the directmap region
pub fn va_to_pa(va: u64, directmap_base: u64) -> u64 {
    va.wrapping_sub(directmap_base)
}

/// Convert physical address to virtual address using directmap
///
/// # Arguments
/// * `pa` - Physical address
/// * `directmap_base` - Base of the directmap region
pub fn pa_to_va(pa: u64, directmap_base: u64) -> u64 {
    directmap_base.wrapping_add(pa)
}

/// Get the current kernel CR3 (page table root physical address)
#[cfg(target_arch = "x86_64")]
pub fn get_kernel_cr3() -> u64 {
    let cr3: u64;
    unsafe {
        asm!("mov {}, cr3", out(reg) cr3, options(nomem, nostack));
    }
    cr3
}

#[cfg(not(target_arch = "x86_64"))]
pub fn get_kernel_cr3() -> u64 {
    0
}

/// Walk kernel page tables to get the physical address for a virtual address
///
/// # Safety
/// - The virtual address must be mapped
/// - Called in a context where page tables won't change
#[cfg(target_arch = "x86_64")]
pub unsafe fn virt_to_phys(va: u64, directmap_base: u64) -> Option<u64> {
    let cr3 = get_kernel_cr3() & !0xFFF; // Mask off flags
    
    // PML4 index (bits 47:39)
    let pml4_idx = ((va >> 39) & 0x1FF) as usize;
    
    // Read PML4 entry via directmap
    let pml4_va = pa_to_va(cr3, directmap_base);
    let pml4_entry = core::ptr::read_volatile((pml4_va as *const u64).add(pml4_idx));
    
    if (pml4_entry & 1) == 0 {
        return None; // Not present
    }
    
    // PDPT index (bits 38:30)
    let pdpt_pa = pml4_entry & 0x000F_FFFF_FFFF_F000;
    let pdpt_idx = ((va >> 30) & 0x1FF) as usize;
    let pdpt_va = pa_to_va(pdpt_pa, directmap_base);
    let pdpt_entry = core::ptr::read_volatile((pdpt_va as *const u64).add(pdpt_idx));
    
    if (pdpt_entry & 1) == 0 {
        return None; // Not present
    }
    
    // Check for 1GB page
    if (pdpt_entry & 0x80) != 0 {
        let page_pa = pdpt_entry & 0x000F_FFFF_C000_0000;
        return Some(page_pa | (va & 0x3FFF_FFFF));
    }
    
    // PD index (bits 29:21)
    let pd_pa = pdpt_entry & 0x000F_FFFF_FFFF_F000;
    let pd_idx = ((va >> 21) & 0x1FF) as usize;
    let pd_va = pa_to_va(pd_pa, directmap_base);
    let pd_entry = core::ptr::read_volatile((pd_va as *const u64).add(pd_idx));
    
    if (pd_entry & 1) == 0 {
        return None; // Not present
    }
    
    // Check for 2MB page
    if (pd_entry & 0x80) != 0 {
        let page_pa = pd_entry & 0x000F_FFFF_FFE0_0000;
        return Some(page_pa | (va & 0x1F_FFFF));
    }
    
    // PT index (bits 20:12)
    let pt_pa = pd_entry & 0x000F_FFFF_FFFF_F000;
    let pt_idx = ((va >> 12) & 0x1FF) as usize;
    let pt_va = pa_to_va(pt_pa, directmap_base);
    let pt_entry = core::ptr::read_volatile((pt_va as *const u64).add(pt_idx));
    
    if (pt_entry & 1) == 0 {
        return None; // Not present
    }
    
    let page_pa = pt_entry & 0x000F_FFFF_FFFF_F000;
    Some(page_pa | (va & 0xFFF))
}

#[cfg(not(target_arch = "x86_64"))]
pub unsafe fn virt_to_phys(_va: u64, _directmap_base: u64) -> Option<u64> {
    None
}
