//! Unikraft Execution Engine Driver for Dandelion
//!
//! This module provides the compute driver for the Unikraft execution engine.
//! Unlike KVM which uses hardware virtualization, Unikraft runs user code in
//! Ring 3 with separate page tables within the same kernel address space.
//!
//! # Architecture
//!
//! The Unikraft engine uses CR3-switching trampolines to transition between
//! kernel and user contexts:
//!
//! ```text
//! ┌─────────────────────────────────────────────────────────────┐
//! │ Kernel (Ring 0)                                             │
//! │  ┌────────────────────┐    ┌─────────────────────────────┐  │
//! │  │ UnikraftLoop       │    │ User Space (Ring 3)         │  │
//! │  │ - setup page tables│◄──►│ - Own CR3 (page tables)     │  │
//! │  │ - setup GDT/TSS/IDT│    │ - Own GDT/IDT/TSS           │  │
//! │  │ - K2U/U2K trampoline│   │ - User code execution       │  │
//! │  └────────────────────┘    └─────────────────────────────┘  │
//! └─────────────────────────────────────────────────────────────┘
//! ```
//!
//! # Module Structure
//!
//! - `mod.rs` (this file): Main driver and engine loop
//! - `x86_64.rs`: Page table and interrupt table setup
//! - `trampolines.rs`: K2U/U2K assembly trampolines
//! - `handlers.rs`: Interrupt handlers
//! - `ap_startup.rs`: Application Processor (multi-core) management

#[cfg(target_arch = "x86_64")]
pub mod x86_64;
#[cfg(target_arch = "x86_64")]
pub mod trampolines;
#[cfg(target_arch = "x86_64")]
pub mod handlers;
pub mod ap_startup;

use crate::{
    function_driver::{
        thread_utils::{start_thread, EngineLoop},
        ComputeResource, Driver, ElfConfig, Function, FunctionConfig, WorkQueue,
    },
    interface::{read_output_structs, setup_input_structs},
    memory_domain::{
        unikraft::UnikraftContext, 
        system_domain::SystemContext,
        Context, ContextTrait, ContextType, MemoryDomain
    },
    util::elf_parser,
    DataItem, DataRequirement, DataRequirementList, DataSet, Position,
};
use dandelion_commons::{DandelionError, DandelionResult};
use log::{debug, info};
use std::collections::BTreeMap;
use std::sync::Arc;

#[cfg(target_arch = "x86_64")]
use x86_64 as arch;

// ============================================================================
// Constants
// ============================================================================

/// Page size
pub const PAGE_SIZE: usize = 4096;

// ============================================================================
// Unikraft Engine Loop
// ============================================================================

/// Engine loop state for Unikraft execution
struct UnikraftLoop {
    /// CPU slot this loop is bound to
    cpu_slot: u8,
}

impl EngineLoop for UnikraftLoop {
    fn init(core_id: u8) -> DandelionResult<Box<Self>> {
        debug!("Initializing UnikraftLoop on core {}", core_id);
        Ok(Box::new(UnikraftLoop { cpu_slot: core_id }))
    }

    fn run(
        &mut self,
        config: FunctionConfig,
        mut context: Context,
        output_sets: Arc<Vec<String>>,
    ) -> DandelionResult<Context> {
        info!("UnikraftLoop::run ENTRY on core {}", self.cpu_slot);

        let elf_config = match config {
            FunctionConfig::ElfConfig(conf) => conf,
            _ => {
                debug!("UnikraftLoop::run - config mismatch");
                return Err(DandelionError::ConfigMissmatch);
            }
        };

        debug!("UnikraftLoop::run - ELF entry point: {:#x}, system_data_offset: {:#x}", 
               elf_config.entry_point, elf_config.system_data_offset);

        // Setup input structures for the Dandelion interface
        debug!("UnikraftLoop::run - setting up input structs");
        setup_input_structs::<u64, u64>(
            &mut context,
            elf_config.system_data_offset,
            &output_sets,
        )?;
        debug!("UnikraftLoop::run - input structs done");

        // Get the Unikraft context
        let unikraft_context = match &mut context.context {
            ContextType::Unikraft(ctx) => ctx,
            _ => {
                debug!("UnikraftLoop::run - context mismatch");
                return Err(DandelionError::ContextMissmatch);
            }
        };

        // Initialize the context if not already done
        #[cfg(target_arch = "x86_64")]
        if !unikraft_context.initialized {
            debug!("UnikraftLoop::run - initializing context");
            self.initialize_context(unikraft_context, elf_config.entry_point)?;
            debug!("UnikraftLoop::run - context initialized");
        }

        // Run the user code
        debug!("UnikraftLoop::run - executing user code");
        self.execute_user_code(unikraft_context, elf_config.entry_point)?;
        debug!("UnikraftLoop::run - user code done");

        // Read output structures
        debug!("UnikraftLoop::run - reading output structs");
        read_output_structs::<u64, u64>(&mut context, elf_config.system_data_offset)?;
        debug!("UnikraftLoop::run - output structs done");

        info!("UnikraftLoop::run EXIT on core {}", self.cpu_slot);
        Ok(context)
    }
}

impl UnikraftLoop {
    /// Initialize the Unikraft context (page tables, GDT, TSS, IDT, etc.)
    #[cfg(target_arch = "x86_64")]
    fn initialize_context(
        &self,
        context: &mut UnikraftContext,
        _entry_point: usize,
    ) -> DandelionResult<()> {
        debug!("Initializing Unikraft context");

        // Extract values we need before any mutable borrow
        let phys_base = context.get_phys_base();
        let context_size = context.layout.total_size;

        // Track available memory (grows downward from top)
        let mut stack_start = context_size;

        // Set up page tables
        let pml4_phys = {
            let storage = unsafe { context.as_slice_mut() };
            arch::set_page_table(
                storage,
                &mut stack_start,
                context_size,
                phys_base,
                true, // use 2MB large pages
            ).map_err(|_| DandelionError::MemoryAllocationError)?
        };

        context.layout.pml4_phys = pml4_phys;
        context.layout.page_table_offset = stack_start;

        // Set up GDT, TSS, IDT
        let (gdt_offset, tss_offset, idt_offset, handler_offset, interrupt_stack_top) = {
            let storage = unsafe { context.as_slice_mut() };
            arch::set_interrupt_tables(
                storage,
                &mut stack_start,
                phys_base,
            ).map_err(|_| DandelionError::MemoryAllocationError)?
        };

        context.layout.gdt_offset = gdt_offset;
        context.layout.tss_offset = tss_offset;
        context.layout.idt_offset = idt_offset;
        context.layout.handler_offset = handler_offset;
        context.layout.interrupt_stack_offset = (interrupt_stack_top - phys_base) as usize;

        // Copy trampolines
        let trampoline_offset = stack_start - crate::memory_domain::unikraft::TRAMPOLINE_REGION_SIZE;
        if trampoline_offset < handler_offset {
            return Err(DandelionError::MemoryAllocationError);
        }

        {
            let storage = unsafe { context.as_slice_mut() };
            trampolines::copy_trampolines(
                storage,
                trampoline_offset,
                trampoline_offset + PAGE_SIZE,
            );
        }
        
        context.layout.trampoline_offset = trampoline_offset;
        context.layout.k2u_entry_offset = trampoline_offset;
        context.layout.u2k_entry_offset = trampoline_offset + PAGE_SIZE;
        
        stack_start = trampoline_offset;
        context.layout.user_stack_top = stack_start;

        context.initialized = true;
        debug!("Unikraft context initialized, user stack top: {:#x}", stack_start);

        Ok(())
    }

    /// Execute user code via the K2U trampoline
    fn execute_user_code(
        &self,
        context: &mut UnikraftContext,
        entry_point: usize,
    ) -> DandelionResult<()> {
        debug!("Executing user code at entry point {:#x}", entry_point);

        // In a real Unikraft environment, we would:
        // 1. Patch the K2U trampoline with user CR3, entry point, stack, GDT/IDT/TSS addresses
        // 2. Call the K2U trampoline
        // 3. User code executes in Ring 3
        // 4. User code returns via INT 32 -> U2K trampoline
        // 5. We return here

        #[cfg(target_arch = "x86_64")]
        {
            // Copy layout values before borrowing storage mutably
            let phys_base = context.get_phys_base();
            let user_cr3 = context.layout.pml4_phys;
            let gdt_offset = context.layout.gdt_offset;
            let idt_offset = context.layout.idt_offset;
            let user_stack_top_offset = context.layout.user_stack_top;
            let k2u_entry_offset = context.layout.k2u_entry_offset;
            let u2k_entry_offset = context.layout.u2k_entry_offset;
            let handler_offset = context.layout.handler_offset;

            // Calculate addresses
            let gdt_base = phys_base + gdt_offset as u64;
            let gdt_limit = (crate::memory_domain::unikraft::GDT_SIZE - 1) as u16;
            let idt_base = phys_base + idt_offset as u64;
            let idt_limit = (crate::memory_domain::unikraft::IDT_SIZE - 1) as u16;
            let tss_selector = arch::selectors::TSS;
            let user_stack_top = phys_base + user_stack_top_offset as u64;
            let user_entry_point = entry_point as u64;

            // Now borrow storage mutably
            let storage = unsafe { context.as_slice_mut() };

            // Patch K2U trampoline
            trampolines::patch_k2u(
                storage,
                k2u_entry_offset,
                user_cr3,
                gdt_base,
                gdt_limit,
                idt_base,
                idt_limit,
                tss_selector,
                user_stack_top,
                user_entry_point,
            );

            // Get current CR3 (kernel) for U2K
            let kernel_cr3: u64;
            unsafe {
                core::arch::asm!("mov {}, cr3", out(reg) kernel_cr3);
            }

            // Patch U2K trampoline
            trampolines::patch_u2k(
                storage,
                u2k_entry_offset,
                kernel_cr3,
            );

            // Patch handlers with U2K address
            let u2k_va = phys_base + u2k_entry_offset as u64;
            handlers::patch_handlers(
                storage,
                handler_offset,
                u2k_va,
            );

            // TODO: In a real implementation, we would call the K2U trampoline here
            // For now, this is a placeholder that logs the execution would happen
            info!("Unikraft: Would execute user code at {:#x} with CR3={:#x}", 
                  entry_point, user_cr3);
            info!("Unikraft: User stack at {:#x}, GDT at {:#x}", 
                  user_stack_top, gdt_base);
        }

        #[cfg(not(target_arch = "x86_64"))]
        {
            warn!("Unikraft execution only supported on x86_64");
        }

        Ok(())
    }
}

// ============================================================================
// Unikraft Driver
// ============================================================================

/// Driver for the Unikraft execution engine
pub struct UnikraftDriver {}

impl UnikraftDriver {
    /// Create a new Unikraft driver
    pub fn new() -> Self {
        UnikraftDriver {}
    }
}

impl Default for UnikraftDriver {
    fn default() -> Self {
        Self::new()
    }
}

impl Driver for UnikraftDriver {
    fn start_engine(
        &self,
        resource: ComputeResource,
        queue: Box<dyn WorkQueue + Send>,
    ) -> DandelionResult<()> {
        let cpu_slot = match resource {
            ComputeResource::CPU(core) => core,
            _ => return Err(DandelionError::EngineResourceError),
        };

        info!("Starting Unikraft engine on CPU {}", cpu_slot);
        start_thread::<UnikraftLoop>(cpu_slot, queue);
        info!("Unikraft engine thread started on CPU {}", cpu_slot);
        
        Ok(())
    }

    fn parse_function(
        &self,
        function_bin: Arc<[u8]>,
        _static_domain: &Box<dyn MemoryDomain>,
    ) -> DandelionResult<Function> {
        let elf = elf_parser::ParsedElf::new(&function_bin)?;
        let system_data = elf.get_symbol_by_name(&function_bin, "__dandelion_system_data")?;
        let entry = elf.get_entry_point();

        let config = FunctionConfig::ElfConfig(ElfConfig {
            system_data_offset: system_data.0,
            #[cfg(feature = "cheri")]
            return_offset: (0, 0),
            entry_point: entry,
            protection_flags: Arc::new(elf.get_memory_protection_layout()),
        });

        let (static_requirements, source_layout) = elf.get_layout_pair();
        let requirements = DataRequirementList {
            input_requirements: Vec::<DataRequirement>::new(),
            static_requirements,
        };

        // Calculate total size needed for storing parsed ELF segments
        let mut total_size = 0;
        for position in source_layout.iter() {
            total_size += position.size;
        }

        // Use SystemContext for the static parsed data (not UnikraftContext)
        // since we're just storing ELF segments, not executing code
        let mut context = Box::new(Context::new(
            ContextType::System(Box::new(SystemContext {
                local_offset_to_data_position: BTreeMap::new(),
                size: total_size,
            })),
            total_size,
        ));

        // Copy ELF segments
        let mut write_counter = 0;
        let mut new_content = DataSet {
            ident: String::from("static"),
            buffers: vec![],
        };
        let buffers = &mut new_content.buffers;

        for position in source_layout.iter() {
            context.write(
                write_counter,
                &function_bin[position.offset..position.offset + position.size],
            )?;
            buffers.push(DataItem {
                ident: String::from(""),
                data: Position {
                    offset: write_counter,
                    size: position.size,
                },
                key: 0,
            });
            write_counter += position.size;
        }

        context.content = vec![Some(new_content)];

        Ok(Function {
            requirements,
            context: Arc::from(context),
            config,
        })
    }
}
