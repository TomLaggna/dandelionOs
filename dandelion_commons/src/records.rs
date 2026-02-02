use crate::FunctionId;
use core::fmt;

#[cfg(not(feature = "timestamp"))]
use std::time::Instant;

/// CPU clock speed in MHz for converting cycles to time
const CPU_MHZ: f64 = 2304.009; // read as the TSC clock speed on my test machine

/// Read the CPU timestamp counter (RDTSC)
#[cfg(feature = "timestamp")]
#[inline(always)]
fn rdtsc() -> u64 {
    #[cfg(target_arch = "x86_64")]
    unsafe {
        core::arch::x86_64::_rdtsc()
    }
    #[cfg(target_arch = "aarch64")]
    unsafe {
        let cnt: u64;
        core::arch::asm!("mrs {}, cntvct_el0", out(reg) cnt);
        cnt
    }
}

/// Maximum usize to expect when converting a record point to a usize
/// By setting the last element to this explicitly, the compiler will throw an error,
/// if there are more than this, because it enumerates from 0 and won't allow a number to be assigned twice.
const LAST_RECORD_POINT: usize = 17;

#[repr(usize)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum RecordPoint {
    /// Queue to load the function code + ctx
    PrepareEnvQueue,
    /// Enqueue parsing operation (async)
    ParsingQueue,
    /// Start parsing (sync)
    ParsingStart,
    /// Finished Parsing (sync)
    ParsingEnd,
    /// Dequeue from parsing (async)
    ParsingDequeue,
    /// Start loading code + alloc ctx (sync)
    LoadStart,
    /// Start data transfer to the ctx (sync)
    TransferStart,
    /// Queue to get an engine for execution
    GetEngineQueue,
    /// Queue to get the function executed on the engine (async)
    ExecutionQueue,
    /// Start execution of the function on the engine (sync)
    EngineStart,
    /// KVM: Buffer allocation complete (VM memory region attached)
    BufferAllocationComplete,
    /// KVM: User code mapping complete (code pages mapped in page tables)
    UserCodeMappingComplete,
    /// KVM: User stack mapping complete (stack pages mapped)
    UserStackMappingComplete,
    /// KVM: Interrupt setup complete (IDT and handlers installed)
    InterruptSetupComplete,
    /// End of engine setup, start of actual function execution (sync)
    EngineSetupEnd,
    /// End of actual function execution, start of cleanup (sync)
    EngineExecEnd,
    /// End execution of the function on the engine (sync)
    EngineEnd,
    /// Return from execution engine (async)
    FutureReturn = LAST_RECORD_POINT,
}

const RECORD_POINT_NAMES: [&str; LAST_RECORD_POINT + 1] = [
    "PrepareEnvQueue",
    "ParsingQueue",
    "ParsingStart",
    "ParsingEnd",
    "ParsingDequeue",
    "LoadStart",
    "TransferStart",
    "GetEngineQueue",
    "ExecutionQueue",
    "EngineStart",
    "BufferAllocationComplete",
    "UserCodeMappingComplete",
    "UserStackMappingComplete",
    "InterruptSetupComplete",
    "EngineSetupEnd",
    "EngineExecEnd",
    "EngineEnd",
    "FutureReturn",
];

#[cfg(feature = "timestamp")]
struct FunctionTimestamp {
    function_id: FunctionId,
    creation: u64,
    time_points: [core::cell::UnsafeCell<u64>; LAST_RECORD_POINT + 1],
    children: std::sync::Mutex<Vec<FunctionTimestamp>>,
}
#[cfg(feature = "timestamp")]
unsafe impl Send for FunctionTimestamp {}
#[cfg(feature = "timestamp")]
unsafe impl Sync for FunctionTimestamp {}

#[cfg(feature = "timestamp")]
impl FunctionTimestamp {
    fn new(function_id: FunctionId, creation: u64) -> std::sync::Arc<Self> {
        return std::sync::Arc::new(Self {
            function_id,
            creation,
            time_points: [const { core::cell::UnsafeCell::new(0u64) }; LAST_RECORD_POINT + 1],
            children: std::sync::Mutex::new(Vec::new()),
        });
    }

    fn record(self: &std::sync::Arc<Self>, current_point: RecordPoint) {
        let current_tsc = rdtsc();
        let elapsed_cycles = current_tsc.saturating_sub(self.creation);
        // each point is only present once in the code, so we can be sure we can write there safely,
        // and sice it is in arc know the memory exists and will not be dropped during writing
        let reference = core::cell::UnsafeCell::raw_get(&self.time_points[current_point as usize]);
        unsafe { *reference = elapsed_cycles };
    }

    fn add_children(self: &mut std::sync::Arc<Self>, new_child: std::sync::Arc<Self>) {
        let mut guard = self.children.lock().unwrap();
        guard.push(std::sync::Arc::into_inner(new_child).unwrap());
    }
}

#[cfg(feature = "timestamp")]
impl fmt::Display for FunctionTimestamp {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        writeln!(f, "function_id: {}", self.function_id)?;
        // write own time points with names (cycles and nanoseconds)
        for index in 0..=LAST_RECORD_POINT {
            let cycles = unsafe { *self.time_points[index].get() };
            let micros = ((cycles as f64) / CPU_MHZ) as u64;
            writeln!(
                f,
                "  {}: {} cycles ({} μs)",
                RECORD_POINT_NAMES[index], cycles, micros
            )?;
        }
        let child_guard = self.children.lock().unwrap();
        if !child_guard.is_empty() {
            writeln!(f, "  children: {{")?;
            for child in child_guard.iter() {
                // Indent child output
                let child_str = format!("{}", child);
                for line in child_str.lines() {
                    writeln!(f, "    {}", line)?;
                }
            }
            write!(f, "  }}")?;
        }
        Ok(())
    }
}

#[cfg(feature = "timestamp")]
struct TimestampArchive {
    collected_timestamps: std::sync::Mutex<Vec<FunctionTimestamp>>,
}

#[cfg(feature = "timestamp")]
impl TimestampArchive {
    fn init() -> Self {
        return Self {
            collected_timestamps: std::sync::Mutex::new(Vec::new()),
        };
    }

    fn insert(&self, new_timestamp: FunctionTimestamp) {
        let mut guard = self.collected_timestamps.lock().unwrap();
        guard.push(new_timestamp);
    }

    fn reset(&self) {
        let mut guard = self.collected_timestamps.lock().unwrap();
        *guard = Vec::new();
    }

    fn append_timestamps(
        &self,
        timestamp: &FunctionTimestamp,
        summary: &mut String,
        indent: usize,
    ) {
        // push self with proper indentation, then use Display impl for the timestamp
        summary.push_str(&"-".repeat(indent));
        summary.push_str(&format!("{}", timestamp));
        summary.push('\n');
        let child_guard = timestamp.children.lock().unwrap();
        for child in child_guard.iter() {
            self.append_timestamps(child, summary, indent + 1);
        }
    }

    fn get_summary(&self, summary: &mut String) {
        for recorder in self.collected_timestamps.lock().unwrap().iter() {
            self.append_timestamps(recorder, summary, 0);
            summary.push_str("\n");
        }
    }
}

/// General implementation of recorder struct, additional functionality enabled by flags
pub struct Recorder {
    #[cfg(feature = "timestamp")]
    timestamps: std::sync::Arc<FunctionTimestamp>,
}

impl Recorder {
    #[cfg(feature = "timestamp")]
    pub fn new(_function_id: FunctionId) -> Self {
        return Self {
            timestamps: FunctionTimestamp::new(_function_id, rdtsc()),
        };
    }

    #[cfg(not(feature = "timestamp"))]
    pub fn new(_function_id: FunctionId, _start: Instant) -> Self {
        return Self {};
    }

    #[cfg(feature = "timestamp")]
    pub fn new_from_parent(_function_id: FunctionId, _parent: &Self) -> Self {
        return Self {
            timestamps: FunctionTimestamp::new(_function_id, _parent.timestamps.creation),
        };
    }

    #[cfg(not(feature = "timestamp"))]
    pub fn new_from_parent(_function_id: FunctionId, _parent: &Self) -> Self {
        return Self {};
    }

    pub fn record(&mut self, _current_point: RecordPoint) {
        #[cfg(feature = "timestamp")]
        self.timestamps.record(_current_point);
    }

    pub fn add_children(&mut self, _new_children: Vec<Recorder>) {
        #[cfg(feature = "timestamp")]
        for child in _new_children {
            self.timestamps.add_children(child.timestamps);
        }
    }

    pub fn get_sub_recorder(&self) -> Self {
        let recorder = Recorder {
            #[cfg(feature = "timestamp")]
            timestamps: self.timestamps.clone(),
        };
        return recorder;
    }
}

impl fmt::Display for Recorder {
    fn fmt(&self, _f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        #[cfg(feature = "timestamp")]
        {
            if std::sync::Arc::strong_count(&self.timestamps) != 1
                && std::sync::Arc::weak_count(&self.timestamps) != 0
            {
                panic!("Trying to format recorder that still has more than one reference");
            }
            self.timestamps.fmt(_f)?;
        }
        Ok(())
    }
}

pub struct Archive {
    #[cfg(feature = "timestamp")]
    timestamp_archive: TimestampArchive,
}

pub struct ArchiveInit {
    #[cfg(feature = "timestamp")]
    pub timestamp_count: usize,
}

impl Archive {
    pub fn init() -> Self {
        return Archive {
            #[cfg(feature = "timestamp")]
            timestamp_archive: TimestampArchive::init(),
        };
    }

    pub fn insert_recorder(&self, _recorder: Recorder) {
        #[cfg(feature = "timestamp")]
        self.timestamp_archive
            .insert(std::sync::Arc::into_inner(_recorder.timestamps).unwrap());
    }

    pub fn get_summary(&self) -> String {
        // For each recorder, print the timestamps
        #[allow(unused_mut)]
        let mut summary = String::new();
        #[cfg(feature = "timestamp")]
        self.timestamp_archive.get_summary(&mut summary);
        return summary;
    }

    pub fn reset(&self) {
        #[cfg(feature = "timestamp")]
        self.timestamp_archive.reset();
    }
}
