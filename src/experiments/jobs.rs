//! Owned worker thread; no OS startup hooks or child processes.
use crate::{
    config::LoadedConfig,
    sim::{RunOptions, RunSummary, SimulationCore},
    telemetry::TelemetrySample,
};
use std::{
    collections::VecDeque,
    sync::{Arc, Condvar, Mutex},
    thread::{self, JoinHandle},
    time::{Duration, Instant},
};
#[derive(Debug, Clone)]
pub struct Preview {
    pub contacts: crate::models::contact::ContactState,
    pub power: Option<PowerPreview>,
    pub sample: TelemetrySample,
    pub progress: f64,
    pub sensors: crate::sensor::SensorOutput,
    pub estimate: crate::control::estimator::Estimate,
    pub reason: String,
    pub steps: u64,
}
#[derive(Debug, Clone)]
pub struct PowerPreview {
    pub voltage: f64,
    pub current: f64,
    pub soc: f64,
    pub iterations: u32,
    pub residual: f64,
}
#[derive(Default)]
struct State {
    paused: bool,
    cancelled: bool,
    quota: u64,
    frames: VecDeque<Preview>,
    dropped: u64,
}
pub struct RunControl {
    state: Mutex<State>,
    wake: Condvar,
    capacity: usize,
    last: Mutex<Option<Instant>>,
}
impl RunControl {
    pub fn new(capacity: usize, paused: bool) -> Self {
        Self {
            state: Mutex::new(State {
                paused,
                ..Default::default()
            }),
            wake: Condvar::new(),
            capacity: capacity.clamp(1, 256),
            last: Mutex::new(None),
        }
    }
    pub fn pause(&self) {
        let mut state = self.state.lock().unwrap();
        state.paused = true;
        state.quota = 0;
    }
    pub fn resume(&self) {
        let mut state = self.state.lock().unwrap();
        state.paused = false;
        state.quota = 0;
        self.wake.notify_all();
    }
    pub fn step(&self, n: u64) {
        let mut s = self.state.lock().unwrap();
        s.paused = true;
        s.quota = s.quota.saturating_add(n);
        self.wake.notify_all();
    }
    pub fn cancel(&self) {
        self.state.lock().unwrap().cancelled = true;
        self.wake.notify_all();
    }
    pub fn cancelled(&self) -> bool {
        self.state.lock().unwrap().cancelled
    }
    pub fn dropped_frames(&self) -> u64 {
        self.state.lock().unwrap().dropped
    }
    pub fn latest(&self) -> Option<Preview> {
        let mut s = self.state.lock().unwrap();
        let v = s.frames.pop_back();
        s.dropped += s.frames.len() as u64;
        s.frames.clear();
        v
    }
    pub(crate) fn boundary(&self, core: &SimulationCore) -> bool {
        let mut s = self.state.lock().unwrap();
        let publish = s.paused || s.cancelled || core.is_finished();
        let mut last = self.last.lock().unwrap();
        if publish || last.is_none_or(|t| t.elapsed() >= Duration::from_millis(16)) {
            let frame = Preview {
                contacts: core.contact_state().clone(),
                power: core.power_state().map(|p| PowerPreview {
                    voltage: p.voltage_v,
                    current: p.current_a,
                    soc: p.soc,
                    iterations: p.iterations,
                    residual: p.residual_v,
                }),
                sample: core.sample(),
                progress: core.progress(),
                sensors: core.sensor_readings().clone(),
                estimate: core.estimated_state().clone(),
                reason: core.termination_reason().into(),
                steps: core.steps(),
            };
            if s.frames.len() == self.capacity {
                s.frames.pop_front();
                s.dropped += 1;
            }
            s.frames.push_back(frame);
            *last = Some(Instant::now());
        }
        drop(last);
        while s.paused && s.quota == 0 && !s.cancelled && !core.is_finished() {
            s = self.wake.wait(s).unwrap();
        }
        if s.quota > 0 {
            s.quota -= 1;
        }
        !s.cancelled
    }
}
pub struct SimulationWorker {
    pub run_id: String,
    pub config: LoadedConfig,
    pub control: Arc<RunControl>,
    result: Arc<Mutex<Option<Result<RunSummary, String>>>>,
    thread: Option<JoinHandle<()>>,
}
impl SimulationWorker {
    pub fn spawn(cfg: LoadedConfig, options: RunOptions, capacity: usize, paused: bool) -> Self {
        let control = Arc::new(RunControl::new(capacity, paused));
        let result = Arc::new(Mutex::new(None));
        let run_id = crate::sim::new_run_id();
        let id = run_id.clone();
        let frozen = cfg.clone();
        let c = control.clone();
        let r = result.clone();
        let thread = thread::spawn(move || {
            let outcome = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                crate::sim::run_controlled(frozen, options, Some(&c), id)
            }))
            .unwrap_or_else(|_| Err("simulation worker panicked".into()));
            *r.lock().unwrap() = Some(outcome);
        });
        Self {
            run_id,
            config: cfg,
            control,
            result,
            thread: Some(thread),
        }
    }
    pub fn take_result(&self) -> Option<Result<RunSummary, String>> {
        self.result.lock().unwrap().take()
    }
    pub fn is_finished(&self) -> bool {
        self.thread.as_ref().is_none_or(|t| t.is_finished())
    }
    pub fn join(&mut self) -> Result<(), String> {
        if let Some(t) = self.thread.take() {
            t.join().map_err(|_| "worker join failed")?;
        }
        Ok(())
    }
}
impl Drop for SimulationWorker {
    fn drop(&mut self) {
        self.control.cancel();
        let _ = self.join();
    }
}

// Auxiliary jobs use an owned thread and cooperative cancellation at work boundaries.
thread_local! {static CANCEL:std::cell::RefCell<Option<Arc<std::sync::atomic::AtomicBool>>>=const{std::cell::RefCell::new(None)};}
pub fn check_cancelled() -> Result<(), String> {
    if CANCEL.with(|c| {
        c.borrow()
            .as_ref()
            .is_some_and(|v| v.load(std::sync::atomic::Ordering::Relaxed))
    }) {
        Err("job cancelled".into())
    } else {
        Ok(())
    }
}
pub struct BackgroundJob<T: Send + 'static> {
    cancel: Arc<std::sync::atomic::AtomicBool>,
    result: Arc<Mutex<Option<Result<T, String>>>>,
    thread: Option<JoinHandle<()>>,
}
impl<T: Send + 'static> BackgroundJob<T> {
    pub fn spawn(f: impl FnOnce() -> Result<T, String> + Send + 'static) -> Self {
        let cancel = Arc::new(std::sync::atomic::AtomicBool::new(false));
        let result = Arc::new(Mutex::new(None));
        let c = cancel.clone();
        let r = result.clone();
        let thread = thread::spawn(move || {
            CANCEL.with(|token| *token.borrow_mut() = Some(c));
            let value = std::panic::catch_unwind(std::panic::AssertUnwindSafe(f))
                .unwrap_or_else(|_| Err("job panicked".into()));
            *r.lock().unwrap() = Some(value);
        });
        Self {
            cancel,
            result,
            thread: Some(thread),
        }
    }
    pub fn cancel(&self) {
        self.cancel
            .store(true, std::sync::atomic::Ordering::Relaxed);
    }
    pub fn take_result(&self) -> Option<Result<T, String>> {
        self.result.lock().unwrap().take()
    }
}
impl<T: Send + 'static> Drop for BackgroundJob<T> {
    fn drop(&mut self) {
        self.cancel();
        if let Some(t) = self.thread.take() {
            let _ = t.join();
        }
    }
}

/// Publish a generated artifact only after success. A cancelled job leaves existing output intact.
pub fn write_output_atomic<T>(
    path: &std::path::Path,
    write: impl FnOnce(&std::path::Path) -> Result<T, String>,
) -> Result<T, String> {
    let name = path
        .file_name()
        .ok_or("output filename missing")?
        .to_string_lossy();
    let temporary = path.with_file_name(format!(".{name}.{}.partial", crate::sim::new_run_id()));
    struct Cleanup(std::path::PathBuf);
    impl Drop for Cleanup {
        fn drop(&mut self) {
            let _ = std::fs::remove_file(&self.0);
        }
    }
    let _cleanup = Cleanup(temporary.clone());
    check_cancelled()?;
    let result = write(&temporary)?;
    check_cancelled()?;
    std::fs::rename(&temporary, path)
        .map_err(|e| format!("cannot publish {}: {e}", path.display()))?;
    Ok(result)
}
