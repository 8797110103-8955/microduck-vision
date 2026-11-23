//! Canonical sensor/target adapter for physical and virtual servo drivers.

use crate::BackendPositionTargets;
use duck_body::BodyBinding;
use std::collections::VecDeque;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Condvar, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use crate::{ControlBackend, GainCapability, LoopError, SensorFrame};

pub(crate) struct ServoAdapter {
    driver: Box<dyn ControlBackend>,
    binding: BodyBinding,
}

/// A sensor-only backend attachment. It owns no policy, target queue, lifecycle state machine,
/// or execution clock; callers can sample it beside a single active RuntimeLoop execution.
pub struct ReadOnlySource {
    adapter: Option<ServoAdapter>,
    sequence: u64,
    sampled_at: crate::SampleTime,
    latest: Option<crate::BackendState>,
    last_error: Option<String>,
    worker: Option<ReadWorker>,
    generation: u64,
}

static NEXT_SOURCE_GENERATION: AtomicU64 = AtomicU64::new(1);

struct ReadWorker {
    cancel: Arc<AtomicBool>,
    shared: Arc<(Mutex<SourceCache>, Condvar)>,
    thread: Option<JoinHandle<ServoAdapter>>,
    config: crate::ReadOnlySourceConfig,
}

#[derive(Default)]
struct SourceCache {
    cancel: bool,
    samples: VecDeque<crate::BackendState>,
    last_error: Option<String>,
    last_success: Option<Instant>,
    failures: u32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SourceHealth {
    Starting,
    Fresh,
    Stale,
    Offline,
}

impl ReadOnlySource {
    pub fn open(binding: crate::LoopBinding, driver: Box<dyn crate::ControlBackend>) -> Self {
        Self {
            adapter: Some(ServoAdapter::new(driver, binding.calibration)),
            sequence: 0,
            sampled_at: crate::SampleTime(0),
            latest: None,
            last_error: None,
            worker: None,
            generation: NEXT_SOURCE_GENERATION.fetch_add(1, Ordering::Relaxed),
        }
    }

    /// Move blocking sensor I/O to a single owned task. The cache is bounded and never feeds
    /// another execution's observation; `read` only collects the newest completed snapshot.
    pub fn start(
        &mut self,
        config: crate::ReadOnlySourceConfig,
    ) -> Result<(), crate::RuntimeConfigError> {
        config.validate()?;
        if self.worker.is_some() {
            return Ok(());
        }
        let mut adapter = self.adapter.take().ok_or_else(|| {
            crate::RuntimeConfigError::Provider("source adapter unavailable".into())
        })?;
        let cancel = Arc::new(AtomicBool::new(false));
        adapter.driver.set_read_cancel(Arc::clone(&cancel));
        let shared = Arc::new((Mutex::new(SourceCache::default()), Condvar::new()));
        let task_shared = Arc::clone(&shared);
        let task_config = config.clone();
        let mut sequence = self.sequence;
        let thread = std::thread::spawn(move || {
            let origin = Instant::now();
            loop {
                if task_shared.0.lock().map_or(true, |cache| cache.cancel) {
                    break;
                }
                let started = Instant::now();
                let result = adapter.read();
                let completed = Instant::now();
                let Ok(mut cache) = task_shared.0.lock() else {
                    break;
                };
                if cache.cancel {
                    break;
                }
                match result {
                    Ok(frame) if completed.duration_since(started) <= task_config.timeout => {
                        sequence = sequence.saturating_add(1);
                        let state = crate::BackendState {
                            valid_positions: frame.valid.clone(),
                            valid_velocities: frame.valid,
                            positions: frame.positions,
                            velocities: frame.velocities,
                            imu: frame.imu,
                            sequence,
                            sampled_at: crate::SampleTime(
                                origin.elapsed().as_micros().min(u128::from(u64::MAX)) as u64,
                            ),
                            source: Some(task_config.id.clone()),
                        };
                        if cache.samples.len() == task_config.cache_capacity {
                            cache.samples.pop_front();
                        }
                        cache.samples.push_back(state);
                        cache.last_success = Some(completed);
                        cache.last_error = None;
                        cache.failures = 0;
                    }
                    result => {
                        cache.failures = cache.failures.saturating_add(1);
                        cache.last_error = Some(match result {
                            Ok(_) => "read-only source timed out; late sample discarded".into(),
                            Err(error) => error.to_string(),
                        });
                    }
                }
                let wait = task_config.sample_period.saturating_sub(started.elapsed());
                let Ok((cache, _)) = task_shared
                    .1
                    .wait_timeout_while(cache, wait, |cache| !cache.cancel)
                else {
                    break;
                };
                if cache.cancel {
                    break;
                }
            }
            adapter
        });
        self.worker = Some(ReadWorker {
            cancel,
            shared,
            thread: Some(thread),
            config,
        });
        Ok(())
    }

    /// Cancellation is checked at transaction boundaries. A timed out join retains ownership
    /// so the caller cannot hand the same physical resource to a replacement session.
    pub fn stop(&mut self) -> Result<(), String> {
        let Some(worker) = self.worker.as_mut() else {
            return Ok(());
        };
        worker.cancel.store(true, Ordering::Release);
        if let Ok(mut cache) = worker.shared.0.lock() {
            cache.cancel = true;
        }
        worker.shared.1.notify_all();
        let deadline = Instant::now() + worker.config.timeout;
        while worker
            .thread
            .as_ref()
            .is_some_and(|thread| !thread.is_finished())
        {
            if Instant::now() >= deadline {
                return Err("read-only source shutdown timed out; resource remains owned".into());
            }
            std::thread::sleep(Duration::from_millis(1));
        }
        if let Some(thread) = worker.thread.take() {
            self.adapter = Some(
                thread
                    .join()
                    .map_err(|_| "read-only source task panicked".to_owned())?,
            );
        }
        self.worker = None;
        Ok(())
    }

    pub fn health(&self) -> SourceHealth {
        let Some(worker) = &self.worker else {
            return if self.last_error.is_some() {
                SourceHealth::Offline
            } else if self.latest.is_some() {
                SourceHealth::Fresh
            } else {
                SourceHealth::Starting
            };
        };
        let Ok(cache) = worker.shared.0.lock() else {
            return SourceHealth::Offline;
        };
        if cache.failures > 0 {
            return SourceHealth::Offline;
        }
        match cache.last_success {
            Some(time) if time.elapsed() > worker.config.max_age => SourceHealth::Stale,
            Some(_) => SourceHealth::Fresh,
            None => SourceHealth::Starting,
        }
    }

    pub fn generation(&self) -> u64 {
        self.generation
    }

    pub fn id(&self) -> &str {
        self.worker
            .as_ref()
            .map_or("read_only", |worker| worker.config.id.as_str())
    }

    pub fn sample_age(&self) -> Option<Duration> {
        self.worker.as_ref().and_then(|worker| {
            worker
                .shared
                .0
                .lock()
                .ok()?
                .last_success
                .map(|time| time.elapsed())
        })
    }

    pub fn failures(&self) -> u32 {
        self.worker
            .as_ref()
            .and_then(|worker| worker.shared.0.lock().ok().map(|cache| cache.failures))
            .unwrap_or_default()
    }

    pub fn read(&mut self) -> Result<crate::BackendState, crate::LoopError> {
        if let Some(worker) = &self.worker {
            let mut cache = worker.shared.0.lock().map_err(|_| {
                crate::LoopError::Servo(servo::ServoError::Bus("source cache poisoned".into()))
            })?;
            if let Some(state) = cache.samples.pop_back() {
                cache.samples.clear();
                self.sequence = state.sequence;
                self.sampled_at = state.sampled_at;
                self.latest = Some(state);
            }
            self.last_error = cache.last_error.clone();
            return self.latest.clone().ok_or_else(|| {
                crate::LoopError::Servo(servo::ServoError::Bus(
                    self.last_error
                        .clone()
                        .unwrap_or_else(|| "read-only source has no sample yet".into()),
                ))
            });
        }
        let frame = match self
            .adapter
            .as_mut()
            .expect("unmanaged source adapter")
            .read()
        {
            Ok(frame) => frame,
            Err(error) => {
                self.last_error = Some(error.to_string());
                return Err(error);
            }
        };
        self.sequence = self.sequence.saturating_add(1);
        self.sampled_at.0 = self.sampled_at.0.saturating_add(1);
        let state = crate::BackendState {
            valid_positions: frame.valid.clone(),
            valid_velocities: frame.valid,
            positions: frame.positions,
            velocities: frame.velocities,
            imu: frame.imu,
            sequence: self.sequence,
            sampled_at: self.sampled_at,
            source: Some("read_only".into()),
        };
        self.last_error = None;
        self.latest = Some(state.clone());
        Ok(state)
    }

    pub fn latest(&self) -> Option<&crate::BackendState> {
        self.latest.as_ref()
    }

    pub fn sequence(&self) -> u64 {
        self.sequence
    }

    pub fn last_error(&self) -> Option<&str> {
        self.last_error.as_deref()
    }
}

impl Drop for ReadOnlySource {
    fn drop(&mut self) {
        if let Some(worker) = &self.worker {
            worker.cancel.store(true, Ordering::Release);
            if let Ok(mut cache) = worker.shared.0.lock() {
                cache.cancel = true;
            }
            worker.shared.1.notify_all();
        }
    }
}

impl ServoAdapter {
    pub(crate) fn servo_ids(&self) -> impl Iterator<Item = u8> + '_ {
        self.binding.joints.iter().map(|joint| joint.physical_id)
    }
    pub(crate) fn new(driver: Box<dyn ControlBackend>, binding: BodyBinding) -> Self {
        Self { driver, binding }
    }

    pub(crate) fn read(&mut self) -> Result<SensorFrame, LoopError> {
        let ids = self.binding.physical_ids();
        let sample = self.driver.read_positions(&ids)?;
        if sample.positions.len() != ids.len() || sample.velocities.len() != ids.len() {
            return Err(LoopError::SensorLength {
                actual: sample.positions.len(),
                expected: ids.len(),
            });
        }
        Ok(SensorFrame {
            valid: self.driver.feedback_validity(&ids),
            positions: self.binding.to_body_positions(&sample.positions),
            velocities: self.binding.to_body_velocities(&sample.velocities),
            imu: self.driver.read_imu()?,
        })
    }

    pub(crate) fn write(&mut self, targets: &[f64]) -> Result<(), LoopError> {
        let ids = self.binding.physical_ids();
        let positions = self.binding.to_servo_positions_unchecked(targets);
        self.driver
            .write_positions(&ids, &BackendPositionTargets { positions })?;
        Ok(())
    }

    pub(crate) fn advance_execution(
        &mut self,
        physics_steps: u32,
        physics_dt: std::time::Duration,
    ) -> Result<(), LoopError> {
        self.driver
            .advance_execution(physics_steps, physics_dt)
            .map_err(LoopError::Servo)
    }

    pub(crate) fn reset_execution(&mut self) -> Result<(), LoopError> {
        self.driver.reset_execution().map_err(LoopError::Servo)
    }

    pub(crate) fn set_torque(&mut self, enabled: bool) -> Result<(), LoopError> {
        self.driver.set_torque(enabled)?;
        Ok(())
    }

    pub(crate) fn configure_gain(&mut self, gain: u16) -> Result<(), LoopError> {
        if self.driver.gain_capability() == GainCapability::Configurable {
            self.driver.set_gain(gain)?;
        }
        Ok(())
    }
}
