//! The product-agnostic single runtime container.
//!
//! `RuntimeLoop` owns at most one execution session and any number of read-only sessions. It is
//! intentionally independent of Tauri, MuJoCo, serial transports and controller storage. Products
//! resolve registrations and controller contracts before opening it.

use servo::ServoLoopDriver;
use std::time::Duration;

use crate::{
    BackendAccess, BackendKind, BackendRegistry, BackendSession, ControlBackend, ControlTick,
    ExecutionInterval, SampleTime,
};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PhysicalAccess {
    Disabled,
    ReadOnly,
    Control,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ControlSource {
    None,
    Manual,
    Controller,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ReadOnlySourceConfig {
    pub id: String,
    pub sample_period: Duration,
    pub timeout: Duration,
    pub max_age: Duration,
    pub cache_capacity: usize,
}

impl ReadOnlySourceConfig {
    pub fn validate(&self) -> Result<(), RuntimeConfigError> {
        if self.id.trim().is_empty()
            || self.sample_period.is_zero()
            || self.timeout.is_zero()
            || self.max_age.is_zero()
            || self.cache_capacity == 0
        {
            return Err(RuntimeConfigError::InvalidSource(self.id.clone()));
        }
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct LoopRuntimeConfig {
    pub execution_backend: Option<String>,
    pub control_source: ControlSource,
    pub physical_access: PhysicalAccess,
    pub read_sources: Vec<ReadOnlySourceConfig>,
    pub servo_ids: Vec<u8>,
    pub control_period: Duration,
    pub physics_dt: Duration,
    pub physics_substeps: u32,
}

impl Default for LoopRuntimeConfig {
    fn default() -> Self {
        Self {
            execution_backend: None,
            control_source: ControlSource::None,
            physical_access: PhysicalAccess::Disabled,
            read_sources: Vec::new(),
            servo_ids: Vec::new(),
            control_period: Duration::from_millis(20),
            physics_dt: Duration::from_millis(5),
            physics_substeps: 4,
        }
    }
}

#[derive(Debug, thiserror::Error, PartialEq, Eq)]
pub enum RuntimeConfigError {
    #[error("runtime requires an execution backend for this control source")]
    MissingExecutionBackend,
    #[error("runtime has no control source for its execution backend")]
    MissingControlSource,
    #[error("runtime physical access is incompatible with the selected backend")]
    InvalidPhysicalAccess,
    #[error("runtime backend is unknown: {0}")]
    UnknownBackend(String),
    #[error("runtime backend does not support control")]
    BackendCannotControl,
    #[error("runtime backend provider failed: {0}")]
    Provider(String),
    #[error("runtime source is invalid: {0}")]
    InvalidSource(String),
    #[error("runtime timing values are invalid")]
    InvalidTiming,
    #[error("runtime source duplicates the execution backend")]
    DuplicateExecutionSource,
}

impl LoopRuntimeConfig {
    pub fn validate(&self, registry: &BackendRegistry) -> Result<(), RuntimeConfigError> {
        if self.control_period.is_zero() || self.physics_dt.is_zero() || self.physics_substeps == 0
        {
            return Err(RuntimeConfigError::InvalidTiming);
        }
        let execution = self
            .execution_backend
            .as_deref()
            .map(|id| {
                registry
                    .describe(id)
                    .ok_or_else(|| RuntimeConfigError::UnknownBackend(id.to_owned()))
            })
            .transpose()?;
        match (execution, self.control_source, self.physical_access) {
            (None, ControlSource::None, PhysicalAccess::Disabled | PhysicalAccess::ReadOnly) => {}
            (None, _, _) => return Err(RuntimeConfigError::MissingExecutionBackend),
            (
                Some(descriptor),
                ControlSource::None,
                PhysicalAccess::Disabled | PhysicalAccess::ReadOnly,
            ) if descriptor.kind == BackendKind::Physical => {
                return Err(RuntimeConfigError::InvalidPhysicalAccess);
            }
            (Some(_), ControlSource::None, _) => {
                return Err(RuntimeConfigError::MissingControlSource);
            }
            (Some(descriptor), _, PhysicalAccess::Disabled)
                if descriptor.kind == BackendKind::Physical =>
            {
                return Err(RuntimeConfigError::InvalidPhysicalAccess);
            }
            (Some(descriptor), _, PhysicalAccess::ReadOnly)
                if descriptor.kind == BackendKind::Physical =>
            {
                return Err(RuntimeConfigError::InvalidPhysicalAccess);
            }
            (Some(descriptor), _, PhysicalAccess::Control)
                if descriptor.kind != BackendKind::Physical =>
            {
                return Err(RuntimeConfigError::InvalidPhysicalAccess);
            }
            (Some(descriptor), _, PhysicalAccess::Control) if !descriptor.supports_control => {
                return Err(RuntimeConfigError::BackendCannotControl);
            }
            _ => {}
        }
        for source in &self.read_sources {
            source.validate()?;
            if self.execution_backend.as_deref() == Some(source.id.as_str()) {
                return Err(RuntimeConfigError::DuplicateExecutionSource);
            }
            let descriptor = registry
                .describe(&source.id)
                .ok_or_else(|| RuntimeConfigError::UnknownBackend(source.id.clone()))?;
            if descriptor.kind != BackendKind::Physical {
                return Err(RuntimeConfigError::InvalidPhysicalAccess);
            }
            if self.physical_access == PhysicalAccess::Disabled {
                return Err(RuntimeConfigError::InvalidPhysicalAccess);
            }
        }
        Ok(())
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RuntimeGeneration(pub u64);

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ExecutionGeneration(pub u64);

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SourceGeneration(pub u64);

pub struct RuntimeTick {
    pub runtime_generation: RuntimeGeneration,
    pub execution_generation: Option<ExecutionGeneration>,
    pub tick: ControlTick,
    pub sampled_at: SampleTime,
    pub advanced_physics_steps: u64,
    pub source_generations: Vec<SourceGeneration>,
}

/// Host execution adapters share one scheduling decision. Controller and device-specific work
/// remains in the adapter; the runtime decides which executor and sources advance.
pub trait RuntimeExecutor {
    fn control_tick(&mut self, period: Duration) -> Result<(), String>;
    fn stop_execution(&mut self) -> Result<(), String>;
}

pub trait RuntimeSource {
    /// Collect a managed worker's completed sample without waiting for device I/O.
    fn collect(&mut self);
}

impl RuntimeSource for crate::ReadOnlySource {
    fn collect(&mut self) {
        let _ = self.read();
    }
}

/// One active runtime. Read-only source failures are isolated from the execution session; an
/// execution failure is returned to the caller; ownership is retained until stop is acknowledged.
pub struct RuntimeLoop<E = BackendSession, S = BackendSession> {
    config: LoopRuntimeConfig,
    runtime_generation: RuntimeGeneration,
    execution_generation: Option<ExecutionGeneration>,
    execution: Option<E>,
    read_sources: Vec<(SourceGeneration, S)>,
    tick: ControlTick,
    sampled_at: SampleTime,
    advanced_physics_steps: u64,
    pending_targets: Option<Vec<f64>>,
}

impl<E, S> RuntimeLoop<E, S> {
    /// Adopt sessions assembled by a host after resolving and validating its registrations.
    /// The host can wrap controller execution and managed I/O workers without opening another loop.
    pub fn from_sessions(
        config: LoopRuntimeConfig,
        execution: Option<E>,
        read_sources: Vec<(SourceGeneration, S)>,
    ) -> Self {
        Self {
            config,
            runtime_generation: RuntimeGeneration(1),
            execution_generation: execution.as_ref().map(|_| ExecutionGeneration(1)),
            execution,
            read_sources,
            tick: ControlTick(0),
            sampled_at: SampleTime(0),
            advanced_physics_steps: 0,
            pending_targets: None,
        }
    }
    pub fn execution(&self) -> Option<&E> {
        self.execution.as_ref()
    }
    pub fn execution_mut(&mut self) -> Option<&mut E> {
        self.execution.as_mut()
    }
    pub fn sources(&self) -> &[(SourceGeneration, S)] {
        &self.read_sources
    }
    pub fn sources_mut(&mut self) -> &mut [(SourceGeneration, S)] {
        &mut self.read_sources
    }
    pub fn config(&self) -> &LoopRuntimeConfig {
        &self.config
    }

    pub fn tick_index(&self) -> ControlTick {
        self.tick
    }
    pub fn set_control_source(&mut self, source: ControlSource) -> Result<(), RuntimeConfigError> {
        if self.execution.is_none() && source != ControlSource::None {
            return Err(RuntimeConfigError::MissingExecutionBackend);
        }
        if self.execution.is_some() && source == ControlSource::None {
            return Err(RuntimeConfigError::MissingControlSource);
        }
        self.config.control_source = source;
        Ok(())
    }
    pub fn attach_source(
        &mut self,
        config: ReadOnlySourceConfig,
        generation: SourceGeneration,
        source: S,
    ) -> Result<(), RuntimeConfigError> {
        config.validate()?;
        if self
            .config
            .read_sources
            .iter()
            .any(|entry| entry.id == config.id)
            || self.config.execution_backend.as_ref() == Some(&config.id)
        {
            return Err(RuntimeConfigError::DuplicateExecutionSource);
        }
        self.config.read_sources.push(config);
        if self.config.physical_access == PhysicalAccess::Disabled {
            self.config.physical_access = PhysicalAccess::ReadOnly;
        }
        self.read_sources.push((generation, source));
        Ok(())
    }
    /// Call only after the worker has acknowledged shutdown; failed shutdown retains ownership.
    pub fn remove_source(&mut self, index: usize) -> S {
        self.config.read_sources.remove(index);
        if self.read_sources.len() == 1 && self.config.physical_access == PhysicalAccess::ReadOnly {
            self.config.physical_access = PhysicalAccess::Disabled;
        }
        self.read_sources.remove(index).1
    }
}

impl<E: RuntimeExecutor, S: RuntimeSource> RuntimeLoop<E, S> {
    pub fn advance(&mut self) -> Result<(), String> {
        for (_, source) in &mut self.read_sources {
            source.collect();
        }
        if let Some(execution) = &mut self.execution {
            execution.control_tick(self.config.control_period)?;
        }
        self.tick.0 = self.tick.0.saturating_add(1);
        Ok(())
    }

    /// Failed stop retains the owner and its resources for an explicit retry.
    pub fn stop_execution(&mut self) -> Result<(), String> {
        if let Some(execution) = &mut self.execution {
            execution.stop_execution()?;
        }
        self.pending_targets = None;
        Ok(())
    }
}

impl RuntimeLoop {
    pub fn open(
        registry: &mut BackendRegistry,
        config: LoopRuntimeConfig,
    ) -> Result<Self, RuntimeConfigError> {
        config.validate(registry)?;
        let runtime_generation = RuntimeGeneration(1);
        let execution = if let Some(id) = config.execution_backend.as_deref() {
            let access = if config.control_source != ControlSource::None {
                BackendAccess::Control
            } else {
                BackendAccess::ReadOnly
            };
            Some(
                registry
                    .open(id, access)
                    .map_err(|error| RuntimeConfigError::Provider(error.to_string()))?,
            )
        } else {
            None
        };
        let execution_generation = execution
            .as_ref()
            .map(|session| ExecutionGeneration(session.generation()));
        let mut read_sources = Vec::new();
        for source in &config.read_sources {
            let session = registry
                .open(&source.id, BackendAccess::ReadOnly)
                .map_err(|error| RuntimeConfigError::Provider(error.to_string()))?;
            let generation = SourceGeneration(session.generation());
            read_sources.push((generation, session));
        }
        Ok(Self {
            config,
            runtime_generation,
            execution_generation,
            execution,
            read_sources,
            tick: ControlTick(0),
            sampled_at: SampleTime(0),
            advanced_physics_steps: 0,
            pending_targets: None,
        })
    }

    pub fn runtime_generation(&self) -> RuntimeGeneration {
        self.runtime_generation
    }
    pub fn execution_generation(&self) -> Option<ExecutionGeneration> {
        self.execution_generation
    }

    /// Replace the runtime configuration only after the current execution lease has been
    /// stopped and dropped. The returned instance receives a fresh runtime generation, so
    /// commands and samples from the previous configuration cannot be replayed into it.
    pub fn reconfigure(
        &mut self,
        registry: &mut BackendRegistry,
        config: LoopRuntimeConfig,
    ) -> Result<(), RuntimeConfigError> {
        config.validate(registry)?;
        let next_generation = RuntimeGeneration(self.runtime_generation.0.saturating_add(1));
        self.stop()?;
        let mut next = Self::open(registry, config)?;
        next.runtime_generation = next_generation;
        *self = next;
        Ok(())
    }
    pub fn submit_targets(&mut self, targets: Vec<f64>) -> Result<(), RuntimeConfigError> {
        if self.config.control_source == ControlSource::None || self.execution.is_none() {
            return Err(RuntimeConfigError::MissingControlSource);
        }
        if targets.iter().any(|target| !target.is_finite()) {
            return Err(RuntimeConfigError::InvalidTiming);
        }
        self.pending_targets = Some(targets);
        Ok(())
    }

    pub fn tick(&mut self) -> Result<RuntimeTick, RuntimeConfigError> {
        self.tick.0 = self.tick.0.saturating_add(1);
        self.sampled_at.0 = self.sampled_at.0.saturating_add(1);
        if let Some(execution) = self.execution.as_mut() {
            let _ = execution
                .read_positions(&self.config.servo_ids)
                .map_err(|error| RuntimeConfigError::Provider(error.to_string()))?;
            if let Some(targets) = self.pending_targets.take() {
                execution
                    .write_positions(
                        &self.config.servo_ids,
                        &servo::ServoPositionTargets { positions: targets },
                    )
                    .map_err(|error| RuntimeConfigError::Provider(error.to_string()))?;
            }
            execution
                .advance_execution(self.config.physics_substeps, self.config.physics_dt)
                .map_err(|error| RuntimeConfigError::Provider(error.to_string()))?;
            self.advanced_physics_steps = self
                .advanced_physics_steps
                .saturating_add(u64::from(self.config.physics_substeps));
        }
        for (_, source) in &mut self.read_sources {
            let _ = source.read_positions(&self.config.servo_ids);
        }
        Ok(RuntimeTick {
            runtime_generation: self.runtime_generation,
            execution_generation: self.execution_generation,
            tick: self.tick,
            sampled_at: self.sampled_at,
            advanced_physics_steps: self.advanced_physics_steps,
            source_generations: self
                .read_sources
                .iter()
                .map(|(generation, _)| *generation)
                .collect(),
        })
    }

    pub fn stop(&mut self) -> Result<(), RuntimeConfigError> {
        self.pending_targets = None;
        if let Some(execution) = self.execution.as_mut() {
            execution
                .set_torque(false)
                .map_err(|e| RuntimeConfigError::Provider(e.to_string()))?;
        }
        self.execution.take();
        self.read_sources.clear();
        self.execution_generation = None;
        self.config = LoopRuntimeConfig::default();
        Ok(())
    }
}

impl<E, S> std::fmt::Debug for RuntimeLoop<E, S> {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        formatter
            .debug_struct("RuntimeLoop")
            .field("config", &self.config)
            .field("runtime_generation", &self.runtime_generation)
            .field("execution_generation", &self.execution_generation)
            .field("tick", &self.tick)
            .finish_non_exhaustive()
    }
}

// Keep the import visible in generated API docs and make the intended backend boundary explicit.
fn _backend_boundary(_: &dyn ControlBackend, _: ExecutionInterval) {}

#[cfg(test)]
mod tests {
    use super::*;
    use servo::{ServoError, ServoLoopDriver, ServoPositionSample, ServoPositionTargets};
    use std::sync::{Arc, Mutex};

    #[derive(Default)]
    struct Recorder {
        calls: Arc<Mutex<Vec<&'static str>>>,
    }

    impl ServoLoopDriver for Recorder {
        fn read_positions(&mut self, _: &[u8]) -> Result<ServoPositionSample, ServoError> {
            self.calls.lock().unwrap().push("read");
            Ok(ServoPositionSample {
                positions: vec![0.0],
                velocities: vec![0.0],
            })
        }
        fn write_positions(
            &mut self,
            _: &[u8],
            _: &ServoPositionTargets,
        ) -> Result<(), ServoError> {
            self.calls.lock().unwrap().push("write");
            Ok(())
        }
        fn set_torque(&mut self, _: bool) -> Result<(), ServoError> {
            self.calls.lock().unwrap().push("torque");
            Ok(())
        }
        fn set_gain(&mut self, _: u16) -> Result<(), ServoError> {
            Ok(())
        }
        fn reboot(&mut self, _: u8) -> Result<(), ServoError> {
            Ok(())
        }
        fn advance_execution(&mut self, steps: u32, _: Duration) -> Result<(), ServoError> {
            if steps == 4 {
                self.calls.lock().unwrap().push("advance");
            }
            Ok(())
        }
    }

    fn registry(calls: Arc<Mutex<Vec<&'static str>>>) -> BackendRegistry {
        let mut registry = BackendRegistry::default();
        let factory_calls = calls.clone();
        registry
            .register(
                crate::BackendDescriptor {
                    id: "sim".into(),
                    kind: BackendKind::Simulation,
                    supports_control: true,
                    contract: "test".into(),
                },
                move || {
                    Ok(Box::new(Recorder {
                        calls: factory_calls.clone(),
                    }))
                },
            )
            .unwrap();
        let physical_calls = calls.clone();
        registry
            .register(
                crate::BackendDescriptor {
                    id: "physical".into(),
                    kind: BackendKind::Physical,
                    supports_control: true,
                    contract: "test".into(),
                },
                move || {
                    Ok(Box::new(Recorder {
                        calls: physical_calls.clone(),
                    }))
                },
            )
            .unwrap();
        registry
    }

    #[test]
    fn configuration_matrix_rejects_simulation_control_and_physical_readonly_execution() {
        let calls = Arc::new(Mutex::new(Vec::new()));
        let registry = registry(calls);
        let simulation_control = LoopRuntimeConfig {
            execution_backend: Some("sim".into()),
            control_source: ControlSource::Manual,
            physical_access: PhysicalAccess::Control,
            ..LoopRuntimeConfig::default()
        };
        assert_eq!(
            simulation_control.validate(&registry),
            Err(RuntimeConfigError::InvalidPhysicalAccess)
        );
        let physical_readonly = LoopRuntimeConfig {
            execution_backend: Some("physical".into()),
            control_source: ControlSource::None,
            physical_access: PhysicalAccess::ReadOnly,
            ..LoopRuntimeConfig::default()
        };
        assert_eq!(
            physical_readonly.validate(&registry),
            Err(RuntimeConfigError::InvalidPhysicalAccess)
        );
    }

    #[test]
    fn simulation_runtime_has_one_clock_and_four_physics_steps_per_tick() {
        let calls = Arc::new(Mutex::new(Vec::new()));
        let mut registry = registry(calls.clone());
        registry.select("sim").unwrap();
        let config = LoopRuntimeConfig {
            execution_backend: Some("sim".into()),
            control_source: ControlSource::Manual,
            physical_access: PhysicalAccess::Disabled,
            servo_ids: vec![1],
            ..LoopRuntimeConfig::default()
        };
        let mut runtime = RuntimeLoop::open(&mut registry, config).unwrap();
        runtime.submit_targets(vec![0.2]).unwrap();
        let first = runtime.tick().unwrap();
        let second = runtime.tick().unwrap();
        assert_eq!(first.tick, ControlTick(1));
        assert_eq!(second.tick, ControlTick(2));
        assert_eq!(second.advanced_physics_steps, 8);
        assert_eq!(
            *calls.lock().unwrap(),
            ["read", "write", "advance", "read", "advance"]
        );
    }

    #[test]
    fn read_only_source_has_no_execution_or_write_path() {
        let calls = Arc::new(Mutex::new(Vec::new()));
        let mut registry = registry(calls.clone());
        let config = LoopRuntimeConfig {
            physical_access: PhysicalAccess::ReadOnly,
            read_sources: vec![ReadOnlySourceConfig {
                id: "physical".into(),
                sample_period: Duration::from_millis(20),
                timeout: Duration::from_millis(10),
                max_age: Duration::from_millis(100),
                cache_capacity: 2,
            }],
            servo_ids: vec![1],
            ..LoopRuntimeConfig::default()
        };
        let mut runtime = RuntimeLoop::open(&mut registry, config).unwrap();
        runtime.tick().unwrap();
        assert_eq!(*calls.lock().unwrap(), ["read"]);
        assert_eq!(runtime.execution_generation(), None);
        assert_eq!(
            runtime.submit_targets(vec![0.2]),
            Err(RuntimeConfigError::MissingControlSource)
        );
    }

    #[test]
    fn reconfigure_revokes_the_old_runtime_generation_before_opening_the_new_one() {
        let calls = Arc::new(Mutex::new(Vec::new()));
        let mut registry = registry(calls);
        registry.select("sim").unwrap();
        let config = LoopRuntimeConfig {
            execution_backend: Some("sim".into()),
            control_source: ControlSource::Manual,
            servo_ids: vec![1],
            ..LoopRuntimeConfig::default()
        };
        let mut runtime = RuntimeLoop::open(&mut registry, config.clone()).unwrap();
        let old_generation = runtime.runtime_generation();
        runtime.reconfigure(&mut registry, config).unwrap();
        assert_eq!(old_generation, RuntimeGeneration(1));
        assert_eq!(runtime.runtime_generation(), RuntimeGeneration(2));
    }
}
