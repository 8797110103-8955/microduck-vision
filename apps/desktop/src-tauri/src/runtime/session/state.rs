//! Desktop runtime state, control clock, and execution ownership.

use std::collections::HashMap;
use std::path::Path;
use std::path::PathBuf;
use std::process::Child;
use std::sync::{
    Arc, Mutex,
    atomic::{AtomicBool, AtomicU64, Ordering},
};
use std::thread::JoinHandle;
use std::time::Duration;

use crate::runtime::policy::inference::MicroDuckPolicyRunner;
use loop_body::{BackendRegistry, ExecutionGate, ReadOnlySource};
use tauri::{AppHandle, Wry};

use crate::platform::lifecycle::runtime_events;
use crate::platform::state::{MotionTracker, machine_store, world_placements};
use crate::runtime::adapters::servo_bus::ConnectedBus;
use crate::runtime::session::execution::PolicyExecution;
use crate::types::{
    ActionKind, ActionRecord, ActionSource, PhysicalMachine, TrainingProgress, VirtualMachine,
};

pub(crate) struct DesktopState {
    pub(crate) recorder: Arc<crate::runtime::recordings::Recorder>,
    pub(crate) backend_registry: Mutex<BackendRegistry>,
    pub(crate) execution_gate: ExecutionGate,
    pub(crate) execution_transition: Mutex<()>,
    pub(crate) config_dir: Option<PathBuf>,
    pub(crate) auto_mount_enabled: Mutex<bool>,
    pub(crate) auto_reconnect_enabled: Mutex<bool>,
    pub(crate) virtual_machines: Arc<Mutex<Vec<VirtualMachine>>>,
    pub(crate) physical_machines: Arc<Mutex<Vec<PhysicalMachine>>>,
    pub(crate) encoder_calibrations: Arc<Mutex<Vec<machine_store::PendingCalibration>>>,
    pub(crate) runtime_loop: Arc<Mutex<Option<RuntimeInstance>>>,
    pub(crate) world_placements: Arc<world_placements::WorldPlacements>,
    /// Backend identity is latched for the lifetime of an open loop so a running loop cannot
    /// silently switch between physical and simulation execution.
    pub(crate) loop_backend_kind: Mutex<Option<String>>,
    /// The physical bus is moved into `MicroDuckPolicyRunner` while a hardware loop runs.  Remember which
    /// machine owns it so `loop_close` can reopen the bus for the debug panel.
    pub(crate) loop_physical_machine_id: Mutex<Option<String>>,
    pub(crate) loop_virtual_machine_id: Mutex<Option<String>>,
    /// Debug loops keep the connected bus behind a shared lock so direct debug commands can
    /// operate on the same serial connection while the loop samples positions.
    pub(crate) loop_physical_bus: Mutex<Option<Arc<Mutex<ConnectedBus>>>>,
    pub(crate) motion: Mutex<MotionTracker>,
    /// The preview loop is always virtual and never owns a connected servo bus.
    /// Rust-owned control clock. Pages submit commands and subscribe to snapshots; they do not
    /// drive execution frequency.
    pub(crate) loop_clock_stop: Arc<AtomicBool>,
    pub(crate) loop_clock_thread: Mutex<Option<JoinHandle<()>>>,
    pub(crate) runtime_generation: Arc<AtomicU64>,
    pub(crate) execution_generation: Arc<AtomicU64>,
    pub(crate) runtime_fault: Arc<Mutex<Option<String>>>,
    pub(crate) command_sequence: AtomicU64,
    pub(crate) config_revision: Arc<AtomicU64>,
    pub(crate) connected_buses: Mutex<HashMap<String, ConnectedBus>>,
    pub(crate) actions: Mutex<Vec<ActionRecord>>,
    /// The optional local trainer is owned by the desktop process so its
    /// output can continue to reach the UI while the command itself returns.
    pub(crate) training_start: Mutex<()>,
    pub(crate) training_process: Mutex<Option<Arc<Mutex<Child>>>>,
    pub(crate) training_progress: Mutex<Option<TrainingProgress>>,
    pub(crate) training_stop_requested: Mutex<bool>,
    pub(crate) app_handle: Arc<Mutex<Option<AppHandle<Wry>>>>,
}

impl Default for DesktopState {
    fn default() -> Self {
        Self::new(None)
    }
}

impl DesktopState {
    pub(crate) fn new(config_dir: Option<PathBuf>) -> Self {
        Self::try_new(config_dir).expect("read desktop configuration")
    }

    pub(crate) fn ensure_encoder_calibration_ready(&self, physical: &str) -> Result<(), String> {
        if self
            .encoder_calibrations
            .lock()
            .map_err(|error| error.to_string())?
            .iter()
            .any(|entry| entry.physical_id == physical)
        {
            return Err(
                "这台实机有未完成的编码器校准；请在调试页再次执行编码器中位校准以恢复，当前不能驱动"
                    .into(),
            );
        }
        Ok(())
    }

    fn try_new(config_dir: Option<PathBuf>) -> Result<Self, String> {
        let machine_store::LoadedMachines {
            mut virtual_machines,
            physical_machines,
            auto_mount_enabled,
            auto_reconnect_enabled,
        } = machine_store::load_machines(config_dir.as_deref())?;
        Self::rebuild_references(&mut virtual_machines, &physical_machines);
        let backend_registry = BackendRegistry::default();
        let execution_gate = backend_registry.gate();
        Ok(Self {
            backend_registry: Mutex::new(backend_registry),
            execution_gate,
            execution_transition: Mutex::new(()),
            encoder_calibrations: Arc::new(Mutex::new(machine_store::load_pending_calibrations(
                config_dir.as_deref(),
            )?)),
            world_placements: Arc::new(world_placements::WorldPlacements::load(
                config_dir.as_deref(),
            )?),
            config_dir,
            auto_mount_enabled: Mutex::new(auto_mount_enabled),
            auto_reconnect_enabled: Mutex::new(auto_reconnect_enabled),
            virtual_machines: Arc::new(Mutex::new(virtual_machines)),
            physical_machines: Arc::new(Mutex::new(physical_machines)),
            runtime_loop: Arc::new(Mutex::new(None)),
            recorder: Arc::new(crate::runtime::recordings::Recorder::default()),
            loop_backend_kind: Mutex::new(None),
            loop_physical_machine_id: Mutex::new(None),
            loop_virtual_machine_id: Mutex::new(None),
            loop_physical_bus: Mutex::new(None),
            motion: Mutex::new(MotionTracker::default()),
            loop_clock_stop: Arc::new(AtomicBool::new(true)),
            loop_clock_thread: Mutex::new(None),
            runtime_generation: Arc::new(AtomicU64::new(0)),
            execution_generation: Arc::new(AtomicU64::new(0)),
            runtime_fault: Arc::new(Mutex::new(None)),
            command_sequence: AtomicU64::new(0),
            config_revision: Arc::new(AtomicU64::new(0)),
            connected_buses: Mutex::new(HashMap::new()),
            actions: Mutex::new(vec![
                ActionRecord {
                    id: "stand".into(),
                    name: "站立".into(),
                    kind: ActionKind::Pose,
                    updated_at: "内置".into(),
                    source: ActionSource::Builtin,
                },
                ActionRecord {
                    id: "sit".into(),
                    name: "坐下".into(),
                    kind: ActionKind::Pose,
                    updated_at: "内置".into(),
                    source: ActionSource::Builtin,
                },
            ]),
            training_start: Mutex::new(()),
            training_process: Mutex::new(None),
            training_progress: Mutex::new(None),
            training_stop_requested: Mutex::new(false),
            app_handle: Arc::new(Mutex::new(None)),
        })
    }

    pub(crate) fn set_app_handle(&self, app: AppHandle<Wry>) {
        if let Ok(mut current) = self.app_handle.lock() {
            *current = Some(app);
        }
    }

    pub(crate) fn emit_joint_range_states(&self, states: &[crate::types::JointRangeState]) {
        runtime_events::emit_joint_range_states(&self.app_handle, states);
    }

    pub(crate) fn load(config_dir: PathBuf) -> Result<Self, String> {
        let state = Self::try_new(Some(config_dir.join("config")))?;
        // Persist a newly created default machine's generated identity before exposing it.
        state.persist()?;
        Ok(state)
    }

    fn rebuild_references(
        virtual_machines: &mut [VirtualMachine],
        physical_machines: &[PhysicalMachine],
    ) {
        for machine in virtual_machines.iter_mut() {
            machine.referenced_by.clear();
        }
        for physical in physical_machines {
            if let Some(virtual_id) = &physical.virtual_machine_id
                && let Some(virtual_machine) = virtual_machines
                    .iter_mut()
                    .find(|machine| &machine.id == virtual_id)
            {
                virtual_machine.referenced_by.push(physical.id.clone());
            }
        }
    }

    pub(crate) fn persist(&self) -> Result<(), String> {
        self.persist_virtual_machines()?;
        self.persist_physical_machines()
    }

    pub(crate) fn persist_virtual_machines(&self) -> Result<(), String> {
        let Some(dir) = &self.config_dir else {
            return Ok(());
        };
        let virtual_machines = self
            .virtual_machines
            .lock()
            .map_err(|error| error.to_string())?
            .clone();
        let auto_mount = *self
            .auto_mount_enabled
            .lock()
            .map_err(|error| error.to_string())?;
        let auto_reconnect = *self
            .auto_reconnect_enabled
            .lock()
            .map_err(|error| error.to_string())?;
        machine_store::save_virtual_machines(dir, virtual_machines, auto_mount, auto_reconnect)
    }

    pub(crate) fn persist_physical_machines(&self) -> Result<(), String> {
        let Some(dir) = &self.config_dir else {
            return Ok(());
        };
        let machines = self.physical_machines.lock().map_err(|e| e.to_string())?;
        machine_store::save_physical_machines(dir, &machines)
    }

    pub(crate) fn start_loop_clock(&self) -> Result<(), String> {
        #[cfg(test)]
        {
            Ok(())
        }
        #[cfg(not(test))]
        {
            self.start_loop_clock_impl()
        }
    }

    pub(crate) fn start_loop_clock_impl(&self) -> Result<(), String> {
        let mut thread = self
            .loop_clock_thread
            .lock()
            .map_err(|error| error.to_string())?;
        if thread.is_some() {
            return Ok(());
        }
        self.loop_clock_stop.store(false, Ordering::Release);
        let stop = Arc::clone(&self.loop_clock_stop);
        let runtime = Arc::clone(&self.runtime_loop);
        let app_handle = Arc::clone(&self.app_handle);
        let generation = Arc::clone(&self.runtime_generation);
        let revision = Arc::clone(&self.config_revision);
        let execution_generation = Arc::clone(&self.execution_generation);
        let fault = Arc::clone(&self.runtime_fault);
        let gate = self.execution_gate.clone();
        let config_dir = self.config_dir.clone();
        let recorder = Arc::clone(&self.recorder);
        *thread = Some(std::thread::spawn(move || {
            let mut deadline = std::time::Instant::now();
            while !stop.load(Ordering::Acquire) {
                let mut period = loop_body::LoopRuntimeConfig::default().control_period;
                if let Ok(mut guard) = runtime.lock()
                    && let Some(instance) = guard.as_mut()
                {
                    period = instance.period();
                    let tick_started = std::time::Instant::now();
                    let mut tick_context = instance.snapshot();
                    tick_context.runtime_generation = generation.load(Ordering::Acquire);
                    tick_context.execution_generation =
                        execution_generation.load(Ordering::Acquire);
                    tick_context.config_revision = revision.load(Ordering::Acquire);
                    tick_context.tick += 1;
                    crate::platform::logging::set_context(
                        config_dir.as_deref(),
                        recorder.context(&tick_context),
                    );
                    let existing_fault = fault.lock().ok().and_then(|fault| fault.clone());
                    let mut status = if existing_fault.is_some() {
                        // Keep polling an isolated source cache, but never retry failed execution
                        // or silently resume inference after an execution/stop failure.
                        if let Some(source) = instance.read_source_mut() {
                            let _ = source.read();
                        }
                        instance.snapshot()
                    } else {
                        match instance.clock_tick(config_dir.as_deref()) {
                            Ok(status) => status,
                            Err(error) => {
                                let stop_error = instance.stop_execution().err();
                                execution_generation.fetch_add(1, Ordering::AcqRel);
                                revision.fetch_add(1, Ordering::AcqRel);
                                let detail = match stop_error {
                                    Some(stop_error) => {
                                        format!("{error}; stop failed: {stop_error}")
                                    }
                                    None => error,
                                };
                                let _ = gate.set_physical_writes_enabled(false);
                                crate::platform::logging::append_control_event(
                                    config_dir.as_deref(),
                                    format!("runtime_faulted error={detail}"),
                                );
                                if let Ok(mut fault) = fault.lock() {
                                    *fault = Some(detail);
                                }
                                instance.snapshot()
                            }
                        }
                    };
                    status.runtime_generation = generation.load(Ordering::Acquire);
                    status.execution_generation = execution_generation.load(Ordering::Acquire);
                    status.config_revision = revision.load(Ordering::Acquire);
                    if let Some(error) = fault.lock().ok().and_then(|fault| fault.clone()) {
                        status.runtime_state = crate::types::RuntimeState::Faulted;
                        status.last_error = Some(error);
                    }
                    recorder.capture(
                        &status,
                        if status.runtime_state == crate::types::RuntimeState::Faulted {
                            None
                        } else {
                            instance.policy_trace()
                        },
                        || instance.recording_metadata(),
                        tick_started,
                    );
                    runtime_events::emit_loop_status(&app_handle, status);
                }
                deadline += period;
                let now = std::time::Instant::now();
                if deadline > now {
                    std::thread::sleep(deadline - now);
                } else {
                    crate::platform::logging::append_control_event(
                        config_dir.as_deref(),
                        "control_deadline_missed",
                    );
                    deadline = now;
                }
            }
        }));
        Ok(())
    }

    pub(crate) fn activate_runtime(&self) -> u64 {
        self.command_sequence.store(0, Ordering::Release);
        if let Ok(mut fault) = self.runtime_fault.lock() {
            *fault = None;
        }
        self.execution_generation.fetch_add(1, Ordering::AcqRel);
        self.config_revision.fetch_add(1, Ordering::AcqRel);
        self.runtime_generation.fetch_add(1, Ordering::AcqRel) + 1
    }

    pub(crate) fn current_runtime_generation(&self) -> u64 {
        self.runtime_generation.load(Ordering::Acquire)
    }

    pub(crate) fn current_config_revision(&self) -> u64 {
        self.config_revision.load(Ordering::Acquire)
    }

    pub(crate) fn current_execution_generation(&self) -> u64 {
        self.execution_generation.load(Ordering::Acquire)
    }

    pub(crate) fn ensure_runtime_healthy(&self) -> Result<(), String> {
        if let Some(error) = self
            .runtime_fault
            .lock()
            .map_err(|error| error.to_string())?
            .as_ref()
        {
            return Err(format!(
                "runtime is Faulted; close it before reopening: {error}"
            ));
        }
        Ok(())
    }

    pub(crate) fn validate_runtime_generation(&self, expected: Option<u64>) -> Result<(), String> {
        if let Some(expected) = expected
            && expected != self.current_runtime_generation()
        {
            return Err(format!(
                "stale runtime generation {expected}, current is {}",
                self.current_runtime_generation()
            ));
        }
        Ok(())
    }

    pub(crate) fn stop_loop_clock(&self) -> Result<(), String> {
        self.loop_clock_stop.store(true, Ordering::Release);
        if let Some(thread) = self
            .loop_clock_thread
            .lock()
            .map_err(|error| error.to_string())?
            .take()
        {
            thread
                .join()
                .map_err(|_| "loop clock thread panicked".to_owned())?;
        }
        Ok(())
    }
}

#[allow(clippy::large_enum_variant)]
enum RuntimeExecution {
    Physical(MicroDuckPolicyRunner),
    Policy(PolicyExecution),
}

impl loop_body::RuntimeExecutor for RuntimeExecution {
    fn control_tick(&mut self, period: Duration) -> Result<(), String> {
        match self {
            Self::Physical(execution) => execution.tick(period).map_err(|e| e.to_string()),
            Self::Policy(execution) => execution.tick().map(|_| ()),
        }
    }

    fn stop_execution(&mut self) -> Result<(), String> {
        match self {
            Self::Physical(execution) => execution
                .send(loop_body::LoopAction::Stop)
                .map_err(|e| e.to_string()),
            Self::Policy(execution) => execution.stop(),
        }
    }
}

/// One common owner of the optional executor and independently sampled read sources.
pub(crate) struct RuntimeInstance {
    managed: loop_body::RuntimeLoop<RuntimeExecution, ReadOnlySource>,
    physical_recording_context: Option<serde_json::Value>,
}

impl RuntimeInstance {
    pub(crate) fn policy_trace(&self) -> Option<&crate::runtime::policy::inference::PolicyTrace> {
        match self.managed.execution()? {
            RuntimeExecution::Physical(execution) => execution.policy_trace(),
            RuntimeExecution::Policy(execution) => execution.runner().policy_trace(),
        }
    }

    pub(crate) fn recording_metadata(&self) -> serde_json::Value {
        match self.managed.execution() {
            Some(RuntimeExecution::Policy(execution)) => execution.recording_metadata(),
            _ => self.physical_recording_context.clone().unwrap_or_else(
                || serde_json::json!({"control":"manual","action_space":"canonical_joint_targets"}),
            ),
        }
    }
    pub(crate) fn physical_execution(execution: MicroDuckPolicyRunner, id: String) -> Self {
        let control_source = if execution.has_policy() {
            loop_body::ControlSource::Controller
        } else {
            loop_body::ControlSource::Manual
        };
        Self {
            physical_recording_context: None,
            managed: loop_body::RuntimeLoop::from_sessions(
                loop_body::LoopRuntimeConfig {
                    execution_backend: Some(id),
                    control_period: execution.control_period(),
                    control_source,
                    physical_access: loop_body::PhysicalAccess::Control,
                    ..Default::default()
                },
                Some(RuntimeExecution::Physical(execution)),
                Vec::new(),
            ),
        }
    }
    pub(crate) fn simulation_execution(execution: PolicyExecution, id: String) -> Self {
        Self {
            physical_recording_context: None,
            managed: loop_body::RuntimeLoop::from_sessions(
                loop_body::LoopRuntimeConfig {
                    execution_backend: Some(id),
                    control_period: execution.control_period(),
                    control_source: loop_body::ControlSource::Manual,
                    ..Default::default()
                },
                Some(RuntimeExecution::Policy(execution)),
                Vec::new(),
            ),
        }
    }
    pub(crate) fn read_only(source: ReadOnlySource) -> Self {
        let mut runtime = Self {
            physical_recording_context: None,
            managed: loop_body::RuntimeLoop::from_sessions(Default::default(), None, Vec::new()),
        };
        runtime
            .attach_source(source)
            .expect("validated source configuration");
        runtime
    }
    pub(crate) fn with_physical_recording_context(mut self, context: serde_json::Value) -> Self {
        self.physical_recording_context = Some(context);
        self
    }

    pub(crate) fn attach_source(&mut self, source: ReadOnlySource) -> Result<(), String> {
        let id = if source.id().is_empty() {
            "physical-read".into()
        } else {
            source.id().into()
        };
        let config = loop_body::ReadOnlySourceConfig {
            id,
            sample_period: Duration::from_millis(40),
            timeout: Duration::from_secs(2),
            max_age: Duration::from_millis(200),
            cache_capacity: 1,
        };
        let generation = loop_body::SourceGeneration(source.generation());
        self.managed
            .attach_source(config, generation, source)
            .map_err(|error| error.to_string())
    }
    pub(crate) fn take_source(&mut self) -> Option<ReadOnlySource> {
        (!self.managed.sources().is_empty()).then(|| self.managed.remove_source(0))
    }
    pub(crate) fn is_simulation(&self) -> bool {
        matches!(
            self.managed.execution(),
            Some(RuntimeExecution::Policy(execution)) if execution.physical_id().is_none()
        )
    }
    pub(crate) fn has_execution(&self) -> bool {
        self.managed.execution().is_some()
    }
    pub(crate) fn set_controller_source(&mut self) -> Result<(), String> {
        self.managed
            .set_control_source(loop_body::ControlSource::Controller)
            .map_err(|e| e.to_string())
    }
    pub(crate) fn period(&self) -> Duration {
        self.managed.config().control_period
    }

    pub(crate) fn clock_tick(
        &mut self,
        config_dir: Option<&Path>,
    ) -> Result<crate::types::LoopStatus, String> {
        self.managed.advance()?;
        if let Some(RuntimeExecution::Physical(execution)) = self.managed.execution()
            && let Some(trace) = execution.policy_trace()
        {
            crate::platform::logging::append_policy_trace(
                config_dir,
                execution.status().tick,
                trace,
            );
        }
        Ok(self.snapshot())
    }
    pub(crate) fn stop_execution(&mut self) -> Result<(), String> {
        self.managed.stop_execution()
    }
    pub(crate) fn snapshot(&self) -> crate::types::LoopStatus {
        let mut status = match self.managed.execution() {
            Some(RuntimeExecution::Physical(execution)) => {
                let mut status = crate::types::LoopStatus::from(execution.status());
                status.active_body = Some(crate::types::ActiveBody {
                    virtual_machine_id: None,
                    physical_machine_id: self.managed.config().execution_backend.clone(),
                });
                status
            }
            Some(RuntimeExecution::Policy(execution)) => {
                let mut status = execution.status();
                status.tick = self.managed.tick_index().0;
                status
            }
            None => {
                Self::source_status(self.read_source().expect("pure read runtime owns a source"))
            }
        };
        status.read_sources = self
            .managed
            .sources()
            .iter()
            .map(|(_, source)| source.into())
            .collect();
        status
    }
    fn source_status(source: &ReadOnlySource) -> crate::types::LoopStatus {
        let sample = source.latest();
        crate::types::LoopStatus {
            has_execution: false,
            active_body: None,
            runtime_state: crate::types::RuntimeState::Running,
            simulation: None,
            phase: crate::types::LoopPhase::Holding,
            session: 0,
            tick: source.sequence(),
            torque: false,
            positions: sample.map_or_else(Vec::new, |sample| sample.positions.clone()),
            targets: Vec::new(),
            world: crate::types::WorldState {
                position: [0.0; 2],
                yaw: 0.0,
                distance_m: 0.0,
            },
            sensor_failures: source.failures(),
            last_error: source.last_error().map(str::to_owned),
            joint_range_states: Vec::new(),
            runtime_generation: 0,
            execution_generation: 0,
            config_revision: 0,
            read_sources: vec![source.into()],
        }
    }

    pub(crate) fn has_policy(&self) -> bool {
        self.physical().is_ok_and(MicroDuckPolicyRunner::has_policy)
    }
    pub(crate) fn physical(&self) -> Result<&MicroDuckPolicyRunner, String> {
        match self.managed.execution() {
            Some(RuntimeExecution::Physical(execution)) => Ok(execution),
            Some(RuntimeExecution::Policy(execution)) if execution.physical_id().is_some() => {
                Ok(execution.runner())
            }
            _ => Err("active runtime has no physical execution".into()),
        }
    }
    pub(crate) fn physical_mut(&mut self) -> Result<&mut MicroDuckPolicyRunner, String> {
        match self.managed.execution_mut() {
            Some(RuntimeExecution::Physical(execution)) => Ok(execution),
            Some(RuntimeExecution::Policy(execution)) if execution.physical_id().is_some() => {
                Ok(execution.runner_mut())
            }
            _ => Err("active runtime has no physical execution".into()),
        }
    }
    pub(crate) fn policy_execution_mut(&mut self) -> Result<&mut PolicyExecution, String> {
        match self.managed.execution_mut() {
            Some(RuntimeExecution::Policy(execution)) => Ok(execution),
            _ => Err("active runtime has no preview policy execution".into()),
        }
    }
    pub(crate) fn has_policy_execution(&self) -> bool {
        matches!(self.managed.execution(), Some(RuntimeExecution::Policy(_)))
    }
    pub(crate) fn read_source(&self) -> Option<&ReadOnlySource> {
        self.managed.sources().first().map(|(_, source)| source)
    }
    pub(crate) fn read_source_mut(&mut self) -> Option<&mut ReadOnlySource> {
        self.managed
            .sources_mut()
            .first_mut()
            .map(|(_, source)| source)
    }
}

impl Drop for DesktopState {
    fn drop(&mut self) {
        self.loop_clock_stop.store(true, Ordering::Release);
        if let Ok(mut thread) = self.loop_clock_thread.lock()
            && let Some(thread) = thread.take()
        {
            let _ = thread.join();
        }
        let _ = self.recorder.stop("app_shutdown");
        crate::platform::logging::flush();
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn generated_default_identity_survives_reload_and_invalid_files_are_preserved() {
        let root = crate::platform::helpers::test_directory().join(
            crate::platform::helpers::new_machine_id("machine-config-test"),
        );
        let first = DesktopState::load(root.clone()).unwrap();
        let id = first.virtual_machines.lock().unwrap()[0].id.clone();
        assert!(uuid::Uuid::parse_str(id.strip_prefix("virtual-").unwrap()).is_ok());
        drop(first);
        let reloaded = DesktopState::load(root.clone()).unwrap();
        assert_eq!(reloaded.virtual_machines.lock().unwrap()[0].id, id);
        drop(reloaded);
        let invalid = "[[robot]\n";
        std::fs::write(root.join("config/robots.toml"), invalid).unwrap();
        assert!(DesktopState::load(root.clone()).is_err());
        assert_eq!(
            std::fs::read_to_string(root.join("config/robots.toml")).unwrap(),
            invalid
        );
        std::fs::remove_dir_all(root).unwrap();
    }
    #[test]
    fn physical_direction_configuration_loads_and_round_trips_without_zero_overrides() {
        let saved: machine_store::PersistedRobotTable = toml::from_str(
            r#"
[[robot]]
id = "physical-test"
name = "test"
executionKind = "physical"
modelKind = "hd1910"
servoModel = "hd1910"
virtualMachineId = "official-hd1910"
[[robot.census]]
servoId = 9
model = "hd1910"
[[robot.jointDirections]]
slotId = "left_hip_yaw"
direction = -1
[[robot.jointDirections]]
slotId = "left_hip_roll"
direction = 1
"#,
        )
        .unwrap();
        let physical = PhysicalMachine::from(saved.physical_machines[0].clone());
        assert_eq!(physical.joint_directions[0].direction, -1);
        assert_eq!(physical.joint_directions[1].direction, 1);
        assert!(physical.joint_bindings.is_empty());
        let encoded =
            toml::to_string(&machine_store::PersistedPhysicalMachine::from(&physical)).unwrap();
        let reloaded = PhysicalMachine::from(
            toml::from_str::<machine_store::PersistedPhysicalMachine>(&encoded).unwrap(),
        );
        assert_eq!(physical.joint_directions, reloaded.joint_directions);
    }

    use super::{DesktopState, machine_store};
    use crate::types::{
        PhysicalMachine, ReconnectHint, ServoIdentity, ServoJointCalibration, ServoJointDirection,
    };

    fn servo(id: u8, model: &str) -> ServoIdentity {
        ServoIdentity {
            servo_id: id,
            model: model.into(),
        }
    }

    #[test]
    fn system_config_round_trips_physical_weak_reference() {
        let config_dir = crate::platform::helpers::test_directory().join(format!(
            "microduck-vision-system-config-{}",
            std::process::id()
        ));
        let _ = std::fs::remove_dir_all(&config_dir);
        let state = DesktopState::new(Some(config_dir.clone()));
        let virtual_id = state.virtual_machines.lock().unwrap()[0].id.clone();
        state
            .physical_machines
            .lock()
            .unwrap()
            .push(PhysicalMachine {
                execution_kind: crate::types::ExecutionKindKey::Physical,
                model_kind: Some(crate::types::ModelKindKey::Hd1910),
                servo_model: Some(crate::types::ServoModelKey::Hd1910),
                id: "physical-1".into(),
                name: "鸭子".into(),
                usb_port: "COM7".into(),
                machine_code: "census-md5".into(),
                census: vec![servo(1, "hd1910")],
                reconnect_hint: Some(ReconnectHint {
                    port_name: "COM7".into(),
                    vid: Some(1),
                    pid: Some(2),
                    adapter_serial: Some("adapter-1".into()),
                    product: Some("USB bridge".into()),
                }),
                virtual_machine_id: Some(virtual_id.clone()),
                online_servo_ids: vec![1],
                connected: true,
                last_seen: Some("刚刚".into()),
                torque_enabled: false,
                joint_directions: vec![ServoJointDirection {
                    slot_id: "left_hip_yaw".into(),
                    direction: -1,
                }],
                joint_bindings: vec![ServoJointCalibration {
                    zero_offset_rad: 0.2,
                    min_rad: None,
                    max_rad: None,
                    slot_id: "left_hip_yaw".into(),
                }],
            });
        state.virtual_machines.lock().unwrap()[0]
            .simulation
            .world_position = [1.25, -0.4, 2.0];
        state.virtual_machines.lock().unwrap()[0].joint_bindings[0].physical_id = 1;
        let mut second = state.physical_machines.lock().unwrap()[0].clone();
        second.id = "physical-2".into();
        second.census = vec![servo(7, "hd1910")];
        second.joint_bindings[0].zero_offset_rad = 1.7;
        second.joint_directions[0].direction = 1;
        state.physical_machines.lock().unwrap().push(second);
        state.persist().unwrap();

        let loaded = DesktopState::new(Some(config_dir.clone()));
        let physical_machines = loaded.physical_machines.lock().unwrap();
        let physical = &physical_machines[0];
        assert_eq!(physical_machines[1].id, "physical-2");
        assert_eq!(physical_machines[1].joint_bindings[0].zero_offset_rad, 1.7);
        assert_eq!(physical_machines[1].joint_directions[0].direction, 1);
        assert!(!physical.connected);
        assert_eq!(
            physical
                .reconnect_hint
                .as_ref()
                .unwrap()
                .adapter_serial
                .as_deref(),
            Some("adapter-1")
        );
        assert_eq!(
            physical.virtual_machine_id.as_deref(),
            Some(virtual_id.as_str())
        );
        assert_eq!(physical.joint_directions[0].direction, -1);
        assert_eq!(physical.joint_bindings[0].zero_offset_rad, 0.2);
        assert_eq!(
            loaded.virtual_machines.lock().unwrap()[0].referenced_by,
            vec!["physical-1", "physical-2"]
        );
        assert_eq!(
            loaded.virtual_machines.lock().unwrap()[0]
                .simulation
                .world_position,
            [1.25, -0.4, 2.0]
        );
        assert_eq!(
            loaded.virtual_machines.lock().unwrap()[0].joint_bindings[0].physical_id,
            1
        );
        assert!(config_dir.join("machines.toml").is_file());
        let machines_text = std::fs::read_to_string(config_dir.join("machines.toml")).unwrap();
        assert!(machines_text.contains("autoMount = true"));
        assert!(machines_text.contains("autoReconnect = true"));
        let robots_text = std::fs::read_to_string(config_dir.join("robots.toml")).unwrap();
        assert!(robots_text.contains("reconnectHint"));
        assert!(robots_text.contains("jointBindings"));
        assert!(robots_text.contains("jointDirections"));
        assert!(!robots_text.contains("machine_code"));
        assert!(!robots_text.contains("connected"));
        let _ = std::fs::remove_dir_all(config_dir);
    }
}
