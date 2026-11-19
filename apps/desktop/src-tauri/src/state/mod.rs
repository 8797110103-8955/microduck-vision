//! Client-owned runtime state and TOML persistence.

mod motion;
pub mod types;

pub(crate) use motion::MotionTracker;

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

use crate::backend::policy_runtime::MicroDuckPolicyRunner;
use duck_body::ServoModelKey;
use loop_body::{BackendRegistry, ExecutionGate, ReadOnlySource};
use serde::{Deserialize, Serialize};
use tauri::{AppHandle, Emitter, Wry};

use crate::backend::control::ConnectedBus;
use crate::backend::physical;
use crate::backend::simulation::PreviewMicroDuckPolicyRunner;
use crate::helpers::default_simulation;
use crate::types::{
    ActionKind, ActionRecord, ActionSource, PhysicalMachine, ReconnectHint, ServoIdentity,
    ServoJointDirection, TrainingProgress, VirtualMachine,
};

#[derive(Debug, Clone, Default, Deserialize, Serialize)]
struct PersistedMachineTable {
    #[serde(default)]
    default: Option<String>,
    #[serde(default = "default_auto_mount")]
    auto_mount: bool,
    #[serde(default = "default_auto_reconnect")]
    auto_reconnect: bool,
    #[serde(rename = "machine", default)]
    virtual_machines: Vec<VirtualMachine>,
}

fn default_auto_mount() -> bool {
    true
}

fn default_auto_reconnect() -> bool {
    true
}

#[derive(Debug, Clone, Default, Deserialize, Serialize)]
struct PersistedRobotTable {
    #[serde(rename = "robot", default)]
    physical_machines: Vec<PersistedPhysicalMachine>,
}

#[derive(Debug, Clone, Deserialize, Serialize)]
#[serde(rename_all = "snake_case")]
struct PersistedPhysicalMachine {
    id: String,
    name: String,
    census: Vec<PersistedServoIdentity>,
    reconnect_hint: Option<PersistedReconnectHint>,
    virtual_machine_id: Option<String>,
    #[serde(default)]
    joint_directions: Vec<ServoJointDirection>,
}

#[derive(Debug, Clone, Deserialize, Serialize)]
#[serde(rename_all = "snake_case")]
struct PersistedServoIdentity {
    servo_id: u8,
    model: String,
}

#[derive(Debug, Clone, Deserialize, Serialize)]
#[serde(rename_all = "snake_case")]
struct PersistedReconnectHint {
    port_name: String,
    vid: Option<u16>,
    pid: Option<u16>,
    adapter_serial: Option<String>,
    product: Option<String>,
}

impl From<&PhysicalMachine> for PersistedPhysicalMachine {
    fn from(machine: &PhysicalMachine) -> Self {
        Self {
            id: machine.id.clone(),
            name: machine.name.clone(),
            census: machine
                .census
                .iter()
                .map(|servo| PersistedServoIdentity {
                    servo_id: servo.servo_id,
                    model: servo.model.clone(),
                })
                .collect(),
            reconnect_hint: machine
                .reconnect_hint
                .as_ref()
                .map(|hint| PersistedReconnectHint {
                    port_name: hint.port_name.clone(),
                    vid: hint.vid,
                    pid: hint.pid,
                    adapter_serial: hint.adapter_serial.clone(),
                    product: hint.product.clone(),
                }),
            virtual_machine_id: machine.virtual_machine_id.clone(),
            joint_directions: machine.joint_directions.clone(),
        }
    }
}

impl From<PersistedPhysicalMachine> for PhysicalMachine {
    fn from(machine: PersistedPhysicalMachine) -> Self {
        let reconnect_hint = machine.reconnect_hint.map(|hint| ReconnectHint {
            port_name: hint.port_name,
            vid: hint.vid,
            pid: hint.pid,
            adapter_serial: hint.adapter_serial,
            product: hint.product,
        });
        let census = machine
            .census
            .into_iter()
            .map(|servo| ServoIdentity {
                servo_id: servo.servo_id,
                model: servo.model,
            })
            .collect::<Vec<_>>();
        let machine_code = physical::census_md5(&census).unwrap_or_default();
        let name = if machine.name.trim().is_empty() {
            machine_code.clone()
        } else {
            machine.name
        };
        Self {
            id: machine.id,
            name,
            usb_port: reconnect_hint
                .as_ref()
                .map(|hint| hint.port_name.clone())
                .unwrap_or_default(),
            machine_code,
            census,
            reconnect_hint,
            virtual_machine_id: machine.virtual_machine_id,
            online_servo_ids: Vec::new(),
            connected: false,
            last_seen: None,
            torque_enabled: false,
            joint_directions: machine
                .joint_directions
                .into_iter()
                .filter(|binding| matches!(binding.direction, -1 | 1))
                .collect(),
        }
    }
}

pub(crate) struct DesktopState {
    pub(crate) backend_registry: Mutex<BackendRegistry>,
    pub(crate) execution_gate: ExecutionGate,
    pub(crate) execution_transition: Mutex<()>,
    pub(crate) config_dir: Option<PathBuf>,
    pub(crate) auto_mount_enabled: Mutex<bool>,
    pub(crate) auto_reconnect_enabled: Mutex<bool>,
    pub(crate) virtual_machines: Mutex<Vec<VirtualMachine>>,
    pub(crate) physical_machines: Mutex<Vec<PhysicalMachine>>,
    pub(crate) runtime_loop: Arc<Mutex<Option<RuntimeInstance>>>,
    /// Backend identity is latched for the lifetime of an open loop so a running loop cannot
    /// silently switch between physical and simulation execution.
    pub(crate) loop_backend_kind: Mutex<Option<String>>,
    /// The physical bus is moved into `MicroDuckPolicyRunner` while a hardware loop runs.  Remember which
    /// machine owns it so `loop_close` can reopen the bus for the debug panel.
    pub(crate) loop_physical_machine_id: Mutex<Option<String>>,
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
        let persisted_machines = config_dir.as_ref().and_then(|dir| {
            std::fs::read_to_string(dir.join("machines.toml"))
                .ok()
                .and_then(|text| toml::from_str::<PersistedMachineTable>(&text).ok())
        });
        let persisted_robots = config_dir.as_ref().and_then(|dir| {
            std::fs::read_to_string(dir.join("robots.toml"))
                .ok()
                .and_then(|text| toml::from_str::<PersistedRobotTable>(&text).ok())
        });
        let mut virtual_machines = persisted_machines
            .as_ref()
            .map(|config| config.virtual_machines.clone())
            .unwrap_or_else(|| {
                vec![VirtualMachine {
                    id: "official-hd1910".into(),
                    name: "机器 0".into(),
                    body_id: "official".into(),
                    servo_model: ServoModelKey::Hd1910,
                    simulation: default_simulation(ServoModelKey::Hd1910),
                    startup_selected: true,
                    referenced_by: Vec::new(),
                    joint_bindings: Vec::new(),
                }]
            });
        let auto_mount_enabled = persisted_machines
            .as_ref()
            .map(|config| config.auto_mount)
            .unwrap_or_else(default_auto_mount);
        let auto_reconnect_enabled = persisted_machines
            .as_ref()
            .map(|config| config.auto_reconnect)
            .unwrap_or_else(default_auto_reconnect);
        if let Some(default) = persisted_machines
            .as_ref()
            .and_then(|config| config.default.clone())
        {
            for machine in &mut virtual_machines {
                machine.startup_selected = machine.id == default;
            }
        }
        let physical_machines: Vec<PhysicalMachine> = persisted_robots
            .map(|config| config.physical_machines)
            .unwrap_or_default()
            .into_iter()
            .map(Into::into)
            .collect();
        Self::rebuild_references(&mut virtual_machines, &physical_machines);
        let backend_registry = BackendRegistry::default();
        let execution_gate = backend_registry.gate();
        Self {
            backend_registry: Mutex::new(backend_registry),
            execution_gate,
            execution_transition: Mutex::new(()),
            config_dir,
            auto_mount_enabled: Mutex::new(auto_mount_enabled),
            auto_reconnect_enabled: Mutex::new(auto_reconnect_enabled),
            virtual_machines: Mutex::new(virtual_machines),
            physical_machines: Mutex::new(physical_machines),
            runtime_loop: Arc::new(Mutex::new(None)),
            loop_backend_kind: Mutex::new(None),
            loop_physical_machine_id: Mutex::new(None),
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
            training_process: Mutex::new(None),
            training_progress: Mutex::new(None),
            training_stop_requested: Mutex::new(false),
            app_handle: Arc::new(Mutex::new(None)),
        }
    }

    pub(crate) fn set_app_handle(&self, app: AppHandle<Wry>) {
        if let Ok(mut current) = self.app_handle.lock() {
            *current = Some(app);
        }
    }

    pub(crate) fn emit_joint_range_states(&self, states: &[crate::types::JointRangeState]) {
        if let Ok(handle) = self.app_handle.lock()
            && let Some(app) = handle.as_ref()
        {
            let _ = app.emit("joint-range-state", states);
        }
    }

    pub(crate) fn load(config_dir: PathBuf) -> Self {
        Self::new(Some(config_dir.join("config")))
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
        let Some(dir) = &self.config_dir else {
            return Ok(());
        };
        let virtual_machines = self
            .virtual_machines
            .lock()
            .map_err(|error| error.to_string())?
            .clone();
        let physical_machines = self
            .physical_machines
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
        std::fs::create_dir_all(dir).map_err(|error| error.to_string())?;
        let default = virtual_machines
            .iter()
            .find(|machine| machine.startup_selected)
            .map(|machine| machine.id.clone());
        let machines = toml::to_string_pretty(&PersistedMachineTable {
            default,
            auto_mount,
            auto_reconnect,
            virtual_machines,
        })
        .map_err(|error| error.to_string())?;
        let robots = toml::to_string_pretty(&PersistedRobotTable {
            physical_machines: physical_machines.iter().map(Into::into).collect(),
        })
        .map_err(|error| error.to_string())?;
        std::fs::write(dir.join("machines.toml"), machines).map_err(|error| error.to_string())?;
        std::fs::write(dir.join("robots.toml"), robots).map_err(|error| error.to_string())
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
        *thread = Some(std::thread::spawn(move || {
            let period = Duration::from_millis(20);
            let mut deadline = std::time::Instant::now();
            while !stop.load(Ordering::Acquire) {
                if let Ok(mut guard) = runtime.lock()
                    && let Some(instance) = guard.as_mut()
                {
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
                                crate::logging::append_control_event(
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
                    crate::logging::append_control_event(
                        config_dir.as_deref(),
                        format!(
                            "runtime_tick runtime_generation={} execution_generation={} config_revision={} tick={} sources={:?} simulation={}",
                            status.runtime_generation,
                            status.execution_generation,
                            status.config_revision,
                            status.tick,
                            status
                                .read_sources
                                .iter()
                                .map(|source| (
                                    &source.id,
                                    source.generation,
                                    source.sequence,
                                    &source.health,
                                    source.age_ms
                                ))
                                .collect::<Vec<_>>(),
                            serde_json::to_string(&status.simulation)
                                .unwrap_or_else(|_| "null".into())
                        ),
                    );
                    if let Ok(handle) = app_handle.lock()
                        && let Some(handle) = handle.as_ref()
                    {
                        let _ = handle.emit("loop-status", status);
                    }
                }
                deadline += period;
                let now = std::time::Instant::now();
                if deadline > now {
                    std::thread::sleep(deadline - now);
                } else {
                    crate::logging::append_control_event(
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
    Simulation(PreviewMicroDuckPolicyRunner),
}

impl loop_body::RuntimeExecutor for RuntimeExecution {
    fn control_tick(&mut self, period: Duration) -> Result<(), String> {
        match self {
            Self::Physical(execution) => execution.tick(period).map_err(|e| e.to_string()),
            Self::Simulation(execution) => execution.tick().map(|_| ()),
        }
    }

    fn stop_execution(&mut self) -> Result<(), String> {
        match self {
            Self::Physical(execution) => execution
                .send(loop_body::LoopAction::Stop)
                .map_err(|e| e.to_string()),
            Self::Simulation(execution) => {
                execution.action(loop_body::LoopAction::Stop).map(|_| ())
            }
        }
    }
}

/// One common owner of the optional executor and independently sampled read sources.
pub(crate) struct RuntimeInstance {
    managed: loop_body::RuntimeLoop<RuntimeExecution, ReadOnlySource>,
}

impl RuntimeInstance {
    pub(crate) fn physical_execution(execution: MicroDuckPolicyRunner, id: String) -> Self {
        let control_source = if execution.has_policy() {
            loop_body::ControlSource::Controller
        } else {
            loop_body::ControlSource::Manual
        };
        Self {
            managed: loop_body::RuntimeLoop::from_sessions(
                loop_body::LoopRuntimeConfig {
                    execution_backend: Some(id),
                    control_source,
                    physical_access: loop_body::PhysicalAccess::Control,
                    ..Default::default()
                },
                Some(RuntimeExecution::Physical(execution)),
                Vec::new(),
            ),
        }
    }
    pub(crate) fn simulation_execution(
        execution: PreviewMicroDuckPolicyRunner,
        id: String,
    ) -> Self {
        Self {
            managed: loop_body::RuntimeLoop::from_sessions(
                loop_body::LoopRuntimeConfig {
                    execution_backend: Some(id),
                    control_source: loop_body::ControlSource::Manual,
                    ..Default::default()
                },
                Some(RuntimeExecution::Simulation(execution)),
                Vec::new(),
            ),
        }
    }
    pub(crate) fn read_only(source: ReadOnlySource) -> Self {
        let mut runtime = Self {
            managed: loop_body::RuntimeLoop::from_sessions(Default::default(), None, Vec::new()),
        };
        runtime
            .attach_source(source)
            .expect("validated source configuration");
        runtime
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
            Some(RuntimeExecution::Simulation(_))
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
    pub(crate) fn clock_tick(
        &mut self,
        config_dir: Option<&Path>,
    ) -> Result<crate::types::LoopStatus, String> {
        self.managed.advance()?;
        if let Some(RuntimeExecution::Physical(execution)) = self.managed.execution()
            && let Some(trace) = execution.policy_trace()
        {
            crate::logging::append_policy_trace(config_dir, execution.status().tick, trace);
        }
        Ok(self.snapshot())
    }
    pub(crate) fn stop_execution(&mut self) -> Result<(), String> {
        self.managed.stop_execution()
    }
    pub(crate) fn snapshot(&self) -> crate::types::LoopStatus {
        let mut status = match self.managed.execution() {
            Some(RuntimeExecution::Physical(execution)) => execution.status().into(),
            Some(RuntimeExecution::Simulation(execution)) => execution.status(),
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
        matches!(self.managed.execution(), Some(RuntimeExecution::Physical(execution)) if execution.has_policy())
    }
    pub(crate) fn physical(&self) -> Result<&MicroDuckPolicyRunner, String> {
        match self.managed.execution() {
            Some(RuntimeExecution::Physical(execution)) => Ok(execution),
            _ => Err("active runtime has no physical execution".into()),
        }
    }
    pub(crate) fn physical_mut(&mut self) -> Result<&mut MicroDuckPolicyRunner, String> {
        match self.managed.execution_mut() {
            Some(RuntimeExecution::Physical(execution)) => Ok(execution),
            _ => Err("active runtime has no physical execution".into()),
        }
    }
    pub(crate) fn simulation_mut(&mut self) -> Result<&mut PreviewMicroDuckPolicyRunner, String> {
        match self.managed.execution_mut() {
            Some(RuntimeExecution::Simulation(execution)) => Ok(execution),
            _ => Err("active runtime has no simulation execution".into()),
        }
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
    }
}

#[cfg(test)]
mod tests {
    use super::DesktopState;
    use crate::types::{PhysicalMachine, ReconnectHint, ServoIdentity, ServoJointDirection};

    fn servo(id: u8, model: &str) -> ServoIdentity {
        ServoIdentity {
            servo_id: id,
            model: model.into(),
        }
    }

    #[test]
    fn system_config_round_trips_physical_weak_reference() {
        let config_dir = std::env::temp_dir().join(format!(
            "microduck-vision-system-config-{}",
            std::process::id()
        ));
        let _ = std::fs::remove_dir_all(&config_dir);
        let state = DesktopState::new(Some(config_dir.clone()));
        state
            .physical_machines
            .lock()
            .unwrap()
            .push(PhysicalMachine {
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
                virtual_machine_id: Some("official-hd1910".into()),
                online_servo_ids: vec![1],
                connected: true,
                last_seen: Some("刚刚".into()),
                torque_enabled: false,
                joint_directions: vec![ServoJointDirection {
                    slot_id: "left_hip_yaw".into(),
                    direction: -1,
                }],
            });
        state.virtual_machines.lock().unwrap()[0]
            .simulation
            .world_position = [1.25, -0.4, 2.0];
        state.virtual_machines.lock().unwrap()[0]
            .joint_bindings
            .push(crate::types::ServoJointBinding {
                slot_id: "left_hip_yaw".into(),
                physical_id: 1,
                direction: -1,
                zero_offset_rad: -0.35,
                min_rad: None,
                max_rad: None,
            });
        state.persist().unwrap();

        let loaded = DesktopState::new(Some(config_dir.clone()));
        let physical = &loaded.physical_machines.lock().unwrap()[0];
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
            Some("official-hd1910")
        );
        assert_eq!(physical.joint_directions[0].direction, -1);
        assert_eq!(
            loaded.virtual_machines.lock().unwrap()[0].referenced_by,
            vec!["physical-1"]
        );
        assert_eq!(
            loaded.virtual_machines.lock().unwrap()[0]
                .simulation
                .world_position,
            [1.25, -0.4, 2.0]
        );
        assert_eq!(
            loaded.virtual_machines.lock().unwrap()[0].joint_bindings[0].zero_offset_rad,
            -0.35
        );
        assert!(config_dir.join("machines.toml").is_file());
        let machines_text = std::fs::read_to_string(config_dir.join("machines.toml")).unwrap();
        assert!(machines_text.contains("auto_mount = true"));
        assert!(machines_text.contains("auto_reconnect = true"));
        let robots_text = std::fs::read_to_string(config_dir.join("robots.toml")).unwrap();
        assert!(robots_text.contains("reconnect_hint"));
        assert!(robots_text.contains("joint_directions"));
        assert!(!robots_text.contains("machine_code"));
        assert!(!robots_text.contains("connected"));
        let _ = std::fs::remove_dir_all(config_dir);
    }
}
