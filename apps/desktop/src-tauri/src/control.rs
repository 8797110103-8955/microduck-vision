//! Runtime control orchestration.
//!
//! This module owns control-loop ticks and lifecycle state transitions. It may depend on the
//! shared `loop-body` contract, backend adapters, state, and logging, but it must not depend on
//! Tauri command registration or frontend DTO construction beyond the final response mapping.

use std::path::PathBuf;
use std::sync::{Arc, Mutex};

use crate::backend::policy_runtime::{MicroDuckPolicyRunner, PolicyConfig, PolicyDescriptor};
use duck_control::obs::{BodyPose, Command};
#[cfg(test)]
use loop_body::{BackendState, ReadOnlySource};
use loop_body::{ControlBackend, LoopBinding, LoopConfig};

use crate::backend::control::ConnectedBus;
use crate::backend::physical_backend::PhysicalBackend;
#[cfg(test)]
use crate::logging::append_policy_trace;
#[cfg(test)]
use crate::logging::saturation_reason_for_error;
use crate::logging::{append_control_event, log_joint_range_states};
use crate::state::DesktopState;
use crate::types::LoopStatus as FrontendLoopStatus;
#[cfg(test)]
use crate::types::WorldState;

#[derive(Debug, Clone, Copy)]
pub(crate) struct PolicyCommandInput {
    pub twist: [f64; 3],
    pub head: [f64; 4],
    pub body: [f64; 3],
}

pub(crate) struct OpenLoopInput {
    pub binding: LoopBinding,
    pub driver: Box<dyn ControlBackend>,
    pub policy: Option<(PathBuf, PolicyConfig, PolicyDescriptor)>,
    pub config_dir: Option<PathBuf>,
    pub backend_kind: &'static str,
    pub backend_profile: String,
    pub contract_hash: String,
}

/// Result of selecting the execution backend for a loop.  The command layer supplies only the
/// resolved machine mapping; serial ownership and the read-only shared-driver decision stay here.
pub(crate) struct OpenBackendDriver {
    pub driver: Box<dyn ControlBackend>,
    pub shared_bus: Option<Arc<Mutex<ConnectedBus>>>,
}

#[derive(Debug, Clone, Copy, PartialEq)]
struct RuntimeTiming {
    physics_dt: f64,
    physics_substeps: usize,
    delay_timebase: &'static str,
    delay_steps: &'static str,
    decimation: usize,
}

fn runtime_timing(backend_kind: &str, control_dt: f64) -> RuntimeTiming {
    if backend_kind == "simulation" {
        // The preview driver advances four 5ms physics steps per 20ms policy tick and owns the
        // vendor's [3, 6] physics-step actuator delay queue.
        RuntimeTiming {
            physics_dt: 0.005,
            physics_substeps: 4,
            delay_timebase: "physics_step",
            delay_steps: "3..6",
            decimation: 4,
        }
    } else {
        // A physical servo firmware owns its internal timing and delay. The host only observes
        // the result, so there is no host-side physics substep to report.
        RuntimeTiming {
            physics_dt: control_dt,
            physics_substeps: 1,
            delay_timebase: "transport",
            delay_steps: "0",
            decimation: 1,
        }
    }
}

pub(crate) fn open_backend_driver(
    state: &DesktopState,
    physical_id: &str,
    model: servo::ServoModel,
    access: loop_body::BackendAccess,
    active_ids: Vec<Option<u8>>,
    contract: &str,
) -> Result<OpenBackendDriver, String> {
    let mut registry = state
        .backend_registry
        .lock()
        .map_err(|error| error.to_string())?;
    let bus = state
        .connected_buses
        .lock()
        .map_err(|error| error.to_string())?
        .remove(physical_id)
        .ok_or_else(|| "physical machine has no connected bus".to_owned())?;
    let shared = Arc::new(Mutex::new(bus));
    let weak_bus = Arc::downgrade(&shared);
    let opened = (|| {
        if registry.describe(physical_id).is_some() {
            if registry
                .describe(physical_id)
                .is_some_and(|descriptor| descriptor.kind != loop_body::BackendKind::Physical)
            {
                return Err("backend ID is already registered with another kind".into());
            }
            registry
                .unregister(physical_id)
                .map_err(|error| error.to_string())?;
        }
        registry
            .register(
                loop_body::BackendDescriptor {
                    id: physical_id.to_owned(),
                    kind: loop_body::BackendKind::Physical,
                    supports_control: true,
                    contract: contract.to_owned(),
                },
                move || {
                    let bus = weak_bus.upgrade().ok_or_else(|| {
                        servo::ServoError::Bus("physical connection closed".into())
                    })?;
                    Ok(Box::new(PhysicalBackend::new(
                        bus,
                        model,
                        active_ids.clone(),
                    )))
                },
            )
            .map_err(|error| error.to_string())?;
        if access == loop_body::BackendAccess::Control {
            registry
                .select(physical_id)
                .map_err(|error| error.to_string())?;
        }
        registry
            .open(physical_id, access)
            .map_err(|error| error.to_string())
    })();
    match opened {
        Ok(driver) => {
            if access == loop_body::BackendAccess::ReadOnly {
                state
                    .execution_gate
                    .set_physical_writes_enabled(false)
                    .map_err(|error| error.to_string())?;
            } else {
                state
                    .execution_gate
                    .set_physical_writes_enabled(true)
                    .map_err(|error| error.to_string())?;
            }
            append_control_event(
                state.config_dir.as_deref(),
                format!(
                    "backend_session backend_id={} backend_kind=physical access={access:?} generation={} contract_hash={contract}",
                    physical_id,
                    driver.generation()
                ),
            );
            Ok(OpenBackendDriver {
                driver: Box::new(driver),
                shared_bus: Some(shared),
            })
        }
        Err(error) => {
            if let Ok(shared) = Arc::try_unwrap(shared)
                && let Ok(bus) = shared.into_inner()
            {
                state
                    .connected_buses
                    .lock()
                    .map_err(|error| error.to_string())?
                    .insert(physical_id.to_owned(), bus);
            }
            Err(error)
        }
    }
}

/// Advance one control tick. A loop without a policy is deliberately limited to sensor reads;
/// this is the read-only debug path used while a physical machine is inspected.
#[cfg(test)]
pub(crate) fn tick(state: &DesktopState) -> Result<FrontendLoopStatus, String> {
    let mut guard = state.runtime_loop.lock().map_err(|e| e.to_string())?;
    let loop_state = guard.as_mut().ok_or_else(|| "loop is closed".to_string())?;
    let physical_status = if let Some(source) = loop_state.read_source_mut() {
        let sample = match source.read() {
            Ok(sample) => sample,
            Err(error) => {
                append_control_event(
                    state.config_dir.as_deref(),
                    format!("read_source_error source=physical error={error}"),
                );
                source.latest().cloned().ok_or_else(|| error.to_string())?
            }
        };
        return read_only_status(sample, source, state);
    } else {
        let loop_state = loop_state.physical_mut()?;
        if let Err(error) = loop_state.tick(LoopConfig::default().period) {
            append_control_event(
                state.config_dir.as_deref(),
                format!(
                    "saturation_reason={} error={}",
                    saturation_reason_for_error(&error.to_string()),
                    error
                ),
            );
            return Err(error.to_string());
        }
        if let Some(trace) = loop_state.policy_trace() {
            append_policy_trace(state.config_dir.as_deref(), loop_state.status().tick, trace);
        }
        loop_state.status()
    };
    let world: WorldState = state.motion.lock().map_err(|e| e.to_string())?.world();
    let mut status = FrontendLoopStatus::from(physical_status);
    decorate_status(state, &mut status);
    status.world = world;
    state.emit_joint_range_states(&status.joint_range_states);
    log_joint_range_states(state.config_dir.as_deref(), &status.joint_range_states);
    Ok(status)
}

#[cfg(test)]
fn read_only_status(
    sample: BackendState,
    source: &ReadOnlySource,
    app_state: &DesktopState,
) -> Result<FrontendLoopStatus, String> {
    let mut status = FrontendLoopStatus {
        has_execution: false,
        runtime_state: crate::types::RuntimeState::Running,
        simulation: None,
        read_sources: vec![source.into()],
        phase: crate::state::types::LoopPhase::Holding,
        session: 0,
        tick: sample.sequence,
        torque: false,
        positions: sample.positions,
        targets: vec![0.0; sample.velocities.len()],
        world: app_state.motion.lock().map_err(|e| e.to_string())?.world(),
        sensor_failures: 0,
        last_error: source.last_error().map(str::to_owned),
        joint_range_states: Vec::new(),
        runtime_generation: app_state.current_runtime_generation(),
        execution_generation: 0,
        config_revision: app_state.current_config_revision(),
    };
    decorate_status(app_state, &mut status);
    Ok(status)
}

pub(crate) fn ensure_backend_available(
    state: &DesktopState,
    requested_kind: &str,
) -> Result<(), String> {
    ensure_backend_access(state, requested_kind, true)
}

pub(crate) fn ensure_backend_access(
    state: &DesktopState,
    requested_kind: &str,
    control_requested: bool,
) -> Result<(), String> {
    state.ensure_runtime_healthy()?;
    let active_kind = state
        .loop_backend_kind
        .lock()
        .map_err(|error| error.to_string())?;
    let readonly_physical = requested_kind == "simulation"
        && active_kind.as_deref() == Some("physical")
        && state
            .runtime_loop
            .lock()
            .map_err(|error| error.to_string())?
            .as_ref()
            .is_some_and(|runtime| !runtime.has_execution());
    let attaching_read_source = requested_kind == "physical"
        && !control_requested
        && state
            .runtime_loop
            .lock()
            .map_err(|error| error.to_string())?
            .as_ref()
            .is_some_and(crate::state::RuntimeInstance::is_simulation);
    if attaching_read_source {
        return Ok(());
    }
    if let Some(active_kind) = active_kind.as_deref()
        && !readonly_physical
    {
        return Err(format!(
            "cannot open {requested_kind} backend while {active_kind} loop is active; close it first"
        ));
    }
    if !readonly_physical
        && state
            .runtime_loop
            .lock()
            .map_err(|error| error.to_string())?
            .is_some()
    {
        return Err("cannot open a new backend while a loop is active; close it first".into());
    }
    if control_requested
        && !readonly_physical
        && state
            .runtime_loop
            .lock()
            .map_err(|error| error.to_string())?
            .is_some()
    {
        return Err(
            "cannot open a backend while the preview loop is active; close it first".into(),
        );
    }
    Ok(())
}

pub(crate) fn record_backend_open(state: &DesktopState, backend_kind: &str) -> Result<(), String> {
    *state
        .loop_backend_kind
        .lock()
        .map_err(|error| error.to_string())? = Some(backend_kind.to_owned());
    Ok(())
}

pub(crate) fn open_loop(input: OpenLoopInput) -> Result<MicroDuckPolicyRunner, String> {
    let mut loop_state =
        MicroDuckPolicyRunner::open(input.binding, input.driver, LoopConfig::default())
            .map_err(|error| error.to_string())?;
    if let Some((path, config, descriptor)) = input.policy.as_ref() {
        append_control_event(
            input.config_dir.as_deref(),
            format!("policy_selected path={path:?} config={config:?}"),
        );
        loop_state
            .select_policy_with_descriptor(path, descriptor, *config)
            .map_err(|error| {
                append_control_event(
                    input.config_dir.as_deref(),
                    format!("policy_load_error path={path:?} error={error}"),
                );
                error.to_string()
            })?;
    } else {
        append_control_event(input.config_dir.as_deref(), "policy_selected none");
    }
    let runtime_config = input
        .policy
        .as_ref()
        .map(|(_, config, _)| *config)
        .unwrap_or_default();
    let control_dt = LoopConfig::default().period.as_secs_f64();
    let timing = runtime_timing(input.backend_kind, control_dt);
    append_control_event(
        input.config_dir.as_deref(),
        format!(
            "policy_runtime backend_kind={} backend_profile={} contract_hash={} config={runtime_config:?} observation_dim=61 action_dim=14 control_dt={control_dt} physics_dt={} physics_substeps={} decimation={} delay_timebase={} delay_steps={} observation_delay_ticks=1 imu_source=backend_or_default gain={}",
            input.backend_kind,
            input.backend_profile,
            input.contract_hash,
            timing.physics_dt,
            timing.physics_substeps,
            timing.decimation,
            timing.delay_timebase,
            timing.delay_steps,
            LoopConfig::default().gain
        ),
    );
    Ok(loop_state)
}

pub(crate) fn status(state: &DesktopState) -> Result<Option<FrontendLoopStatus>, String> {
    let runtime = state.runtime_loop.lock().map_err(|e| e.to_string())?;
    let mut status = runtime
        .as_ref()
        .map(crate::state::RuntimeInstance::snapshot);
    if status.is_none()
        && let Some(error) = state
            .runtime_fault
            .lock()
            .map_err(|e| e.to_string())?
            .clone()
    {
        status = Some(FrontendLoopStatus {
            has_execution: false,
            runtime_state: crate::types::RuntimeState::Faulted,
            simulation: None,
            phase: crate::types::LoopPhase::Stopped,
            session: 0,
            tick: 0,
            torque: false,
            positions: Vec::new(),
            targets: Vec::new(),
            world: state.motion.lock().map_err(|e| e.to_string())?.world(),
            sensor_failures: 0,
            last_error: Some(error),
            joint_range_states: Vec::new(),
            runtime_generation: state.current_runtime_generation(),
            execution_generation: state.current_execution_generation(),
            config_revision: state.current_config_revision(),
            read_sources: Vec::new(),
        });
    }
    if let Some(status) = &mut status {
        status.read_sources = runtime
            .as_ref()
            .and_then(|runtime| runtime.read_source())
            .map(|source| vec![source.into()])
            .unwrap_or_default();
        decorate_status(state, status);
        if !runtime
            .as_ref()
            .is_some_and(crate::state::RuntimeInstance::is_simulation)
        {
            status.world = state.motion.lock().map_err(|e| e.to_string())?.world();
        }
        state.emit_joint_range_states(&status.joint_range_states);
    }
    Ok(status)
}

pub(crate) fn set_targets(state: &DesktopState, targets: Vec<f64>) -> Result<(), String> {
    let mut guard = state.runtime_loop.lock().map_err(|e| e.to_string())?;
    let loop_state = guard.as_mut().ok_or_else(|| "loop is closed".to_string())?;
    if loop_state.has_policy() {
        return Err(
            "policy owns the target source; stop the policy before submitting manual targets"
                .to_owned(),
        );
    }
    let session = loop_state.physical()?.status().session;
    append_control_event(
        state.config_dir.as_deref(),
        format!("loop_command session={} body_targets={targets:?}", session),
    );
    loop_state
        .physical_mut()?
        .set_command(loop_body::BodyCommand { session, targets })
        .map_err(|e| e.to_string())
}

pub(crate) fn set_policy_command(
    state: &DesktopState,
    input: PolicyCommandInput,
) -> Result<(), String> {
    let mut guard = state.runtime_loop.lock().map_err(|e| e.to_string())?;
    let loop_state = guard.as_mut().ok_or_else(|| "loop is closed".to_string())?;
    if !loop_state.has_policy() {
        return Err("debug loop is read-only".to_owned());
    }
    loop_state.physical_mut()?.set_policy_command(Command {
        twist: input.twist,
        head: input.head,
        body: BodyPose {
            z: input.body[0],
            roll: input.body[1],
            pitch: input.body[2],
        },
    });
    Ok(())
}

pub(crate) fn action(
    state: &DesktopState,
    action: loop_body::LoopAction,
) -> Result<FrontendLoopStatus, String> {
    let mut guard = state.runtime_loop.lock().map_err(|e| e.to_string())?;
    let loop_state = guard.as_mut().ok_or_else(|| "loop is closed".to_string())?;
    if loop_state.physical().is_err() {
        return Err("debug loop is read-only".to_owned());
    }
    append_control_event(
        state.config_dir.as_deref(),
        format!("loop_action action={action:?}"),
    );
    loop_state
        .physical_mut()?
        .send(action)
        .map_err(|e| e.to_string())?;
    let mut status = FrontendLoopStatus::from(loop_state.physical()?.status());
    decorate_status(state, &mut status);
    status.world = state.motion.lock().map_err(|e| e.to_string())?.world();
    state.emit_joint_range_states(&status.joint_range_states);
    log_joint_range_states(state.config_dir.as_deref(), &status.joint_range_states);
    Ok(status)
}

fn decorate_status(state: &DesktopState, status: &mut FrontendLoopStatus) {
    status.runtime_generation = state.current_runtime_generation();
    status.config_revision = state.current_config_revision();
    status.execution_generation = state.current_execution_generation();
    if let Some(error) = state
        .runtime_fault
        .lock()
        .ok()
        .and_then(|fault| fault.clone())
    {
        status.runtime_state = crate::types::RuntimeState::Faulted;
        status.last_error = Some(error);
    }
}

pub(crate) fn restore_shared_physical_bus(
    state: &DesktopState,
    physical_id: &str,
) -> Result<(), String> {
    let shared: Option<Arc<Mutex<ConnectedBus>>> = state
        .loop_physical_bus
        .lock()
        .map_err(|error| error.to_string())?
        .take();
    let Some(shared) = shared else { return Ok(()) };
    let bus = Arc::try_unwrap(shared)
        .map_err(|_| "debug physical bus is still in use".to_owned())?
        .into_inner()
        .map_err(|_| "physical bus lock poisoned".to_owned())?;
    state
        .connected_buses
        .lock()
        .map_err(|error| error.to_string())?
        .insert(physical_id.to_owned(), bus);
    Ok(())
}

pub(crate) fn close(state: &DesktopState) -> Result<(), String> {
    let _transition = state
        .execution_transition
        .lock()
        .map_err(|e| e.to_string())?;
    close_locked(state)
}

pub(crate) fn close_locked(state: &DesktopState) -> Result<(), String> {
    append_control_event(state.config_dir.as_deref(), "loop_close");
    state.stop_loop_clock()?;
    let mut guard = state.runtime_loop.lock().map_err(|e| e.to_string())?;
    let source_only = guard
        .as_ref()
        .is_some_and(|runtime| runtime.is_simulation() && runtime.read_source().is_some());
    if let Some(source) = guard.as_mut().and_then(|runtime| runtime.read_source_mut())
        && let Err(error) = source.stop()
    {
        drop(guard);
        if source_only {
            state.start_loop_clock()?;
        }
        return Err(error);
    }
    let stop_result = if !source_only && let Some(loop_state) = guard.as_mut() {
        loop_state.stop_execution()
    } else {
        Ok(())
    };
    if let Err(error) = stop_result {
        // Keep both the execution lease and its transport so close can be retried. Releasing
        // ownership here could let a second controller run while torque-off is unconfirmed.
        *state
            .runtime_fault
            .lock()
            .map_err(|error| error.to_string())? = Some(error.clone());
        state
            .execution_gate
            .set_physical_writes_enabled(false)
            .map_err(|error| error.to_string())?;
        append_control_event(
            state.config_dir.as_deref(),
            format!("runtime_faulted stage=stop error={error}"),
        );
        return Err(error);
    }
    if source_only {
        if let Some(instance) = guard.as_mut() {
            instance.take_source();
        }
    } else {
        drop(guard.take());
    }
    drop(guard);
    let physical_id = state
        .loop_physical_machine_id
        .lock()
        .map_err(|e| e.to_string())?
        .take();
    let mut restore_result = Ok(());
    if let Some(physical_id) = physical_id {
        let result = restore_shared_physical_bus(state, &physical_id);
        if let Err(error) = result {
            append_control_event(
                state.config_dir.as_deref(),
                format!("loop_bus_restore_error physical_machine_id={physical_id} error={error}"),
            );
            restore_result = Err(error);
        }
    }
    *state.loop_backend_kind.lock().map_err(|e| e.to_string())? = None;
    state
        .execution_gate
        .set_physical_writes_enabled(false)
        .map_err(|error| error.to_string())?;
    *state
        .runtime_fault
        .lock()
        .map_err(|error| error.to_string())? = None;
    state.motion.lock().map_err(|e| e.to_string())?.reset();
    state.emit_joint_range_states(&[]);
    if source_only {
        state.start_loop_clock()?;
    }
    restore_result
}

/// A disappearing connection revokes its sessions before releasing transport ownership. It
/// never replays torque-off or reconnects a controller: a new connection needs a new session.
pub(crate) fn detach_physical_connection(state: &DesktopState, id: &str) -> Result<(), String> {
    let _transition = state
        .execution_transition
        .lock()
        .map_err(|error| error.to_string())?;
    if state
        .loop_physical_machine_id
        .lock()
        .map_err(|error| error.to_string())?
        .as_deref()
        == Some(id)
    {
        let mut runtime = state
            .runtime_loop
            .lock()
            .map_err(|error| error.to_string())?;
        if let Some(source) = runtime
            .as_mut()
            .and_then(|runtime| runtime.read_source_mut())
        {
            source.stop()?;
        }
    }
    {
        let mut registry = state
            .backend_registry
            .lock()
            .map_err(|error| error.to_string())?;
        if registry.describe(id).is_some() {
            registry.unregister(id).map_err(|error| error.to_string())?;
        }
    }
    let mut physical_id = state
        .loop_physical_machine_id
        .lock()
        .map_err(|error| error.to_string())?;
    if physical_id.as_deref() == Some(id) {
        let mut runtime = state
            .runtime_loop
            .lock()
            .map_err(|error| error.to_string())?;
        if runtime
            .as_ref()
            .is_some_and(crate::state::RuntimeInstance::is_simulation)
        {
            if let Some(instance) = runtime.as_mut() {
                instance.take_source();
            }
        } else {
            runtime.take();
        }
        state
            .loop_physical_bus
            .lock()
            .map_err(|error| error.to_string())?
            .take();
        *state
            .loop_backend_kind
            .lock()
            .map_err(|error| error.to_string())? = None;
        *physical_id = None;
    }
    Ok(())
}

#[cfg(test)]
fn should_read_sensors_only(has_policy: bool) -> bool {
    !has_policy
}

#[cfg(test)]
pub(crate) mod tests {
    use std::sync::{Arc, Mutex};
    use std::time::Duration;

    use duck_body::BodyTemplate;
    use loop_body::{
        BodyRuntimeView, Calibration, JointCalibration, LoopBinding, LoopConfig, ReadOnlySource,
        ServoBinding,
    };
    use servo::{
        ServoError, ServoLoopDriver, ServoModel, ServoPositionSample, ServoPositionTargets,
    };

    use super::{RuntimeTiming, ensure_backend_available, record_backend_open, runtime_timing};
    use crate::state::DesktopState;

    pub(crate) struct ReadOnlyDriver {
        pub(crate) positions: Vec<f64>,
        pub(crate) events: Arc<Mutex<Vec<&'static str>>>,
    }

    impl ServoLoopDriver for ReadOnlyDriver {
        fn read_positions(&mut self, _ids: &[u8]) -> Result<ServoPositionSample, ServoError> {
            self.events.lock().unwrap().push("read");
            Ok(ServoPositionSample {
                positions: self.positions.clone(),
                velocities: vec![0.0; self.positions.len()],
            })
        }

        fn write_positions(
            &mut self,
            _ids: &[u8],
            _targets: &ServoPositionTargets,
        ) -> Result<(), ServoError> {
            self.events.lock().unwrap().push("write");
            Ok(())
        }

        fn set_torque(&mut self, _enabled: bool) -> Result<(), ServoError> {
            self.events.lock().unwrap().push("torque");
            Ok(())
        }

        fn set_gain(&mut self, _gain: u16) -> Result<(), ServoError> {
            self.events.lock().unwrap().push("gain");
            Ok(())
        }

        fn reboot(&mut self, _id: u8) -> Result<(), ServoError> {
            Ok(())
        }
    }

    fn read_only_loop(events: Arc<Mutex<Vec<&'static str>>>) -> ReadOnlySource {
        read_only_loop_with_driver(Box::new(ReadOnlyDriver {
            positions: BodyTemplate::official().unwrap().control.home_pose,
            events,
        }))
    }

    pub(crate) fn read_only_loop_with_driver(
        driver: Box<dyn loop_body::ControlBackend>,
    ) -> ReadOnlySource {
        let body = BodyTemplate::official().unwrap();
        let slots: Vec<_> = body.actuators.iter().map(|slot| slot.id.clone()).collect();
        let binding = LoopBinding {
            body: BodyRuntimeView::new(body.clone()).unwrap(),
            servos: ServoBinding {
                model: ServoModel::Xl330M288,
                slot_ids: slots.clone(),
            },
            calibration: Calibration {
                body_id: body.id,
                body_version: body.version,
                joints: slots
                    .into_iter()
                    .enumerate()
                    .map(|(index, slot_id)| JointCalibration {
                        slot_id,
                        physical_id: (index + 1) as u8,
                        direction: 1,
                        zero_offset_rad: 0.0,
                        min_rad: None,
                        max_rad: None,
                    })
                    .collect(),
            },
        };
        ReadOnlySource::open(binding, driver)
    }

    #[test]
    fn loop_without_policy_is_sensor_only() {
        assert!(super::should_read_sensors_only(false));
        assert!(!super::should_read_sensors_only(true));
    }

    #[test]
    fn runtime_log_timing_matches_the_selected_backend() {
        assert_eq!(
            runtime_timing("simulation", 0.02),
            RuntimeTiming {
                physics_dt: 0.005,
                physics_substeps: 4,
                delay_timebase: "physics_step",
                delay_steps: "3..6",
                decimation: 4,
            }
        );
        assert_eq!(
            runtime_timing("physical", 0.02),
            RuntimeTiming {
                physics_dt: 0.02,
                physics_substeps: 1,
                delay_timebase: "transport",
                delay_steps: "0",
                decimation: 1,
            }
        );
    }

    #[test]
    fn debug_tick_reads_physical_state_without_policy_or_actuator_writes() {
        let state = DesktopState::default();
        let events = Arc::new(Mutex::new(Vec::new()));
        *state.runtime_loop.lock().unwrap() = Some(crate::state::RuntimeInstance::read_only(
            read_only_loop(Arc::clone(&events)),
        ));

        let status = super::tick(&state).unwrap();

        assert_eq!(status.tick, 1);
        assert!(!status.torque);
        assert_eq!(*events.lock().unwrap(), vec!["read"]);
        assert!(
            !state
                .runtime_loop
                .lock()
                .unwrap()
                .as_ref()
                .unwrap()
                .has_policy()
        );
        assert_eq!(LoopConfig::default().period, Duration::from_millis(20));
    }

    #[test]
    fn backend_selection_is_latched_until_loop_close() {
        let state = DesktopState::default();
        ensure_backend_available(&state, "physical").unwrap();
        record_backend_open(&state, "physical").unwrap();
        let error = ensure_backend_available(&state, "simulation").unwrap_err();
        assert!(error.contains("physical"));
    }
}
