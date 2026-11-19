//! Application service for opening a control loop.
//!
//! This module owns machine/policy resolution, physical mapping, backend construction, and
//! control-loop lifecycle. Tauri commands only adapt the frontend input and output.

use std::collections::{HashMap, HashSet};
use std::sync::Arc;

use duck_body::BodyTemplate;
use loop_body::{
    BodyRuntimeView, Calibration, ControlBackend, JointCalibration, LoopBinding, ReadOnlySource,
    ServoBinding,
};

use crate::application::policy;
use crate::backend::contract::ResolvedContract;
use crate::control;
use crate::helpers::model;
use crate::logging::{
    LoggedServoDriver, append_control_event, begin_control_loop, log_joint_range_states,
};
use crate::state::DesktopState;
use crate::types::{
    LoopOpenInput, LoopStatus as FrontendLoopStatus, PhysicalMachine, ServoIdentity,
    ServoJointBinding,
};

pub(crate) fn start(
    state: &DesktopState,
    input: crate::types::RuntimeOpenInput,
) -> Result<FrontendLoopStatus, String> {
    let _transition = state
        .execution_transition
        .lock()
        .map_err(|e| e.to_string())?;
    start_locked(state, input)
}

fn start_locked(
    state: &DesktopState,
    input: crate::types::RuntimeOpenInput,
) -> Result<FrontendLoopStatus, String> {
    if matches!(input.access, crate::types::LoopAccess::ReadOnly)
        && input.policy_id.is_none()
        && let Some(id) = input.physical_machine_id.as_deref()
        && state
            .loop_physical_machine_id
            .lock()
            .map_err(|error| error.to_string())?
            .as_deref()
            == Some(id)
    {
        return control::status(state)?.ok_or_else(|| "physical source is unavailable".into());
    }
    match input.physical_machine_id {
        Some(physical_machine_id) => open_locked(
            state,
            LoopOpenInput {
                virtual_machine_id: input.virtual_machine_id,
                physical_machine_id,
                access: input.access,
                policy_id: input.policy_id,
            },
        ),
        None => {
            if !matches!(input.access, crate::types::LoopAccess::ReadOnly) {
                return Err("simulation runtime only supports read-only physical access".into());
            }
            if input.policy_id.is_some() {
                return Err("simulation runtime policy is selected by a runtime command".into());
            }
            super::preview::open_locked(state, input.virtual_machine_id)
        }
    }
}

/// Replace execution under one transition lock. Failed shutdown keeps the old owner; failed
/// loading leaves an explicit fault and never restores the previous controller or permissions.
pub(crate) fn reconfigure(
    state: &DesktopState,
    generation: u64,
    config_revision: u64,
    input: crate::types::RuntimeOpenInput,
) -> Result<FrontendLoopStatus, String> {
    let _transition = state
        .execution_transition
        .lock()
        .map_err(|e| e.to_string())?;
    state.validate_runtime_generation(Some(generation))?;
    state.ensure_runtime_healthy()?;
    if state.current_config_revision() != config_revision {
        return Err("stale runtime config revision".into());
    }
    close_locked(state, crate::types::RuntimeCloseScope::Runtime)?;
    match start_locked(state, input) {
        Ok(status) => Ok(status),
        Err(error) => {
            state
                .execution_gate
                .set_physical_writes_enabled(false)
                .map_err(|e| e.to_string())?;
            *state.runtime_fault.lock().map_err(|e| e.to_string())? = Some(error.clone());
            append_control_event(
                state.config_dir.as_deref(),
                format!("runtime_faulted stage=load error={error}"),
            );
            Err(error)
        }
    }
}

/// Serialize frontend mutations with start/stop before validating their runtime ownership.
/// The transition lock remains held through the operation, preventing a stale request from
/// being reinterpreted after a concurrent execution replacement.
pub(crate) fn command(
    state: &DesktopState,
    generation: u64,
    execution_generation: u64,
    sequence: u64,
    request: crate::types::RuntimeCommand,
) -> Result<FrontendLoopStatus, String> {
    let _transition = state
        .execution_transition
        .lock()
        .map_err(|e| e.to_string())?;
    state.validate_runtime_generation(Some(generation))?;
    state.ensure_runtime_healthy()?;
    let simulation = {
        let runtime = state.runtime_loop.lock().map_err(|e| e.to_string())?;
        let current_execution = state.current_execution_generation();
        if execution_generation != current_execution {
            return Err(format!(
                "stale execution generation {execution_generation}, current is {current_execution}"
            ));
        }
        let runtime = runtime.as_ref().ok_or("runtime is closed")?;
        if !runtime.has_execution() {
            return Err("runtime is read-only".into());
        }
        runtime.is_simulation()
    };
    let previous = state
        .command_sequence
        .load(std::sync::atomic::Ordering::Acquire);
    if sequence <= previous {
        return Err(format!(
            "stale command sequence {sequence}, latest is {previous}"
        ));
    }
    state
        .command_sequence
        .store(sequence, std::sync::atomic::Ordering::Release);
    use crate::types::RuntimeCommand;
    match request {
        RuntimeCommand::Input { input } if simulation => super::preview::set_input(state, input),
        RuntimeCommand::Input { input } => {
            control::set_policy_command(
                state,
                control::PolicyCommandInput {
                    twist: input.twist,
                    head: input.head,
                    body: input.body,
                },
            )?;
            control::status(state)?.ok_or_else(|| "runtime is closed".into())
        }
        RuntimeCommand::Targets { targets } if !simulation => {
            control::set_targets(state, targets)?;
            control::status(state)?.ok_or_else(|| "runtime is closed".into())
        }
        RuntimeCommand::Policy { policy_id } if simulation => {
            super::preview::select_policy(state, policy_id)
        }
        RuntimeCommand::ResetWorld if simulation => super::preview::reset_world(state),
        RuntimeCommand::Action { action } => {
            let action = match action {
                crate::types::LoopAction::Power { on } => loop_body::LoopAction::Power(on),
                crate::types::LoopAction::Drive { on } => loop_body::LoopAction::Drive(on),
                crate::types::LoopAction::Stop => loop_body::LoopAction::Stop,
                crate::types::LoopAction::Estop => loop_body::LoopAction::Estop,
                crate::types::LoopAction::Reconnect { session } => {
                    loop_body::LoopAction::Reconnect { session }
                }
            };
            if simulation {
                super::preview::action(state, action)
            } else {
                control::action(state, action)
            }
        }
        _ => Err("command is unsupported by the active execution session".into()),
    }
}

/// Close whichever runtime is active while preserving the same generation and transition
/// boundary for physical, simulation, combined, and read-only sessions.
pub(crate) fn close(
    state: &DesktopState,
    generation: u64,
    scope: crate::types::RuntimeCloseScope,
) -> Result<(), String> {
    let _transition = state
        .execution_transition
        .lock()
        .map_err(|e| e.to_string())?;
    state.validate_runtime_generation(Some(generation))?;
    close_locked(state, scope)
}

fn close_locked(
    state: &DesktopState,
    scope: crate::types::RuntimeCloseScope,
) -> Result<(), String> {
    if matches!(scope, crate::types::RuntimeCloseScope::ReadOnlySource) {
        let has_source = state
            .runtime_loop
            .lock()
            .map_err(|e| e.to_string())?
            .as_ref()
            .is_some_and(|runtime| runtime.read_source().is_some());
        if !has_source {
            return Ok(());
        }
        let result = control::close_locked(state);
        state
            .config_revision
            .fetch_add(1, std::sync::atomic::Ordering::AcqRel);
        return result;
    }
    // Release any physical attachment first, then stop the remaining execution. Sensor
    // detachment may preserve simulation for debug consumers, but runtime close never does.
    control::close_locked(state)?;
    // A first pass may detach a source while preserving simulation. Full runtime shutdown
    // still acknowledges the execution stop before dropping the remaining owner.
    if state
        .runtime_loop
        .lock()
        .map_err(|e| e.to_string())?
        .is_some()
    {
        control::close_locked(state)?;
    }
    state.activate_runtime();
    Ok(())
}
pub(crate) fn binding_conflict_error(bindings: &[ServoJointBinding]) -> Option<String> {
    let mut by_id = HashMap::<u8, Vec<&str>>::new();
    for binding in bindings {
        by_id
            .entry(binding.physical_id)
            .or_default()
            .push(binding.slot_id.as_str());
    }
    let mut conflicts = by_id
        .into_iter()
        .filter(|(_, slots)| slots.len() > 1)
        .map(|(id, mut slots)| {
            slots.sort_unstable();
            format!("{id} → {}", slots.join(","))
        })
        .collect::<Vec<_>>();
    conflicts.sort_unstable();
    (!conflicts.is_empty()).then(|| format!("绑定冲突：{}", conflicts.join("；")))
}

pub(crate) fn official_servo_bindings(census: &[ServoIdentity]) -> Vec<ServoJointBinding> {
    duck_control::model::JOINT_NAMES
        .iter()
        .copied()
        .zip(duck_control::model::JOINT_IDS)
        .filter(|(_, id)| census.iter().any(|servo| servo.servo_id == *id))
        .map(|(slot_id, physical_id)| ServoJointBinding {
            slot_id: slot_id.to_owned(),
            physical_id,
            direction: 1,
            zero_offset_rad: 0.0,
            min_rad: None,
            max_rad: None,
        })
        .collect()
}

pub(crate) fn effective_servo_bindings(
    census: &[ServoIdentity],
    virtual_bindings: &[ServoJointBinding],
) -> Vec<ServoJointBinding> {
    let official = official_servo_bindings(census);
    duck_control::model::JOINT_NAMES
        .iter()
        .filter_map(|slot_id| {
            virtual_bindings
                .iter()
                .find(|binding| binding.slot_id == *slot_id)
                .or_else(|| official.iter().find(|binding| binding.slot_id == *slot_id))
                .filter(|binding| {
                    census
                        .iter()
                        .any(|servo| servo.servo_id == binding.physical_id)
                })
                .cloned()
        })
        .collect()
}

/// Resolve each slot from its saved binding, falling back to its official ID when unset.
/// A slot whose explicitly selected physical ID is not present in the
/// census stays unmapped so the read-only debug loop can continue with the other joints.
pub(crate) fn debug_servo_bindings(
    census: &[ServoIdentity],
    virtual_bindings: &[ServoJointBinding],
    slots: &[String],
) -> Vec<Option<ServoJointBinding>> {
    let bindings = effective_servo_bindings(census, virtual_bindings);
    slots
        .iter()
        .map(|slot_id| {
            bindings
                .iter()
                .find(|binding| binding.slot_id == *slot_id)
                .cloned()
        })
        .collect()
}

pub(crate) fn debug_driver_mapping(
    census: &[ServoIdentity],
    bindings: &[Option<ServoJointBinding>],
) -> Result<(Vec<u8>, Vec<Option<u8>>), String> {
    let census_ids = census
        .iter()
        .map(|servo| servo.servo_id)
        .collect::<HashSet<_>>();
    let mut used = HashSet::new();
    let mut driver_ids = Vec::with_capacity(bindings.len());
    let mut active_ids = Vec::with_capacity(bindings.len());
    for binding in bindings {
        if let Some(binding) = binding {
            if !used.insert(binding.physical_id) {
                return Err(format!("实机舵机 {} 被重复绑定", binding.physical_id));
            }
            active_ids.push(Some(binding.physical_id));
            driver_ids.push(binding.physical_id);
        } else {
            let placeholder = (0..=u8::MAX)
                .find(|id| !census_ids.contains(id) && !used.contains(id))
                .ok_or_else(|| "没有可用的未映射舵机占位 ID".to_owned())?;
            used.insert(placeholder);
            active_ids.push(None);
            driver_ids.push(placeholder);
        }
    }
    Ok((driver_ids, active_ids))
}

/// Resolve the physical mapping shared by debug and policy loops.  Missing virtual slots are
/// represented by `None` in `active_ids`, allowing the loop to keep running the matched joints
/// without attempting a bus read or write for the missing ones.
type DebugLoopMapping = (Vec<ServoJointBinding>, Vec<u8>, Vec<Option<u8>>);

pub(crate) fn resolve_physical_loop_mapping(
    physical: &PhysicalMachine,
    virtual_bindings: &[ServoJointBinding],
    slots: &[String],
) -> Result<DebugLoopMapping, String> {
    let mut bindings = debug_servo_bindings(&physical.census, virtual_bindings, slots)
        .into_iter()
        .flatten()
        .collect::<Vec<_>>();
    apply_physical_directions(&mut bindings, physical);
    let resolved = slots
        .iter()
        .map(|slot_id| {
            bindings
                .iter()
                .find(|binding| binding.slot_id == *slot_id)
                .cloned()
        })
        .collect::<Vec<_>>();
    let (driver_ids, active_ids) = debug_driver_mapping(&physical.census, &resolved)?;
    Ok((bindings, driver_ids, active_ids))
}

pub(crate) fn apply_physical_directions(
    bindings: &mut [ServoJointBinding],
    physical: &PhysicalMachine,
) {
    for binding in bindings {
        if let Some(override_direction) = physical
            .joint_directions
            .iter()
            .find(|item| item.slot_id == binding.slot_id)
        {
            binding.direction = override_direction.direction;
        }
    }
}

#[cfg(test)]
pub(crate) fn open(
    state: &DesktopState,
    input: LoopOpenInput,
) -> Result<FrontendLoopStatus, String> {
    let _transition = state
        .execution_transition
        .lock()
        .map_err(|e| e.to_string())?;
    open_locked(state, input)
}

fn open_locked(state: &DesktopState, input: LoopOpenInput) -> Result<FrontendLoopStatus, String> {
    let LoopOpenInput {
        virtual_machine_id,
        physical_machine_id: requested_physical_id,
        access,
        policy_id,
    } = input;
    if requested_physical_id.is_empty() {
        return Err(
            "physical machine ID is required; simulation uses the simulation runtime entry".into(),
        );
    }
    let access = match (access, policy_id.is_some()) {
        (crate::types::LoopAccess::ReadOnly, false) => loop_body::BackendAccess::ReadOnly,
        (crate::types::LoopAccess::Control, _) => loop_body::BackendAccess::Control,
        (crate::types::LoopAccess::ReadOnly, true) => {
            return Err("read-only access cannot load a policy".into());
        }
    };
    let machine = state
        .virtual_machines
        .lock()
        .map_err(|e| e.to_string())?
        .iter()
        .find(|machine| machine.id == virtual_machine_id)
        .cloned()
        .ok_or_else(|| "unknown virtual machine".to_string())?;
    if policy_id.is_some()
        && let Some(error) = binding_conflict_error(&machine.joint_bindings)
    {
        return Err(error);
    }
    let template = BodyTemplate::official().map_err(|e| e.to_string())?;
    let model = model(&machine.servo_model);
    let policy_path = if let Some(policy_id) = policy_id {
        let selected = policy::list(state.config_dir.as_deref())?
            .into_iter()
            .find(|entry| entry.id == policy_id && entry.cached && entry.selected)
            .ok_or_else(|| "selected ONNX policy is not downloaded".to_owned())?;
        let path = policy::selected_path(state.config_dir.as_deref())
            .ok_or_else(|| format!("selected ONNX policy is unavailable: {}", selected.id))?;
        let config = policy::runtime_config(state.config_dir.as_deref(), &selected)?;
        let descriptor = policy::descriptor(state.config_dir.as_deref(), &selected)?;
        Some((path, config, descriptor))
    } else {
        None
    };
    let physical = state
        .physical_machines
        .lock()
        .map_err(|e| e.to_string())?
        .iter()
        .find(|physical| physical.id == requested_physical_id)
        .cloned()
        .ok_or_else(|| "unknown physical machine".to_string())?;
    if !physical.connected
        || physical
            .virtual_machine_id
            .as_deref()
            .is_some_and(|id| id != machine.id)
    {
        return Err("physical machine is not available for this virtual machine".into());
    }
    let physical_id = physical.id.clone();
    let backend_kind = "physical";
    control::ensure_backend_access(
        state,
        backend_kind,
        access == loop_body::BackendAccess::Control,
    )?;
    let mut contract_sources = crate::backend::simulation::official_sources()?;
    contract_sources.push((
        "physical/profile".into(),
        format!(
            "{:?};{:?};{:?};{:?}",
            machine.servo_model, machine.joint_bindings, physical.joint_directions, physical.census
        )
        .into_bytes(),
    ));
    if let Some((path, config, descriptor)) = &policy_path {
        contract_sources.push((
            "policy/onnx".into(),
            std::fs::read(path).map_err(|e| e.to_string())?,
        ));
        contract_sources.push((
            "policy/resolved".into(),
            format!("{config:?};{descriptor:?}").into_bytes(),
        ));
    }
    let resolved_contract = ResolvedContract::with_sources(
        &template.id,
        template.version,
        policy_path
            .as_ref()
            .map(|(_, _, descriptor)| descriptor.id.clone())
            .unwrap_or_else(|| "none".to_owned()),
        backend_kind,
        format!("{model:?}"),
        contract_sources,
    );
    append_control_event(
        state.config_dir.as_deref(),
        format!(
            "resolved_contract body_id={} body_version={} policy_id={} backend_kind={} backend_profile={} contract_hash={}",
            resolved_contract.body_id,
            resolved_contract.body_version,
            resolved_contract.policy_id,
            resolved_contract.backend_kind,
            resolved_contract.backend_profile,
            resolved_contract.hash,
        ),
    );
    let slot_ids = template
        .actuators
        .iter()
        .map(|slot| slot.id.clone())
        .collect::<Vec<_>>();
    let (saved_bindings, bound_servo_ids, mapped_active_ids) =
        resolve_physical_loop_mapping(&physical, &machine.joint_bindings, &slot_ids)?;
    let calibration = Calibration {
        body_id: template.id.clone(),
        body_version: template.version,
        joints: template
            .actuators
            .iter()
            .zip(bound_servo_ids.iter().copied())
            .map(|(slot, id)| {
                let saved = saved_bindings
                    .iter()
                    .find(|binding| binding.slot_id == slot.id && binding.physical_id == id);
                JointCalibration {
                    slot_id: slot.id.clone(),
                    physical_id: id,
                    direction: saved.map(|binding| binding.direction).unwrap_or(1),
                    zero_offset_rad: saved.map(|binding| binding.zero_offset_rad).unwrap_or(0.0),
                    min_rad: saved.and_then(|binding| binding.min_rad),
                    max_rad: saved.and_then(|binding| binding.max_rad),
                }
            })
            .collect(),
    };
    let binding_summary = calibration
        .joints
        .iter()
        .map(|joint| {
            format!(
                "{}->{} dir={} offset={} min={:?} max={:?}",
                joint.slot_id,
                joint.physical_id,
                joint.direction,
                joint.zero_offset_rad,
                joint.min_rad,
                joint.max_rad
            )
        })
        .collect::<Vec<_>>()
        .join(" | ");
    let non_identity_calibration = calibration.joints.iter().any(|joint| {
        joint.direction != 1
            || joint.zero_offset_rad != 0.0
            || joint.min_rad.is_some()
            || joint.max_rad.is_some()
    });
    let nonzero_offsets = calibration
        .joints
        .iter()
        .filter(|joint| joint.zero_offset_rad != 0.0)
        .map(|joint| format!("{}={}", joint.slot_id, joint.zero_offset_rad))
        .collect::<Vec<_>>();
    let binding = LoopBinding {
        body: BodyRuntimeView::new(template.clone()).map_err(|e| e.to_string())?,
        servos: ServoBinding::all_slots(&template, model),
        calibration: calibration.clone(),
    };
    let loop_physical_id = physical_id.clone();
    let log_machine_id = physical.id.as_str();
    let log_machine_name = physical.name.as_str();
    let log_handle = begin_control_loop(
        state.config_dir.as_deref(),
        log_machine_id,
        log_machine_name,
        &binding_summary,
    );
    append_control_event(
        state.config_dir.as_deref(),
        format!(
            "binding_check servo_model={model:?} physical_driver={} actuator_count={} non_identity_calibration={} zero_offset_frame=raw_servo position_transform=subtract_zero_then_direction velocity_transform=direction_only nonzero_offsets={:?}",
            true,
            calibration.joints.len(),
            non_identity_calibration,
            nonzero_offsets
        ),
    );
    let mut debug_shared_bus;
    // Sensor loops share their connection with direct reads. Explicit debug writes use the
    // application execution gate; a read-only backend session itself never receives writes.
    let opened_driver = control::open_backend_driver(
        state,
        &physical_id,
        model,
        access,
        mapped_active_ids,
        &resolved_contract.hash,
    )?;
    debug_shared_bus = opened_driver.shared_bus;
    let driver: Box<dyn ControlBackend> = if let Some(handle) = log_handle {
        Box::new(LoggedServoDriver::new(
            opened_driver.driver,
            handle,
            log_machine_id.to_owned(),
        ))
    } else {
        opened_driver.driver
    };
    let (runtime_instance, mut status) = if access == loop_body::BackendAccess::Control {
        let body_loop = match control::open_loop(control::OpenLoopInput {
            binding,
            driver,
            policy: policy_path.clone(),
            config_dir: state.config_dir.clone(),
            backend_kind,
            backend_profile: format!("{model:?}"),
            contract_hash: resolved_contract.hash.clone(),
        }) {
            Ok(loop_state) => loop_state,
            Err(error) => {
                if let Some(shared) = debug_shared_bus.take()
                    && let Ok(shared) = Arc::try_unwrap(shared)
                    && let Ok(bus) = shared.into_inner()
                    && let Ok(mut buses) = state.connected_buses.lock()
                {
                    buses.insert(physical_id.clone(), bus);
                }
                return Err(error);
            }
        };
        let status = FrontendLoopStatus::from(body_loop.status());
        (
            crate::state::RuntimeInstance::physical_execution(body_loop, physical_id.clone()),
            status,
        )
    } else {
        append_control_event(
            state.config_dir.as_deref(),
            "policy_selected none; opening read-only source",
        );
        let mut source = ReadOnlySource::open(binding, driver);
        source
            .start(loop_body::ReadOnlySourceConfig {
                id: physical_id.clone(),
                sample_period: std::time::Duration::from_millis(40),
                timeout: std::time::Duration::from_secs(2),
                max_age: std::time::Duration::from_millis(200),
                cache_capacity: 1,
            })
            .map_err(|error| error.to_string())?;
        let status = FrontendLoopStatus {
            has_execution: false,
            runtime_state: crate::types::RuntimeState::Running,
            simulation: None,
            read_sources: vec![(&source).into()],
            phase: crate::state::types::LoopPhase::Holding,
            session: 0,
            tick: 0,
            torque: false,
            positions: Vec::new(),
            targets: Vec::new(),
            world: crate::state::types::WorldState {
                position: [0.0; 2],
                yaw: 0.0,
                distance_m: 0.0,
            },
            sensor_failures: 0,
            last_error: None,
            joint_range_states: Vec::new(),
            runtime_generation: 0,
            execution_generation: 0,
            config_revision: 0,
        };
        (crate::state::RuntimeInstance::read_only(source), status)
    };
    let mut motion = state.motion.lock().map_err(|e| e.to_string())?;
    motion.reset();
    // Joint feedback cannot supply physical odometry without a separate sensor provider.
    drop(motion);
    status.world = state.motion.lock().map_err(|e| e.to_string())?.world();
    state.emit_joint_range_states(&status.joint_range_states);
    log_joint_range_states(state.config_dir.as_deref(), &status.joint_range_states);
    *state
        .loop_physical_machine_id
        .lock()
        .map_err(|e| e.to_string())? = Some(loop_physical_id);
    *state.loop_physical_bus.lock().map_err(|e| e.to_string())? = debug_shared_bus;
    let mut runtime = state.runtime_loop.lock().map_err(|e| e.to_string())?;
    let instance = match (runtime.take(), runtime_instance) {
        (Some(mut existing), mut next) if existing.is_simulation() && !next.has_execution() => {
            state
                .config_revision
                .fetch_add(1, std::sync::atomic::Ordering::AcqRel);
            existing.attach_source(next.take_source().ok_or("missing read source")?)?;
            existing
        }
        (None, instance) => {
            state.activate_runtime();
            instance
        }
        (Some(existing), _) => {
            *runtime = Some(existing);
            return Err("runtime was replaced while attaching source".into());
        }
    };
    *runtime = Some(instance);
    drop(runtime);
    status.runtime_generation = state.current_runtime_generation();
    status.execution_generation = state.current_execution_generation();
    status.config_revision = state.current_config_revision();
    state.start_loop_clock()?;
    control::record_backend_open(state, backend_kind)?;
    Ok(status)
}

#[cfg(test)]
mod tests {
    use super::open;
    use super::resolve_physical_loop_mapping;
    use crate::state::DesktopState;
    use crate::types::{LoopOpenInput, PhysicalMachine, ServoIdentity};
    use duck_body::BodyTemplate;

    fn open_simulation(state: &DesktopState) -> crate::types::LoopStatus {
        let virtual_machine_id = state.virtual_machines.lock().unwrap()[0].id.clone();
        super::start(
            state,
            crate::types::RuntimeOpenInput {
                virtual_machine_id,
                physical_machine_id: None,
                access: crate::types::LoopAccess::ReadOnly,
                policy_id: None,
            },
        )
        .unwrap()
    }

    #[test]
    fn unified_commands_reject_old_generation_and_status_reads_do_not_advance_execution() {
        let state = DesktopState::default();
        let initial = open_simulation(&state);
        super::command(
            &state,
            initial.runtime_generation,
            initial.execution_generation,
            2,
            crate::types::RuntimeCommand::ResetWorld,
        )
        .unwrap();
        assert!(
            super::command(
                &state,
                initial.runtime_generation,
                initial.execution_generation,
                1,
                crate::types::RuntimeCommand::ResetWorld
            )
            .unwrap_err()
            .contains("stale command sequence")
        );
        let clock_status = state
            .runtime_loop
            .lock()
            .unwrap()
            .as_mut()
            .unwrap()
            .clock_tick(None)
            .unwrap();
        let first = crate::control::status(&state).unwrap().unwrap();
        let second = crate::control::status(&state).unwrap().unwrap();
        assert_eq!(first.tick, clock_status.tick);
        assert_eq!(first.tick, second.tick);
        assert_eq!(first.runtime_generation, initial.runtime_generation);
        super::close(
            &state,
            initial.runtime_generation,
            crate::types::RuntimeCloseScope::Runtime,
        )
        .unwrap();
        assert!(state.runtime_loop.lock().unwrap().is_none());
        assert!(state.execution_gate.physical_write().is_err());
        let next = open_simulation(&state);
        assert_ne!(initial.runtime_generation, next.runtime_generation);
        let error = super::command(
            &state,
            initial.runtime_generation,
            initial.execution_generation,
            1,
            crate::types::RuntimeCommand::ResetWorld,
        )
        .unwrap_err();
        assert!(error.contains("stale runtime generation"));
        assert!(
            super::close(
                &state,
                initial.runtime_generation,
                crate::types::RuntimeCloseScope::Runtime
            )
            .is_err()
        );
        assert!(state.runtime_loop.lock().unwrap().is_some());
        super::close(
            &state,
            next.runtime_generation,
            crate::types::RuntimeCloseScope::Runtime,
        )
        .unwrap();
    }

    #[test]
    fn reconfigure_serializes_replacement_and_failed_load_stays_faulted() {
        use crate::types::{LoopAccess, RuntimeCloseScope, RuntimeOpenInput, RuntimeState};
        let state = DesktopState::default();
        let first = open_simulation(&state);
        let id = state.virtual_machines.lock().unwrap()[0].id.clone();
        let input = || RuntimeOpenInput {
            virtual_machine_id: id.clone(),
            physical_machine_id: None,
            access: LoopAccess::ReadOnly,
            policy_id: None,
        };
        assert!(
            super::reconfigure(
                &state,
                first.runtime_generation,
                first.config_revision + 1,
                input()
            )
            .is_err()
        );
        assert_eq!(
            crate::control::status(&state)
                .unwrap()
                .unwrap()
                .runtime_generation,
            first.runtime_generation
        );
        let second = super::reconfigure(
            &state,
            first.runtime_generation,
            first.config_revision,
            input(),
        )
        .unwrap();
        assert_ne!(first.runtime_generation, second.runtime_generation);
        assert_ne!(first.execution_generation, second.execution_generation);
        let mut invalid = input();
        invalid.virtual_machine_id = "missing".into();
        assert!(
            super::reconfigure(
                &state,
                second.runtime_generation,
                second.config_revision,
                invalid
            )
            .is_err()
        );
        assert!(state.runtime_loop.lock().unwrap().is_none());
        let fault = crate::control::status(&state).unwrap().unwrap();
        assert_eq!(fault.runtime_state, RuntimeState::Faulted);
        assert!(
            fault
                .last_error
                .unwrap()
                .contains("unknown virtual machine")
        );
        assert!(state.execution_gate.physical_write().is_err());
        assert!(
            super::start(&state, input())
                .unwrap_err()
                .contains("Faulted")
        );
        super::close(&state, fault.runtime_generation, RuntimeCloseScope::Runtime).unwrap();
        assert!(crate::control::status(&state).unwrap().is_none());
        super::start(&state, input()).unwrap();
    }

    #[test]
    fn detaching_source_preserves_simulation_but_runtime_close_stops_both() {
        use crate::control::tests::{ReadOnlyDriver, read_only_loop_with_driver};
        use std::sync::{Arc, Mutex};
        let state = DesktopState::default();
        let status = open_simulation(&state);
        let events = Arc::new(Mutex::new(Vec::new()));
        let attach = || {
            let source = read_only_loop_with_driver(Box::new(ReadOnlyDriver {
                positions: vec![0.123; 15],
                events: events.clone(),
            }));
            let mut guard = state.runtime_loop.lock().unwrap();
            let mut simulation = guard.take().expect("expected simulation");
            simulation.attach_source(source).unwrap();
            *guard = Some(simulation);
        };
        attach();
        let snapshot = crate::control::status(&state).unwrap().unwrap();
        assert_eq!(snapshot.positions, status.positions);
        assert_eq!(snapshot.read_sources.len(), 1);
        super::close(
            &state,
            status.runtime_generation,
            crate::types::RuntimeCloseScope::ReadOnlySource,
        )
        .unwrap();
        assert!(matches!(
            state.runtime_loop.lock().unwrap().as_ref(),
            Some(runtime) if runtime.is_simulation()
        ));
        assert_eq!(
            state.current_runtime_generation(),
            status.runtime_generation
        );
        attach();
        super::close(
            &state,
            status.runtime_generation,
            crate::types::RuntimeCloseScope::Runtime,
        )
        .unwrap();
        assert!(state.runtime_loop.lock().unwrap().is_none());
        assert!(
            events.lock().unwrap().is_empty(),
            "close cannot emit hardware reads or writes"
        );
    }

    fn missing_physical_input(state: &DesktopState) -> LoopOpenInput {
        let virtual_machine_id = state
            .virtual_machines
            .lock()
            .expect("virtual machines lock")
            .first()
            .expect("default virtual machine")
            .id
            .clone();
        LoopOpenInput {
            virtual_machine_id,
            access: crate::types::LoopAccess::ReadOnly,
            physical_machine_id: String::new(),
            policy_id: None,
        }
    }

    #[test]
    fn physical_loop_requires_an_explicit_id_and_never_uses_an_echo_backend() {
        let state = DesktopState::default();
        let error = open(&state, missing_physical_input(&state)).unwrap_err();
        assert!(error.contains("physical machine ID is required"));
        assert!(state.runtime_loop.lock().unwrap().is_none());
        assert!(
            state
                .backend_registry
                .lock()
                .unwrap()
                .list()
                .next()
                .is_none()
        );
    }

    #[test]
    fn access_is_required_and_read_only_requests_cannot_load_a_policy() {
        assert!(
            serde_json::from_value::<LoopOpenInput>(serde_json::json!({
                "virtualMachineId": "virtual", "physicalMachineId": "physical"
            }))
            .is_err()
        );
        let state = DesktopState::default();
        let mut input = missing_physical_input(&state);
        input.physical_machine_id = "physical".into();
        input.policy_id = Some("velstand".into());
        assert!(
            open(&state, input.clone())
                .unwrap_err()
                .contains("read-only access")
        );
        input.access = crate::types::LoopAccess::Control;
        input.policy_id = None;
        assert!(
            open(&state, input)
                .unwrap_err()
                .contains("unknown physical machine")
        );
        assert!(state.runtime_loop.lock().unwrap().is_none());
    }
    #[test]
    fn physical_open_failure_does_not_leave_a_partial_loop_or_backend_marker() {
        let state = DesktopState::default();
        let virtual_machine_id = state
            .virtual_machines
            .lock()
            .expect("virtual machines lock")
            .first()
            .expect("default virtual machine")
            .id
            .clone();
        let physical = crate::types::PhysicalMachine {
            id: "physical-test".into(),
            name: "测试实机".into(),
            usb_port: "COM-test".into(),
            machine_code: "test".into(),
            census: vec![],
            reconnect_hint: None,
            virtual_machine_id: Some(virtual_machine_id.clone()),
            online_servo_ids: vec![],
            connected: true,
            last_seen: None,
            torque_enabled: false,
            joint_directions: vec![],
        };
        state
            .physical_machines
            .lock()
            .expect("physical machines lock")
            .push(physical);
        let error = open(
            &state,
            LoopOpenInput {
                virtual_machine_id,
                access: crate::types::LoopAccess::ReadOnly,
                physical_machine_id: "physical-test".into(),
                policy_id: None,
            },
        )
        .expect_err("physical open without a connected bus must fail");
        assert!(error.contains("connected bus"));
        assert!(
            state
                .runtime_loop
                .lock()
                .expect("loop state lock")
                .is_none()
        );
        assert!(
            state
                .loop_backend_kind
                .lock()
                .expect("backend kind lock")
                .is_none()
        );
    }

    #[test]
    fn partial_physical_mapping_keeps_unmatched_slots_as_read_only_placeholders() {
        let template = BodyTemplate::official().expect("official body template");
        let slots = template
            .actuators
            .iter()
            .map(|slot| slot.id.clone())
            .collect::<Vec<_>>();
        let physical = PhysicalMachine {
            id: "physical-partial".into(),
            name: "部分实机".into(),
            usb_port: "COM-test".into(),
            machine_code: "test".into(),
            census: vec![ServoIdentity {
                servo_id: duck_control::model::JOINT_IDS[0],
                model: "hd1910".into(),
            }],
            reconnect_hint: None,
            virtual_machine_id: None,
            online_servo_ids: vec![duck_control::model::JOINT_IDS[0]],
            connected: true,
            last_seen: None,
            torque_enabled: false,
            joint_directions: vec![],
        };
        let (_, driver_ids, active_ids) =
            resolve_physical_loop_mapping(&physical, &[], &slots).expect("partial mapping");
        assert_eq!(driver_ids.len(), slots.len());
        assert_eq!(active_ids.iter().filter(|id| id.is_some()).count(), 1);
        assert_eq!(
            active_ids.iter().filter(|id| id.is_none()).count(),
            slots.len() - 1
        );
    }

    #[test]
    fn custom_ids_keep_calibration_without_requiring_the_official_id_in_census() {
        let census = vec![ServoIdentity {
            servo_id: 7,
            model: "hd1910".into(),
        }];
        let binding = crate::types::ServoJointBinding {
            slot_id: "left_hip_yaw".into(),
            physical_id: 7,
            direction: -1,
            zero_offset_rad: 2.7,
            min_rad: Some(-0.5),
            max_rad: Some(0.7),
        };
        assert_eq!(
            super::effective_servo_bindings(&census, std::slice::from_ref(&binding)),
            vec![binding]
        );
    }
    #[test]
    fn faulted_runtime_blocks_commands_until_a_successful_close() {
        let state = DesktopState::default();
        let id = state.virtual_machines.lock().unwrap()[0].id.clone();
        let opened = crate::application::loop_service::start(
            &state,
            crate::types::RuntimeOpenInput {
                virtual_machine_id: id.clone(),
                physical_machine_id: None,
                access: crate::types::LoopAccess::ReadOnly,
                policy_id: None,
            },
        )
        .unwrap();
        state
            .backend_registry
            .lock()
            .unwrap()
            .unregister(&id)
            .unwrap();
        state.start_loop_clock_impl().unwrap();
        let deadline = std::time::Instant::now() + std::time::Duration::from_secs(2);
        while state.ensure_runtime_healthy().is_ok() && std::time::Instant::now() < deadline {
            std::thread::sleep(std::time::Duration::from_millis(5));
        }
        state.stop_loop_clock().unwrap();
        let failed = crate::control::status(&state).unwrap().unwrap();
        assert_eq!(failed.runtime_state, crate::types::RuntimeState::Faulted);
        assert!(failed.last_error.unwrap().contains("revoked"));
        assert!(state.execution_gate.physical_write().is_err());
        assert!(
            crate::application::loop_service::command(
                &state,
                opened.runtime_generation,
                opened.execution_generation,
                1,
                crate::types::RuntimeCommand::ResetWorld
            )
            .unwrap_err()
            .contains("Faulted")
        );
        // A failed cleanup retains the execution owner; it cannot be replaced behind the UI.
        assert!(
            crate::application::loop_service::close(
                &state,
                opened.runtime_generation,
                crate::types::RuntimeCloseScope::Runtime
            )
            .is_err()
        );
        assert!(state.runtime_loop.lock().unwrap().is_some());
    }
}
