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

pub(crate) fn select_body(
    state: &DesktopState,
    generation: u64,
    execution_generation: u64,
    virtual_machine_id: String,
    physical_machine_id: Option<String>,
) -> Result<FrontendLoopStatus, String> {
    use std::sync::atomic::Ordering;
    let _transition = state
        .execution_transition
        .lock()
        .map_err(|e| e.to_string())?;
    state.validate_runtime_generation(Some(generation))?;
    let (old_physical_id, old_virtual_id) = {
        let mut runtime = state.runtime_loop.lock().map_err(|e| e.to_string())?;
        validate_execution_generation(state, execution_generation)?;
        let runtime = runtime.as_mut().ok_or("runtime is closed")?;
        let execution = runtime.policy_execution_mut()?;
        if physical_machine_id.is_some() {
            state.ensure_runtime_healthy()?;
        }
        let old_physical_id = execution.physical_id().map(str::to_owned);
        let old_virtual_id = execution.virtual_machine_id().to_owned();
        if !execution.is_detached()
            && old_physical_id == physical_machine_id
            && old_virtual_id == virtual_machine_id
            && state
                .runtime_fault
                .lock()
                .map_err(|error| error.to_string())?
                .is_none()
        {
            return Ok(super::preview::with_runtime_generation(
                state,
                runtime.snapshot(),
            ));
        }
        if physical_machine_id.is_some() {
            if old_physical_id.is_some() {
                return Err("请先切回模拟机，再选择另一台真机".into());
            }
            if old_virtual_id != virtual_machine_id {
                return Err("请先切换到真机对应的模拟机".into());
            }
        }
        (old_physical_id, old_virtual_id)
    };
    let machines = state.virtual_machines.lock().map_err(|e| e.to_string())?;
    let machine = machines
        .iter()
        .find(|machine| machine.id == virtual_machine_id)
        .cloned()
        .ok_or("unknown virtual machine")?;
    let fallback = machines
        .iter()
        .find(|machine| machine.id == old_virtual_id)
        .cloned();
    drop(machines);
    let physical_body = if let Some(id) = &physical_machine_id {
        let body = prepare_physical_body(state, &virtual_machine_id, id, None)?;
        body.check_feedback(state)?;
        Some(body)
    } else {
        None
    };
    // Normal ticks can continue while hardware feedback is checked. Acquisition of this
    // lock is the handoff boundary, and a fault during preflight invalidates the request.
    let mut active = state.runtime_loop.lock().map_err(|e| e.to_string())?;
    validate_execution_generation(state, execution_generation)?;
    let runtime = active.as_mut().ok_or("runtime is closed")?;
    if physical_body.is_some()
        && let Some(source) = runtime.read_source_mut()
    {
        source.stop()?;
        runtime.take_source();
        if let Some(id) = state
            .loop_physical_machine_id
            .lock()
            .map_err(|e| e.to_string())?
            .take()
        {
            control::restore_shared_physical_bus(state, &id)?;
        }
        *state.loop_backend_kind.lock().map_err(|e| e.to_string())? = None;
    }
    let execution = runtime.policy_execution_mut()?;
    execution.action(loop_body::LoopAction::Drive(false))?;
    let released = execution.release_body(
        &mut *state.backend_registry.lock().map_err(|e| e.to_string())?,
        old_physical_id.as_deref().unwrap_or(&old_virtual_id),
    );
    state.execution_generation.fetch_add(1, Ordering::AcqRel);
    state.config_revision.fetch_add(1, Ordering::AcqRel);
    let result = released.and_then(|()| {
        if let Some(body) = physical_body {
            let id = body.physical.id.clone();
            let profile = format!("{:?}", body.model);
            let contract = body.contract.hash.clone();
            let (binding, driver, bus) = body.open(state, loop_body::BackendAccess::Control)?;
            *state
                .loop_physical_machine_id
                .lock()
                .map_err(|e| e.to_string())? = Some(id.clone());
            *state.loop_physical_bus.lock().map_err(|e| e.to_string())? = bus;
            *state
                .loop_virtual_machine_id
                .lock()
                .map_err(|e| e.to_string())? = Some(virtual_machine_id.clone());
            let mut runner = control::open_loop(control::OpenLoopInput {
                binding,
                driver,
                policy: None,
                config_dir: state.config_dir.clone(),
                backend_kind: "physical",
                backend_profile: profile.clone(),
                contract_hash: contract.clone(),
            })?;
            runner.sample_body().map_err(|e| e.to_string())?;
            execution.switch_to_physical(id, runner, profile, contract)?;
            control::record_backend_open(state, "physical")?;
            execution.action(loop_body::LoopAction::Power(true))?;
        } else {
            let next = super::preview::open_body(state, &machine)?;
            execution.switch_to_simulation(next)?;
            if let Some(id) = &old_physical_id {
                control::restore_shared_physical_bus(state, id)?;
                *state
                    .loop_physical_machine_id
                    .lock()
                    .map_err(|e| e.to_string())? = None;
            }
            if old_physical_id.is_some() {
                *state.loop_backend_kind.lock().map_err(|e| e.to_string())? = None;
            }
            state
                .execution_gate
                .set_physical_writes_enabled(false)
                .map_err(|e| e.to_string())?;
        }
        Ok(())
    });
    if let Err(error) = result {
        // Once the outgoing lease has been released, reopen its simulation on a failed
        // hardware attach. An attached physical body still owns torque-off and is faulted.
        state
            .execution_gate
            .set_physical_writes_enabled(false)
            .map_err(|e| e.to_string())?;
        let recovered = execution.is_detached()
            && old_physical_id.is_none()
            && fallback.is_some_and(|machine| {
                if let Ok(mut id) = state.loop_physical_machine_id.lock()
                    && let Some(id) = id.take()
                {
                    let _ = control::restore_shared_physical_bus(state, &id);
                }
                super::preview::open_body(state, &machine)
                    .and_then(|next| execution.switch_to_simulation(next))
                    .is_ok()
            });
        if !recovered {
            let error = match execution.stop() {
                Ok(()) => error,
                Err(stop) => format!("{error}; stop failed: {stop}"),
            };
            *state.runtime_fault.lock().map_err(|e| e.to_string())? = Some(error.clone());
            return Err(error);
        }
        return Err(error);
    }
    *state.runtime_fault.lock().map_err(|e| e.to_string())? = None;
    Ok(super::preview::with_runtime_generation(
        state,
        runtime.snapshot(),
    ))
}

fn validate_execution_generation(state: &DesktopState, expected: u64) -> Result<(), String> {
    let current = state.current_execution_generation();
    if expected != current {
        return Err(format!(
            "stale execution generation {expected}, current is {current}"
        ));
    }
    Ok(())
}

/// Configuration mutations hold the transition lock through persistence and source replacement.
pub(crate) fn ensure_mapping_editable(
    state: &DesktopState,
    virtual_id: &str,
) -> Result<(), String> {
    let physical_ids = state
        .physical_machines
        .lock()
        .map_err(|e| e.to_string())?
        .iter()
        .filter(|machine| {
            machine.virtual_machine_id.as_deref() == Some(virtual_id)
                || machine.virtual_machine_id.is_none()
        })
        .map(|machine| machine.id.clone())
        .collect::<Vec<_>>();
    for id in physical_ids {
        super::encoder_calibration::ensure_ready(state, &id)?;
    }
    let active_body = state
        .runtime_loop
        .lock()
        .map_err(|e| e.to_string())?
        .as_ref()
        .and_then(|runtime| runtime.snapshot().active_body);
    let bound_virtual = state
        .loop_virtual_machine_id
        .lock()
        .map_err(|e| e.to_string())?
        .clone();
    if active_body.is_some_and(|body| {
        body.physical_machine_id.is_some()
            && (body.virtual_machine_id.as_deref() == Some(virtual_id)
                || bound_virtual.as_deref() == Some(virtual_id))
    }) {
        return Err("请先切回模拟机，再修改正在驱动实机的肢体映射".into());
    }
    Ok(())
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
    let (simulation, managed_policy) = {
        let runtime = state.runtime_loop.lock().map_err(|e| e.to_string())?;
        validate_execution_generation(state, execution_generation)?;
        let runtime = runtime.as_ref().ok_or("runtime is closed")?;
        if !runtime.has_execution() {
            return Err("runtime is read-only".into());
        }
        (runtime.is_simulation(), runtime.has_policy_execution())
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
        RuntimeCommand::Input { input } if managed_policy => {
            super::preview::set_input(state, input)
        }
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
        RuntimeCommand::Targets { targets } if !simulation && !managed_policy => {
            control::set_targets(state, targets)?;
            control::status(state)?.ok_or_else(|| "runtime is closed".into())
        }
        RuntimeCommand::Policy { policy_id, input } if managed_policy => {
            super::preview::select_policy(state, policy_id, input)
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
            if managed_policy {
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

#[cfg(test)]
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
    virtual_bindings: &[crate::types::VirtualJointBinding],
) -> Vec<ServoJointBinding> {
    duck_control::model::JOINT_NAMES
        .iter()
        .filter_map(|slot_id| {
            let physical_id = virtual_bindings
                .iter()
                .find(|binding| binding.slot_id == *slot_id)
                .map(|binding| binding.physical_id)?;
            census
                .iter()
                .any(|servo| servo.servo_id == physical_id)
                .then_some(ServoJointBinding {
                    slot_id: (*slot_id).to_owned(),
                    physical_id,
                    direction: 1,
                    zero_offset_rad: 0.0,
                    min_rad: None,
                    max_rad: None,
                })
        })
        .collect()
}

/// Resolve each slot from the virtual machine's explicit binding.
/// A slot whose explicitly selected physical ID is not present in the
/// census stays unmapped so the read-only debug loop can continue with the other joints.
pub(crate) fn debug_servo_bindings(
    census: &[ServoIdentity],
    virtual_bindings: &[crate::types::VirtualJointBinding],
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
    virtual_bindings: &[crate::types::VirtualJointBinding],
    physical: &PhysicalMachine,
    slots: &[String],
) -> Result<DebugLoopMapping, String> {
    // The virtual machine owns the slot -> physical ID relation.  The physical
    // machine contributes installation direction and calibration by limb name.
    // A census confirms only that an ID exists, never which limb it operates.
    let bindings = debug_servo_bindings(
        &physical
            .census
            .iter()
            .filter(|servo| physical.online_servo_ids.contains(&servo.servo_id))
            .cloned()
            .collect::<Vec<_>>(),
        virtual_bindings,
        slots,
    )
    .into_iter()
    .flatten()
    .map(|mut binding| {
        if let Some(calibration) = physical
            .joint_bindings
            .iter()
            .find(|calibration| calibration.slot_id == binding.slot_id)
        {
            binding.zero_offset_rad = calibration.zero_offset_rad;
            binding.min_rad = calibration.min_rad;
            binding.max_rad = calibration.max_rad;
        }
        if let Some(installation) = physical
            .joint_directions
            .iter()
            .find(|direction| direction.slot_id == binding.slot_id)
        {
            binding.direction = installation.direction;
        }
        binding
    })
    .collect::<Vec<_>>();
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

type PolicyLoad = (
    std::path::PathBuf,
    crate::backend::policy_runtime::PolicyConfig,
    crate::backend::policy_runtime::PolicyDescriptor,
);

struct PhysicalBody {
    virtual_id: String,
    physical: PhysicalMachine,
    binding: LoopBinding,
    model: servo::ServoModel,
    mapped_active_ids: Vec<Option<u8>>,
    binding_summary: String,
    contract: ResolvedContract,
    simulation: crate::types::SimulationParameters,
    simulation_model: servo::ServoModel,
}

fn prepare_physical_body(
    state: &DesktopState,
    virtual_machine_id: &str,
    requested_physical_id: &str,
    policy_path: Option<&PolicyLoad>,
) -> Result<PhysicalBody, String> {
    let machine = state
        .virtual_machines
        .lock()
        .map_err(|e| e.to_string())?
        .iter()
        .find(|machine| machine.id == virtual_machine_id)
        .cloned()
        .ok_or_else(|| "unknown virtual machine".to_string())?;
    let template = BodyTemplate::official().map_err(|e| e.to_string())?;
    let physical = state
        .physical_machines
        .lock()
        .map_err(|e| e.to_string())?
        .iter()
        .find(|physical| physical.id == requested_physical_id)
        .cloned()
        .ok_or_else(|| "unknown physical machine".to_string())?;
    if !physical.connected {
        return Err("physical machine is offline".into());
    }
    let model = super::physical_machine::physical_servo_model(&physical)?;
    let backend_kind = "physical";
    let mut contract_sources = crate::backend::simulation::official_sources()?;
    contract_sources.push((
        "physical/profile".into(),
        format!(
            "{model:?};{:?};{:?};{:?}",
            (&machine.joint_bindings, &physical.joint_bindings),
            physical.joint_directions,
            physical.online_servo_ids
        )
        .into_bytes(),
    ));
    if let Some((path, config, descriptor)) = policy_path {
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
        resolve_physical_loop_mapping(&machine.joint_bindings, &physical, &slot_ids)?;
    crate::logging::diagnostic(
        state.config_dir.as_deref(),
        "binding",
        "info",
        "body_mapping_resolved",
        serde_json::json!({"duck_id":physical.id,"virtual_machine_id":machine.id,
            "virtual_bindings":machine.joint_bindings,"joint_order":slot_ids,
            "active_servo_ids":mapped_active_ids,"calibrations":saved_bindings,
            "unmatched_feedback":"mujoco_in_control_invalid_in_read_only",
            "association":"configured_id_presence_only"}),
    );
    if let Some(error) = binding_conflict_error(&saved_bindings) {
        return Err(error);
    }
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
    let binding = LoopBinding {
        body: BodyRuntimeView::new(template.clone()).map_err(|e| e.to_string())?,
        servos: ServoBinding::all_slots(&template, model),
        calibration,
    };
    binding
        .calibration
        .validate(&template)
        .map_err(|error| error.to_string())?;
    Ok(PhysicalBody {
        virtual_id: machine.id,
        physical,
        binding,
        model,
        mapped_active_ids,
        binding_summary,
        contract: resolved_contract,
        simulation: machine.simulation,
        simulation_model: crate::helpers::model(&machine.servo_model),
    })
}

type OpenedPhysicalBody = (
    LoopBinding,
    Box<dyn ControlBackend>,
    Option<Arc<std::sync::Mutex<crate::backend::control::ConnectedBus>>>,
);

impl PhysicalBody {
    fn check_feedback(&self, state: &DesktopState) -> Result<(), String> {
        let ids = self
            .mapped_active_ids
            .iter()
            .flatten()
            .copied()
            .collect::<Vec<_>>();
        if ids.is_empty() {
            return Ok(());
        }
        if ids
            .iter()
            .any(|id| !self.physical.online_servo_ids.contains(id))
        {
            return Err("真机关节绑定的舵机未全部在线".into());
        }
        super::physical_io::with_bus_locked(
            state,
            &self.physical.id,
            loop_body::BackendAccess::ReadOnly,
            |bus| {
                let positions = bus
                    .ok_or("实机没有可用连接")?
                    .read_positions(self.model, &ids)?;
                if positions.len() != ids.len()
                    || positions.iter().any(|position| !position.is_finite())
                {
                    return Err("真机返回的关节反馈不完整".into());
                }
                Ok(())
            },
        )
    }

    fn open(
        self,
        state: &DesktopState,
        access: loop_body::BackendAccess,
    ) -> Result<OpenedPhysicalBody, String> {
        let Self {
            virtual_id,
            physical,
            mut binding,
            model,
            mapped_active_ids,
            binding_summary,
            contract,
            simulation,
            simulation_model,
        } = self;
        // Unmatched limbs keep a MuJoCo actuator. This simulation has its own lease;
        // it must never select/revoke the application's physical execution owner.
        let fallback = if access == loop_body::BackendAccess::Control {
            let mut registry = loop_body::BackendRegistry::default();
            crate::backend::simulation::register_machine(
                &mut registry,
                "unmatched-limbs",
                simulation_model,
                &simulation,
            )?;
            registry
                .select("unmatched-limbs")
                .map_err(|e| e.to_string())?;
            Some((
                Box::new(
                    registry
                        .open("unmatched-limbs", access)
                        .map_err(|e| e.to_string())?,
                ) as Box<dyn ControlBackend>,
                registry,
            ))
        } else {
            None
        };
        let calibration = &binding.calibration;
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

        // Sensor loops share their connection with direct reads. Explicit debug writes use the
        // application execution gate; a read-only backend session itself never receives writes.
        let read_only = access == loop_body::BackendAccess::ReadOnly;
        let raw_ids = physical.online_servo_ids.clone();
        let active_ids = mapped_active_ids.clone();
        let driver_ids = if read_only {
            raw_ids.iter().copied().map(Some).collect()
        } else {
            mapped_active_ids
        };
        let opened_driver = control::open_backend_driver(
            state,
            log_machine_id,
            model,
            access,
            driver_ids,
            &contract.hash,
        )?;

        let periodic_ranges = (model == servo::ServoModel::Hd1910).then(|| {
            let template = binding.body.template();
            template
                .actuators
                .iter()
                .map(|slot| {
                    template
                        .joint_range(&slot.joint)
                        .expect("validated actuator joint")
                })
                .collect::<Vec<_>>()
        });

        let driver: Box<dyn ControlBackend> = if let Some(handle) = log_handle {
            let logger =
                LoggedServoDriver::new(opened_driver.driver, handle, log_machine_id.to_owned());
            Box::new(if read_only {
                logger
            } else {
                logger.with_calibration(binding.calibration.clone(), periodic_ranges.clone())
            })
        } else {
            opened_driver.driver
        };
        if read_only {
            let virtuals = Arc::clone(&state.virtual_machines);
            let physicals = Arc::clone(&state.physical_machines);
            let pending_calibrations = Arc::clone(&state.encoder_calibrations);
            let revision = Arc::clone(&state.config_revision);
            let physical_id = physical.id.clone();
            let slots = binding
                .calibration
                .joints
                .iter()
                .map(|joint| joint.slot_id.clone())
                .collect::<Vec<_>>();
            let mut driver =
                crate::backend::read_only_body::ReadOnlyBody::new(driver, raw_ids, move || {
                    let virtual_bindings = virtuals
                        .lock()
                        .map_err(|e| e.to_string())?
                        .iter()
                        .find(|machine| machine.id == virtual_id)
                        .ok_or("unknown virtual machine")?
                        .joint_bindings
                        .clone();
                    let physicals = physicals.lock().map_err(|e| e.to_string())?;
                    let physical = physicals
                        .iter()
                        .find(|machine| machine.id == physical_id)
                        .ok_or("unknown physical machine")?;
                    let (bindings, _, _) =
                        resolve_physical_loop_mapping(&virtual_bindings, physical, &slots)?;
                    let pending = pending_calibrations.lock().map_err(|e| e.to_string())?;
                    Ok(slots
                        .iter()
                        .map(|slot| {
                            if super::encoder_calibration::is_pending(&pending, &physical_id, slot)
                            {
                                return None;
                            }
                            bindings
                                .iter()
                                .find(|joint| &joint.slot_id == slot)
                                .map(|joint| JointCalibration {
                                    slot_id: joint.slot_id.clone(),
                                    physical_id: joint.physical_id,
                                    direction: joint.direction,
                                    zero_offset_rad: joint.zero_offset_rad,
                                    min_rad: joint.min_rad,
                                    max_rad: joint.max_rad,
                                })
                        })
                        .collect())
                })
                .with_mapping_revision(move || revision.load(std::sync::atomic::Ordering::Acquire));
            if let Some(ranges) = periodic_ranges {
                driver = driver.with_periodic_ranges(ranges);
            }
            // The source adapter receives canonical body angles. ID/direction conversion is
            // resolved from the shared configuration above for every raw sample.
            for (index, joint) in binding.calibration.joints.iter_mut().enumerate() {
                joint.physical_id = (index + 1) as u8;
                joint.direction = 1;
                joint.zero_offset_rad = 0.0;
                joint.min_rad = None;
                joint.max_rad = None;
            }
            return Ok((binding, Box::new(driver), opened_driver.shared_bus));
        }
        let driver: Box<dyn ControlBackend> = if let Some(ranges) = periodic_ranges {
            Box::new(crate::backend::hd1910_body::Hd1910Body::new(
                driver,
                binding.calibration.clone(),
                ranges,
                active_ids.clone(),
                opened_driver.hd1910_position_domains,
            ))
        } else {
            driver
        };
        let driver = match fallback {
            Some((simulation, registry)) => {
                Box::new(crate::backend::attached_body::AttachedBody::new(
                    simulation,
                    driver,
                    active_ids,
                    binding.calibration.clone(),
                    registry,
                )) as Box<dyn ControlBackend>
            }
            None => driver,
        };
        Ok((binding, driver, opened_driver.shared_bus))
    }
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
    control::ensure_backend_access(
        state,
        "physical",
        access == loop_body::BackendAccess::Control,
    )?;
    let body = prepare_physical_body(
        state,
        &virtual_machine_id,
        &requested_physical_id,
        policy_path.as_ref(),
    )?;
    let physical_id = body.physical.id.clone();
    let binding_summary = body.binding_summary.clone();
    let backend_kind = "physical";
    let backend_profile = format!("{:?}", body.model);
    let contract_hash = body.contract.hash.clone();
    let (binding, driver, mut debug_shared_bus) = body.open(state, access)?;
    let (runtime_instance, mut status) = if access == loop_body::BackendAccess::Control {
        let body_loop = match control::open_loop(control::OpenLoopInput {
            binding,
            driver,
            policy: policy_path.clone(),
            config_dir: state.config_dir.clone(),
            backend_kind,
            backend_profile,
            contract_hash,
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
        crate::logging::diagnostic(
            state.config_dir.as_deref(),
            "binding",
            "info",
            "read_source_started",
            serde_json::json!({"duck_id":physical_id,"source_generation":source.generation(),
                "joint_order":duck_control::model::JOINT_NAMES,"binding_summary":binding_summary}),
        );
        let status = FrontendLoopStatus {
            has_execution: false,
            active_body: None,
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
        .map_err(|e| e.to_string())? = Some(physical_id);
    *state.loop_physical_bus.lock().map_err(|e| e.to_string())? = debug_shared_bus;
    *state
        .loop_virtual_machine_id
        .lock()
        .map_err(|e| e.to_string())? = Some(virtual_machine_id);
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
    control::status(state)?.ok_or_else(|| "runtime is closed".into())
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
    fn a_faulted_execution_can_return_to_sampling_without_rearming_physical_control() {
        let state = DesktopState::default();
        let before = open_simulation(&state);
        let virtual_id = before.active_body.unwrap().virtual_machine_id.unwrap();
        *state.runtime_fault.lock().unwrap() = Some("goal_position cannot encode".into());
        let after = super::select_body(
            &state,
            before.runtime_generation,
            before.execution_generation,
            virtual_id,
            None,
        )
        .unwrap();
        assert!(state.runtime_fault.lock().unwrap().is_none());
        assert!(after.active_body.unwrap().physical_machine_id.is_none());
        assert!(after.execution_generation > before.execution_generation);
        assert_eq!(after.phase, crate::types::LoopPhase::Ready);
    }

    #[test]
    fn body_handoff_isolates_feedback_and_releases_physical_control_without_restarting_runtime() {
        use loop_body::{BackendAccess, BackendDescriptor, BackendKind};
        use std::sync::{Arc, Mutex};

        let state = DesktopState::default();
        let id = state.virtual_machines.lock().unwrap()[0].id.clone();
        let placement = crate::types::WorldState {
            position: [1.4, -0.6],
            yaw: 0.9,
            distance_m: 2.0,
        };
        state.world_placements.remember(&id, placement);
        let initial = open_simulation(&state);
        assert_eq!(initial.world.position, placement.position);
        let virtual_id = initial.active_body.unwrap().virtual_machine_id.unwrap();
        let events = Arc::new(Mutex::new(Vec::new()));
        let mut active = state.runtime_loop.lock().unwrap();
        let runtime = active.as_mut().unwrap();
        runtime.clock_tick(None).unwrap();
        let runner = runtime.policy_execution_mut().unwrap();
        let mut registry = state.backend_registry.lock().unwrap();
        runner.release_body(&mut registry, &virtual_id).unwrap();
        let fake_events = Arc::clone(&events);
        registry
            .register(
                BackendDescriptor {
                    id: "test-physical".into(),
                    kind: BackendKind::Physical,
                    supports_control: true,
                    contract: "test".into(),
                },
                move || {
                    Ok(Box::new(crate::control::tests::ReadOnlyDriver {
                        positions: vec![0.25; 15],
                        events: Arc::clone(&fake_events),
                    }))
                },
            )
            .unwrap();
        registry.select("test-physical").unwrap();
        let driver = registry
            .open("test-physical", BackendAccess::Control)
            .unwrap();
        drop(registry);
        state
            .execution_gate
            .set_physical_writes_enabled(true)
            .unwrap();
        let template = BodyTemplate::official().unwrap();
        let binding = loop_body::LoopBinding {
            body: loop_body::BodyRuntimeView::new(template.clone()).unwrap(),
            servos: loop_body::ServoBinding::all_slots(&template, servo::ServoModel::Hd1910),
            calibration: loop_body::Calibration {
                body_id: template.id.clone(),
                body_version: template.version,
                joints: template
                    .actuators
                    .iter()
                    .enumerate()
                    .map(|(index, slot)| loop_body::JointCalibration {
                        slot_id: slot.id.clone(),
                        physical_id: (index + 1) as u8,
                        direction: 1,
                        zero_offset_rad: 0.0,
                        min_rad: None,
                        max_rad: None,
                    })
                    .collect(),
            },
        };
        let mut next = crate::backend::policy_runtime::MicroDuckPolicyRunner::open(
            binding,
            Box::new(driver),
            loop_body::LoopConfig::default(),
        )
        .unwrap();
        next.sample_body().unwrap();
        runner
            .switch_to_physical("test-physical".into(), next, "test".into(), "test".into())
            .unwrap();
        runner.action(loop_body::LoopAction::Power(true)).unwrap();
        runner.stop().unwrap();
        let stopped_events = events.lock().unwrap().len();
        runner.stop().unwrap();
        assert_eq!(
            events.lock().unwrap().len(),
            stopped_events,
            "a confirmed torque-off must not touch a subsequently disappearing bus"
        );
        runner.action(loop_body::LoopAction::Power(true)).unwrap();
        runner.stop().unwrap();
        assert!(
            events.lock().unwrap().len() > stopped_events,
            "arming again requires a new torque-off confirmation"
        );
        runner.action(loop_body::LoopAction::Power(true)).unwrap();
        let physical = runtime.clock_tick(None).unwrap();
        assert_eq!(
            physical.active_body.unwrap().physical_machine_id.as_deref(),
            Some("test-physical")
        );
        assert_eq!(physical.positions, vec![0.25; 15]);
        assert_eq!(physical.world.position, placement.position);
        assert_eq!(physical.world.yaw, placement.yaw);
        assert!(physical.simulation.is_none());
        drop(active);

        let simulation = super::select_body(
            &state,
            initial.runtime_generation,
            initial.execution_generation,
            virtual_id,
            None,
        )
        .unwrap();
        assert_eq!(simulation.world.position, placement.position);
        assert_eq!(simulation.world.yaw, placement.yaw);
        assert_eq!(simulation.world.distance_m, placement.distance_m);
        assert_eq!(initial.runtime_generation, simulation.runtime_generation);
        assert!(initial.execution_generation < simulation.execution_generation);
        assert_eq!(simulation.tick, physical.tick);
        assert!(
            simulation
                .active_body
                .unwrap()
                .physical_machine_id
                .is_none()
        );
        assert_eq!(events.lock().unwrap().last(), Some(&"torque"));
        let mut active = state.runtime_loop.lock().unwrap();
        let simulation = active.as_mut().unwrap().clock_tick(None).unwrap();
        assert!(simulation.simulation.is_some());
        assert_ne!(simulation.positions, vec![0.25; 15]);
        assert!(state.execution_gate.physical_write().is_err());
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
            census: vec![ServoIdentity {
                servo_id: 20,
                model: "hd1910".into(),
            }],
            reconnect_hint: None,
            virtual_machine_id: Some(virtual_machine_id.clone()),
            online_servo_ids: vec![],
            connected: true,
            last_seen: None,
            torque_enabled: false,
            joint_directions: Vec::new(),
            joint_bindings: vec![],
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
            joint_directions: Vec::new(),
            joint_bindings: vec![],
        };
        let virtual_bindings = vec![crate::types::VirtualJointBinding {
            slot_id: slots[0].clone(),
            physical_id: duck_control::model::JOINT_IDS[0],
        }];
        let (_, driver_ids, active_ids) =
            resolve_physical_loop_mapping(&virtual_bindings, &physical, &slots)
                .expect("partial mapping");
        assert_eq!(driver_ids.len(), slots.len());
        assert_eq!(active_ids.iter().filter(|id| id.is_some()).count(), 1);
        assert_eq!(
            active_ids.iter().filter(|id| id.is_none()).count(),
            slots.len() - 1
        );
    }

    #[test]
    fn knee_direction_reaches_read_only_feedback_and_the_official_3d_pose() {
        use crate::types::{ServoJointCalibration, ServoJointDirection};
        use std::sync::{Arc, Mutex};

        let state = DesktopState::default();
        let virtual_id = state.virtual_machines.lock().unwrap()[0].id.clone();
        let slots = BodyTemplate::official().unwrap().actuators;
        state.virtual_machines.lock().unwrap()[0].joint_bindings = slots
            .iter()
            .enumerate()
            .map(|(index, slot)| crate::types::VirtualJointBinding {
                slot_id: slot.id.clone(),
                physical_id: (index + 1) as u8,
            })
            .collect();
        let physical = PhysicalMachine {
            id: "knee-test".into(),
            name: "test".into(),
            usb_port: "test".into(),
            machine_code: "test".into(),
            reconnect_hint: None,
            virtual_machine_id: Some(virtual_id.clone()),
            connected: true,
            census: (1..=15)
                .map(|servo_id| ServoIdentity {
                    servo_id,
                    model: "hd1910".into(),
                })
                .collect(),
            online_servo_ids: (1..=15).collect(),
            last_seen: None,
            torque_enabled: false,
            joint_directions: vec![ServoJointDirection {
                slot_id: "left_knee".into(),
                direction: 1,
            }],
            joint_bindings: slots
                .iter()
                .map(|slot| ServoJointCalibration {
                    slot_id: slot.id.clone(),

                    zero_offset_rad: if slot.id == "left_knee" {
                        2.8194566881338083
                    } else {
                        0.0
                    },
                    min_rad: None,
                    max_rad: None,
                })
                .collect(),
        };
        let mut other = physical.clone();
        other.id = "other-duck".into();
        state
            .physical_machines
            .lock()
            .unwrap()
            .extend([physical, other]);
        let events = Arc::new(Mutex::new(Vec::new()));
        let mut raw = vec![0.0; 15];
        raw[3] = 3.9592044135328397;
        let scene = duck_body::OfficialScene::from_path(
            &crate::backend::mujoco::resource_directory()
                .unwrap()
                .join("robot_walk.xml"),
        )
        .unwrap();
        let knee_body = scene
            .model()
            .joints
            .iter()
            .find(|joint| joint.name == "left_knee")
            .unwrap()
            .body as usize;
        let hip_body = scene
            .model()
            .joints
            .iter()
            .find(|joint| joint.name == "left_hip_pitch")
            .unwrap()
            .body as usize;
        let mut frames = Vec::new();
        for direction in [1, -1, 1] {
            crate::application::binding::physical_machine_bind(
                &state,
                crate::types::PhysicalMachineBindingInput {
                    physical_machine_id: "knee-test".into(),

                    slot_id: "left_knee".into(),

                    direction,
                },
            )
            .unwrap();
            let body =
                super::prepare_physical_body(&state, &virtual_id, "knee-test", None).unwrap();
            let mut source = loop_body::ReadOnlySource::open(
                body.binding,
                Box::new(crate::control::tests::ReadOnlyDriver {
                    positions: raw.clone(),
                    events: events.clone(),
                }),
            );
            let feedback = source.read().unwrap();
            let status: crate::types::ReadSourceStatus = (&source).into();
            assert_eq!(status.positions, feedback.positions);
            assert!(
                (status.positions[3] - 1.1397477253990314 * f64::from(direction)).abs() < 1e-12
            );
            frames.push(scene.frame(&status.positions).unwrap());
        }
        assert_ne!(frames[0].bodies[knee_body], frames[1].bodies[knee_body]);
        assert_eq!(frames[0].bodies[knee_body], frames[2].bodies[knee_body]);
        assert_eq!(frames[0].bodies[hip_body], frames[1].bodies[hip_body]);
        assert!(events.lock().unwrap().iter().all(|event| *event == "read"));
        assert_eq!(
            state.physical_machines.lock().unwrap()[1].joint_directions[0].direction,
            1
        );

        // A physical executor must never keep an old mapping behind a newly saved value.
        let body = super::prepare_physical_body(&state, &virtual_id, "knee-test", None).unwrap();
        let execution = crate::backend::policy_runtime::MicroDuckPolicyRunner::open(
            body.binding,
            Box::new(crate::control::tests::ReadOnlyDriver {
                positions: raw,
                events: events.clone(),
            }),
            loop_body::LoopConfig::default(),
        )
        .unwrap();
        *state.runtime_loop.lock().unwrap() = Some(
            crate::state::RuntimeInstance::physical_execution(execution, "knee-test".into()),
        );
        *state.loop_physical_machine_id.lock().unwrap() = Some("knee-test".into());
        let rejected = crate::application::binding::physical_machine_bind(
            &state,
            crate::types::PhysicalMachineBindingInput {
                physical_machine_id: "knee-test".into(),

                slot_id: "left_knee".into(),

                direction: -1,
            },
        )
        .unwrap_err();
        assert!(rejected.contains("请先停止"));
        assert_eq!(
            state.physical_machines.lock().unwrap()[0].joint_directions[0].direction,
            1
        );
    }

    #[test]
    fn virtual_mapping_combines_limb_calibration_without_using_census_order() {
        let physical = PhysicalMachine {
            id: "physical-test".into(),
            name: "test".into(),
            usb_port: "test".into(),
            machine_code: "test".into(),
            census: [7, 20]
                .into_iter()
                .map(|servo_id| ServoIdentity {
                    servo_id,
                    model: "hd1910".into(),
                })
                .collect(),
            reconnect_hint: None,
            virtual_machine_id: None,
            online_servo_ids: vec![7, 20],
            connected: true,
            last_seen: None,
            torque_enabled: false,
            joint_directions: vec![crate::types::ServoJointDirection {
                slot_id: "left_hip_yaw".into(),
                direction: -1,
            }],
            joint_bindings: vec![crate::types::ServoJointCalibration {
                slot_id: "left_hip_yaw".into(),

                zero_offset_rad: 0.5,
                min_rad: None,
                max_rad: None,
            }],
        };
        let virtual_bindings = vec![crate::types::VirtualJointBinding {
            slot_id: "left_hip_yaw".into(),
            physical_id: 7,
        }];
        let (resolved, ids, _) =
            resolve_physical_loop_mapping(&virtual_bindings, &physical, &["left_hip_yaw".into()])
                .unwrap();
        assert_eq!(ids, [7]);
        let calibration = loop_body::Calibration {
            body_id: "official".into(),
            body_version: 1,
            joints: resolved
                .iter()
                .map(|binding| loop_body::JointCalibration {
                    slot_id: binding.slot_id.clone(),
                    physical_id: binding.physical_id,
                    direction: binding.direction,
                    zero_offset_rad: binding.zero_offset_rad,
                    min_rad: binding.min_rad,
                    max_rad: binding.max_rad,
                })
                .collect(),
        };
        assert!((calibration.to_body_positions(&[0.7])[0] + 0.2).abs() < 1e-12);
        assert_eq!(calibration.to_body_velocities(&[0.4]), [-0.4]);
        assert!((calibration.to_servo_positions(&[-0.2])[0] - 0.7).abs() < 1e-12);
    }

    #[test]
    fn merged_mapping_uses_virtual_ids_and_physical_direction() {
        let mut physical = PhysicalMachine {
            id: "physical-merge".into(),
            name: "merge".into(),
            usb_port: "test".into(),
            machine_code: "test".into(),
            census: vec![
                ServoIdentity {
                    servo_id: 7,
                    model: "hd1910".into(),
                },
                ServoIdentity {
                    servo_id: 8,
                    model: "hd1910".into(),
                },
            ],
            reconnect_hint: None,
            virtual_machine_id: Some("virtual-merge".into()),
            online_servo_ids: vec![7, 8],
            connected: true,
            last_seen: None,
            torque_enabled: false,
            joint_directions: vec![crate::types::ServoJointDirection {
                slot_id: "left_knee".into(),
                direction: -1,
            }],
            // Physical calibration belongs to the limb regardless of its current bus ID.
            joint_bindings: vec![crate::types::ServoJointCalibration {
                slot_id: "left_knee".into(),

                zero_offset_rad: 1.5,
                min_rad: None,
                max_rad: None,
            }],
        };
        let virtual_bindings = vec![crate::types::VirtualJointBinding {
            slot_id: "left_knee".into(),
            physical_id: 7,
        }];
        let (resolved, ids, _) =
            resolve_physical_loop_mapping(&virtual_bindings, &physical, &["left_knee".into()])
                .unwrap();
        assert_eq!(ids, [7]);
        assert_eq!(resolved[0].physical_id, 7);
        assert_eq!(resolved[0].direction, -1);
        assert_eq!(resolved[0].zero_offset_rad, 1.5);
        physical.census.reverse();
        let (reordered, reordered_ids, _) =
            resolve_physical_loop_mapping(&virtual_bindings, &physical, &["left_knee".into()])
                .unwrap();
        assert_eq!(reordered, resolved);
        assert_eq!(reordered_ids, ids);
        // Neither an offline configured ID nor a different online ID implies another limb.
        physical.online_servo_ids = vec![8];
        let (unmatched, _, active) =
            resolve_physical_loop_mapping(&virtual_bindings, &physical, &["left_knee".into()])
                .unwrap();
        assert!(unmatched.is_empty());
        assert_eq!(active, [None]);
    }

    #[test]
    fn custom_ids_keep_calibration_without_requiring_the_official_id_in_census() {
        let census = vec![ServoIdentity {
            servo_id: 7,
            model: "hd1910".into(),
        }];
        let binding = crate::types::VirtualJointBinding {
            slot_id: "left_hip_yaw".into(),
            physical_id: 7,
        };
        let expected = crate::types::ServoJointBinding {
            slot_id: "left_hip_yaw".into(),
            physical_id: 7,
            direction: 1,
            zero_offset_rad: 0.0,
            min_rad: None,
            max_rad: None,
        };
        assert_eq!(
            super::effective_servo_bindings(&census, std::slice::from_ref(&binding)),
            vec![expected]
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
