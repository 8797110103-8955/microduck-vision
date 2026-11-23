//! Application service for opening a control loop.
//!
//! This module owns machine/policy resolution, physical mapping, backend construction, and
//! control-loop lifecycle. Tauri commands only adapt the frontend input and output.

use std::sync::{Arc, Mutex};

#[cfg(test)]
use crate::platform::logging::{append_policy_trace, saturation_reason_for_error};
use crate::runtime::adapters::physical_backend::{PhysicalOpenContext, open_backend_driver};
use crate::runtime::adapters::servo_bus::ConnectedBus;
use crate::runtime::policy::inference::{OpenLoopInput, open_loop};
#[cfg(test)]
use crate::types::WorldState;
use duck_control::obs::{BodyPose, Command};
#[cfg(test)]
use loop_body::BackendState;
use loop_body::LoopConfig;
use microduck_body_runtime::PhysicalGains;

use loop_body::{
    BodyRuntimeView, Calibration, ControlBackend, JointCalibration, LoopBinding, ReadOnlySource,
    ServoBinding,
};

use crate::platform::logging::{append_control_event, begin_control_loop, log_joint_range_states};
use crate::runtime::adapters::logged_body::LoggedBody;
use crate::runtime::machines::machine_resolution::{
    binding_conflict_error, resolve_physical_loop_mapping,
};
use crate::runtime::policy::catalog as policy;
use crate::runtime::session::contract::{
    PhysicalExecutionSources, ResolvedContract, SlotExecutionSource, SlotSource,
    register_prepared_simulation, simulation_contract,
};
use crate::runtime::session::state::DesktopState;
use crate::types::{LoopOpenInput, LoopStatus as FrontendLoopStatus, PhysicalMachine};

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
        return status(state)?.ok_or_else(|| "physical source is unavailable".into());
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
            crate::runtime::session::preview::open_locked(state, input.virtual_machine_id)
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
            if execution.has_training_policy() {
                return Err("请选择官方策略，再切换到真机".into());
            }
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
            return Ok(crate::runtime::session::preview::with_runtime_generation(
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
        let body = prepare_physical_body(
            state,
            &virtual_machine_id,
            id,
            loop_body::BackendAccess::Control,
            None,
        )?;
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
            restore_shared_physical_bus(state, &id)?;
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
            let contract = body.contract.clone();
            let sources = body.sources.clone();
            let loop_config = body.loop_config();
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
            let mut runner = open_loop(OpenLoopInput {
                loop_config,
                binding,
                driver,
                policy: None,
                config_dir: state.config_dir.clone(),
                backend_kind: "physical",
                backend_profile: profile.clone(),
                contract_hash: contract.hash.clone(),
            })?;
            runner.sample_body().map_err(|e| e.to_string())?;
            execution.switch_to_physical(
                id,
                virtual_machine_id.clone(),
                runner,
                contract,
                sources,
            )?;
            record_backend_open(state, "physical")?;
            execution.action(loop_body::LoopAction::Power(true))?;
        } else {
            let next = crate::runtime::session::preview::open_body(state, &machine)?;
            execution.switch_to_simulation(next)?;
            if let Some(id) = &old_physical_id {
                restore_shared_physical_bus(state, id)?;
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
                    let _ = restore_shared_physical_bus(state, &id);
                }
                crate::runtime::session::preview::open_body(state, &machine)
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
    Ok(crate::runtime::session::preview::with_runtime_generation(
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
        crate::runtime::machines::encoder_calibration::ensure_ready(state, &id)?;
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
            crate::runtime::session::preview::set_input(state, input)
        }
        RuntimeCommand::Input { input } => {
            set_policy_command(
                state,
                PolicyCommandInput {
                    twist: input.twist,
                    head: input.head,
                    body: input.body,
                },
            )?;
            self::status(state)?.ok_or_else(|| "runtime is closed".into())
        }
        RuntimeCommand::Targets { targets } if !simulation && !managed_policy => {
            set_targets(state, targets)?;
            self::status(state)?.ok_or_else(|| "runtime is closed".into())
        }
        RuntimeCommand::Policy { policy_id, input } if managed_policy => {
            crate::runtime::session::preview::select_policy(state, policy_id, input)
        }
        RuntimeCommand::ResetWorld if simulation => {
            crate::runtime::session::preview::reset_world(state)
        }
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
                crate::runtime::session::preview::action(state, action)
            } else {
                self::action(state, action)
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
        let result = close_backend_locked(state);
        state
            .config_revision
            .fetch_add(1, std::sync::atomic::Ordering::AcqRel);
        return result;
    }
    // Release any physical attachment first, then stop the remaining execution. Sensor
    // detachment may preserve simulation for debug consumers, but runtime close never does.
    close_backend_locked(state)?;
    // A first pass may detach a source while preserving simulation. Full runtime shutdown
    // still acknowledges the execution stop before dropping the remaining owner.
    if state
        .runtime_loop
        .lock()
        .map_err(|e| e.to_string())?
        .is_some()
    {
        close_backend_locked(state)?;
    }
    state.activate_runtime();
    Ok(())
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
    crate::runtime::policy::inference::PolicyConfig,
    crate::runtime::policy::inference::PolicyDescriptor,
);

struct PhysicalBody {
    virtual_id: String,
    physical: PhysicalMachine,
    binding: LoopBinding,
    model: servo::ServoModel,
    sources: PhysicalExecutionSources,
    gains: PhysicalGains,
    mapped_active_ids: Vec<Option<u8>>,
    binding_summary: String,
    contract: ResolvedContract,
    policy_sha256: Option<String>,
}

fn prepare_physical_body(
    state: &DesktopState,
    virtual_machine_id: &str,
    requested_physical_id: &str,
    access: loop_body::BackendAccess,
    policy_path: Option<&PolicyLoad>,
) -> Result<PhysicalBody, String> {
    let machine =
        crate::runtime::machines::machine_resolution::virtual_machine(state, virtual_machine_id)?;
    let body = crate::runtime::machines::machine_resolution::body_profile(&machine.body_id)?;
    let template = body.template().clone();
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
    let selection = crate::runtime::machines::physical_machine::physical_selection(&physical)?;
    let model = selection.servo_model;
    let profile = microduck_body_runtime::resolve_physical(selection, body)?;
    let gains = profile.physical_gains.ok_or("missing physical gains")?;
    let backend_kind = "physical";
    let mut contract_sources = crate::runtime::adapters::simulation::official_sources()?;
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
    let policy_sha256 = if let Some((path, config, descriptor)) = policy_path {
        use sha2::{Digest, Sha256};

        let bytes = std::fs::read(path).map_err(|e| e.to_string())?;
        let sha256 = format!("{:x}", Sha256::digest(&bytes));
        contract_sources.push(("policy/onnx".into(), bytes));
        contract_sources.push((
            "policy/resolved".into(),
            format!("{config:?};{descriptor:?}").into_bytes(),
        ));
        Some(sha256)
    } else {
        None
    };
    let slot_ids = template
        .actuators
        .iter()
        .map(|slot| slot.id.clone())
        .collect::<Vec<_>>();
    let (saved_bindings, bound_servo_ids, mapped_active_ids) =
        resolve_physical_loop_mapping(&machine.joint_bindings, &physical, &slot_ids)?;
    crate::platform::logging::diagnostic(
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
    let fallback_simulation = if access == loop_body::BackendAccess::Control {
        let selection =
            crate::runtime::machines::machine_resolution::simulation_selection(&machine)?;
        Some(crate::runtime::adapters::simulation::resolve(
            selection,
            &machine.simulation,
            &template,
        )?)
    } else {
        None
    };
    let sources = PhysicalExecutionSources {
        physical: profile,
        slots: slot_ids
            .into_iter()
            .zip(&mapped_active_ids)
            .map(|(slot_id, &physical_id)| {
                let source = match physical_id {
                    Some(_) => SlotSource::Physical,
                    None if fallback_simulation.is_some() => SlotSource::Simulation,
                    None => SlotSource::Unavailable,
                };
                SlotExecutionSource {
                    slot_id,
                    physical_id,
                    source,
                }
            })
            .collect(),
        fallback_simulation,
    };
    contract_sources.push((
        "physical/execution-sources".into(),
        serde_json::to_vec(&sources).map_err(|error| error.to_string())?,
    ));
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
    Ok(PhysicalBody {
        virtual_id: machine.id.clone(),
        physical,
        binding,
        model,
        sources,
        gains,
        mapped_active_ids,
        binding_summary,
        contract: resolved_contract,
        policy_sha256,
    })
}

type OpenedPhysicalBody = (
    LoopBinding,
    Box<dyn ControlBackend>,
    Option<Arc<std::sync::Mutex<crate::runtime::adapters::servo_bus::ConnectedBus>>>,
);

impl PhysicalBody {
    fn loop_config(&self) -> LoopConfig {
        crate::runtime::adapters::physical_backend::physical_loop_config(
            &self.sources.physical,
            self.gains,
        )
    }

    fn recording_context(
        &self,
        policy: Option<&PolicyLoad>,
        period: std::time::Duration,
    ) -> serde_json::Value {
        let descriptor = policy.map(|(_, _, descriptor)| descriptor);
        serde_json::json!({
            "body_id":self.contract.body_id,
            "body_version":self.contract.body_version,
            "backend_profile":self.contract.backend_profile,
            "backend_contract":self.contract.hash,
            "physical_execution":self.sources,
            "policy_id":descriptor.map(|policy| &policy.id),
            "policy_contract":descriptor,
            "policy_sha256":self.policy_sha256,
            "policy_config":policy.map(|(_, config, _)| format!("{config:?}")),
            "control":if descriptor.is_some() { "policy" } else { "manual" },
            "action_space":if descriptor.is_some() { "raw_policy" } else { "canonical_joint_targets" },
            "observation_len":descriptor.map(|policy| policy.obs_len),
            "action_len":descriptor.map(|policy| policy.action_len),
            "control_period_s":period.as_secs_f64(),
            "joint_units":"rad",
            "joint_order":duck_control::model::JOINT_NAMES,
        })
    }

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
        crate::runtime::machines::physical_io::with_bus_locked(
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
            sources,
            gains,
            mapped_active_ids,
            binding_summary,
            contract,
            policy_sha256: _,
        } = self;
        // Unmatched limbs keep a MuJoCo actuator. This simulation has its own lease;
        // it must never select/revoke the application's physical execution owner.
        let fallback = if let Some(profile) = sources.fallback_simulation {
            let mut registry = loop_body::BackendRegistry::default();
            let contract = simulation_contract(&profile)?;
            register_prepared_simulation(
                &mut registry,
                "unmatched-limbs",
                binding.body.template(),
                &profile,
                &contract,
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
        if access == loop_body::BackendAccess::Control {
            state.ensure_encoder_calibration_ready(&physical.id)?;
        }
        let opened_driver = open_backend_driver(
            PhysicalOpenContext {
                registry: &state.backend_registry,
                buses: &state.connected_buses,
                config_dir: state.config_dir.as_deref(),
            },
            log_machine_id,
            gains,
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
            let logger = LoggedBody::new(opened_driver.driver, handle, log_machine_id.to_owned());
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
            let mut driver = crate::runtime::adapters::read_only_body::ReadOnlyBody::new(
                driver,
                raw_ids,
                move || {
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
                            if crate::runtime::machines::encoder_calibration::is_pending(
                                &pending,
                                &physical_id,
                                slot,
                            ) {
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
                },
            )
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
            Box::new(crate::runtime::adapters::hd1910_body::Hd1910Body::new(
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
                Box::new(crate::runtime::adapters::attached_body::AttachedBody::new(
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
        if policy::is_training_policy(&policy_id) {
            return Err("本地训练策略需要模拟机预览".into());
        }
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
    ensure_backend_access(
        state,
        "physical",
        access == loop_body::BackendAccess::Control,
    )?;
    let body = prepare_physical_body(
        state,
        &virtual_machine_id,
        &requested_physical_id,
        access,
        policy_path.as_ref(),
    )?;
    let physical_id = body.physical.id.clone();
    let binding_summary = body.binding_summary.clone();
    let backend_kind = "physical";
    let backend_profile = format!("{:?}", body.model);
    let contract_hash = body.contract.hash.clone();
    let config = body.loop_config();
    let recording_context = (access == loop_body::BackendAccess::Control)
        .then(|| body.recording_context(policy_path.as_ref(), config.period));
    let (binding, driver, mut debug_shared_bus) = body.open(state, access)?;
    let (runtime_instance, mut status) = if access == loop_body::BackendAccess::Control {
        let body_loop = match open_loop(OpenLoopInput {
            loop_config: config,
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
            crate::runtime::session::state::RuntimeInstance::physical_execution(
                body_loop,
                physical_id.clone(),
            )
            .with_physical_recording_context(recording_context.expect("physical control context")),
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
        crate::platform::logging::diagnostic(
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
            phase: crate::platform::state::types::LoopPhase::Holding,
            session: 0,
            tick: 0,
            torque: false,
            positions: Vec::new(),
            targets: Vec::new(),
            world: crate::platform::state::types::WorldState {
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
        (
            crate::runtime::session::state::RuntimeInstance::read_only(source),
            status,
        )
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
    record_backend_open(state, backend_kind)?;
    self::status(state)?.ok_or_else(|| "runtime is closed".into())
}

#[cfg(test)]
mod tests {
    use super::open;
    use super::resolve_physical_loop_mapping;
    use crate::runtime::session::state::DesktopState;
    use crate::types::{LoopOpenInput, PhysicalMachine, ServoIdentity};

    #[test]
    fn hd1910_physical_control_resolves_native_and_compatible_fallback_independently() {
        use crate::types::{ModelKindKey, PhysicalMachineSaveInput};
        use loop_body::BackendAccess;
        use microduck_body_runtime::{ExecutionKind, ModelKind};

        let state = DesktopState::new(None);
        let virtual_id = state.virtual_machines.lock().unwrap()[0].id.clone();
        let first_id = duck_control::model::JOINT_IDS[0];
        let physical = crate::runtime::machines::physical_machine::physical_machine_save(
            &state,
            PhysicalMachineSaveInput {
                name: Some("partial HD".into()),
                census: vec![ServoIdentity {
                    servo_id: first_id,
                    model: "hd1910".into(),
                }],
                reconnect_hint: None,
            },
        )
        .unwrap();
        let body = super::prepare_physical_body(
            &state,
            &virtual_id,
            &physical.id,
            BackendAccess::Control,
            None,
        )
        .unwrap();
        let metadata = body.recording_context(None, body.loop_config().period);
        assert_eq!(
            body.sources.physical.execution_kind,
            ExecutionKind::Physical
        );
        assert_eq!(body.sources.physical.model_kind, ModelKind::Hd1910);
        let fallback = body.sources.fallback_simulation.as_ref().unwrap();
        assert_eq!(fallback.execution_kind, ExecutionKind::Simulation);
        assert_eq!(fallback.model_kind, ModelKind::Hd1910Compatible);
        assert_eq!(body.loop_config().gain, 32);
        assert_ne!(
            body.sources.physical.execution_fingerprint,
            fallback.execution_fingerprint
        );
        assert_eq!(
            metadata["physical_execution"]["slots"][0]["source"],
            "physical"
        );
        assert_eq!(
            metadata["physical_execution"]["slots"][0]["physical_id"],
            first_id
        );
        assert_eq!(
            metadata["physical_execution"]["slots"][1]["source"],
            "simulation"
        );

        let sensing = super::prepare_physical_body(
            &state,
            &virtual_id,
            &physical.id,
            BackendAccess::ReadOnly,
            None,
        )
        .unwrap();
        assert!(sensing.sources.fallback_simulation.is_none());
        assert!(matches!(
            sensing.sources.slots[1].source,
            super::SlotSource::Unavailable
        ));

        state.physical_machines.lock().unwrap()[0].model_kind =
            Some(ModelKindKey::Hd1910Compatible);
        let error = super::open(
            &state,
            LoopOpenInput {
                virtual_machine_id: virtual_id,
                physical_machine_id: physical.id.clone(),
                access: crate::types::LoopAccess::Control,
                policy_id: None,
            },
        )
        .unwrap_err();
        assert_eq!(
            error,
            microduck_body_runtime::Selection {
                execution_kind: ExecutionKind::Physical,
                model_kind: ModelKind::Hd1910Compatible,
                servo_model: servo::ServoModel::Hd1910,
            }
            .validate()
            .unwrap_err()
        );
        assert!(state.execution_gate.selected().unwrap().is_none());
        let write = crate::runtime::machines::physical_io::with_bus(
            &state,
            &physical.id,
            BackendAccess::Control,
            |_| -> Result<(), String> { panic!("unselected hardware must not receive writes") },
        );
        assert!(write.is_err());
    }

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
                    Ok(Box::new(
                        crate::runtime::session::loop_service::control_tests::ReadOnlyDriver {
                            positions: vec![0.25; 15],
                            events: Arc::clone(&fake_events),
                        },
                    ))
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
        let template = crate::runtime::machines::body_catalog::official_template().unwrap();
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
        let mut next = crate::runtime::policy::inference::MicroDuckPolicyRunner::open(
            binding,
            Box::new(driver),
            loop_body::LoopConfig::default(),
        )
        .unwrap();
        next.sample_body().unwrap();
        let body = crate::runtime::adapters::body_assets::test_profile();
        let profile = crate::runtime::machines::machine_resolution::physical_profile(
            &body,
            servo::ServoModel::Hd1910,
        )
        .unwrap();
        let mut contract = super::ResolvedContract::new("official", 1, "none", "physical", "test");
        contract.hash = "test".into();
        runner
            .switch_to_physical(
                "test-physical".into(),
                virtual_id.clone(),
                next,
                contract,
                super::PhysicalExecutionSources {
                    physical: profile,
                    fallback_simulation: None,
                    slots: vec![],
                },
            )
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
        let first = super::status(&state).unwrap().unwrap();
        let second = super::status(&state).unwrap().unwrap();
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
            super::status(&state).unwrap().unwrap().runtime_generation,
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
        let fault = super::status(&state).unwrap().unwrap();
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
        assert!(super::status(&state).unwrap().is_none());
        super::start(&state, input()).unwrap();
    }

    #[test]
    fn detaching_source_preserves_simulation_but_runtime_close_stops_both() {
        use crate::runtime::session::loop_service::control_tests::{
            ReadOnlyDriver, read_only_loop_with_driver,
        };
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
        let snapshot = super::status(&state).unwrap().unwrap();
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
            execution_kind: crate::types::ExecutionKindKey::Physical,
            model_kind: Some(crate::types::ModelKindKey::Hd1910),
            servo_model: Some(crate::types::ServoModelKey::Hd1910),
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
        let template = crate::runtime::machines::body_catalog::official_template()
            .expect("official body template");
        let slots = template
            .actuators
            .iter()
            .map(|slot| slot.id.clone())
            .collect::<Vec<_>>();
        let physical = PhysicalMachine {
            execution_kind: crate::types::ExecutionKindKey::Physical,
            model_kind: Some(crate::types::ModelKindKey::Hd1910),
            servo_model: Some(crate::types::ServoModelKey::Hd1910),
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
        let slots = crate::runtime::machines::body_catalog::official_template()
            .unwrap()
            .actuators;
        state.virtual_machines.lock().unwrap()[0].joint_bindings = slots
            .iter()
            .enumerate()
            .map(|(index, slot)| crate::types::VirtualJointBinding {
                slot_id: slot.id.clone(),
                physical_id: (index + 1) as u8,
            })
            .collect();
        let physical = PhysicalMachine {
            execution_kind: crate::types::ExecutionKindKey::Physical,
            model_kind: Some(crate::types::ModelKindKey::Hd1910),
            servo_model: Some(crate::types::ServoModelKey::Hd1910),
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
        let scene = crate::runtime::machines::body_catalog::official_scene().unwrap();
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
            crate::runtime::machines::binding::physical_machine_bind(
                &state,
                crate::types::PhysicalMachineBindingInput {
                    physical_machine_id: "knee-test".into(),

                    slot_id: "left_knee".into(),

                    direction,
                },
            )
            .unwrap();
            let body = super::prepare_physical_body(
                &state,
                &virtual_id,
                "knee-test",
                loop_body::BackendAccess::ReadOnly,
                None,
            )
            .unwrap();
            let mut source = loop_body::ReadOnlySource::open(
                body.binding,
                Box::new(
                    crate::runtime::session::loop_service::control_tests::ReadOnlyDriver {
                        positions: raw.clone(),
                        events: events.clone(),
                    },
                ),
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
        let body = super::prepare_physical_body(
            &state,
            &virtual_id,
            "knee-test",
            loop_body::BackendAccess::ReadOnly,
            None,
        )
        .unwrap();
        let execution = crate::runtime::policy::inference::MicroDuckPolicyRunner::open(
            body.binding,
            Box::new(
                crate::runtime::session::loop_service::control_tests::ReadOnlyDriver {
                    positions: raw,
                    events: events.clone(),
                },
            ),
            loop_body::LoopConfig::default(),
        )
        .unwrap();
        *state.runtime_loop.lock().unwrap() = Some(
            crate::runtime::session::state::RuntimeInstance::physical_execution(
                execution,
                "knee-test".into(),
            ),
        );
        *state.loop_physical_machine_id.lock().unwrap() = Some("knee-test".into());
        let rejected = crate::runtime::machines::binding::physical_machine_bind(
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
            execution_kind: crate::types::ExecutionKindKey::Physical,
            model_kind: Some(crate::types::ModelKindKey::Hd1910),
            servo_model: Some(crate::types::ServoModelKey::Hd1910),
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
            execution_kind: crate::types::ExecutionKindKey::Physical,
            model_kind: Some(crate::types::ModelKindKey::Hd1910),
            servo_model: Some(crate::types::ServoModelKey::Hd1910),
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
            crate::runtime::machines::machine_resolution::effective_servo_bindings(
                &census,
                std::slice::from_ref(&binding)
            ),
            vec![expected]
        );
    }
    #[test]
    fn faulted_runtime_blocks_commands_until_a_successful_close() {
        let state = DesktopState::default();
        let id = state.virtual_machines.lock().unwrap()[0].id.clone();
        let opened = crate::runtime::session::loop_service::start(
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
        let failed = super::status(&state).unwrap().unwrap();
        assert_eq!(failed.runtime_state, crate::types::RuntimeState::Faulted);
        assert!(failed.last_error.unwrap().contains("revoked"));
        assert!(state.execution_gate.physical_write().is_err());
        assert!(
            crate::runtime::session::loop_service::command(
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
            crate::runtime::session::loop_service::close(
                &state,
                opened.runtime_generation,
                crate::types::RuntimeCloseScope::Runtime
            )
            .is_err()
        );
        assert!(state.runtime_loop.lock().unwrap().is_some());
    }
}

#[derive(Debug, Clone, Copy)]
pub(crate) struct PolicyCommandInput {
    pub twist: [f64; 3],
    pub head: [f64; 4],
    pub body: [f64; 3],
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
        active_body: None,
        runtime_state: crate::types::RuntimeState::Running,
        simulation: None,
        read_sources: vec![source.into()],
        phase: crate::platform::state::types::LoopPhase::Holding,
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
            .is_some_and(crate::runtime::session::state::RuntimeInstance::is_simulation);
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

pub(crate) fn status(state: &DesktopState) -> Result<Option<FrontendLoopStatus>, String> {
    let runtime = state.runtime_loop.lock().map_err(|e| e.to_string())?;
    let mut status = runtime
        .as_ref()
        .map(crate::runtime::session::state::RuntimeInstance::snapshot);
    if status.is_none()
        && let Some(error) = state
            .runtime_fault
            .lock()
            .map_err(|e| e.to_string())?
            .clone()
    {
        status = Some(FrontendLoopStatus {
            has_execution: false,
            active_body: None,
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
            .is_some_and(crate::runtime::session::state::RuntimeInstance::is_simulation)
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
    let mut status = loop_state.snapshot();
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

pub(crate) fn close_on_exit(state: &DesktopState) -> Result<(), String> {
    let _transition = state
        .execution_transition
        .lock()
        .map_err(|e| e.to_string())?;
    close_backend_locked(state)
}

pub(crate) fn close_backend_locked(state: &DesktopState) -> Result<(), String> {
    append_control_event(state.config_dir.as_deref(), "loop_close");
    state.stop_loop_clock()?;
    let mut guard = state.runtime_loop.lock().map_err(|e| e.to_string())?;
    let source_only = guard
        .as_ref()
        .is_some_and(|runtime| runtime.is_simulation() && runtime.read_source().is_some());
    if !source_only {
        state.recorder.stop("runtime_closed")?;
    }
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
    *state
        .loop_virtual_machine_id
        .lock()
        .map_err(|e| e.to_string())? = None;
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
        if let Some(instance) = runtime.as_mut()
            && let Ok(execution) = instance.policy_execution_mut()
            && execution.physical_id() == Some(id)
        {
            let stop = execution.stop();
            state
                .execution_gate
                .set_physical_writes_enabled(false)
                .map_err(|e| e.to_string())?;
            let error = match &stop {
                Ok(()) => "真机已断开，可切回模拟机".to_owned(),
                Err(error) => format!("无法确认真机扭矩已关闭：{error}"),
            };
            let mut fault = state.runtime_fault.lock().map_err(|e| e.to_string())?;
            *fault = Some(match fault.as_ref() {
                Some(original) => format!("{original}; {error}"),
                None => error,
            });
            drop(fault);
            state
                .execution_generation
                .fetch_add(1, std::sync::atomic::Ordering::AcqRel);
            state
                .config_revision
                .fetch_add(1, std::sync::atomic::Ordering::AcqRel);
            stop?;
            execution.mark_detached();
        }
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
            .is_some_and(crate::runtime::session::state::RuntimeInstance::is_simulation)
        {
            if let Some(instance) = runtime.as_mut() {
                instance.take_source();
            }
        } else if !runtime
            .as_ref()
            .is_some_and(crate::runtime::session::state::RuntimeInstance::has_policy_execution)
        {
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
pub(crate) mod control_tests {
    use std::sync::{Arc, Mutex};
    use std::time::Duration;

    use loop_body::{BackendJointSample, BackendPositionTargets, ControlBackend};
    use loop_body::{
        BodyRuntimeView, Calibration, JointCalibration, LoopBinding, LoopConfig, ReadOnlySource,
        ServoBinding,
    };
    use servo::{ServoError, ServoModel};

    use super::{ensure_backend_available, record_backend_open};
    use crate::runtime::policy::inference::{RuntimeTiming, runtime_timing};
    use crate::runtime::session::state::DesktopState;

    pub(crate) struct ReadOnlyDriver {
        pub(crate) positions: Vec<f64>,
        pub(crate) events: Arc<Mutex<Vec<&'static str>>>,
    }

    impl ControlBackend for ReadOnlyDriver {
        fn read_positions(&mut self, _ids: &[u8]) -> Result<BackendJointSample, ServoError> {
            self.events.lock().unwrap().push("read");
            Ok(BackendJointSample {
                positions: self.positions.clone(),
                velocities: vec![0.0; self.positions.len()],
            })
        }

        fn write_positions(
            &mut self,
            _ids: &[u8],
            _targets: &BackendPositionTargets,
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
            positions: crate::runtime::adapters::body_assets::test_template()
                .control
                .home_pose,
            events,
        }))
    }

    pub(crate) fn read_only_loop_with_driver(
        driver: Box<dyn loop_body::ControlBackend>,
    ) -> ReadOnlySource {
        let body = crate::runtime::adapters::body_assets::test_template();
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
        *state.runtime_loop.lock().unwrap() =
            Some(crate::runtime::session::state::RuntimeInstance::read_only(
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

    #[test]
    fn physical_loop_uses_the_readback_hd1910_runtime_gain_while_simulation_keeps_its_loop_default()
    {
        let body = crate::runtime::adapters::body_assets::test_profile();
        let profile = crate::runtime::machines::machine_resolution::physical_profile(
            &body,
            servo::ServoModel::Hd1910,
        )
        .unwrap();
        let config = crate::runtime::adapters::physical_backend::physical_loop_config(
            &profile,
            profile.physical_gains.unwrap(),
        );
        assert_eq!(config.gain, 32);
        assert_eq!(
            crate::runtime::session::execution::simulation_loop_config().gain,
            LoopConfig::default().gain
        );
    }

    #[test]
    fn xl330_control_uses_the_native_position_controller_defaults() {
        let body = crate::runtime::adapters::body_assets::test_profile();
        let profile = crate::runtime::machines::machine_resolution::physical_profile(
            &body,
            servo::ServoModel::Xl330M288,
        )
        .unwrap();
        let gains = profile.physical_gains.unwrap();
        assert_eq!(
            gains,
            microduck_body_runtime::PhysicalGains::Xl330 { p: 200, i: 0, d: 0 }
        );
        let config =
            crate::runtime::adapters::physical_backend::physical_loop_config(&profile, gains);
        assert_eq!(config.gain, 200);
        assert_eq!(config.period, std::time::Duration::from_millis(20));
    }
}
