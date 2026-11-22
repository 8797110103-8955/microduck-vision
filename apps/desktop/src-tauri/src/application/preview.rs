//! Application service for the simulation preview loop.
//!
//! ONNX selection, simulation state and preview diagnostics are coordinated here so the Tauri
//! command layer remains a transport adapter.

use crate::application::policy;
use crate::backend::policy_runtime::PolicyExecution;
use crate::control;
use crate::helpers::model;
use crate::logging::append_control_event;
#[cfg(test)]
use crate::logging::log_joint_range_states;
use crate::state::DesktopState;
use crate::types::{LoopStatus, PreviewInput};
type FrontendLoopStatus = LoopStatus;

pub(crate) fn select_machine(
    state: &DesktopState,
    virtual_machine_id: String,
) -> Result<(), String> {
    let _transition = state
        .execution_transition
        .lock()
        .map_err(|error| error.to_string())?;
    let machine = state
        .virtual_machines
        .lock()
        .map_err(|error| error.to_string())?
        .iter()
        .find(|machine| machine.id == virtual_machine_id)
        .cloned()
        .ok_or_else(|| "unknown virtual machine".to_owned())?;
    if state
        .execution_gate
        .selected()
        .map_err(|error| error.to_string())?
        .is_some_and(|selected| {
            selected.id == machine.id && selected.kind == loop_body::BackendKind::Simulation
        })
        && state
            .backend_registry
            .lock()
            .map_err(|error| error.to_string())?
            .describe(&machine.id)
            .is_some()
    {
        return Ok(());
    }
    control::ensure_backend_available(state, "simulation")?;
    state.ensure_runtime_healthy()?;
    let mut registry = state
        .backend_registry
        .lock()
        .map_err(|error| error.to_string())?;
    crate::backend::simulation::register_machine(
        &mut registry,
        &machine.id,
        model(&machine.servo_model),
        &machine.simulation,
    )
    .map(|_| ())?;
    registry
        .select(&machine.id)
        .map_err(|error| error.to_string())?;
    append_control_event(
        state.config_dir.as_deref(),
        format!(
            "execution_selected backend_id={} backend_kind=simulation physical_write_allowed=false",
            machine.id
        ),
    );
    Ok(())
}

#[cfg(test)]
pub(crate) fn open(
    state: &DesktopState,
    virtual_machine_id: String,
) -> Result<FrontendLoopStatus, String> {
    let _transition = state
        .execution_transition
        .lock()
        .map_err(|error| error.to_string())?;
    open_locked(state, virtual_machine_id)
}

pub(crate) fn open_locked(
    state: &DesktopState,
    virtual_machine_id: String,
) -> Result<FrontendLoopStatus, String> {
    control::ensure_backend_available(state, "simulation")?;
    let machine = state
        .virtual_machines
        .lock()
        .map_err(|e| e.to_string())?
        .iter()
        .find(|machine| machine.id == virtual_machine_id)
        .cloned()
        .ok_or_else(|| "unknown virtual machine".to_string())?;
    if state
        .runtime_loop
        .lock()
        .map_err(|e| e.to_string())?
        .as_ref()
        .is_some_and(crate::state::RuntimeInstance::is_simulation)
    {
        return Err("preview loop is already active".into());
    }
    let preview = open_body(state, &machine)?;
    let status = preview.status();
    state.emit_joint_range_states(&status.joint_range_states);
    let mut runtime = state.runtime_loop.lock().map_err(|e| e.to_string())?;
    let mut instance =
        crate::state::RuntimeInstance::simulation_execution(preview, machine.id.clone());
    let instance = match runtime.take() {
        Some(mut existing) if !existing.has_execution() => {
            while let Some(source) = existing.take_source() {
                instance.attach_source(source)?;
            }
            instance
        }
        Some(existing) => {
            *runtime = Some(existing);
            return Err("preview loop is already active".into());
        }
        None => instance,
    };
    *runtime = Some(instance);
    state.activate_runtime();
    state.start_loop_clock()?;
    Ok(with_runtime_generation(state, status))
}

pub(crate) fn open_body(
    state: &DesktopState,
    machine: &crate::types::VirtualMachine,
) -> Result<PolicyExecution, String> {
    let mut execution = PolicyExecution::open(
        model(&machine.servo_model),
        machine.simulation.clone(),
        state.config_dir.as_deref(),
        &machine.id,
        &machine.name,
        &mut *state
            .backend_registry
            .lock()
            .map_err(|error| error.to_string())?,
    )?;
    execution.restore_placement(state.world_placements.clone());
    Ok(execution)
}

pub(crate) fn select_policy(
    state: &DesktopState,
    policy_id: String,
    input: PreviewInput,
) -> Result<FrontendLoopStatus, String> {
    let selected = policy::list(state.config_dir.as_deref())?
        .into_iter()
        .find(|entry| entry.id == policy_id && entry.cached && entry.selected)
        .ok_or_else(|| "selected ONNX policy is not downloaded".to_owned())?;
    let mut guard = state.runtime_loop.lock().map_err(|e| e.to_string())?;
    let preview = guard
        .as_mut()
        .ok_or_else(|| "preview loop is closed".to_owned())?
        .policy_execution_mut()?;
    preview.select_policy(&selected.id)?;
    // Policy and its command become visible to the clock together. No tick may
    // run the new network with the previous policy's input (or the startup zero).
    preview.set_input(input);
    guard
        .as_mut()
        .ok_or("runtime is closed")?
        .set_controller_source()?;
    state
        .config_revision
        .fetch_add(1, std::sync::atomic::Ordering::AcqRel);
    state
        .execution_generation
        .fetch_add(1, std::sync::atomic::Ordering::AcqRel);
    let status = guard.as_ref().ok_or("runtime is closed")?.snapshot();
    state.emit_joint_range_states(&status.joint_range_states);
    Ok(with_runtime_generation(state, status))
}

pub(crate) fn set_input(
    state: &DesktopState,
    input: PreviewInput,
) -> Result<FrontendLoopStatus, String> {
    let mut guard = state.runtime_loop.lock().map_err(|e| e.to_string())?;
    let preview = guard
        .as_mut()
        .ok_or_else(|| "preview loop is closed".to_owned())?
        .policy_execution_mut()?;
    preview.set_input(input);
    let status = guard.as_ref().ok_or("runtime is closed")?.snapshot();
    state.emit_joint_range_states(&status.joint_range_states);
    Ok(with_runtime_generation(state, status))
}

pub(crate) fn reset_world(state: &DesktopState) -> Result<FrontendLoopStatus, String> {
    let mut guard = state.runtime_loop.lock().map_err(|e| e.to_string())?;
    let preview = guard
        .as_mut()
        .ok_or_else(|| "preview loop is closed".to_owned())?
        .policy_execution_mut()?;
    preview.reset_world()?;
    state
        .config_revision
        .fetch_add(1, std::sync::atomic::Ordering::AcqRel);
    let status = guard.as_ref().ok_or("runtime is closed")?.snapshot();
    state.emit_joint_range_states(&status.joint_range_states);
    Ok(with_runtime_generation(state, status))
}

#[cfg(test)]
pub(crate) fn tick(state: &DesktopState) -> Result<FrontendLoopStatus, String> {
    let mut guard = state.runtime_loop.lock().map_err(|e| e.to_string())?;
    let status = guard
        .as_mut()
        .ok_or_else(|| "preview loop is closed".to_owned())?
        .policy_execution_mut()?
        .tick()?;
    state.emit_joint_range_states(&status.joint_range_states);
    log_joint_range_states(state.config_dir.as_deref(), &status.joint_range_states);
    Ok(status)
}

pub(crate) fn action(
    state: &DesktopState,
    action: loop_body::LoopAction,
) -> Result<FrontendLoopStatus, String> {
    let mut runtime = state.runtime_loop.lock().map_err(|e| e.to_string())?;
    let preview = runtime
        .as_mut()
        .ok_or("runtime is closed")?
        .policy_execution_mut()?;
    preview.action(action)?;
    let status = runtime.as_ref().ok_or("runtime is closed")?.snapshot();
    Ok(with_runtime_generation(state, status))
}

#[cfg(test)]
pub(crate) fn close(state: &DesktopState) -> Result<(), String> {
    super::loop_service::close(
        state,
        state.current_runtime_generation(),
        crate::types::RuntimeCloseScope::Runtime,
    )
}

pub(crate) fn with_runtime_generation(
    state: &DesktopState,
    mut status: FrontendLoopStatus,
) -> FrontendLoopStatus {
    status.runtime_generation = state.current_runtime_generation();
    status.execution_generation = state.current_execution_generation();
    status.config_revision = state.current_config_revision();
    status
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn closing_an_already_closed_preview_is_idempotent() {
        let state = DesktopState::new(None);
        close(&state).expect("preview close should be safe when no loop is open");
        assert!(state.runtime_loop.lock().expect("preview lock").is_none());
    }
    #[test]
    fn selecting_an_idle_simulation_registers_a_provider_and_blocks_hardware_writes() {
        let state = DesktopState::default();
        let id = state.virtual_machines.lock().unwrap()[0].id.clone();
        select_machine(&state, id.clone()).unwrap();
        assert!(state.runtime_loop.lock().unwrap().is_none());
        assert_eq!(state.backend_registry.lock().unwrap().list().count(), 1);
        assert!(state.execution_gate.physical_write().is_err());
        select_machine(&state, id.clone()).unwrap();
        open(&state, id.clone()).unwrap();
        select_machine(&state, id).unwrap();
        assert_eq!(tick(&state).unwrap().tick, 1);
        close(&state).unwrap();
        assert!(state.execution_gate.physical_write().is_err());
    }
    #[test]
    fn simulation_and_physical_sensor_loop_coexist_and_direct_writes_are_rejected() {
        use crate::application::{debug_machine, preview, registers};
        use crate::control::tests::{ReadOnlyDriver, read_only_loop_with_driver};
        use crate::types::{PhysicalMachine, PhysicalMachineRegisterWriteInput, ServoIdentity};
        use std::sync::{Arc, Mutex};
        let state = DesktopState::default();
        let machine_id = state.virtual_machines.lock().unwrap()[0].id.clone();
        let events = Arc::new(Mutex::new(Vec::new()));
        let driver_events = events.clone();
        let physical_positions = vec![0.123; 15];
        let session = {
            let mut registry = state.backend_registry.lock().unwrap();
            registry
                .register(
                    loop_body::BackendDescriptor {
                        id: "physical-test".into(),
                        kind: loop_body::BackendKind::Physical,
                        supports_control: true,
                        contract: "test".into(),
                    },
                    move || {
                        Ok(Box::new(ReadOnlyDriver {
                            positions: vec![0.123; 15],
                            events: driver_events.clone(),
                        }))
                    },
                )
                .unwrap();
            registry
                .open("physical-test", loop_body::BackendAccess::ReadOnly)
                .unwrap()
        };
        *state.runtime_loop.lock().unwrap() = Some(crate::state::RuntimeInstance::read_only(
            read_only_loop_with_driver(Box::new(session)),
        ));
        control::record_backend_open(&state, "physical").unwrap();
        state
            .physical_machines
            .lock()
            .unwrap()
            .push(PhysicalMachine {
                id: "physical-test".into(),
                name: "test".into(),
                usb_port: "test".into(),
                machine_code: "test".into(),
                census: vec![ServoIdentity {
                    servo_id: 1,
                    model: "hd1910".into(),
                }],
                reconnect_hint: None,
                virtual_machine_id: Some(machine_id.clone()),
                online_servo_ids: vec![1],
                connected: true,
                last_seen: None,
                torque_enabled: false,
                joint_directions: Vec::new(),
                joint_bindings: vec![],
            });

        preview::open(&state, machine_id.clone()).unwrap();
        let initial = preview::tick(&state).unwrap();
        assert_eq!(initial.tick, 1);
        assert_eq!(control::tick(&state).unwrap().positions, physical_positions);
        assert_ne!(initial.positions, physical_positions);
        assert!(
            preview::open(&state, machine_id)
                .unwrap_err()
                .contains("loop is active")
        );

        for enabled in [true, false] {
            assert!(
                debug_machine::set_torque_state(&state, "physical-test".into(), enabled, None)
                    .unwrap_err()
                    .contains("physical writes are disabled")
            );
        }
        assert!(
            debug_machine::write_servo_position(&state, "physical-test".into(), 1, 0.5)
                .unwrap_err()
                .contains("physical writes are disabled")
        );
        assert!(
            registers::physical_machine_write_register(
                &state,
                PhysicalMachineRegisterWriteInput {
                    physical_machine_id: "physical-test".into(),
                    servo_id: 1,
                    name: "torque_enable".into(),
                    value: 1,
                }
            )
            .unwrap_err()
            .contains("physical writes are disabled")
        );
        assert!(
            control::action(&state, loop_body::LoopAction::Power(true))
                .unwrap_err()
                .contains("read-only")
        );
        assert_eq!(preview::tick(&state).unwrap().tick, 2);
        control::close(&state).unwrap();
        assert_eq!(*events.lock().unwrap(), vec!["read"]);
        // Closing the independent sensor session cannot release the simulation selection.
        assert!(state.execution_gate.physical_write().is_err());
        let session = state
            .backend_registry
            .lock()
            .unwrap()
            .open("physical-test", loop_body::BackendAccess::ReadOnly)
            .unwrap();
        let physical_read = read_only_loop_with_driver(Box::new(session));
        let mut runtime = state.runtime_loop.lock().unwrap();
        let mut simulation = runtime.take().unwrap();
        simulation.attach_source(physical_read).unwrap();
        *runtime = Some(simulation);
        drop(runtime);
        *state.loop_physical_machine_id.lock().unwrap() = Some("physical-test".into());
        control::record_backend_open(&state, "physical").unwrap();
        // A USB refresh must retain the session's connection without opening another port.
        crate::application::devices::auto_reconnect(&state, "physical-test".into(), 1_000_000)
            .unwrap();
        crate::application::physical_machine::physical_machine_disconnect(
            &state,
            "physical-test".into(),
        )
        .unwrap();
        assert!(matches!(
            state.runtime_loop.lock().unwrap().as_ref(),
            Some(runtime) if runtime.is_simulation()
        ));
        assert!(
            state
                .backend_registry
                .lock()
                .unwrap()
                .describe("physical-test")
                .is_none()
        );
        assert_eq!(preview::tick(&state).unwrap().tick, 3);
        assert_eq!(*events.lock().unwrap(), vec!["read"]);
        preview::close(&state).unwrap();
        assert!(state.execution_gate.physical_write().is_err());
    }
}
