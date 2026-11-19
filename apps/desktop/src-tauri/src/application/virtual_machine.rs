//! Virtual-machine application service.
//!
//! This service owns persisted virtual-machine state and binding updates. Tauri command files
//! only adapt command arguments and preserve the frontend command names.

use crate::helpers::model_key;
use crate::state::DesktopState;
use crate::types::{VirtualMachine, VirtualMachineInput};

pub(crate) fn virtual_machine_create(
    state: &DesktopState,
    input: VirtualMachineInput,
) -> Result<VirtualMachine, String> {
    let servo_model = model_key(&input.servo_model)?;
    let mut machines = state.virtual_machines.lock().map_err(|e| e.to_string())?;
    let name = if input.name.trim().is_empty() {
        format!("机器 {}", machines.len())
    } else {
        input.name.trim().to_owned()
    };
    let machine = VirtualMachine {
        id: format!("virtual-{}", machines.len() + 1),
        name,
        body_id: input.body_id,
        servo_model,
        simulation: input.simulation,
        startup_selected: false,
        referenced_by: Vec::new(),
        joint_bindings: Vec::new(),
    };
    machines.push(machine.clone());
    drop(machines);
    state.persist()?;
    Ok(machine)
}

pub(crate) fn virtual_machine_set_startup(state: &DesktopState, id: String) -> Result<(), String> {
    let mut machines = state.virtual_machines.lock().map_err(|e| e.to_string())?;
    if !machines.iter().any(|machine| machine.id == id) {
        return Err(format!("unknown virtual machine: {id}"));
    }
    for machine in &mut *machines {
        machine.startup_selected = machine.id == id;
    }
    drop(machines);
    state.persist()?;
    Ok(())
}

pub(crate) fn virtual_machine_set_world_position(
    state: &DesktopState,
    id: String,
    position: [f64; 3],
) -> Result<VirtualMachine, String> {
    if position.iter().any(|value| !value.is_finite()) {
        return Err("世界位置必须是有限数字".into());
    }
    let mut machines = state.virtual_machines.lock().map_err(|e| e.to_string())?;
    let machine = machines
        .iter_mut()
        .find(|machine| machine.id == id)
        .ok_or_else(|| format!("unknown virtual machine: {id}"))?;
    machine.simulation.world_position = position;
    let result = machine.clone();
    drop(machines);
    state.persist()?;
    Ok(result)
}

pub(crate) fn virtual_machine_delete(state: &DesktopState, id: String) -> Result<(), String> {
    let mut machines = state.virtual_machines.lock().map_err(|e| e.to_string())?;
    let Some(machine) = machines.iter().find(|machine| machine.id == id) else {
        return Err(format!("unknown virtual machine: {id}"));
    };
    if machine.startup_selected || !machine.referenced_by.is_empty() {
        return Err("cannot delete selected or referenced virtual machine".into());
    }
    machines.retain(|machine| machine.id != id);
    drop(machines);
    state.persist()?;
    Ok(())
}

pub(crate) fn virtual_machine_rename(
    state: &DesktopState,
    id: String,
    name: String,
) -> Result<VirtualMachine, String> {
    let name = name.trim();
    if name.is_empty() {
        return Err("virtual machine name cannot be empty".into());
    }
    let mut machines = state.virtual_machines.lock().map_err(|e| e.to_string())?;
    let machine = machines
        .iter_mut()
        .find(|machine| machine.id == id)
        .ok_or_else(|| format!("unknown virtual machine: {id}"))?;
    machine.name = name.to_string();
    let result = machine.clone();
    drop(machines);
    state.persist()?;
    Ok(result)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::helpers::default_simulation;

    #[test]
    fn virtual_machine_service_updates_world_position_without_tauri() {
        let state = DesktopState::new(None);
        let machine = virtual_machine_create(
            &state,
            VirtualMachineInput {
                name: "测试机".into(),
                body_id: "official".into(),
                servo_model: "hd1910".into(),
                simulation: default_simulation(duck_body::ServoModelKey::Hd1910),
            },
        )
        .expect("virtual machine should be created");

        let updated = virtual_machine_set_world_position(&state, machine.id, [1.0, 2.0, 0.5])
            .expect("world position should be persisted");
        assert_eq!(updated.simulation.world_position, [1.0, 2.0, 0.5]);
    }

    #[test]
    fn virtual_machine_service_rejects_nonfinite_world_position() {
        let state = DesktopState::new(None);
        let error = virtual_machine_set_world_position(
            &state,
            "official-hd1910".into(),
            [f64::NAN, 0.0, 0.0],
        )
        .expect_err("nonfinite world positions must be rejected");
        assert!(error.contains("有限"));
    }
}
