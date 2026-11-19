//! Binding application service.
//!
//! Body-to-servo binding, duplicate-ID validation, and persisted binding updates live here so
//! physical and virtual command domains share the same rules.

use crate::state::DesktopState;
use crate::types::{
    PhysicalMachine, PhysicalMachineBindingInput, ServoJointBinding, VirtualMachine,
    VirtualMachineBindingInput,
};
pub(crate) fn physical_machine_bind(
    state: &DesktopState,
    input: PhysicalMachineBindingInput,
) -> Result<PhysicalMachine, String> {
    if !duck_control::model::JOINT_NAMES.contains(&input.slot_id.as_str()) {
        return Err(format!("unknown joint slot: {}", input.slot_id));
    }
    if let Some(direction) = input.direction
        && !matches!(direction, -1 | 1)
    {
        return Err(format!("invalid joint direction: {direction}"));
    }
    let physical = state
        .physical_machines
        .lock()
        .map_err(|e| e.to_string())?
        .iter()
        .find(|machine| machine.id == input.physical_machine_id)
        .ok_or_else(|| "unknown physical machine".to_owned())?
        .clone();
    if let Some(physical_id) = input.physical_id
        && !physical
            .census
            .iter()
            .any(|servo| servo.servo_id == physical_id)
    {
        return Err(format!("servo {physical_id} is not in the machine census"));
    }
    let virtual_id = input
        .virtual_machine_id
        .or_else(|| physical.virtual_machine_id.clone());
    let Some(virtual_id) = virtual_id else {
        return Err("实机尚未关联模拟机，绑定关系必须保存到模拟机".into());
    };
    let mut virtual_machines = state.virtual_machines.lock().map_err(|e| e.to_string())?;
    let virtual_machine = virtual_machines
        .iter_mut()
        .find(|machine| machine.id == virtual_id)
        .ok_or_else(|| "unknown virtual machine".to_owned())?;
    if let Some(physical_id) = input.physical_id {
        if let Some(binding) = virtual_machine
            .joint_bindings
            .iter_mut()
            .find(|binding| binding.slot_id == input.slot_id)
        {
            binding.physical_id = physical_id;
        } else {
            virtual_machine.joint_bindings.push(ServoJointBinding {
                slot_id: input.slot_id.clone(),
                physical_id,
                direction: 1,
                zero_offset_rad: 0.0,
                min_rad: None,
                max_rad: None,
            });
        }
    } else {
        virtual_machine
            .joint_bindings
            .retain(|binding| binding.slot_id != input.slot_id);
    }
    virtual_machine.joint_bindings.sort_by_key(|binding| {
        duck_control::model::joint_index(&binding.slot_id).unwrap_or(usize::MAX)
    });
    drop(virtual_machines);
    let physical = if let Some(direction) = input.direction {
        let mut physical_machines = state
            .physical_machines
            .lock()
            .map_err(|error| error.to_string())?;
        let machine = physical_machines
            .iter_mut()
            .find(|machine| machine.id == input.physical_machine_id)
            .ok_or_else(|| "unknown physical machine".to_owned())?;
        if let Some(binding) = machine
            .joint_directions
            .iter_mut()
            .find(|binding| binding.slot_id == input.slot_id)
        {
            binding.direction = direction;
        } else {
            machine
                .joint_directions
                .push(crate::types::ServoJointDirection {
                    slot_id: input.slot_id.clone(),
                    direction,
                });
        }
        machine.joint_directions.sort_by_key(|binding| {
            duck_control::model::joint_index(&binding.slot_id).unwrap_or(usize::MAX)
        });
        machine.clone()
    } else {
        physical
    };
    state.persist()?;
    Ok(physical)
}

pub(crate) fn virtual_machine_bind(
    state: &DesktopState,
    input: VirtualMachineBindingInput,
) -> Result<VirtualMachine, String> {
    if !duck_control::model::JOINT_NAMES.contains(&input.slot_id.as_str()) {
        return Err(format!("unknown joint slot: {}", input.slot_id));
    }
    let mut virtual_machines = state.virtual_machines.lock().map_err(|e| e.to_string())?;
    let machine = virtual_machines
        .iter_mut()
        .find(|machine| machine.id == input.virtual_machine_id)
        .ok_or_else(|| "unknown virtual machine".to_owned())?;
    if let Some(physical_id) = input.physical_id {
        if let Some(binding) = machine
            .joint_bindings
            .iter_mut()
            .find(|binding| binding.slot_id == input.slot_id)
        {
            binding.physical_id = physical_id;
        } else {
            machine.joint_bindings.push(ServoJointBinding {
                slot_id: input.slot_id,
                physical_id,
                direction: 1,
                zero_offset_rad: 0.0,
                min_rad: None,
                max_rad: None,
            });
        }
    } else {
        machine
            .joint_bindings
            .retain(|binding| binding.slot_id != input.slot_id);
    }
    machine.joint_bindings.sort_by_key(|binding| {
        duck_control::model::joint_index(&binding.slot_id).unwrap_or(usize::MAX)
    });
    let result = machine.clone();
    drop(virtual_machines);
    state.persist()?;
    Ok(result)
}

pub(crate) fn virtual_machine_validate_bindings(
    state: &DesktopState,
    id: String,
) -> Result<VirtualMachine, String> {
    let mut virtual_machines = state.virtual_machines.lock().map_err(|e| e.to_string())?;
    let machine = virtual_machines
        .iter_mut()
        .find(|machine| machine.id == id)
        .ok_or_else(|| "unknown virtual machine".to_owned())?;
    let mut conflicting_ids = Vec::new();
    for (index, binding) in machine.joint_bindings.iter().enumerate() {
        if machine.joint_bindings[index + 1..]
            .iter()
            .any(|other| other.physical_id == binding.physical_id)
        {
            conflicting_ids.push(binding.physical_id);
        }
    }
    machine
        .joint_bindings
        .retain(|binding| !conflicting_ids.contains(&binding.physical_id));
    let result = machine.clone();
    drop(virtual_machines);
    if !conflicting_ids.is_empty() {
        state.persist()?;
    }
    Ok(result)
}

pub(crate) fn physical_machine_validate_bindings(
    state: &DesktopState,
    id: String,
) -> Result<PhysicalMachine, String> {
    let physical = state
        .physical_machines
        .lock()
        .map_err(|e| e.to_string())?
        .iter()
        .find(|machine| machine.id == id)
        .cloned()
        .ok_or_else(|| "unknown physical machine".to_owned())?;
    let Some(virtual_id) = physical.virtual_machine_id.clone() else {
        return Ok(physical);
    };
    let mut virtual_machines = state.virtual_machines.lock().map_err(|e| e.to_string())?;
    let virtual_machine = virtual_machines
        .iter_mut()
        .find(|machine| machine.id == virtual_id)
        .ok_or_else(|| "关联的模拟机不存在".to_owned())?;
    let mut bindings = virtual_machine.joint_bindings.clone();
    let mut conflicting_ids = Vec::new();
    for (index, binding) in bindings.iter().enumerate() {
        if bindings[index + 1..]
            .iter()
            .any(|other| other.physical_id == binding.physical_id)
        {
            conflicting_ids.push(binding.physical_id);
        }
    }
    if !conflicting_ids.is_empty() {
        bindings.retain(|binding| !conflicting_ids.contains(&binding.physical_id));
    }
    virtual_machine.joint_bindings = bindings;
    drop(virtual_machines);
    if !conflicting_ids.is_empty() {
        state.persist()?;
    }
    Ok(physical)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn editing_a_binding_preserves_raw_zero_direction_and_installation_limits() {
        let state = DesktopState::new(None);
        let calibration = ServoJointBinding {
            slot_id: "left_hip_yaw".into(),
            physical_id: 7,
            direction: -1,
            zero_offset_rad: 2.7,
            min_rad: Some(-0.5),
            max_rad: Some(0.7),
        };
        state.virtual_machines.lock().unwrap()[0].joint_bindings = vec![calibration.clone()];
        let updated = virtual_machine_bind(
            &state,
            VirtualMachineBindingInput {
                virtual_machine_id: "official-hd1910".into(),
                slot_id: "left_hip_yaw".into(),
                physical_id: Some(8),
            },
        )
        .unwrap();
        assert_eq!(
            updated.joint_bindings[0],
            ServoJointBinding {
                physical_id: 8,
                ..calibration
            }
        );
    }

    #[test]
    fn virtual_binding_service_updates_and_removes_a_slot() {
        let state = DesktopState::new(None);
        let updated = virtual_machine_bind(
            &state,
            VirtualMachineBindingInput {
                virtual_machine_id: "official-hd1910".into(),
                slot_id: "left_hip_yaw".into(),
                physical_id: Some(20),
            },
        )
        .expect("binding should be stored");
        assert_eq!(updated.joint_bindings[0].physical_id, 20);

        let removed = virtual_machine_bind(
            &state,
            VirtualMachineBindingInput {
                virtual_machine_id: "official-hd1910".into(),
                slot_id: "left_hip_yaw".into(),
                physical_id: None,
            },
        )
        .expect("binding should be removed");
        assert!(removed.joint_bindings.is_empty());
    }

    #[test]
    fn virtual_binding_validation_removes_duplicate_ids() {
        let state = DesktopState::new(None);
        for slot_id in ["left_hip_yaw", "right_hip_yaw"] {
            virtual_machine_bind(
                &state,
                VirtualMachineBindingInput {
                    virtual_machine_id: "official-hd1910".into(),
                    slot_id: slot_id.into(),
                    physical_id: Some(20),
                },
            )
            .expect("binding should be stored");
        }

        let validated = virtual_machine_validate_bindings(&state, "official-hd1910".into())
            .expect("validation should complete");
        assert!(validated.joint_bindings.is_empty());
    }
}
