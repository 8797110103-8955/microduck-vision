//! Binding application service.
//!
//! Body-to-servo binding, duplicate-ID validation, and persisted binding updates live here so
//! physical and virtual command domains share the same rules.

use crate::runtime::session::state::DesktopState;
use crate::types::{
    PhysicalMachine, PhysicalMachineBindingInput, VirtualJointBinding, VirtualMachine,
    VirtualMachineBindingInput,
};
pub(crate) fn physical_machine_bind(
    state: &DesktopState,
    input: PhysicalMachineBindingInput,
) -> Result<PhysicalMachine, String> {
    if !duck_control::model::JOINT_NAMES.contains(&input.slot_id.as_str()) {
        return Err(format!("unknown joint slot: {}", input.slot_id));
    }
    if !matches!(input.direction, -1 | 1) {
        return Err(format!("invalid joint direction: {}", input.direction));
    }
    let _transition = state
        .execution_transition
        .lock()
        .map_err(|e| e.to_string())?;
    // A live executor owns an immutable calibration and controller history. Never save a
    // replacement that its next observation/write would silently ignore.
    crate::runtime::machines::encoder_calibration::ensure_ready(state, &input.physical_machine_id)?;
    if state
        .runtime_loop
        .lock()
        .map_err(|e| e.to_string())?
        .as_ref()
        .is_some_and(|runtime| {
            runtime.physical().is_ok()
                && runtime
                    .snapshot()
                    .active_body
                    .as_ref()
                    .and_then(|body| body.physical_machine_id.as_deref())
                    == Some(input.physical_machine_id.as_str())
        })
    {
        return Err("请先停止这台实机的控制环，再修改肢体映射或方向".into());
    }
    let result = {
        let mut machines = state.physical_machines.lock().map_err(|e| e.to_string())?;
        let machine = machines
            .iter_mut()
            .find(|machine| machine.id == input.physical_machine_id)
            .ok_or("unknown physical machine")?;
        if let Some(saved) = machine
            .joint_directions
            .iter_mut()
            .find(|saved| saved.slot_id == input.slot_id)
        {
            saved.direction = input.direction;
        } else {
            machine
                .joint_directions
                .push(crate::types::ServoJointDirection {
                    slot_id: input.slot_id.clone(),
                    direction: input.direction,
                });
        }
        machine.joint_directions.sort_by_key(|binding| {
            duck_control::model::joint_index(&binding.slot_id).unwrap_or(usize::MAX)
        });
        machine.clone()
    };
    state.persist_physical_machines()?;
    state
        .config_revision
        .fetch_add(1, std::sync::atomic::Ordering::AcqRel);
    crate::platform::logging::diagnostic(
        state.config_dir.as_deref(),
        "binding",
        "info",
        "physical_direction_saved",
        serde_json::json!({"duck_id":result.id,"slot_id":input.slot_id,
            "direction":input.direction,"physical_directions":result.joint_directions}),
    );
    Ok(result)
}

fn update_virtual_binding(machine: &mut VirtualMachine, slot_id: &str, physical_id: Option<u8>) {
    if let Some(physical_id) = physical_id {
        if let Some(binding) = machine
            .joint_bindings
            .iter_mut()
            .find(|binding| binding.slot_id == slot_id)
        {
            binding.physical_id = physical_id;
        } else {
            machine.joint_bindings.push(VirtualJointBinding {
                slot_id: slot_id.into(),
                physical_id,
            });
        }
    } else {
        machine
            .joint_bindings
            .retain(|binding| binding.slot_id != slot_id);
    }
    machine.joint_bindings.sort_by_key(|binding| {
        duck_control::model::joint_index(&binding.slot_id).unwrap_or(usize::MAX)
    });
}

/// Exchange IDs together so the runtime never observes an intermediate duplicate mapping.
pub(crate) fn virtual_machine_bindings(
    state: &DesktopState,
    id: String,
    updates: Vec<crate::types::VirtualJointBindingInput>,
) -> Result<VirtualMachine, String> {
    let _transition = state
        .execution_transition
        .lock()
        .map_err(|e| e.to_string())?;
    crate::runtime::session::loop_service::ensure_mapping_editable(state, &id)?;
    let mut machines = state.virtual_machines.lock().map_err(|e| e.to_string())?;
    let machine = machines
        .iter_mut()
        .find(|machine| machine.id == id)
        .ok_or("unknown virtual machine")?;
    let mut next = machine.clone();
    for update in updates {
        if !duck_control::model::JOINT_NAMES.contains(&update.slot_id.as_str()) {
            return Err(format!("unknown joint slot: {}", update.slot_id));
        }
        update_virtual_binding(&mut next, &update.slot_id, update.physical_id);
    }
    let mut ids = std::collections::HashSet::new();
    if next
        .joint_bindings
        .iter()
        .any(|binding| !ids.insert(binding.physical_id))
    {
        return Err("肢体绑定存在重复舵机 ID".into());
    }
    *machine = next.clone();
    drop(machines);
    state.persist_virtual_machines()?;
    state
        .config_revision
        .fetch_add(1, std::sync::atomic::Ordering::AcqRel);
    Ok(next)
}

pub(crate) fn virtual_machine_bind(
    state: &DesktopState,
    input: VirtualMachineBindingInput,
) -> Result<VirtualMachine, String> {
    virtual_machine_bindings(
        state,
        input.virtual_machine_id,
        vec![crate::types::VirtualJointBindingInput {
            slot_id: input.slot_id,
            physical_id: input.physical_id,
        }],
    )
}

pub(crate) fn virtual_machine_validate_bindings(
    state: &DesktopState,
    id: String,
) -> Result<VirtualMachine, String> {
    let _transition = state
        .execution_transition
        .lock()
        .map_err(|e| e.to_string())?;
    crate::runtime::session::loop_service::ensure_mapping_editable(state, &id)?;
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
        state.persist_virtual_machines()?;
        state
            .config_revision
            .fetch_add(1, std::sync::atomic::Ordering::AcqRel);
    }
    Ok(result)
}

pub(crate) fn physical_machine_validate_bindings(
    state: &DesktopState,
    id: String,
) -> Result<PhysicalMachine, String> {
    let _transition = state
        .execution_transition
        .lock()
        .map_err(|e| e.to_string())?;
    let mut machines = state.physical_machines.lock().map_err(|e| e.to_string())?;
    let machine = machines
        .iter_mut()
        .find(|machine| machine.id == id)
        .ok_or("unknown physical machine")?;
    machine
        .joint_bindings
        .sort_by_key(|binding| duck_control::model::joint_index(&binding.slot_id));
    machine
        .joint_bindings
        .dedup_by(|a, b| a.slot_id == b.slot_id);
    let result = machine.clone();
    drop(machines);
    state.persist_physical_machines()?;
    Ok(result)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn exchanging_ids_is_atomic_and_a_duplicate_update_keeps_the_previous_mapping() {
        let state = DesktopState::new(None);
        let original = state.virtual_machines.lock().unwrap()[0].clone();
        let first = &original.joint_bindings[0];
        let second = &original.joint_bindings[1];
        let swapped = virtual_machine_bindings(
            &state,
            original.id.clone(),
            vec![
                crate::types::VirtualJointBindingInput {
                    slot_id: first.slot_id.clone(),
                    physical_id: Some(second.physical_id),
                },
                crate::types::VirtualJointBindingInput {
                    slot_id: second.slot_id.clone(),
                    physical_id: Some(first.physical_id),
                },
            ],
        )
        .unwrap();
        assert_eq!(swapped.joint_bindings[0].physical_id, second.physical_id);
        assert_eq!(swapped.joint_bindings[1].physical_id, first.physical_id);
        let error = virtual_machine_bind(
            &state,
            VirtualMachineBindingInput {
                virtual_machine_id: original.id,
                slot_id: second.slot_id.clone(),
                physical_id: Some(second.physical_id),
            },
        )
        .unwrap_err();
        assert!(error.contains("重复"));
        assert_eq!(
            state.virtual_machines.lock().unwrap()[0].joint_bindings,
            swapped.joint_bindings
        );
    }

    #[test]
    fn serialized_installation_has_no_servo_id_or_implicit_binding() {
        let calibration = crate::types::ServoJointCalibration {
            slot_id: "left_knee".into(),
            zero_offset_rad: 1.2,
            min_rad: None,
            max_rad: None,
        };
        let installation = toml::to_string(&calibration).unwrap();
        assert!(!installation.contains("physicalId"));
        assert!(!installation.contains("direction"));
        let binding = toml::to_string(&VirtualJointBinding {
            slot_id: "left_knee".into(),
            physical_id: 42,
        })
        .unwrap();
        assert!(binding.contains("physicalId = 42"));
        assert!(!binding.contains("zeroOffsetRad"));
        assert!(
            toml::from_str::<crate::types::ServoJointCalibration>(
                "slotId = 'left_knee'\nphysicalId = 7\nzeroOffsetRad = 0"
            )
            .is_err()
        );
    }

    #[test]
    fn physical_direction_never_changes_virtual_ids_or_calibration() {
        let state = DesktopState::new(None);
        state.virtual_machines.lock().unwrap()[0]
            .joint_bindings
            .clear();
        let virtual_id = state.virtual_machines.lock().unwrap()[0].id.clone();
        virtual_machine_bind(
            &state,
            VirtualMachineBindingInput {
                virtual_machine_id: virtual_id.clone(),
                slot_id: "left_hip_yaw".into(),
                physical_id: Some(42),
            },
        )
        .unwrap();
        state
            .physical_machines
            .lock()
            .unwrap()
            .push(PhysicalMachine {
                execution_kind: crate::types::ExecutionKindKey::Physical,
                model_kind: None,
                servo_model: None,
                id: "physical-test".into(),
                name: "test".into(),
                usb_port: "test".into(),
                machine_code: "test".into(),
                reconnect_hint: None,
                census: vec![],
                virtual_machine_id: Some(virtual_id.clone()),
                online_servo_ids: vec![],
                connected: false,
                last_seen: None,
                torque_enabled: false,
                joint_directions: vec![],
                joint_bindings: vec![crate::types::ServoJointCalibration {
                    slot_id: "left_hip_yaw".into(),
                    zero_offset_rad: 1.2,
                    min_rad: Some(-1.0),
                    max_rad: Some(1.0),
                }],
            });
        let hardware = physical_machine_bind(
            &state,
            PhysicalMachineBindingInput {
                physical_machine_id: "physical-test".into(),
                slot_id: "left_hip_yaw".into(),
                direction: -1,
            },
        )
        .unwrap();
        assert_eq!(hardware.joint_directions[0].direction, -1);
        assert_eq!(hardware.joint_bindings[0].zero_offset_rad, 1.2);
        virtual_machine_bind(
            &state,
            VirtualMachineBindingInput {
                virtual_machine_id: virtual_id,
                slot_id: "left_hip_yaw".into(),
                physical_id: Some(99),
            },
        )
        .unwrap();
        let hardware = state.physical_machines.lock().unwrap();
        assert_eq!(hardware[0].joint_bindings[0].zero_offset_rad, 1.2);
        assert_eq!(hardware[0].joint_directions[0].direction, -1);
        assert_eq!(
            state.virtual_machines.lock().unwrap()[0].joint_bindings[0].physical_id,
            99
        );
    }

    #[test]
    fn editing_a_binding_preserves_raw_zero_direction_and_installation_limits() {
        let state = DesktopState::new(None);
        state.virtual_machines.lock().unwrap()[0]
            .joint_bindings
            .clear();
        let virtual_id = state.virtual_machines.lock().unwrap()[0].id.clone();
        let binding = VirtualJointBinding {
            slot_id: "left_hip_yaw".into(),
            physical_id: 7,
        };
        state.virtual_machines.lock().unwrap()[0].joint_bindings = vec![binding];
        let updated = virtual_machine_bind(
            &state,
            VirtualMachineBindingInput {
                virtual_machine_id: virtual_id.clone(),
                slot_id: "left_hip_yaw".into(),
                physical_id: Some(8),
            },
        )
        .unwrap();
        assert_eq!(
            updated.joint_bindings[0],
            VirtualJointBinding {
                physical_id: 8,
                slot_id: "left_hip_yaw".into(),
            }
        );
    }

    #[test]
    fn virtual_binding_service_updates_and_removes_a_slot() {
        let state = DesktopState::new(None);
        state.virtual_machines.lock().unwrap()[0]
            .joint_bindings
            .clear();
        let virtual_id = state.virtual_machines.lock().unwrap()[0].id.clone();
        let updated = virtual_machine_bind(
            &state,
            VirtualMachineBindingInput {
                virtual_machine_id: virtual_id.clone(),
                slot_id: "left_hip_yaw".into(),
                physical_id: Some(20),
            },
        )
        .expect("binding should be stored");
        assert_eq!(updated.joint_bindings[0].physical_id, 20);

        let removed = virtual_machine_bind(
            &state,
            VirtualMachineBindingInput {
                virtual_machine_id: virtual_id.clone(),
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
        state.virtual_machines.lock().unwrap()[0]
            .joint_bindings
            .clear();
        let virtual_id = state.virtual_machines.lock().unwrap()[0].id.clone();
        state.virtual_machines.lock().unwrap()[0].joint_bindings =
            ["left_hip_yaw", "right_hip_yaw"]
                .iter()
                .map(|slot| VirtualJointBinding {
                    slot_id: (*slot).into(),
                    physical_id: 20,
                })
                .collect();

        let validated = virtual_machine_validate_bindings(&state, virtual_id.clone())
            .expect("validation should complete");
        assert!(validated.joint_bindings.is_empty());
    }
}
