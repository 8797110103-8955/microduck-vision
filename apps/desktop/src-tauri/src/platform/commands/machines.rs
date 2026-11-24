//! Tauri command layer for the desktop client.
//!
//! These commands coordinate user-facing state and persistence.  Hardware and
//! loop-body adapters stay in [`crate::runtime::adapters::servo_bus`], while storage lives in
//! [`crate::platform::state`].

pub(crate) mod physical;
pub(crate) mod registers;
pub(crate) mod virtual_machine;
#[cfg(test)]
pub(crate) use crate::runtime::machines::registers::update_servo_id_cache;
#[cfg(test)]
use std::collections::HashSet;
use tauri::State;

#[cfg(test)]
use crate::runtime::adapters::servo_bus::RegisterDescriptor;

use crate::runtime::machines::devices as service;
#[cfg(test)]
pub(crate) use crate::runtime::machines::machine_resolution::{
    binding_conflict_error, debug_driver_mapping, debug_servo_bindings, official_servo_bindings,
    resolve_physical_loop_mapping,
};
use crate::runtime::session::state::DesktopState;

#[cfg(test)]
use crate::types::ServoIdentity;
use crate::types::{
    DetectedPhysicalMachine, PhysicalMachine, PhysicalMachineBindingInput,
    PhysicalMachineDetectInput, PhysicalMachineInput, PhysicalMachineProbeInput,
    PhysicalMachineReconnectInput, PhysicalMachineRegisterWriteInput, PhysicalMachineSaveInput,
    PhysicalServoRegister, SerialPortCandidate, VirtualMachine, VirtualMachineBindingInput,
    VirtualMachineInput,
};

#[cfg(test)]
fn register_table(model: servo::ServoModel) -> Vec<RegisterDescriptor> {
    let registers = match model {
        servo::ServoModel::Hd1910 => servo::hd1910::registers(),
        servo::ServoModel::Xl330M288 => servo::xl330::registers(),
        _ => &[],
    };
    let mut addresses = HashSet::new();
    let mut table = Vec::new();
    for register in registers {
        if model == servo::ServoModel::Hd1910 && register.addr == 3 {
            if !addresses.insert(3) {
                continue;
            }
            // FT exposes the servo model as two one-byte version fields.
            // rustypot's shared STS definition exposes a u16 alias at 0x03.
            table.extend([
                RegisterDescriptor {
                    name: "model",
                    addr: 3,
                    size: 1,
                    access: servo::RegisterAccess::Read,
                    is_default: false,
                },
                RegisterDescriptor {
                    name: "model_number",
                    addr: 4,
                    size: 1,
                    access: servo::RegisterAccess::Read,
                    is_default: false,
                },
            ]);
            addresses.insert(4);
            continue;
        }
        // rustypot's shared STS table has the old two-byte definition at 0x18
        // and the old factory meaning at 0x53. HD1910 uses the FT table here.
        if model == servo::ServoModel::Hd1910 && matches!(register.addr, 24 | 83) {
            continue;
        }
        if model == servo::ServoModel::Hd1910 && register.addr == 44 {
            table.push(RegisterDescriptor {
                name: "pwm_open_speed",
                addr: 44,
                size: 2,
                access: servo::RegisterAccess::ReadWrite,
                is_default: false,
            });
            addresses.insert(44);
            continue;
        }
        if !addresses.insert(register.addr) {
            continue;
        }
        table.push(RegisterDescriptor {
            name: register.name,
            addr: register.addr,
            size: register.size,
            access: register.access,
            is_default: model == servo::ServoModel::Hd1910 && register.addr >= 80,
        });
        if model == servo::ServoModel::Hd1910 && register.addr == 69 {
            table.extend([
                RegisterDescriptor {
                    name: "current_bias",
                    addr: 73,
                    size: 2,
                    access: servo::RegisterAccess::Read,
                    is_default: false,
                },
                RegisterDescriptor {
                    name: "v_fk",
                    addr: 77,
                    size: 1,
                    access: servo::RegisterAccess::Read,
                    is_default: true,
                },
                RegisterDescriptor {
                    name: "v_kgi",
                    addr: 78,
                    size: 1,
                    access: servo::RegisterAccess::Read,
                    is_default: true,
                },
                RegisterDescriptor {
                    name: "p_fk",
                    addr: 79,
                    size: 1,
                    access: servo::RegisterAccess::Read,
                    is_default: true,
                },
            ]);
            addresses.extend([73, 77, 78, 79]);
        }
        if model == servo::ServoModel::Hd1910 && register.addr == 82 {
            // 0x52/0x53 are eFk/Vk on HD1910, rather than the shared STS
            // velocity-unit/minimum-speed pair.
            table.pop();
            addresses.remove(&82);
            table.extend([
                RegisterDescriptor {
                    name: "e_fk",
                    addr: 82,
                    size: 1,
                    access: servo::RegisterAccess::Read,
                    is_default: true,
                },
                RegisterDescriptor {
                    name: "vk",
                    addr: 83,
                    size: 1,
                    access: servo::RegisterAccess::Read,
                    is_default: true,
                },
            ]);
            addresses.extend([82, 83]);
        }
        if model == servo::ServoModel::Hd1910 && register.addr == 23 {
            table.push(RegisterDescriptor {
                name: "minimum_startup_force",
                addr: 24,
                size: 1,
                access: servo::RegisterAccess::ReadWrite,
                is_default: false,
            });
            table.push(RegisterDescriptor {
                name: "integral_limit",
                addr: 25,
                size: 1,
                access: servo::RegisterAccess::ReadWrite,
                is_default: false,
            });
            addresses.extend([24, 25]);
        }
    }
    table
}

#[cfg(test)]
fn register_is_writable(register: RegisterDescriptor) -> bool {
    !register.is_default && !matches!(register.access, servo::RegisterAccess::Read)
}

#[tauri::command]
#[specta::specta]
pub(crate) fn virtual_machines(
    state: State<'_, DesktopState>,
) -> Result<Vec<VirtualMachine>, String> {
    service::virtual_machines(state.inner())
}

#[tauri::command]
#[specta::specta]
pub(crate) fn physical_machines(
    state: State<'_, DesktopState>,
) -> Result<Vec<PhysicalMachine>, String> {
    service::physical_machines(state.inner())
}

#[tauri::command]
#[specta::specta]
pub(crate) fn auto_mount_enabled(state: State<'_, DesktopState>) -> Result<bool, String> {
    service::auto_mount_enabled(state.inner())
}

#[tauri::command]
#[specta::specta]
pub(crate) fn set_auto_mount_enabled(
    state: State<'_, DesktopState>,
    enabled: bool,
) -> Result<bool, String> {
    service::set_auto_mount_enabled(state.inner(), enabled)
}

#[tauri::command]
#[specta::specta]
pub(crate) fn auto_reconnect_enabled(state: State<'_, DesktopState>) -> Result<bool, String> {
    service::auto_reconnect_enabled(state.inner())
}

#[tauri::command]
#[specta::specta]
pub(crate) fn set_auto_reconnect_enabled(
    state: State<'_, DesktopState>,
    enabled: bool,
) -> Result<bool, String> {
    service::set_auto_reconnect_enabled(state.inner(), enabled)
}

#[cfg(test)]
mod register_table_tests {
    use super::{
        binding_conflict_error, debug_driver_mapping, debug_servo_bindings, register_is_writable,
        register_table, resolve_physical_loop_mapping, update_servo_id_cache,
    };
    use crate::runtime::adapters::physical_identity;
    use crate::types::{PhysicalMachine, ServoIdentity, ServoJointBinding};
    use servo::ServoModel;

    #[test]
    fn hd_table_contains_one_version_pair_and_default_rows_are_read_only() {
        let table = register_table(ServoModel::Hd1910);
        assert_eq!(table.iter().filter(|row| row.addr == 3).count(), 1);
        assert_eq!(table.iter().filter(|row| row.addr == 4).count(), 1);
        assert_eq!(
            table.iter().find(|row| row.addr == 3).map(|row| row.name),
            Some("model")
        );
        assert_eq!(
            table.iter().find(|row| row.addr == 4).map(|row| row.name),
            Some("model_number")
        );
        assert_eq!(
            table.iter().find(|row| row.addr == 44).map(|row| row.name),
            Some("pwm_open_speed")
        );
        assert_eq!(
            table.iter().find(|row| row.addr == 24).map(|row| row.size),
            Some(1)
        );
        assert_eq!(
            table.iter().find(|row| row.addr == 25).map(|row| row.size),
            Some(1)
        );
        assert_eq!(
            table.iter().find(|row| row.addr == 44).map(|row| row.size),
            Some(2)
        );
        for address in 80..=86 {
            let row = table
                .iter()
                .find(|row| row.addr == address)
                .expect("HD1910 default row");
            assert!(row.is_default);
            assert!(!register_is_writable(*row));
        }
    }

    #[test]
    fn changing_a_servo_id_updates_census_and_online_ids() {
        let mut machine = PhysicalMachine {
            execution_kind: crate::types::ExecutionKindKey::Physical,
            model_kind: Some(crate::types::ModelKindKey::Hd1910),
            servo_model: Some(crate::types::ServoModelKey::Hd1910),
            id: "physical-1".into(),
            name: "duck".into(),
            usb_port: "COM1".into(),
            machine_code: String::new(),
            census: vec![
                ServoIdentity {
                    servo_id: 5,
                    model: "hd1910".into(),
                },
                ServoIdentity {
                    servo_id: 2,
                    model: "hd1910".into(),
                },
            ],
            reconnect_hint: None,
            virtual_machine_id: Some("virtual-1".into()),
            online_servo_ids: vec![5, 2],
            connected: true,
            last_seen: None,
            torque_enabled: false,
            joint_directions: Vec::new(),
            joint_bindings: Vec::new(),
        };

        update_servo_id_cache(&mut machine, 5, 7).unwrap();
        assert_eq!(
            machine
                .census
                .iter()
                .map(|servo| servo.servo_id)
                .collect::<Vec<_>>(),
            vec![2, 7]
        );
        assert_eq!(machine.online_servo_ids, vec![7, 2]);
        assert_eq!(
            machine.machine_code,
            physical_identity::census_md5(&machine.census).unwrap()
        );
    }

    #[test]
    fn changing_a_servo_id_rejects_missing_or_duplicate_ids_without_mutating() {
        let mut machine = PhysicalMachine {
            execution_kind: crate::types::ExecutionKindKey::Physical,
            model_kind: Some(crate::types::ModelKindKey::Hd1910),
            servo_model: Some(crate::types::ServoModelKey::Hd1910),
            id: "physical-1".into(),
            name: "duck".into(),
            usb_port: "COM1".into(),
            machine_code: String::new(),
            census: vec![
                ServoIdentity {
                    servo_id: 1,
                    model: "hd1910".into(),
                },
                ServoIdentity {
                    servo_id: 2,
                    model: "hd1910".into(),
                },
            ],
            reconnect_hint: None,
            virtual_machine_id: None,
            online_servo_ids: vec![1, 2],
            connected: true,
            last_seen: None,
            torque_enabled: false,
            joint_directions: Vec::new(),
            joint_bindings: Vec::new(),
        };
        let before = machine.clone();
        assert!(update_servo_id_cache(&mut machine, 9, 3).is_err());
        assert_eq!(machine.census, before.census);
        assert_eq!(machine.online_servo_ids, before.online_servo_ids);
        assert_eq!(machine.machine_code, before.machine_code);
        assert!(update_servo_id_cache(&mut machine, 1, 2).is_err());
        assert_eq!(machine.census, before.census);
        assert_eq!(machine.online_servo_ids, before.online_servo_ids);
        assert_eq!(machine.machine_code, before.machine_code);
    }

    #[test]
    fn duplicate_virtual_bindings_are_rejected_before_hardware_use() {
        let bindings = vec![
            ServoJointBinding {
                slot_id: "left_hip_yaw".into(),
                physical_id: 42,
                direction: 1,
                zero_offset_rad: 0.0,
                min_rad: None,
                max_rad: None,
            },
            ServoJointBinding {
                slot_id: "right_hip_yaw".into(),
                physical_id: 42,
                direction: 1,
                zero_offset_rad: 0.0,
                min_rad: None,
                max_rad: None,
            },
        ];
        assert_eq!(
            binding_conflict_error(&bindings).as_deref(),
            Some("绑定冲突：42 → left_hip_yaw,right_hip_yaw")
        );
    }

    #[test]
    fn debug_bindings_use_virtual_ids_and_skip_unmatched_slots() {
        let census = vec![
            ServoIdentity {
                servo_id: 7,
                model: "hd1910".into(),
            },
            ServoIdentity {
                servo_id: 10,
                model: "hd1910".into(),
            },
        ];
        let bindings = vec![
            crate::types::VirtualJointBinding {
                slot_id: "left_hip_yaw".into(),
                physical_id: 7,
            },
            crate::types::VirtualJointBinding {
                slot_id: "right_hip_yaw".into(),
                physical_id: 42,
            },
        ];
        let resolved = debug_servo_bindings(
            &census,
            &bindings,
            &["left_hip_yaw".into(), "right_hip_yaw".into()],
        );
        assert_eq!(
            resolved[0].as_ref().map(|binding| binding.physical_id),
            Some(7)
        );
        assert!(resolved[1].is_none());
    }

    #[test]
    fn debug_bindings_never_guess_limb_ids_from_the_census() {
        let census = vec![
            ServoIdentity {
                servo_id: 20,
                model: "hd1910".into(),
            },
            ServoIdentity {
                servo_id: 10,
                model: "hd1910".into(),
            },
        ];
        let slots = vec!["left_hip_yaw".into(), "right_hip_yaw".into()];
        let resolved = debug_servo_bindings(&census, &[], &slots);
        assert_eq!(
            resolved
                .iter()
                .map(|binding| binding.as_ref().map(|binding| binding.physical_id))
                .collect::<Vec<_>>(),
            vec![None, None]
        );
        let override_binding = crate::types::VirtualJointBinding {
            slot_id: "left_hip_yaw".into(),
            physical_id: 20,
        };
        let resolved =
            debug_servo_bindings(&census, std::slice::from_ref(&override_binding), &slots);
        assert_eq!(
            resolved[0].as_ref().map(|binding| binding.physical_id),
            Some(20)
        );
        assert!(resolved[1].is_none());
    }

    #[test]
    fn debug_driver_mapping_keeps_body_order_and_isolates_missing_slots() {
        let census = vec![
            ServoIdentity {
                servo_id: 7,
                model: "hd1910".into(),
            },
            ServoIdentity {
                servo_id: 10,
                model: "hd1910".into(),
            },
        ];
        let resolved = vec![
            Some(ServoJointBinding {
                slot_id: "left_hip_yaw".into(),
                physical_id: 7,
                direction: 1,
                zero_offset_rad: 0.0,
                min_rad: None,
                max_rad: None,
            }),
            None,
            Some(ServoJointBinding {
                slot_id: "right_hip_yaw".into(),
                physical_id: 10,
                direction: -1,
                zero_offset_rad: 0.2,
                min_rad: Some(-1.0),
                max_rad: Some(1.0),
            }),
        ];
        let (driver_ids, active_ids) = debug_driver_mapping(&census, &resolved).unwrap();
        assert_eq!(active_ids, vec![Some(7), None, Some(10)]);
        assert_eq!(driver_ids.len(), 3);
        assert_eq!(driver_ids[0], 7);
        assert_eq!(driver_ids[2], 10);
        assert!(!census.iter().any(|servo| servo.servo_id == driver_ids[1]));
        assert_ne!(driver_ids[1], driver_ids[0]);
        assert_ne!(driver_ids[1], driver_ids[2]);
    }

    #[test]
    fn debug_driver_mapping_rejects_duplicate_matched_ids_but_allows_missing_slots() {
        let census = vec![ServoIdentity {
            servo_id: 7,
            model: "hd1910".into(),
        }];
        let binding = |slot_id: &str| ServoJointBinding {
            slot_id: slot_id.into(),
            physical_id: 7,
            direction: 1,
            zero_offset_rad: 0.0,
            min_rad: None,
            max_rad: None,
        };
        assert!(
            debug_driver_mapping(
                &census,
                &[
                    Some(binding("left_hip_yaw")),
                    Some(binding("right_hip_yaw"))
                ]
            )
            .is_err()
        );
        let (_, active_ids) = debug_driver_mapping(&census, &[None, None]).unwrap();
        assert_eq!(active_ids, vec![None, None]);
    }

    #[test]
    fn physical_loop_mapping_allows_policy_slots_missing_from_census() {
        let physical = PhysicalMachine {
            execution_kind: crate::types::ExecutionKindKey::Physical,
            model_kind: Some(crate::types::ModelKindKey::Hd1910),
            servo_model: Some(crate::types::ServoModelKey::Hd1910),
            id: "physical-1".into(),
            name: "duck".into(),
            usb_port: "COM1".into(),
            machine_code: String::new(),
            census: vec![ServoIdentity {
                servo_id: 7,
                model: "hd1910".into(),
            }],
            reconnect_hint: None,
            virtual_machine_id: Some("virtual-1".into()),
            online_servo_ids: vec![7],
            connected: true,
            last_seen: None,
            torque_enabled: false,
            joint_directions: Vec::new(),
            joint_bindings: Vec::new(),
        };
        let bindings = [
            crate::types::VirtualJointBinding {
                slot_id: "left_hip_yaw".into(),
                physical_id: 7,
            },
            crate::types::VirtualJointBinding {
                slot_id: "right_hip_yaw".into(),
                physical_id: 42,
            },
        ];
        let slots = vec!["left_hip_yaw".into(), "right_hip_yaw".into()];

        let (resolved, driver_ids, active_ids) =
            resolve_physical_loop_mapping(&bindings, &physical, &slots).unwrap();

        assert_eq!(
            resolved
                .iter()
                .map(|binding| binding.slot_id.as_str())
                .collect::<Vec<_>>(),
            vec!["left_hip_yaw"]
        );
        assert_eq!(active_ids, vec![Some(7), None]);
        assert_eq!(driver_ids[0], 7);
        assert_ne!(driver_ids[1], 7);
    }
}

#[cfg(test)]
mod training_tests {
    use crate::runtime::machines::devices::virtual_machines;
    use crate::runtime::session::state::DesktopState;
    use crate::training::process::training_metadata;
    use crate::types::TrainingStartInput;

    #[test]
    fn training_metadata_uses_the_selected_body_and_fixed_policy_layout() {
        let state = DesktopState::new(None);
        let virtual_machine_id = virtual_machines(&state).unwrap()[0].id.clone();
        let input = TrainingStartInput {
            program: "uv".into(),
            args: vec!["run".into(), "train".into()],
            working_dir: ".".into(),
            virtual_machine_id,
            policy_id: None,
            experiment_id: Some("experiment-1".into()),
            recipe: "student-distillation".into(),
            teacher_motion_provenance: "teacher:official-motion".into(),
            dataset_path: None,
        };

        let metadata = training_metadata(&state, &input, "run-1").expect("training metadata");
        assert_eq!(metadata.experiment_id, "experiment-1");
        assert_eq!(metadata.body_template_id, "official");
        assert_eq!(metadata.body_version, 1);
        assert!(!metadata.body_source_hash.is_empty());
        assert_eq!(metadata.policy_layout.obs_len, 61);
        assert_eq!(metadata.policy_layout.action_len, 14);
        assert_eq!(metadata.policy_layout.control_hz, 50);
        assert_eq!(metadata.servo_profile, crate::types::ServoModelKey::Hd1910);
    }
}

#[cfg(test)]
mod joint_binding_tests {
    use super::{ServoIdentity, official_servo_bindings};

    #[test]
    fn official_census_is_ordered_by_body_slots() {
        let census = [
            ServoIdentity {
                servo_id: 14,
                model: "hd1910".into(),
            },
            ServoIdentity {
                servo_id: 20,
                model: "hd1910".into(),
            },
            ServoIdentity {
                servo_id: 10,
                model: "hd1910".into(),
            },
        ];
        let bindings = official_servo_bindings(&census);
        assert_eq!(
            bindings
                .iter()
                .map(|binding| binding.slot_id.as_str())
                .collect::<Vec<_>>(),
            ["left_hip_yaw", "right_hip_yaw", "right_ankle",]
        );
        assert_eq!(
            bindings
                .iter()
                .map(|binding| binding.physical_id)
                .collect::<Vec<_>>(),
            [20, 10, 14]
        );
    }
}
