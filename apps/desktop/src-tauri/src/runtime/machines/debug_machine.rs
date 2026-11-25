//! Application service for the debug page's explicit physical-machine actions.
//!
//! The debug page is deliberately read-oriented.  Each mutating action is an explicit service
//! call; it never piggybacks on the read-only physical loop.

use loop_body::BackendAccess;

use loop_body::{Calibration, JointCalibration};

use crate::runtime::machines::machine_resolution::{
    binding_conflict_error, resolve_physical_loop_mapping,
};
use crate::runtime::machines::physical_io::with_bus;
use crate::runtime::session::state::DesktopState;
use crate::types::PhysicalMachine;

fn read_positions(
    state: &DesktopState,
    physical_id: &str,
    model: servo::ServoModel,
    ids: &[u8],
) -> Result<Vec<f64>, String> {
    with_bus(state, physical_id, BackendAccess::ReadOnly, |bus| {
        bus.ok_or_else(|| "实机没有可用连接".to_owned())?
            .read_positions(model, ids)
    })
}

fn set_torque(
    state: &DesktopState,
    physical_id: &str,
    model: servo::ServoModel,
    ids: &[u8],
    enabled: bool,
) -> Result<(), String> {
    with_bus(state, physical_id, BackendAccess::Control, |bus| {
        crate::runtime::machines::physical_io::validate_control_ids(state, ids)?;
        bus.ok_or_else(|| "实机没有可用连接".to_owned())?
            .set_torque(model, ids, enabled)
    })
}

fn write_position(
    state: &DesktopState,
    physical_id: &str,
    servo_id: u8,
    position_rad: f64,
) -> Result<(), String> {
    with_bus(state, physical_id, BackendAccess::Control, |bus| {
        crate::runtime::machines::physical_io::validate_control_ids(state, &[servo_id])?;
        let bus = bus.ok_or_else(|| "实机没有可用连接".to_owned())?;
        if matches!(
            bus,
            crate::runtime::adapters::servo_bus::ConnectedBus::Hd1910(_)
        ) {
            let snapshot = bus.read_hd1910_configuration(servo_id)?;
            snapshot
                .domain
                .validate(position_rad)
                .map_err(|error| format!("HD1910 id={servo_id}: {error}"))?;
        }
        bus.write_position(servo_id, position_rad)
    })
}
pub(crate) fn set_pose_zero(
    state: &DesktopState,
    id: String,
    virtual_machine_id: Option<String>,
) -> Result<PhysicalMachine, String> {
    set_pose_zero_with_reader(state, id.clone(), virtual_machine_id, |model, ids| {
        crate::runtime::machines::physical_io::with_bus_locked(
            state,
            &id,
            BackendAccess::ReadOnly,
            |bus| {
                bus.ok_or_else(|| "实机没有可用连接".to_owned())?
                    .read_positions(model, ids)
            },
        )
    })
}

fn set_pose_zero_with_reader(
    state: &DesktopState,
    id: String,
    virtual_machine_id: Option<String>,
    read: impl FnOnce(servo::ServoModel, &[u8]) -> Result<Vec<f64>, String>,
) -> Result<PhysicalMachine, String> {
    // Calibration and execution replacement must not race. The existing read-only source
    // resolves the updated calibration from shared configuration on its next sample.
    let _transition = state
        .execution_transition
        .lock()
        .map_err(|error| error.to_string())?;
    crate::runtime::machines::encoder_calibration::ensure_ready(state, &id)?;
    if state
        .runtime_loop
        .lock()
        .map_err(|error| error.to_string())?
        .as_ref()
        .is_some_and(|runtime| runtime.physical().is_ok())
    {
        return Err("控制环运行时不能绑定姿态零位".into());
    }
    let machine = state
        .physical_machines
        .lock()
        .map_err(|error| error.to_string())?
        .iter()
        .find(|machine| machine.id == id)
        .cloned()
        .ok_or_else(|| "unknown physical machine".to_owned())?;
    let virtual_machine = crate::runtime::machines::machine_resolution::associated_virtual_machine(
        state,
        &machine,
        virtual_machine_id.as_deref(),
    )?;
    let template =
        crate::runtime::machines::machine_resolution::body_profile(&virtual_machine.body_id)?
            .template();
    let slots = template
        .actuators
        .iter()
        .map(|slot| slot.id.clone())
        .collect::<Vec<_>>();
    let (bindings, _, _) =
        resolve_physical_loop_mapping(&virtual_machine.joint_bindings, &machine, &slots)?;
    let model = crate::runtime::machines::physical_machine::physical_servo_model(&machine)?;
    if let Some(error) = binding_conflict_error(&bindings) {
        return Err(error);
    }
    let ids = bindings
        .iter()
        .map(|binding| binding.physical_id)
        .collect::<Vec<_>>();
    let positions = read(model, &ids)?;
    let mut machines = state.physical_machines.lock().map_err(|e| e.to_string())?;
    let machine = machines
        .iter_mut()
        .find(|machine| machine.id == id)
        .ok_or("unknown physical machine")?;
    let mut resolved_calibrations = bindings
        .iter()
        .map(|binding| JointCalibration {
            slot_id: binding.slot_id.clone(),
            physical_id: binding.physical_id,
            direction: binding.direction,
            zero_offset_rad: binding.zero_offset_rad,
            min_rad: binding.min_rad,
            max_rad: binding.max_rad,
        })
        .collect::<Vec<_>>();
    duck_body::apply_pose_zero_positions(&mut resolved_calibrations, &positions)
        .map_err(|error| error.to_string())?;
    let updated_bindings = &mut machine.joint_bindings;
    for calibration in resolved_calibrations {
        if let Some(binding) = updated_bindings
            .iter_mut()
            .find(|binding| binding.slot_id == calibration.slot_id)
        {
            *binding = crate::types::ServoJointCalibration {
                slot_id: calibration.slot_id,

                zero_offset_rad: calibration.zero_offset_rad,
                min_rad: calibration.min_rad,
                max_rad: calibration.max_rad,
            };
        } else {
            updated_bindings.push(crate::types::ServoJointCalibration {
                slot_id: calibration.slot_id,

                zero_offset_rad: calibration.zero_offset_rad,
                min_rad: calibration.min_rad,
                max_rad: calibration.max_rad,
            });
        }
    }
    let result = machine.clone();
    drop(machines);
    state.persist_physical_machines()?;
    state
        .config_revision
        .fetch_add(1, std::sync::atomic::Ordering::AcqRel);
    Ok(result)
}

pub(crate) fn pose(
    state: &DesktopState,
    id: String,
    virtual_machine_id: Option<String>,
) -> Result<Vec<f64>, String> {
    let (model, bindings, template) = {
        let machines = state
            .physical_machines
            .lock()
            .map_err(|error| error.to_string())?;
        let machine = machines
            .iter()
            .find(|machine| machine.id == id)
            .cloned()
            .ok_or_else(|| "unknown physical machine".to_owned())?;
        let model = crate::runtime::machines::physical_machine::physical_servo_model(&machine)?;
        let virtual_machine =
            crate::runtime::machines::machine_resolution::associated_virtual_machine(
                state,
                &machine,
                virtual_machine_id.as_deref(),
            )?;
        let template =
            crate::runtime::machines::machine_resolution::body_profile(&virtual_machine.body_id)?
                .template();
        let slots = template
            .actuators
            .iter()
            .map(|slot| slot.id.clone())
            .collect::<Vec<_>>();
        let (bindings, _, _) =
            resolve_physical_loop_mapping(&virtual_machine.joint_bindings, &machine, &slots)?;
        (model, bindings, template)
    };
    if let Some(error) = binding_conflict_error(&bindings) {
        return Err(error);
    }
    let ids = bindings
        .iter()
        .map(|binding| binding.physical_id)
        .collect::<Vec<_>>();
    let positions = read_positions(state, &id, model, &ids)?;
    let canonical = Calibration {
        body_id: template.id.clone(),
        body_version: template.version,
        joints: bindings
            .iter()
            .map(|binding| JointCalibration {
                slot_id: binding.slot_id.clone(),
                physical_id: binding.physical_id,
                direction: binding.direction,
                zero_offset_rad: binding.zero_offset_rad,
                min_rad: binding.min_rad,
                max_rad: binding.max_rad,
            })
            .collect(),
    }
    .to_body_positions(&positions);
    Ok(template
        .actuators
        .iter()
        .map(|slot| {
            bindings
                .iter()
                .zip(&canonical)
                .find(|(binding, _)| binding.slot_id == slot.id)
                .map(|(_, position)| *position)
                .unwrap_or(0.0)
        })
        .collect())
}

pub(crate) fn set_torque_state(
    state: &DesktopState,
    id: String,
    enabled: bool,
    _virtual_machine_id: Option<String>,
) -> Result<PhysicalMachine, String> {
    let (model, ids) = {
        let machines = state
            .physical_machines
            .lock()
            .map_err(|error| error.to_string())?;
        let machine = machines
            .iter()
            .find(|machine| machine.id == id)
            .ok_or_else(|| "unknown physical machine".to_owned())?;
        let model = crate::runtime::machines::physical_machine::physical_servo_model(machine)?;
        (
            model,
            machine
                .census
                .iter()
                .map(|servo| servo.servo_id)
                .collect::<Vec<_>>(),
        )
    };
    set_torque(state, &id, model, &ids, enabled)?;
    let mut machines = state
        .physical_machines
        .lock()
        .map_err(|error| error.to_string())?;
    let machine = machines
        .iter_mut()
        .find(|machine| machine.id == id)
        .ok_or_else(|| "unknown physical machine".to_owned())?;
    machine.torque_enabled = enabled;
    let result = machine.clone();
    drop(machines);
    Ok(result)
}

pub(crate) fn write_servo_position(
    state: &DesktopState,
    id: String,
    servo_id: u8,
    position_rad: f64,
) -> Result<PhysicalMachine, String> {
    if !position_rad.is_finite() {
        return Err("目标位置必须是有限数字".into());
    }
    let machine = state
        .physical_machines
        .lock()
        .map_err(|error| error.to_string())?
        .iter()
        .find(|machine| machine.id == id)
        .cloned()
        .ok_or_else(|| "unknown physical machine".to_owned())?;
    if !machine.connected {
        return Err("实机没有可用连接".into());
    }
    if !machine.online_servo_ids.contains(&servo_id)
        && !machine
            .census
            .iter()
            .any(|servo| servo.servo_id == servo_id)
    {
        return Err(format!("实机舵机 {servo_id} 当前不在线"));
    }
    write_position(state, &id, servo_id, position_rad)?;
    Ok(machine)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::{Arc, Mutex};

    fn calibrated_machine(state: &DesktopState) -> String {
        let virtual_id = state.virtual_machines.lock().unwrap()[0].id.clone();
        let bindings = vec![crate::types::VirtualJointBinding {
            slot_id: "left_hip_yaw".into(),
            physical_id: 7,
        }];
        state.virtual_machines.lock().unwrap()[0].joint_bindings = bindings.clone();
        state
            .physical_machines
            .lock()
            .unwrap()
            .push(PhysicalMachine {
                execution_kind: crate::types::ExecutionKindKey::Physical,
                model_kind: Some(crate::types::ModelKindKey::Hd1910),
                servo_model: Some(crate::types::ServoModelKey::Hd1910),
                id: "physical-test".into(),
                name: "test".into(),
                usb_port: "test".into(),
                machine_code: "test".into(),
                census: vec![crate::types::ServoIdentity {
                    servo_id: 7,
                    model: "hd1910".into(),
                }],
                reconnect_hint: None,
                virtual_machine_id: Some(virtual_id.clone()),
                online_servo_ids: vec![7],
                connected: true,
                last_seen: None,
                torque_enabled: false,
                joint_directions: vec![crate::types::ServoJointDirection {
                    slot_id: "left_hip_yaw".into(),
                    direction: -1,
                }],
                joint_bindings: bindings
                    .iter()
                    .map(|binding| crate::types::ServoJointCalibration {
                        slot_id: binding.slot_id.clone(),

                        zero_offset_rad: 0.0,
                        min_rad: None,
                        max_rad: None,
                    })
                    .collect(),
            });
        virtual_id
    }

    #[test]
    fn pose_zero_reads_and_saves_calibration_with_a_read_only_runtime() {
        let state = DesktopState::new(None);
        calibrated_machine(&state);
        let events = Arc::new(Mutex::new(Vec::new()));
        let source =
            crate::runtime::session::loop_service::control_tests::read_only_loop_with_driver(
                Box::new(
                    crate::runtime::session::loop_service::control_tests::ReadOnlyDriver {
                        positions: vec![0.0; 15],
                        events: events.clone(),
                    },
                ),
            );
        *state.runtime_loop.lock().unwrap() = Some(
            crate::runtime::session::state::RuntimeInstance::read_only(source),
        );
        assert!(
            !crate::runtime::session::loop_service::status(&state)
                .unwrap()
                .unwrap()
                .has_execution
        );
        set_pose_zero_with_reader(&state, "physical-test".into(), None, |_, ids| {
            assert_eq!(ids, [7]);
            Ok(vec![0.37])
        })
        .unwrap();
        let binding = state.physical_machines.lock().unwrap()[0].joint_bindings[0].clone();
        assert_eq!(binding.zero_offset_rad, 0.37);
        assert_eq!(
            state.physical_machines.lock().unwrap()[0].joint_directions[0].direction,
            -1
        );
        assert_eq!(
            state.virtual_machines.lock().unwrap()[0].joint_bindings[0].physical_id,
            7
        );
        assert!(
            events.lock().unwrap().is_empty(),
            "calibration must not send actuator writes"
        );
        assert!(
            state
                .runtime_loop
                .lock()
                .unwrap()
                .as_ref()
                .unwrap()
                .read_source()
                .is_some()
        );
    }

    #[test]
    fn pose_zero_allows_read_only_physical_calibration_while_simulation_executes() {
        let state = DesktopState::new(None);
        let virtual_id = calibrated_machine(&state);
        crate::runtime::session::preview::open(&state, virtual_id).unwrap();
        set_pose_zero_with_reader(&state, "physical-test".into(), None, |_, _| Ok(vec![0.42]))
            .unwrap();
        assert_eq!(
            state.physical_machines.lock().unwrap()[0].joint_bindings[0].zero_offset_rad,
            0.42
        );
        assert!(
            state
                .runtime_loop
                .lock()
                .unwrap()
                .as_ref()
                .unwrap()
                .is_simulation()
        );
    }

    #[test]
    fn pose_zero_rejects_manual_physical_execution_before_reading() {
        let state = DesktopState::new(None);
        let virtual_id = calibrated_machine(&state);
        let body = crate::runtime::machines::body_catalog::official_template().unwrap();
        let binding = loop_body::LoopBinding {
            body: loop_body::BodyRuntimeView::new(body.clone()).unwrap(),
            servos: loop_body::ServoBinding::all_slots(&body, servo::ServoModel::Hd1910),
            calibration: Calibration {
                body_id: body.id.clone(),
                body_version: body.version,
                joints: body
                    .actuators
                    .iter()
                    .enumerate()
                    .map(|(index, slot)| JointCalibration {
                        slot_id: slot.id.clone(),
                        physical_id: index as u8 + 1,
                        direction: 1,
                        zero_offset_rad: 0.0,
                        min_rad: None,
                        max_rad: None,
                    })
                    .collect(),
            },
        };
        let execution = crate::runtime::policy::inference::MicroDuckPolicyRunner::open(
            binding,
            Box::new(
                crate::runtime::session::loop_service::control_tests::ReadOnlyDriver {
                    positions: vec![0.0; 15],
                    events: Arc::new(Mutex::new(Vec::new())),
                },
            ),
            loop_body::LoopConfig::default(),
        )
        .unwrap();
        *state.runtime_loop.lock().unwrap() = Some(
            crate::runtime::session::state::RuntimeInstance::physical_execution(
                execution, virtual_id,
            ),
        );
        let error = set_pose_zero_with_reader(&state, "physical-test".into(), None, |_, _| {
            panic!("physical execution must be rejected before sensor I/O")
        })
        .unwrap_err();
        assert!(error.contains("控制环运行时不能绑定姿态零位"));
        assert_eq!(
            state.physical_machines.lock().unwrap()[0].joint_bindings[0].zero_offset_rad,
            0.0
        );
    }

    #[test]
    fn debug_position_rejects_non_finite_targets_before_hardware_lookup() {
        let state = DesktopState::new(None);
        let error = write_servo_position(&state, "missing".into(), 1, f64::NAN)
            .expect_err("non-finite debug targets must be rejected");
        assert!(error.contains("有限数字"));
    }

    #[test]
    fn debug_position_requires_a_connected_machine() {
        let state = DesktopState::new(None);
        let error = write_servo_position(&state, "missing".into(), 1, 0.0)
            .expect_err("unknown debug machines must be rejected");
        assert!(error.contains("unknown physical machine"));
    }
}
