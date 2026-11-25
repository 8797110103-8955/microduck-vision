//! Recenter physical encoders without redefining body zero or enabling motion.
//!
//! Hardware and TOML cannot commit atomically. A durable per-servo recovery record precedes
//! each instruction; uncertain hardware is rolled back, verified hardware finishes its TOML commit.

#[cfg(test)]
use std::path::Path;
use std::sync::atomic::Ordering;

use crate::platform::state::machine_store::PendingCalibration;
use crate::runtime::adapters::encoder_calibration::{EncoderBus, EncoderSnapshot};
use crate::runtime::session::state::DesktopState;
use crate::types::{EncoderCalibrationResult, PhysicalMachine, ServoJointCalibration};
use duck_body::{BodyTemplate, JointCalibration};

#[cfg(test)]
pub(crate) fn load_pending(config: Option<&Path>) -> Result<Vec<PendingCalibration>, String> {
    crate::platform::state::machine_store::load_pending_calibrations(config)
}

fn save_pending(state: &DesktopState, entries: Vec<PendingCalibration>) -> Result<(), String> {
    let config = state
        .config_dir
        .as_deref()
        .ok_or("编码器校准需要可写的配置目录")?;
    crate::platform::state::machine_store::save_pending_calibrations(config, &entries)?;
    *state
        .encoder_calibrations
        .lock()
        .map_err(|e| e.to_string())? = entries;
    // A source may have read a raw sample just before this transaction took its bus lock.
    // Its before/after revision check discards that sample rather than applying a new zero to it.
    state.config_revision.fetch_add(1, Ordering::AcqRel);
    Ok(())
}

fn update_pending(
    state: &DesktopState,
    entry: Option<PendingCalibration>,
    physical: &str,
    servo: u8,
) -> Result<(), String> {
    let mut entries = state
        .encoder_calibrations
        .lock()
        .map_err(|e| e.to_string())?
        .clone();
    entries.retain(|pending| pending.physical_id != physical || pending.joint.physical_id != servo);
    if let Some(entry) = entry {
        entries.push(entry);
    }
    save_pending(state, entries)
}

pub(crate) fn ensure_ready(state: &DesktopState, physical: &str) -> Result<(), String> {
    state.ensure_encoder_calibration_ready(physical)
}

pub(crate) fn is_pending(entries: &[PendingCalibration], physical: &str, slot: &str) -> bool {
    entries
        .iter()
        .any(|entry| entry.physical_id == physical && entry.joint.slot_id == slot)
}

fn event(
    state: &DesktopState,
    kind: &str,
    level: &str,
    pending: &PendingCalibration,
    details: serde_json::Value,
) {
    crate::platform::logging::diagnostic(
        state.config_dir.as_deref(),
        "calibration",
        level,
        kind,
        serde_json::json!({
            "duck_id": pending.physical_id, "operation_id": pending.operation_id,
            "servo_id": pending.joint.physical_id, "slot_id": pending.joint.slot_id,
            "before": pending.before, "body_angle_rad": pending.body_angle_rad,
            "old_zero_offset_rad": pending.joint.zero_offset_rad,
            "new_zero_offset_rad": pending.new_zero_offset_rad, "details": details,
        }),
    );
}

fn commit_zero(state: &DesktopState, pending: &PendingCalibration) -> Result<(), String> {
    let zero = pending
        .new_zero_offset_rad
        .ok_or("encoder calibration has not been verified")?;
    let mut machines = state.physical_machines.lock().map_err(|e| e.to_string())?;
    let machine = machines
        .iter_mut()
        .find(|machine| machine.id == pending.physical_id)
        .ok_or("unknown physical machine")?;
    if let Some(joint) = machine
        .joint_bindings
        .iter_mut()
        .find(|joint| joint.slot_id == pending.joint.slot_id)
    {
        if (joint.zero_offset_rad - pending.joint.zero_offset_rad).abs() > 1e-9
            && (joint.zero_offset_rad - zero).abs() > 1e-9
        {
            return Err("校准恢复记录与当前软件零位冲突，不能覆盖人工修改".into());
        }
        joint.zero_offset_rad = zero;
    } else {
        machine.joint_bindings.push(ServoJointCalibration {
            slot_id: pending.joint.slot_id.clone(),
            zero_offset_rad: zero,
            min_rad: pending.joint.min_rad,
            max_rad: pending.joint.max_rad,
        });
    }
    machine.torque_enabled = false;
    drop(machines);
    state.persist_physical_machines()?;
    state.config_revision.fetch_add(1, Ordering::AcqRel);
    Ok(())
}

fn recover(state: &DesktopState, bus: &mut dyn EncoderBus, id: &str) -> Result<(), String> {
    let entries = state
        .encoder_calibrations
        .lock()
        .map_err(|e| e.to_string())?
        .clone();
    for pending in entries
        .into_iter()
        .filter(|pending| pending.physical_id == id)
    {
        let servo = pending.joint.physical_id;
        bus.torque_off(&[servo])?;
        if pending.new_zero_offset_rad.is_some() {
            // No pose assumption and no repeat calibration: only complete the verified commit.
            let expected = pending.after.as_ref().ok_or("校准恢复记录缺少验证快照")?;
            let current = bus.snapshot(servo)?;
            current.validate()?;
            if current.offset_raw != expected.offset_raw
                || current.mode != expected.mode
                || current.phase != expected.phase
                || current.lock != expected.lock
            {
                return Err(format!(
                    "id={servo} 校准后的硬件偏移已被修改，不能恢复软件配置"
                ));
            }
            commit_zero(state, &pending)?;
        } else {
            // The calibration instruction may have succeeded without an acknowledgement.
            // Restore the exact old offset, not an offset inferred from the duck's current pose.
            bus.restore_offset(servo, pending.before.offset_raw, pending.before.lock)?;
            let restored = bus.snapshot(servo)?;
            restored.validate()?;
            if restored.offset_raw != pending.before.offset_raw
                || restored.lock != pending.before.lock
                || restored.phase != pending.before.phase
                || restored.mode != pending.before.mode
            {
                return Err(format!("id={servo} 编码器旧偏移恢复读回不一致"));
            }
        }
        update_pending(state, None, id, servo)?;
        event(
            state,
            "encoder_calibration_recovered",
            "info",
            &pending,
            serde_json::json!({
                "result": if pending.new_zero_offset_rad.is_some() { "configuration_committed" } else { "hardware_offset_restored" }
            }),
        );
    }
    Ok(())
}

fn rad(count: i32) -> f64 {
    servo::hd1910::position_rad_from_steps(servo::ServoModel::Hd1910, count)
}

fn stable(before: &EncoderSnapshot, after: &EncoderSnapshot) -> Result<(), String> {
    after.validate()?;
    let delta = (after.position_count - before.position_count + 2048).rem_euclid(4096) - 2048;
    if delta.abs() > 2 || after.velocity_count.abs() > 1 || before.velocity_count.abs() > 1 {
        return Err("电机仍在转动，请托住机身并保持静止后再校准".into());
    }
    if before.offset_raw != after.offset_raw
        || before.phase != after.phase
        || before.mode != after.mode
    {
        return Err("采样期间电机配置发生变化，请重新校准".into());
    }
    Ok(())
}

fn migrated_zero(
    joint: &JointCalibration,
    range: [f64; 2],
    raw: i32,
) -> Result<(f64, f64), String> {
    let angle = crate::runtime::adapters::hd1910_body::body_position(rad(raw), joint, range)
        .map_err(|e| e.to_string())?;
    let zero = -f64::from(joint.direction) * angle; // count 2048 is raw angle zero.
    for endpoint in range {
        let count = servo::hd1910::position_steps_from_rad(
            servo::ServoModel::Hd1910,
            zero + f64::from(joint.direction) * endpoint,
        )
        .map_err(|e| e.to_string())?;
        if !(4..=4091).contains(&count) {
            return Err(format!(
                "{} 当前姿态不适合将完整关节范围居中，请先将肢体摆回活动范围中部",
                joint.slot_id
            ));
        }
    }
    Ok((angle, zero))
}

pub(crate) fn center_encoders(
    state: &DesktopState,
    id: String,
    virtual_id: Option<String>,
) -> Result<EncoderCalibrationResult, String> {
    let _transition = state
        .execution_transition
        .lock()
        .map_err(|e| e.to_string())?;
    if state
        .runtime_loop
        .lock()
        .map_err(|e| e.to_string())?
        .as_ref()
        .is_some_and(|runtime| {
            runtime
                .snapshot()
                .active_body
                .is_some_and(|body| body.physical_machine_id.is_some())
        })
    {
        return Err("请先切回模拟机；编码器校准只能使用只读采集连接".into());
    }
    let machine = state
        .physical_machines
        .lock()
        .map_err(|e| e.to_string())?
        .iter()
        .find(|machine| machine.id == id)
        .cloned()
        .ok_or("unknown physical machine")?;
    if !machine.connected {
        return Err("实机没有可用连接".into());
    }
    if crate::runtime::machines::physical_machine::physical_servo_model(&machine)?
        != servo::ServoModel::Hd1910
    {
        return Err("编码器中位校准仅支持 HD1910".into());
    }
    let virtual_machine = crate::runtime::machines::machine_resolution::associated_virtual_machine(
        state,
        &machine,
        virtual_id.as_deref(),
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
        crate::runtime::machines::machine_resolution::resolve_physical_loop_mapping(
            &virtual_machine.joint_bindings,
            &machine,
            &slots,
        )?;
    if let Some(error) =
        crate::runtime::machines::machine_resolution::binding_conflict_error(&bindings)
    {
        return Err(error);
    }
    if bindings.is_empty() && ensure_ready(state, &id).is_ok() {
        return Err("没有已绑定且在线的肢体可校准".into());
    }
    let joints = bindings
        .iter()
        .map(|joint| JointCalibration {
            slot_id: joint.slot_id.clone(),
            physical_id: joint.physical_id,
            direction: joint.direction,
            zero_offset_rad: joint.zero_offset_rad,
            min_rad: joint.min_rad,
            max_rad: joint.max_rad,
        })
        .collect::<Vec<_>>();
    // This connection access shares the read-only source's bus. The maintenance transaction
    // is guarded above; it never grants a motion write permit or enables a control runtime.
    crate::platform::logging::diagnostic(
        state.config_dir.as_deref(),
        "calibration",
        "info",
        "encoder_calibration_requested",
        serde_json::json!({
            "duck_id":id,"virtual_machine_id":virtual_id,"servo_ids":joints.iter().map(|joint| joint.physical_id).collect::<Vec<_>>()
        }),
    );
    let result = crate::runtime::machines::physical_io::with_bus_locked(
        state,
        &id,
        loop_body::BackendAccess::ReadOnly,
        |bus| {
            run(
                state,
                bus.ok_or("实机没有可用连接")?,
                &id,
                &machine.online_servo_ids,
                &joints,
                template,
            )
        },
    );
    let result = match result {
        Ok(result) => result,
        Err(error) => EncoderCalibrationResult {
            machine: state
                .physical_machines
                .lock()
                .map_err(|e| e.to_string())?
                .iter()
                .find(|machine| machine.id == id)
                .cloned()
                .ok_or("unknown physical machine")?,
            calibrated_servo_ids: Vec::new(),
            error: Some(error),
        },
    };
    crate::platform::logging::diagnostic(
        state.config_dir.as_deref(),
        "calibration",
        if result.error.is_some() {
            "error"
        } else {
            "info"
        },
        "encoder_calibration_finished",
        serde_json::json!({
            "duck_id":id,"calibrated_servo_ids":result.calibrated_servo_ids,"error":result.error
        }),
    );
    Ok(result)
}

fn run(
    state: &DesktopState,
    bus: &mut dyn EncoderBus,
    id: &str,
    online_ids: &[u8],
    joints: &[JointCalibration],
    template: &BodyTemplate,
) -> Result<EncoderCalibrationResult, String> {
    // Establish writable durable recovery storage before touching hardware.
    let entries = state
        .encoder_calibrations
        .lock()
        .map_err(|e| e.to_string())?
        .clone();
    save_pending(state, entries)?;
    bus.torque_off(online_ids)?;
    if let Some(machine) = state
        .physical_machines
        .lock()
        .map_err(|e| e.to_string())?
        .iter_mut()
        .find(|machine| machine.id == id)
    {
        machine.torque_enabled = false;
    }
    recover(state, bus, id)?;
    // Preflight every mapped servo before the first irreversible instruction.
    for joint in joints {
        bus.snapshot(joint.physical_id)?
            .validate()
            .map_err(|e| format!("{} id={}: {e}", joint.slot_id, joint.physical_id))?;
    }
    let operation = crate::platform::helpers::new_machine_id("encoder-calibration");
    let mut calibrated = Vec::new();
    let mut failure = None;
    for original in joints {
        // A recovery above may have completed a software commit. Resolve its current zero.
        let mut joint = original.clone();
        if let Some(current) = state
            .physical_machines
            .lock()
            .map_err(|e| e.to_string())?
            .iter()
            .find(|machine| machine.id == id)
            .and_then(|machine| {
                machine
                    .joint_bindings
                    .iter()
                    .find(|binding| binding.slot_id == joint.slot_id)
            })
        {
            joint.zero_offset_rad = current.zero_offset_rad;
        }
        let range = template
            .actuators
            .iter()
            .find(|slot| slot.id == joint.slot_id)
            .and_then(|slot| template.joint_range(&slot.joint))
            .ok_or("unknown body joint")?;
        let outcome = (|| {
            let before = bus.snapshot(joint.physical_id)?;
            before.validate()?;
            bus.settle();
            let before_latest = bus.snapshot(joint.physical_id)?;
            stable(&before, &before_latest)?;
            let (angle, _) = migrated_zero(&joint, range, before_latest.position_count)?;
            if (before_latest.position_count - 2048).abs() <= 2 {
                return Ok(());
            }
            let mut pending = PendingCalibration {
                physical_id: id.into(),
                operation_id: operation.clone(),
                joint: joint.clone(),
                before: before_latest,
                body_angle_rad: angle,
                new_zero_offset_rad: None,
                after: None,
            };
            update_pending(state, Some(pending.clone()), id, joint.physical_id)?;
            event(
                state,
                "encoder_calibration_started",
                "info",
                &pending,
                serde_json::json!({"target_count":2048}),
            );
            let acknowledgement = bus.center(joint.physical_id);
            // Read back even after a timeout; never repeat the EEPROM-changing request.
            bus.settle();
            let after = bus.snapshot(joint.physical_id)?;
            after.validate()?;
            if (after.position_count - 2048).abs() > 4
                || after.velocity_count.abs() > 1
                || after.mode != pending.before.mode
                || after.phase != pending.before.phase
                || after.lock != pending.before.lock
            {
                return Err(format!(
                    "校准读回不一致：position={}，指令应答={acknowledgement:?}",
                    after.position_count
                ));
            }
            pending.new_zero_offset_rad =
                Some(rad(after.position_count) - f64::from(joint.direction) * angle);
            pending.after = Some(after.clone());
            update_pending(state, Some(pending.clone()), id, joint.physical_id)?;
            commit_zero(state, &pending)?;
            update_pending(state, None, id, joint.physical_id)?;
            event(
                state,
                "encoder_calibration_completed",
                "info",
                &pending,
                serde_json::json!({"after":after,"acknowledgement":acknowledgement.err()}),
            );
            Ok::<(), String>(())
        })();
        if let Err(error) = outcome {
            let cleanup = recover(state, bus, id);
            let error = match cleanup {
                Ok(()) => error,
                Err(recovery) => format!("{error}；恢复未完成：{recovery}，请再次执行校准恢复"),
            };
            crate::platform::logging::diagnostic(
                state.config_dir.as_deref(),
                "calibration",
                "error",
                "encoder_calibration_failed",
                serde_json::json!({
                    "duck_id":id,"operation_id":operation,"slot_id":joint.slot_id,"servo_id":joint.physical_id,
                    "calibrated_servo_ids":calibrated,"error":error,
                }),
            );
            failure = Some(format!(
                "{} id={}: {error}",
                joint.slot_id, joint.physical_id
            ));
            break;
        }
        calibrated.push(joint.physical_id);
    }
    let machine: PhysicalMachine = state
        .physical_machines
        .lock()
        .map_err(|e| e.to_string())?
        .iter()
        .find(|machine| machine.id == id)
        .cloned()
        .ok_or("unknown physical machine")?;
    Ok(EncoderCalibrationResult {
        machine,
        calibrated_servo_ids: calibrated,
        error: failure,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::{ServoIdentity, ServoJointDirection};
    use std::collections::BTreeMap;

    struct FakeBus {
        motors: BTreeMap<u8, EncoderSnapshot>,
        centered: Vec<u8>,
        restores: Vec<u8>,
        torque_off_calls: Vec<Vec<u8>>,
        acknowledgement_lost: bool,
        readback_fails: bool,
        readback_fail_id: u8,
        restore_fails: bool,
        moving: bool,
    }
    fn signed(raw: u16) -> i32 {
        let value = i32::from(raw & 0x7fff);
        if raw & 0x8000 == 0 { value } else { -value }
    }
    impl EncoderBus for FakeBus {
        fn torque_off(&mut self, ids: &[u8]) -> Result<(), String> {
            self.torque_off_calls.push(ids.to_vec());
            for id in ids {
                self.motors.get_mut(id).ok_or("offline")?.torque = 0;
            }
            Ok(())
        }
        fn snapshot(&mut self, id: u8) -> Result<EncoderSnapshot, String> {
            if self.readback_fails && id == self.readback_fail_id && self.centered.contains(&id) {
                self.readback_fails = false;
                return Err("readback timeout".into());
            }
            self.motors.get(&id).cloned().ok_or("offline".into())
        }
        fn center(&mut self, id: u8) -> Result<(), String> {
            self.centered.push(id);
            let motor = self.motors.get_mut(&id).unwrap();
            assert_eq!(motor.torque, 0);
            let offset = signed(motor.offset_raw) + motor.position_count - 2048;
            motor.offset_raw = offset.unsigned_abs() as u16 | if offset < 0 { 0x8000 } else { 0 };
            motor.position_count = 2048;
            if self.acknowledgement_lost {
                Err("ack timeout".into())
            } else {
                Ok(())
            }
        }
        fn restore_offset(&mut self, id: u8, offset: u16, lock: u8) -> Result<(), String> {
            self.restores.push(id);
            if self.restore_fails {
                return Err("restore disconnected".into());
            }
            let motor = self.motors.get_mut(&id).unwrap();
            assert_eq!(motor.torque, 0);
            motor.position_count += signed(motor.offset_raw) - signed(offset);
            motor.offset_raw = offset;
            motor.lock = lock;
            Ok(())
        }
        fn settle(&mut self) {
            if self.moving {
                for motor in self.motors.values_mut() {
                    motor.position_count += 10;
                }
            }
        }
    }

    fn fixture() -> (DesktopState, FakeBus, Vec<JointCalibration>, BodyTemplate) {
        let config = crate::platform::helpers::test_directory()
            .join(crate::platform::helpers::new_machine_id("encoder-test"))
            .join("config");
        let state = DesktopState::new(Some(config));
        let joints = vec![
            JointCalibration {
                slot_id: "left_hip_roll".into(),
                physical_id: 1,
                direction: -1,
                zero_offset_rad: rad(15),
                min_rad: Some(-0.3),
                max_rad: Some(0.3),
            },
            JointCalibration {
                slot_id: "right_knee".into(),
                physical_id: 7,
                direction: -1,
                zero_offset_rad: rad(4270),
                min_rad: None,
                max_rad: None,
            },
        ];
        state
            .physical_machines
            .lock()
            .unwrap()
            .push(PhysicalMachine {
                execution_kind: crate::types::ExecutionKindKey::Physical,
                model_kind: Some(crate::types::ModelKindKey::Hd1910),
                servo_model: Some(crate::types::ServoModelKey::Hd1910),
                id: "physical-test".into(),
                name: "HD1910".into(),
                usb_port: "test".into(),
                machine_code: "test".into(),
                census: vec![
                    ServoIdentity {
                        servo_id: 1,
                        model: "hd1910".into(),
                    },
                    ServoIdentity {
                        servo_id: 7,
                        model: "hd1910".into(),
                    },
                ],
                reconnect_hint: None,
                virtual_machine_id: None,
                online_servo_ids: vec![1, 7],
                connected: true,
                last_seen: None,
                torque_enabled: false,
                joint_directions: joints
                    .iter()
                    .map(|joint| ServoJointDirection {
                        slot_id: joint.slot_id.clone(),
                        direction: joint.direction,
                    })
                    .collect(),
                joint_bindings: joints
                    .iter()
                    .map(|joint| ServoJointCalibration {
                        slot_id: joint.slot_id.clone(),
                        zero_offset_rad: joint.zero_offset_rad,
                        min_rad: joint.min_rad,
                        max_rad: joint.max_rad,
                    })
                    .collect(),
            });
        state.persist_physical_machines().unwrap();
        let motors = [(1, 4130), (7, -117)]
            .into_iter()
            .map(|(id, position_count)| {
                (
                    id,
                    EncoderSnapshot {
                        firmware: [3, 46],
                        model: [10, 31],
                        mode: 4,
                        phase: 52,
                        resolution: 1,
                        limits: [0, 4095],
                        offset_raw: 0,
                        lock: 1,
                        torque: 0,
                        position_count,
                        velocity_count: 0,
                    },
                )
            })
            .collect();
        (
            state,
            FakeBus {
                motors,
                centered: vec![],
                restores: vec![],
                torque_off_calls: vec![],
                acknowledgement_lost: false,
                readback_fails: false,
                readback_fail_id: 1,
                restore_fails: false,
                moving: false,
            },
            joints,
            crate::runtime::machines::body_catalog::official_template().unwrap(),
        )
    }

    #[test]
    fn logged_4130_pose_centers_without_changing_angle_direction_ids_or_limits() {
        let (state, mut bus, joints, template) = fixture();
        let virtual_before = state.virtual_machines.lock().unwrap().clone();
        let result = run(
            &state,
            &mut bus,
            "physical-test",
            &[1, 7],
            &joints,
            &template,
        )
        .unwrap();
        assert!(result.error.is_none(), "{:?}", result.error);
        assert_eq!(result.calibrated_servo_ids, [1, 7]);
        assert_eq!(bus.centered, [1, 7]);
        assert!(bus.restores.is_empty());
        let migrated = &result.machine.joint_bindings[0];
        let count = servo::hd1910::position_steps_from_rad(
            servo::ServoModel::Hd1910,
            migrated.zero_offset_rad,
        )
        .unwrap();
        assert_eq!(count, 2029);
        assert_eq!(migrated.min_rad, Some(-0.3));
        assert_eq!(migrated.max_rad, Some(0.3));
        assert_eq!(result.machine.joint_directions[0].direction, -1);
        assert_eq!(
            state.virtual_machines.lock().unwrap()[0].joint_bindings,
            virtual_before[0].joint_bindings
        );
        for (index, joint) in joints.iter().enumerate() {
            let range = template.joint_range(&joint.slot_id).unwrap();
            let before = crate::runtime::adapters::hd1910_body::body_position(
                rad(if index == 0 { 4130 } else { -117 }),
                joint,
                range,
            )
            .unwrap();
            let mut migrated = joint.clone();
            migrated.zero_offset_rad = result.machine.joint_bindings[index].zero_offset_rad;
            let after =
                crate::runtime::adapters::hd1910_body::body_position(rad(2048), &migrated, range)
                    .unwrap();
            assert!((before - after).abs() < 1e-12);
        }
        ensure_ready(&state, "physical-test").unwrap();
        assert!(
            load_pending(state.config_dir.as_deref())
                .unwrap()
                .is_empty()
        );
        let log = std::fs::read_to_string(crate::platform::logging::current_log_file(
            state.config_dir.as_deref(),
        ))
        .unwrap();
        assert_eq!(
            log.matches("\"event\":\"encoder_calibration_started\"")
                .count(),
            2
        );
        assert_eq!(
            log.matches("\"event\":\"encoder_calibration_completed\"")
                .count(),
            2
        );
        assert!(log.contains("body_angle_rad") && log.contains("operation_id"));
    }

    #[test]
    fn a_lost_acknowledgement_is_verified_without_sending_calibration_twice() {
        let (state, mut bus, joints, template) = fixture();
        bus.acknowledgement_lost = true;
        let result = run(
            &state,
            &mut bus,
            "physical-test",
            &[1, 7],
            &joints,
            &template,
        )
        .unwrap();
        assert!(result.error.is_none());
        assert_eq!(bus.centered, [1, 7]);
        assert!(bus.restores.is_empty());
    }

    #[test]
    fn incompatible_firmware_or_moving_joints_never_receive_a_calibration_instruction() {
        let (state, mut bus, joints, template) = fixture();
        bus.motors.get_mut(&7).unwrap().firmware = [3, 42];
        assert!(
            run(
                &state,
                &mut bus,
                "physical-test",
                &[1, 7],
                &joints,
                &template
            )
            .is_err()
        );
        assert!(bus.centered.is_empty());
        bus.motors.get_mut(&7).unwrap().firmware = [3, 46];
        bus.moving = true;
        let result = run(
            &state,
            &mut bus,
            "physical-test",
            &[1, 7],
            &joints,
            &template,
        )
        .unwrap();
        assert!(result.error.unwrap().contains("仍在转动"));
        assert!(bus.centered.is_empty());
    }

    #[test]
    fn failed_readback_restores_the_exact_hardware_offset_and_keeps_software_zero() {
        let (state, mut bus, joints, template) = fixture();
        bus.readback_fails = true;
        let result = run(
            &state,
            &mut bus,
            "physical-test",
            &[1, 7],
            &joints,
            &template,
        )
        .unwrap();
        assert!(result.error.unwrap().contains("readback timeout"));
        assert!(result.calibrated_servo_ids.is_empty());
        assert_eq!(bus.centered, [1]);
        assert_eq!(bus.restores, [1]);
        assert_eq!(bus.motors[&1].position_count, 4130);
        assert_eq!(bus.motors[&1].offset_raw, 0);
        assert_eq!(
            result.machine.joint_bindings[0].zero_offset_rad,
            joints[0].zero_offset_rad
        );
        ensure_ready(&state, "physical-test").unwrap();
    }

    #[test]
    fn an_uncertain_write_survives_restart_blocks_control_and_recovers_without_pose_assumptions() {
        let (state, mut bus, joints, template) = fixture();
        bus.readback_fails = true;
        bus.restore_fails = true;
        let result = run(
            &state,
            &mut bus,
            "physical-test",
            &[1, 7],
            &joints,
            &template,
        )
        .unwrap();
        assert!(result.error.unwrap().contains("恢复未完成"));
        assert!(ensure_ready(&state, "physical-test").is_err());
        assert!(
            crate::runtime::machines::physical_machine::physical_machine_delete(
                &state,
                "physical-test".into()
            )
            .is_err()
        );
        let virtual_id = state.virtual_machines.lock().unwrap()[0].id.clone();
        assert!(
            crate::runtime::machines::physical_machine::physical_machine_mount(
                &state,
                "physical-test".into(),
                virtual_id.clone()
            )
            .is_err()
        );
        assert!(
            crate::runtime::session::loop_service::ensure_mapping_editable(&state, &virtual_id)
                .is_err()
        );
        assert!(is_pending(
            &state.encoder_calibrations.lock().unwrap(),
            "physical-test",
            "left_hip_roll"
        ));
        let restarted = DesktopState::new(state.config_dir.clone());
        assert!(ensure_ready(&restarted, "physical-test").is_err());
        bus.restore_fails = false;
        bus.motors.get_mut(&1).unwrap().position_count -= 28; // User moved it after failure.
        recover(&restarted, &mut bus, "physical-test").unwrap();
        assert_eq!(bus.motors[&1].position_count, 4102);
        assert_eq!(bus.centered, [1]);
        assert_eq!(
            restarted.physical_machines.lock().unwrap()[0].joint_bindings[0].zero_offset_rad,
            joints[0].zero_offset_rad
        );
        ensure_ready(&restarted, "physical-test").unwrap();
    }

    #[test]
    fn verified_hardware_finishes_a_failed_toml_commit_without_recalibrating() {
        let (state, mut bus, joints, template) = fixture();
        let config = state.config_dir.as_ref().unwrap();
        let robots = config.join("robots.toml");
        let backup = config.join("robots.saved.toml");
        std::fs::rename(&robots, &backup).unwrap();
        std::fs::create_dir(&robots).unwrap();
        let result = run(
            &state,
            &mut bus,
            "physical-test",
            &[1, 7],
            &joints,
            &template,
        )
        .unwrap();
        assert!(result.error.is_some());
        assert_eq!(bus.centered, [1]);
        assert!(bus.restores.is_empty());
        assert!(ensure_ready(&state, "physical-test").is_err());
        std::fs::remove_dir(&robots).unwrap();
        std::fs::rename(backup, &robots).unwrap();
        let restarted = DesktopState::new(state.config_dir.clone());
        recover(&restarted, &mut bus, "physical-test").unwrap();
        assert_eq!(bus.centered, [1]);
        assert!(bus.restores.is_empty());
        let actual =
            restarted.physical_machines.lock().unwrap()[0].joint_bindings[0].zero_offset_rad;
        assert!((actual - rad(2029)).abs() < 1e-12);
        ensure_ready(&restarted, "physical-test").unwrap();
    }

    #[test]
    fn no_hardware_is_touched_if_the_recovery_record_cannot_be_saved() {
        let (mut state, mut bus, joints, template) = fixture();
        let invalid = state.config_dir.as_ref().unwrap().join("not-a-directory");
        std::fs::write(&invalid, b"file").unwrap();
        state.config_dir = Some(invalid);
        assert!(
            run(
                &state,
                &mut bus,
                "physical-test",
                &[1, 7],
                &joints,
                &template
            )
            .is_err()
        );
        assert!(bus.torque_off_calls.is_empty());
        assert!(bus.centered.is_empty());
    }

    #[test]
    fn a_later_servo_failure_keeps_completed_calibrations_and_rolls_back_only_the_failed_servo() {
        let (state, mut bus, joints, template) = fixture();
        bus.readback_fails = true;
        bus.readback_fail_id = 7;
        let result = run(
            &state,
            &mut bus,
            "physical-test",
            &[1, 7],
            &joints,
            &template,
        )
        .unwrap();
        assert!(result.error.is_some());
        assert_eq!(result.calibrated_servo_ids, [1]);
        assert_eq!(bus.centered, [1, 7]);
        assert_eq!(bus.restores, [7]);
        assert!((result.machine.joint_bindings[0].zero_offset_rad - rad(2029)).abs() < 1e-12);
        assert_eq!(
            result.machine.joint_bindings[1].zero_offset_rad,
            joints[1].zero_offset_rad
        );
        assert_eq!(bus.motors[&7].position_count, -117);
        ensure_ready(&state, "physical-test").unwrap();
    }

    #[test]
    fn recentering_handles_both_directions_and_rejects_a_pose_that_cannot_fit_the_joint_travel() {
        let (_, _, joints, template) = fixture();
        let range = template.joint_range("left_hip_roll").unwrap();
        for direction in [-1, 1] {
            let mut joint = joints[0].clone();
            joint.direction = direction;
            let (angle, zero) = migrated_zero(&joint, range, 4130).unwrap();
            assert!(((-zero) * f64::from(direction) - angle).abs() < 1e-12);
        }
        assert!(migrated_zero(&joints[0], range, 2060).is_err());
    }
}
