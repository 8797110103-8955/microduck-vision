//! Keep a bounded body joint in its calibration turn while preserving the bus's absolute turn.
//!
//! A turn correction belongs to the physical body, not the wire codec or a continuous simulator.
//! In particular, wrapping a goal into 0..4095 can request an almost full turn of real motion.

use std::collections::HashMap;
use std::f64::consts::{PI, TAU};

use loop_body::{BackendJointSample, BackendPositionTargets};
use loop_body::{Calibration, ControlBackend, JointCalibration};
use servo::ServoError;
#[cfg(test)]
use servo::ServoModel;

use servo::hd1910::PositionDomain;

/// Choose the periodic equivalent nearest the model joint's travel. Preserve measured poses
/// outside that travel too: a manually positioned joint must not be clipped to a model limit.
/// A continuous joint or a tie between two turns needs an explicit reference instead.
pub(crate) fn body_position(
    raw: f64,
    joint: &JointCalibration,
    range: [f64; 2],
) -> Result<f64, ServoError> {
    let position = (raw - joint.zero_offset_rad) * f64::from(joint.direction);
    let [min, max] = range;
    if !position.is_finite() || !min.is_finite() || !max.is_finite() || min > max {
        return Err(joint_error(
            joint,
            "non-finite position or invalid joint range",
        ));
    }
    let center = (min + max) * 0.5;
    let projected = position + ((center - position) / TAU).round() * TAU;
    if max - min >= TAU || ((projected - center).abs() - PI).abs() < 1e-10 {
        return Err(joint_error(
            joint,
            format!(
                "反馈无法唯一映射到关节范围：raw_rad={raw} zero_rad={} body_range={range:?}",
                joint.zero_offset_rad
            ),
        ));
    }
    Ok(projected)
}

/// Read-only inspection also shows out-of-travel manual poses; it never feeds a driving policy.
pub(crate) fn display_position(raw: f64, joint: &JointCalibration, range: [f64; 2]) -> f64 {
    body_position(raw, joint, range).unwrap_or_else(|_| {
        let position = (raw - joint.zero_offset_rad) * f64::from(joint.direction);
        (position + PI).rem_euclid(TAU) - PI
    })
}

fn joint_error(joint: &JointCalibration, detail: impl std::fmt::Display) -> ServoError {
    ServoError::Bus(format!(
        "HD1910 slot={} id={}: {detail}",
        joint.slot_id, joint.physical_id
    ))
}

pub(crate) struct Hd1910Body {
    inner: Box<dyn ControlBackend>,
    calibration: Calibration,
    ranges: Vec<[f64; 2]>,
    active_ids: Vec<Option<u8>>,
    domains: HashMap<u8, PositionDomain>,
    // Reference-branch raw angle minus actual bus angle, established by successful feedback.
    offsets: Option<Vec<f64>>,
    hold_written: bool,
}

impl Hd1910Body {
    pub(crate) fn new(
        inner: Box<dyn ControlBackend>,
        calibration: Calibration,
        ranges: Vec<[f64; 2]>,
        active_ids: Vec<Option<u8>>,
        domains: HashMap<u8, PositionDomain>,
    ) -> Self {
        Self {
            inner,
            calibration,
            ranges,
            active_ids,
            domains,
            offsets: None,
            hold_written: false,
        }
    }

    fn validate_position(&self, index: usize, position: f64) -> Result<(), ServoError> {
        let joint = &self.calibration.joints[index];
        self.domains
            .get(&joint.physical_id)
            .ok_or_else(|| joint_error(joint, "未读到硬件位置模式及限位，不能写入目标"))?
            .validate(position)
            .map_err(|error| joint_error(joint, error))
    }
}

impl ControlBackend for Hd1910Body {
    fn set_read_cancel(&mut self, cancel: std::sync::Arc<std::sync::atomic::AtomicBool>) {
        self.inner.set_read_cancel(cancel);
    }
    fn feedback_validity(&self, ids: &[u8]) -> Vec<bool> {
        self.inner.feedback_validity(ids)
    }
    fn read_positions(&mut self, ids: &[u8]) -> Result<BackendJointSample, ServoError> {
        self.offsets = None;
        let result = (|| {
            let mut sample = self.inner.read_positions(ids)?;
            let size = self.calibration.joints.len();
            if ids.len() != size
                || sample.positions.len() != size
                || sample.velocities.len() != size
                || self.ranges.len() != size
                || self.active_ids.len() != size
            {
                return Err(ServoError::Bus(
                    "HD1910 body feedback length mismatch".into(),
                ));
            }
            let valid = self.inner.feedback_validity(ids);
            let mut offsets = vec![0.0; size];
            for (index, joint) in self.calibration.joints.iter().enumerate() {
                if self.active_ids[index].is_none() {
                    continue;
                }
                if !valid.get(index).copied().unwrap_or(false)
                    || !sample.velocities[index].is_finite()
                {
                    return Err(joint_error(joint, "invalid physical feedback"));
                }
                let raw = sample.positions[index];
                let position = body_position(raw, joint, self.ranges[index])?;
                sample.positions[index] =
                    joint.zero_offset_rad + position * f64::from(joint.direction);
                offsets[index] = sample.positions[index] - raw;
            }
            self.offsets = Some(offsets);
            Ok(sample)
        })();
        if result.is_err() {
            self.hold_written = false;
        }
        result
    }
    fn read_imu(&mut self) -> Result<Option<loop_body::BackendImuSample>, ServoError> {
        self.inner.read_imu()
    }
    fn write_positions(
        &mut self,
        ids: &[u8],
        targets: &BackendPositionTargets,
    ) -> Result<(), ServoError> {
        self.hold_written = false;
        let offsets = self.offsets.as_ref().ok_or_else(|| {
            ServoError::Bus("HD1910 requires valid feedback before writing targets".into())
        })?;
        if ids.len() != offsets.len() || targets.positions.len() != offsets.len() {
            return Err(ServoError::Bus("HD1910 body target length mismatch".into()));
        }
        let mut wire = targets.clone();
        for (index, position) in wire.positions.iter_mut().enumerate() {
            if self.active_ids[index].is_none() {
                continue;
            }
            *position -= offsets[index];
            self.validate_position(index, *position)?;
        }
        // Validate every servo before the first bus write. Never wrap a target independently.
        self.inner.write_positions(ids, &wire)?;
        self.hold_written = true;
        Ok(())
    }
    fn set_torque(&mut self, enabled: bool) -> Result<(), ServoError> {
        if enabled && !self.hold_written {
            return Err(ServoError::Bus(
                "HD1910 must write a validated measured hold target before enabling torque".into(),
            ));
        }
        if !enabled {
            self.hold_written = false;
        }
        self.inner.set_torque(enabled)
    }
    fn gain_capability(&self) -> loop_body::GainCapability {
        self.inner.gain_capability()
    }
    fn set_gain(&mut self, gain: u16) -> Result<(), ServoError> {
        self.inner.set_gain(gain)
    }
    fn reboot(&mut self, id: u8) -> Result<(), ServoError> {
        self.offsets = None;
        self.hold_written = false;
        self.inner.reboot(id)
    }
    fn advance_execution(&mut self, steps: u32, dt: std::time::Duration) -> Result<(), ServoError> {
        self.inner.advance_execution(steps, dt)
    }
    fn reset_execution(&mut self) -> Result<(), ServoError> {
        self.offsets = None;
        self.hold_written = false;
        self.inner.reset_execution()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::{Arc, Mutex};

    #[derive(Default)]
    struct Probe {
        counts: Vec<i32>,
        written: Vec<Vec<i32>>,
        events: Vec<&'static str>,
        fail_read: bool,
    }

    struct Driver(Arc<Mutex<Probe>>);
    impl ControlBackend for Driver {
        fn read_positions(&mut self, _: &[u8]) -> Result<BackendJointSample, ServoError> {
            let mut probe = self.0.lock().unwrap();
            probe.events.push("read");
            if probe.fail_read {
                return Err(ServoError::Bus("disconnected".into()));
            }
            Ok(BackendJointSample {
                positions: probe.counts.iter().map(|count| rad(*count)).collect(),
                velocities: vec![0.25; probe.counts.len()],
            })
        }
        fn write_positions(
            &mut self,
            _: &[u8],
            targets: &BackendPositionTargets,
        ) -> Result<(), ServoError> {
            let counts = targets
                .positions
                .iter()
                .map(|position| {
                    servo::hd1910::position_steps_from_rad(ServoModel::Hd1910, *position).unwrap()
                })
                .collect();
            let mut probe = self.0.lock().unwrap();
            probe.events.push("write");
            probe.written.push(counts);
            Ok(())
        }
        fn set_torque(&mut self, enabled: bool) -> Result<(), ServoError> {
            self.0
                .lock()
                .unwrap()
                .events
                .push(if enabled { "torque_on" } else { "torque_off" });
            Ok(())
        }
        fn set_gain(&mut self, _: u16) -> Result<(), ServoError> {
            self.0.lock().unwrap().events.push("gain");
            Ok(())
        }
        fn reboot(&mut self, _: u8) -> Result<(), ServoError> {
            Ok(())
        }
    }

    fn rad(count: i32) -> f64 {
        servo::hd1910::position_rad_from_steps(ServoModel::Hd1910, count)
    }

    fn domain(multi_turn: bool) -> PositionDomain {
        PositionDomain {
            limits: [0, if multi_turn { 0 } else { 4095 }],
            mode: 4,
            resolution: 1,
        }
    }

    // Actual first stand-policy frame from session 1795199548668-27120. Keep both raw turns
    // and saved zeros: replacing these with already-normalized angles would hide the bug.
    fn logged_scene(multi_turn: bool) -> (loop_body::LoopBinding, Hd1910Body, Arc<Mutex<Probe>>) {
        let template = crate::runtime::adapters::body_assets::test_template();
        let ids = [9, 1, 3, 4, 5, 11, 12, 13, 14, 21, 10, 2, 6, 7, 8];
        let zeros = [
            4053, 15, 267, 3847, 4301, -340, 435, 1792, 24, 3902, 11, 1297, 2187, 4270, 2144,
        ];
        let directions = [1, -1, -1, -1, -1, 1, 1, -1, 1, 1, 1, -1, -1, -1, -1];
        let counts = vec![
            3882, 4097, 517, 3875, 1, 5, 449, 1900, 22, 3906, 4094, 1011, 1919, 532, 2653,
        ];
        let calibration = Calibration {
            body_id: template.id.clone(),
            body_version: template.version,
            joints: template
                .actuators
                .iter()
                .enumerate()
                .map(|(i, slot)| JointCalibration {
                    slot_id: slot.id.clone(),
                    physical_id: ids[i],
                    direction: directions[i],
                    zero_offset_rad: rad(zeros[i]),
                    min_rad: None,
                    max_rad: None,
                })
                .collect(),
        };
        let ranges = template
            .actuators
            .iter()
            .map(|slot| template.joint_range(&slot.joint).unwrap())
            .collect();
        let probe = Arc::new(Mutex::new(Probe {
            counts,
            ..Probe::default()
        }));
        let driver = Hd1910Body::new(
            Box::new(Driver(probe.clone())),
            calibration.clone(),
            ranges,
            ids.iter().copied().map(Some).collect(),
            ids.iter().map(|id| (*id, domain(multi_turn))).collect(),
        );
        let binding = loop_body::LoopBinding {
            body: loop_body::BodyRuntimeView::new(template.clone()).unwrap(),
            servos: loop_body::ServoBinding::all_slots(&template, ServoModel::Hd1910),
            calibration,
        };
        (binding, driver, probe)
    }

    #[test]
    fn logged_feedback_and_hold_roundtrip_through_the_real_body_adapter() {
        let (binding, driver, probe) = logged_scene(true);
        let mut body =
            loop_body::BodyLoop::open(binding, Box::new(driver), loop_body::LoopConfig::default())
                .unwrap();
        body.set_velocity_observation_delay(0).unwrap();
        body.arm_at_current_pose().unwrap();
        let positions = body.status().positions;
        for (index, degrees) in [
            (1, 1.23046875),
            (4, 17.9296875),
            (10, -1.142578125),
            (13, -31.46484375),
            // Real telemetry was ~3deg beyond the model roll limit. Keep it measured,
            // rather than clipping it or inventing another turn to fit the model.
            (11, 25.13671875),
        ] {
            assert!((positions[index].to_degrees() - degrees).abs() < 1e-9);
        }
        assert_eq!(body.sensor_frame().velocities[1], -0.25);
        assert_eq!(body.sensor_frame().velocities[10], 0.25);
        let probe = probe.lock().unwrap();
        assert_eq!(
            probe.written.as_slice(),
            std::slice::from_ref(&probe.counts)
        );
        assert_eq!(probe.events, ["read", "write", "torque_on", "gain"]);
    }

    #[test]
    fn actual_single_turn_limits_stop_logged_out_of_turn_hold_before_any_write() {
        let (binding, driver, probe) = logged_scene(false);
        let mut body =
            loop_body::BodyLoop::open(binding, Box::new(driver), loop_body::LoopConfig::default())
                .unwrap();
        let error = body.arm_at_current_pose().unwrap_err().to_string();
        assert!(error.contains("left_hip_roll id=1"), "{error}");
        assert!(
            error.contains("count=4097") && error.contains("0..4095"),
            "{error}"
        );
        assert_eq!(probe.lock().unwrap().events, ["read"]);
        assert!(!body.status().torque);
    }

    #[test]
    fn policy_target_crossing_single_turn_boundary_rejects_the_entire_frame() {
        let (binding, mut driver, probe) = logged_scene(false);
        // Recenter only the fake encoder to make the initial pose executable.
        probe.lock().unwrap().counts[1] = 1;
        let ids = binding.calibration.physical_ids();
        let measured = driver.read_positions(&ids).unwrap();
        let mut target = BackendPositionTargets {
            positions: measured.positions,
        };
        driver.write_positions(&ids, &target).unwrap();
        let held = probe.lock().unwrap().counts.clone();
        assert_eq!(probe.lock().unwrap().written, [held]);
        // right_hip_yaw is currently -1.14deg. Zero on the same physical turn is count 4107,
        // not count 11. Modulo would send it backwards by nearly a complete revolution.
        target.positions[10] = binding.calibration.joints[10].zero_offset_rad;
        let error = driver
            .write_positions(&ids, &target)
            .unwrap_err()
            .to_string();
        assert!(
            error.contains("right_hip_yaw id=10") && error.contains("count=4107"),
            "{error}"
        );
        assert_eq!(probe.lock().unwrap().written.len(), 1);
        assert!(driver.set_torque(true).is_err());
        driver.set_torque(false).unwrap();
        assert_eq!(probe.lock().unwrap().events.last(), Some(&"torque_off"));
    }

    #[test]
    fn loss_of_feedback_or_missing_configuration_cannot_reuse_an_old_turn() {
        let (binding, mut driver, probe) = logged_scene(true);
        let ids = binding.calibration.physical_ids();
        let target = BackendPositionTargets {
            positions: driver.read_positions(&ids).unwrap().positions,
        };
        driver.domains.remove(&1);
        assert!(
            driver
                .write_positions(&ids, &target)
                .unwrap_err()
                .to_string()
                .contains("未读到硬件")
        );
        assert!(probe.lock().unwrap().written.is_empty());
        probe.lock().unwrap().fail_read = true;
        assert!(driver.read_positions(&ids).is_err());
        assert!(driver.write_positions(&ids, &target).is_err());
        assert!(driver.set_torque(true).is_err());
        driver.set_torque(false).unwrap();
    }

    #[test]
    fn ambiguous_or_invalid_feedback_never_enters_the_policy() {
        let (binding, mut driver, probe) = logged_scene(true);
        let joint = &binding.calibration.joints[1];
        assert!(body_position(rad(4097), joint, [-TAU, TAU]).is_err());
        assert!(body_position(joint.zero_offset_rad + PI, joint, [-0.1, 0.1]).is_err());
        assert!(body_position(f64::NAN, joint, [-0.1, 0.1]).is_err());
        driver.ranges[1] = [-TAU, TAU];
        assert!(
            driver
                .read_positions(&binding.calibration.physical_ids())
                .is_err()
        );
        assert!(driver.offsets.is_none());
        assert!(probe.lock().unwrap().written.is_empty());
    }

    #[test]
    fn read_only_hot_mapping_uses_the_same_turn_and_direction_as_control() {
        let (binding, _, probe) = logged_scene(true);
        let current = Arc::new(Mutex::new(binding.calibration.joints.clone()));
        let cache = current.clone();
        let ids = binding.calibration.physical_ids();
        let ranges = binding
            .body
            .template()
            .actuators
            .iter()
            .map(|slot| binding.body.template().joint_range(&slot.joint).unwrap())
            .collect();
        let mut source = super::super::read_only_body::ReadOnlyBody::new(
            Box::new(Driver(probe)),
            ids.clone(),
            move || Ok(cache.lock().unwrap().iter().cloned().map(Some).collect()),
        )
        .with_periodic_ranges(ranges);
        let first = source.read_positions(&ids).unwrap();
        assert!((first.positions[1].to_degrees() - 1.23046875).abs() < 1e-9);
        current.lock().unwrap()[1].direction = 1;
        let second = source.read_positions(&ids).unwrap();
        assert!((second.positions[1] + first.positions[1]).abs() < 1e-12);
        assert_eq!(second.velocities[1], -first.velocities[1]);
    }

    #[test]
    fn diagnostics_keep_absolute_io_and_explain_the_body_turn_correction() {
        let (binding, mut driver, _) = logged_scene(true);
        let suffix = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos();
        let config = crate::platform::helpers::test_directory()
            .join(format!("hd1910-turn-log-{suffix}"))
            .join("config");
        let handle = crate::platform::logging::begin_control_loop(
            Some(&config),
            "physical-test",
            "duck",
            "turn-test",
        )
        .unwrap();
        driver.inner = Box::new(
            crate::runtime::adapters::logged_body::LoggedBody::new(
                driver.inner,
                handle,
                "physical-test",
            )
            .with_calibration(binding.calibration.clone(), Some(driver.ranges.clone())),
        );
        let ids = binding.calibration.physical_ids();
        let measured = driver.read_positions(&ids).unwrap();
        driver
            .write_positions(
                &ids,
                &BackendPositionTargets {
                    positions: measured.positions,
                },
            )
            .unwrap();
        let log =
            std::fs::read_to_string(crate::platform::logging::current_log_file(Some(&config)))
                .unwrap();
        let events = log
            .lines()
            .map(|line| serde_json::from_str::<serde_json::Value>(line).unwrap())
            .collect::<Vec<_>>();
        let message = |kind: &str| {
            events
                .iter()
                .find_map(|event| {
                    let message = event["message"].as_str()?;
                    (event["event"] == kind
                        && message.split_whitespace().any(|part| part == "id=1"))
                    .then_some(message)
                })
                .unwrap()
        };
        let field = |message: &str, key: &str| {
            message
                .split_whitespace()
                .find_map(|part| {
                    let (name, value) = part.split_once('=')?;
                    (name == key).then(|| value.parse::<f64>().unwrap())
                })
                .unwrap()
        };
        let read = message("servo_read");
        assert!((field(read, "position_rad") - rad(4097)).abs() < 1e-12);
        assert!((field(read, "body_position_rad").to_degrees() - 1.23046875).abs() < 1e-9);
        assert!((field(read, "position_turn_offset_rad") + TAU).abs() < 1e-12);
        assert!((field(message("servo_write_request"), "target_rad") - rad(4097)).abs() < 1e-12);
        assert!(log.contains("servo_write_result") && log.contains("result=accepted"));
    }

    #[test]
    fn modes_limits_resolution_and_signed_multi_turn_values_are_distinct() {
        let single = domain(false);
        for count in [0, 4095] {
            single.validate(rad(count)).unwrap();
        }
        for count in [-1, 4096] {
            assert!(single.validate(rad(count)).is_err());
        }
        for count in [-32767, -340, 4301, 32767] {
            domain(true).validate(rad(count)).unwrap();
        }
        assert!(domain(true).validate(rad(32768)).is_err());
        assert!(
            PositionDomain { mode: 1, ..single }
                .validate(rad(2048))
                .is_err()
        );
        assert!(
            PositionDomain {
                resolution: 2,
                ..single
            }
            .validate(rad(2048))
            .is_err()
        );
        assert!(
            PositionDomain {
                limits: [1000, 2000],
                ..single
            }
            .validate(rad(999))
            .is_err()
        );
    }

    #[test]
    fn a_feedback_turn_change_preserves_body_pose_but_updates_the_hold_branch() {
        let (binding, mut driver, probe) = logged_scene(true);
        let ids = binding.calibration.physical_ids();
        let first = driver.read_positions(&ids).unwrap();
        probe.lock().unwrap().counts[1] -= 4096;
        let next = driver.read_positions(&ids).unwrap();
        assert!((first.positions[1] - next.positions[1]).abs() < 1e-12);
        driver
            .write_positions(
                &ids,
                &BackendPositionTargets {
                    positions: next.positions,
                },
            )
            .unwrap();
        assert_eq!(probe.lock().unwrap().written[0][1], 1);
    }
}
