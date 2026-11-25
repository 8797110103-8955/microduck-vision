//! Log a body's backend operations while forwarding them to its execution backend.

use std::sync::Arc;

use loop_body::{BackendImuSample, BackendJointSample, BackendPositionTargets, ControlBackend};
use servo::ServoError;

use crate::platform::logging::{LogHandle, append_handle};

pub(crate) struct LoggedBody {
    inner: Box<dyn ControlBackend>,
    handle: LogHandle,
    machine_id: String,
    calibration: Option<loop_body::Calibration>,
    periodic_ranges: Option<Vec<[f64; 2]>>,
}

impl LoggedBody {
    pub(crate) fn new(
        inner: Box<dyn ControlBackend>,
        handle: LogHandle,
        machine_id: impl Into<String>,
    ) -> Self {
        Self {
            inner,
            handle,
            machine_id: machine_id.into(),
            calibration: None,
            periodic_ranges: None,
        }
    }

    pub(crate) fn with_calibration(
        mut self,
        calibration: loop_body::Calibration,
        periodic_ranges: Option<Vec<[f64; 2]>>,
    ) -> Self {
        self.calibration = Some(calibration);
        self.periodic_ranges = periodic_ranges;
        self
    }

    fn line(&self, message: impl AsRef<str>) {
        append_handle(
            &self.handle,
            &format!("{} machine_id={}", message.as_ref(), self.machine_id),
        );
    }
}

impl ControlBackend for LoggedBody {
    fn set_read_cancel(&mut self, cancel: Arc<std::sync::atomic::AtomicBool>) {
        self.inner.set_read_cancel(cancel);
    }
    fn feedback_validity(&self, ids: &[u8]) -> Vec<bool> {
        self.inner.feedback_validity(ids)
    }
    fn advance_execution(&mut self, steps: u32, dt: std::time::Duration) -> Result<(), ServoError> {
        self.inner.advance_execution(steps, dt)
    }

    fn reset_execution(&mut self) -> Result<(), ServoError> {
        self.inner.reset_execution()
    }
    fn read_positions(&mut self, ids: &[u8]) -> Result<BackendJointSample, ServoError> {
        let result = self.inner.read_positions(ids);
        match &result {
            Ok(sample) => {
                for (index, id) in ids.iter().enumerate() {
                    let mapping = self
                        .calibration
                        .as_ref()
                        .and_then(|binding| {
                            binding
                                .joints
                                .iter()
                                .enumerate()
                                .find(|(_, joint)| joint.physical_id == *id)
                        })
                        .map(|(slot_index, joint)| {
                            let raw = sample.positions.get(index).copied().unwrap_or_default();
                            let position = self
                                .periodic_ranges
                                .as_ref()
                                .and_then(|ranges| ranges.get(slot_index))
                                .map_or_else(
                                    || (raw - joint.zero_offset_rad) * f64::from(joint.direction),
                                    |range| {
                                        crate::runtime::adapters::hd1910_body::display_position(
                                            raw, joint, *range,
                                        )
                                    },
                                );
                            format!(
                                " slot_id={} direction={} zero_offset_rad={} body_position_rad={} position_turn_offset_rad={}",
                                joint.slot_id, joint.direction, joint.zero_offset_rad, position,
                                joint.zero_offset_rad + position * f64::from(joint.direction) - raw
                            )
                        })
                        .unwrap_or_default();
                    self.line(format!(
                        "servo_read id={} position_rad={} velocity_rad_s={}{mapping}",
                        id,
                        sample.positions.get(index).copied().unwrap_or_default(),
                        sample.velocities.get(index).copied().unwrap_or_default()
                    ));
                }
            }
            Err(error) => self.line(format!("servo_read_error ids={ids:?} error={error}")),
        }
        result
    }

    fn read_imu(&mut self) -> Result<Option<BackendImuSample>, ServoError> {
        let result = self.inner.read_imu();
        match &result {
            Ok(Some(sample)) => self.line(format!(
                "imu_read gyro={:?} gravity={:?}",
                sample.gyro, sample.gravity
            )),
            Ok(None) => self.line("imu_read unavailable"),
            Err(error) => self.line(format!("imu_read_error error={error}")),
        }
        result
    }

    fn write_positions(
        &mut self,
        ids: &[u8],
        targets: &BackendPositionTargets,
    ) -> Result<(), ServoError> {
        for (index, id) in ids.iter().enumerate() {
            self.line(format!(
                "servo_write_request id={} target_rad={}",
                id,
                targets.positions.get(index).copied().unwrap_or_default()
            ));
        }
        let result = self.inner.write_positions(ids, targets);
        match &result {
            Ok(()) => self.line(format!("servo_write_result ids={ids:?} result=accepted")),
            Err(error) => self.line(format!(
                "servo_write_result ids={ids:?} result=failed error={error}"
            )),
        }
        result
    }

    fn set_torque(&mut self, enabled: bool) -> Result<(), ServoError> {
        self.line(format!("servo_torque enabled={enabled}"));
        let result = self.inner.set_torque(enabled);
        if let Err(error) = &result {
            self.line(format!("servo_torque_error error={error}"));
        } else {
            self.line(format!("servo_torque_result enabled={enabled} result=ok"));
        }
        result
    }

    fn gain_capability(&self) -> loop_body::GainCapability {
        self.inner.gain_capability()
    }
    fn set_gain(&mut self, gain: u16) -> Result<(), ServoError> {
        self.line(format!("servo_gain value={gain}"));
        let result = self.inner.set_gain(gain);
        if let Err(error) = &result {
            self.line(format!("servo_gain_error error={error}"));
        }
        result
    }

    fn reboot(&mut self, id: u8) -> Result<(), ServoError> {
        self.line(format!("servo_reboot id={id}"));
        let result = self.inner.reboot(id);
        if let Err(error) = &result {
            self.line(format!("servo_reboot_error id={id} error={error}"));
        }
        result
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::platform::logging::{begin_control_loop, current_log_file, frontend_diagnostic};
    use std::time::{SystemTime, UNIX_EPOCH};

    struct FakeDriver;

    impl ControlBackend for FakeDriver {
        fn read_positions(&mut self, ids: &[u8]) -> Result<BackendJointSample, ServoError> {
            Ok(BackendJointSample {
                positions: ids.iter().map(|id| f64::from(*id)).collect(),
                velocities: vec![0.0; ids.len()],
            })
        }

        fn write_positions(
            &mut self,
            _ids: &[u8],
            _targets: &BackendPositionTargets,
        ) -> Result<(), ServoError> {
            Ok(())
        }

        fn set_torque(&mut self, _enabled: bool) -> Result<(), ServoError> {
            Ok(())
        }

        fn set_gain(&mut self, _gain: u16) -> Result<(), ServoError> {
            Ok(())
        }

        fn reboot(&mut self, _id: u8) -> Result<(), ServoError> {
            Ok(())
        }
    }

    #[test]
    fn logged_driver_records_each_motor_input_and_output() {
        let suffix = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_nanos();
        let root = crate::platform::helpers::test_directory()
            .join(format!("microduck-control-log-{suffix}"));
        let config_dir = root.join("config");
        let handle = begin_control_loop(
            Some(&config_dir),
            "physical-1",
            "duck",
            "left_hip_yaw->1 dir=1 offset=0",
        )
        .unwrap();
        let mut driver = LoggedBody::new(Box::new(FakeDriver), handle, "physical-1")
            .with_calibration(
                loop_body::Calibration {
                    body_id: "official".into(),
                    body_version: 1,
                    joints: vec![loop_body::JointCalibration {
                        slot_id: "left_knee".into(),
                        physical_id: 1,
                        direction: -1,
                        zero_offset_rad: 0.4,
                        min_rad: None,
                        max_rad: None,
                    }],
                },
                None,
            );
        driver.read_positions(&[1, 2]).unwrap();
        driver
            .write_positions(
                &[1, 2],
                &BackendPositionTargets {
                    positions: vec![0.1, -0.2],
                },
            )
            .unwrap();
        let text = std::fs::read_to_string(current_log_file(Some(&config_dir))).unwrap();
        assert!(text.contains("servo_read id=1"));
        assert!(text.contains("servo_read id=2"));
        assert!(
            text.contains(
                "slot_id=left_knee direction=-1 zero_offset_rad=0.4 body_position_rad=-0.6"
            )
        );
        frontend_diagnostic(
            Some(&config_dir),
            serde_json::json!({
                "event":"debug_pose_applied", "physicalMachineId":"physical-1",
                "sourceGeneration":7,"positions":[0,0,0,-0.6],
            })
            .to_string(),
        );
        frontend_diagnostic(Some(&config_dir), "a frontend error".into());
        let text = std::fs::read_to_string(current_log_file(Some(&config_dir))).unwrap();
        let events = text
            .lines()
            .map(|line| serde_json::from_str::<serde_json::Value>(line).unwrap())
            .collect::<Vec<_>>();
        let pose = events
            .iter()
            .find(|event| event["event"] == "debug_pose_applied")
            .unwrap();
        assert_eq!(pose["level"], "info");
        assert_eq!(pose["context"]["sourceGeneration"], 7);
        assert_eq!(pose["context"]["positions"][3], -0.6);
        assert!(
            events
                .iter()
                .any(|event| event["event"] == "frontend_error" && event["level"] == "error")
        );
        assert!(text.contains("servo_write_request id=1 target_rad=0.1"));
        assert!(text.contains("servo_write_request id=2 target_rad=-0.2"));
        let _ = std::fs::remove_dir_all(root);
    }
}
