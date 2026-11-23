//! A MuJoCo body with optional physical actuators in canonical joint order.
//!
//! Bus IDs come only from the resolved virtual mapping. Unmatched limbs retain simulated
//! telemetry and targets; an error from a matched servo is never hidden by simulation.

use loop_body::{BackendJointSample, BackendPositionTargets};
use loop_body::{Calibration, ControlBackend};
use servo::ServoError;

pub(crate) struct AttachedBody {
    simulation: Box<dyn ControlBackend>,
    physical: Box<dyn ControlBackend>,
    active_ids: Vec<Option<u8>>,
    calibration: Calibration,
    simulation_ids: Vec<u8>,
    // Dropping a registry revokes its sessions, even if the session itself is retained.
    _simulation_registry: loop_body::BackendRegistry,
}

impl AttachedBody {
    pub(crate) fn new(
        simulation: Box<dyn ControlBackend>,
        physical: Box<dyn ControlBackend>,
        active_ids: Vec<Option<u8>>,
        calibration: Calibration,
        simulation_registry: loop_body::BackendRegistry,
    ) -> Self {
        let simulation_ids = (1..=active_ids.len()).map(|id| id as u8).collect();
        Self {
            simulation,
            physical,
            active_ids,
            calibration,
            simulation_ids,
            _simulation_registry: simulation_registry,
        }
    }
}

impl ControlBackend for AttachedBody {
    fn feedback_validity(&self, ids: &[u8]) -> Vec<bool> {
        let physical = self.physical.feedback_validity(ids);
        let simulated = self.simulation.feedback_validity(&self.simulation_ids);
        self.active_ids
            .iter()
            .enumerate()
            .map(|(index, id)| {
                if id.is_some() {
                    physical.get(index).copied().unwrap_or(false)
                } else {
                    simulated.get(index).copied().unwrap_or(false)
                }
            })
            .collect()
    }
    fn read_positions(&mut self, ids: &[u8]) -> Result<BackendJointSample, ServoError> {
        let simulated = self.simulation.read_positions(&self.simulation_ids)?;
        let mut physical = self.physical.read_positions(ids)?;
        if simulated.positions.len() != self.active_ids.len()
            || simulated.velocities.len() != self.active_ids.len()
            || physical.positions.len() != self.active_ids.len()
            || physical.velocities.len() != self.active_ids.len()
        {
            return Err(ServoError::Bus(
                "attached body feedback length mismatch".into(),
            ));
        }
        for (index, joint) in self.calibration.joints.iter().enumerate() {
            if self.active_ids[index].is_none() {
                physical.positions[index] =
                    joint.zero_offset_rad + f64::from(joint.direction) * simulated.positions[index];
                physical.velocities[index] =
                    f64::from(joint.direction) * simulated.velocities[index];
            }
        }
        Ok(physical)
    }

    fn read_imu(&mut self) -> Result<Option<loop_body::BackendImuSample>, ServoError> {
        self.physical.read_imu()
    }

    fn write_positions(
        &mut self,
        ids: &[u8],
        targets: &BackendPositionTargets,
    ) -> Result<(), ServoError> {
        if targets.positions.len() != self.calibration.joints.len() {
            return Err(ServoError::Bus(
                "attached body target length mismatch".into(),
            ));
        }
        let positions = targets
            .positions
            .iter()
            .zip(&self.calibration.joints)
            .map(|(position, joint)| {
                (position - joint.zero_offset_rad) * f64::from(joint.direction)
            })
            .collect();
        self.simulation
            .write_positions(&self.simulation_ids, &BackendPositionTargets { positions })?;
        self.physical.write_positions(ids, targets)
    }

    fn set_torque(&mut self, enabled: bool) -> Result<(), ServoError> {
        if enabled {
            self.simulation.set_torque(true)?;
            self.physical.set_torque(true)
        } else {
            let physical = self.physical.set_torque(false);
            let simulation = self.simulation.set_torque(false);
            physical.and(simulation)
        }
    }
    fn gain_capability(&self) -> loop_body::GainCapability {
        self.physical.gain_capability()
    }
    fn set_gain(&mut self, gain: u16) -> Result<(), ServoError> {
        if self.gain_capability() != loop_body::GainCapability::Configurable {
            return Err(ServoError::UnsupportedGain);
        }
        self.physical.set_gain(gain)
    }
    fn reboot(&mut self, id: u8) -> Result<(), ServoError> {
        self.physical.reboot(id)
    }
    fn advance_execution(&mut self, steps: u32, dt: std::time::Duration) -> Result<(), ServoError> {
        // Physical firmware runs on its own clock; MuJoCo always retains its 5ms solver step.
        let duration = dt * steps;
        self.simulation.advance_execution(
            (duration.as_secs_f64() / 0.005).round() as u32,
            std::time::Duration::from_millis(5),
        )
    }
    fn reset_execution(&mut self) -> Result<(), ServoError> {
        self.simulation.reset_execution()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::{Arc, Mutex};

    struct PhysicalProbe {
        written: Arc<Mutex<Vec<f64>>>,
        gains: Arc<Mutex<Vec<u16>>>,
        fail: bool,
    }
    impl ControlBackend for PhysicalProbe {
        fn feedback_validity(&self, _: &[u8]) -> Vec<bool> {
            vec![true, false]
        }
        fn read_positions(&mut self, ids: &[u8]) -> Result<BackendJointSample, ServoError> {
            assert_eq!(ids, [42, 0]);
            if self.fail {
                return Err(ServoError::Bus("servo 42 timeout".into()));
            }
            Ok(BackendJointSample {
                positions: vec![1.25, 0.0],
                velocities: vec![0.5, 0.0],
            })
        }
        fn write_positions(
            &mut self,
            ids: &[u8],
            targets: &BackendPositionTargets,
        ) -> Result<(), ServoError> {
            assert_eq!(ids, [42, 0]);
            *self.written.lock().unwrap() = targets.positions.clone();
            Ok(())
        }
        fn set_torque(&mut self, _: bool) -> Result<(), ServoError> {
            Ok(())
        }
        fn set_gain(&mut self, gain: u16) -> Result<(), ServoError> {
            self.gains.lock().unwrap().push(gain);
            Ok(())
        }
        fn reboot(&mut self, _: u8) -> Result<(), ServoError> {
            Ok(())
        }
    }

    struct Fixture {
        body: AttachedBody,
        written: Arc<Mutex<Vec<f64>>>,
        gains: Arc<Mutex<Vec<u16>>>,
    }

    fn body(fail: bool) -> Fixture {
        let mut registry = loop_body::BackendRegistry::default();
        crate::test_support::register_simulation(
            &mut registry,
            "simulation",
            crate::test_support::simulation_selection(
                microduck_body_runtime::ModelKind::Hd1910Compatible,
            ),
            &crate::platform::helpers::default_simulation(crate::types::ServoModelKey::Hd1910),
            &crate::runtime::adapters::body_assets::test_template(),
        )
        .unwrap();
        registry.select("simulation").unwrap();
        let simulation = registry
            .open("simulation", loop_body::BackendAccess::Control)
            .unwrap();
        let written = Arc::new(Mutex::new(Vec::new()));
        let gains = Arc::new(Mutex::new(Vec::new()));
        let calibration = Calibration {
            body_id: "test".into(),
            body_version: 1,
            joints: ["left_hip_yaw", "left_hip_roll"]
                .iter()
                .enumerate()
                .map(|(index, slot)| loop_body::JointCalibration {
                    slot_id: (*slot).into(),
                    physical_id: if index == 0 { 42 } else { 0 },
                    direction: -1,
                    zero_offset_rad: 2.0,
                    min_rad: None,
                    max_rad: None,
                })
                .collect(),
        };
        Fixture {
            body: AttachedBody::new(
                Box::new(simulation),
                Box::new(PhysicalProbe {
                    written: written.clone(),
                    gains: gains.clone(),
                    fail,
                }),
                vec![Some(42), None],
                calibration,
                registry,
            ),
            written,
            gains,
        }
    }

    #[test]
    fn unmatched_limb_keeps_mujoco_feedback_and_targets_in_canonical_coordinates() {
        let Fixture {
            mut body, written, ..
        } = body(false);
        let native = body.simulation.read_positions(&[1, 2]).unwrap();
        let sample = body.read_positions(&[42, 0]).unwrap();
        assert_eq!(sample.positions[0], 1.25);
        assert_eq!(sample.velocities[0], 0.5);
        assert!((sample.positions[1] - (2.0 - native.positions[1])).abs() < 1e-12);
        assert_eq!(sample.velocities[1], -native.velocities[1]);
        assert_eq!(body.feedback_validity(&[42, 0]), [true, true]);
        body.set_torque(true).unwrap();
        body.write_positions(
            &[42, 0],
            &BackendPositionTargets {
                positions: vec![1.8, 1.5],
            },
        )
        .unwrap();
        assert_eq!(*written.lock().unwrap(), [1.8, 1.5]);
        body.advance_execution(4, std::time::Duration::from_millis(5))
            .unwrap();
        let after = body.read_positions(&[42, 0]).unwrap();
        assert!(after.positions.iter().all(|position| position.is_finite()));
        body.set_torque(false).unwrap();
    }

    #[test]
    fn matched_servo_failure_is_never_replaced_by_simulated_feedback() {
        let Fixture { mut body, .. } = body(true);
        assert!(
            body.read_positions(&[42, 0])
                .unwrap_err()
                .to_string()
                .contains("servo 42 timeout")
        );
    }

    #[test]
    fn hd1910_gain_is_sent_only_to_physical_actuators() {
        let Fixture {
            mut body, gains, ..
        } = body(false);
        assert_eq!(
            body.simulation.gain_capability(),
            loop_body::GainCapability::FixedModel
        );
        body.set_gain(32).unwrap();
        assert_eq!(*gains.lock().unwrap(), [32]);
        body.advance_execution(4, std::time::Duration::from_millis(5))
            .unwrap();
    }
}
