//! Project raw physical telemetry into the current body mapping without replacing its source.

use loop_body::{ControlBackend, JointCalibration};
use servo::{ServoError, ServoLoopDriver, ServoPositionSample, ServoPositionTargets};

pub(crate) struct ReadOnlyBody {
    driver: Box<dyn ControlBackend>,
    raw_ids: Vec<u8>,
    mapping: Box<dyn Fn() -> Result<Vec<Option<JointCalibration>>, String> + Send>,
    valid: Vec<bool>,
    periodic_ranges: Option<Vec<[f64; 2]>>,
    mapping_revision: Option<Box<dyn Fn() -> u64 + Send>>,
}

impl ReadOnlyBody {
    pub(crate) fn new(
        driver: Box<dyn ControlBackend>,
        raw_ids: Vec<u8>,
        mapping: impl Fn() -> Result<Vec<Option<JointCalibration>>, String> + Send + 'static,
    ) -> Self {
        Self {
            driver,
            raw_ids,
            mapping: Box::new(mapping),
            valid: Vec::new(),
            periodic_ranges: None,
            mapping_revision: None,
        }
    }

    pub(crate) fn with_periodic_ranges(mut self, ranges: Vec<[f64; 2]>) -> Self {
        self.periodic_ranges = Some(ranges);
        self
    }

    pub(crate) fn with_mapping_revision(
        mut self,
        revision: impl Fn() -> u64 + Send + 'static,
    ) -> Self {
        self.mapping_revision = Some(Box::new(revision));
        self
    }
}

impl ServoLoopDriver for ReadOnlyBody {
    fn set_read_cancel(&mut self, cancel: std::sync::Arc<std::sync::atomic::AtomicBool>) {
        self.driver.set_read_cancel(cancel);
    }
    fn feedback_validity(&self, ids: &[u8]) -> Vec<bool> {
        if self.valid.len() == ids.len() {
            self.valid.clone()
        } else {
            vec![false; ids.len()]
        }
    }
    fn read_positions(&mut self, ids: &[u8]) -> Result<ServoPositionSample, ServoError> {
        let revision = self.mapping_revision.as_ref().map(|revision| revision());
        let raw = self.driver.read_positions(&self.raw_ids)?;
        let raw_valid = self.driver.feedback_validity(&self.raw_ids);
        if raw.positions.len() != self.raw_ids.len() || raw.velocities.len() != self.raw_ids.len() {
            return Err(ServoError::Bus("raw telemetry length mismatch".into()));
        }
        // Resolve after I/O. A calibration revision also invalidates the old raw coordinate
        // frame, so that sample is discarded without replacing the running source.
        let mapping = (self.mapping)().map_err(ServoError::Bus)?;
        if revision != self.mapping_revision.as_ref().map(|revision| revision()) {
            return Err(ServoError::Bus(
                "body mapping changed during telemetry read; sample discarded".into(),
            ));
        }
        if mapping.len() != ids.len() {
            return Err(ServoError::Bus("body mapping length mismatch".into()));
        }
        let mut sample = ServoPositionSample {
            positions: vec![0.0; ids.len()],
            velocities: vec![0.0; ids.len()],
        };
        self.valid = vec![false; ids.len()];
        for (slot, joint) in mapping.iter().enumerate() {
            if let Some(joint) = joint
                && let Some(index) = self.raw_ids.iter().position(|id| *id == joint.physical_id)
            {
                sample.positions[slot] = if let Some(range) = self
                    .periodic_ranges
                    .as_ref()
                    .and_then(|ranges| ranges.get(slot))
                {
                    super::hd1910_body::display_position(raw.positions[index], joint, *range)
                } else {
                    (raw.positions[index] - joint.zero_offset_rad) * f64::from(joint.direction)
                };
                sample.velocities[slot] = raw.velocities[index] * f64::from(joint.direction);
                self.valid[slot] = raw_valid.get(index).copied().unwrap_or(false);
            }
        }
        Ok(sample)
    }
    fn read_imu(&mut self) -> Result<Option<servo::ServoImuSample>, ServoError> {
        self.driver.read_imu()
    }
    fn write_positions(&mut self, _: &[u8], _: &ServoPositionTargets) -> Result<(), ServoError> {
        Err(ServoError::Bus(
            "read-only body cannot write targets".into(),
        ))
    }
    fn set_torque(&mut self, _: bool) -> Result<(), ServoError> {
        Err(ServoError::Bus("read-only body cannot set torque".into()))
    }
    fn set_gain(&mut self, _: u16) -> Result<(), ServoError> {
        Err(ServoError::Bus("read-only body cannot set gain".into()))
    }
    fn reboot(&mut self, _: u8) -> Result<(), ServoError> {
        Err(ServoError::Bus("read-only body cannot reboot".into()))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::{Arc, Mutex};
    use std::time::{Duration, Instant};

    struct Telemetry;
    impl ServoLoopDriver for Telemetry {
        fn read_positions(&mut self, ids: &[u8]) -> Result<ServoPositionSample, ServoError> {
            assert_eq!(ids, [7, 42]);
            Ok(ServoPositionSample {
                positions: vec![1.25, 2.5],
                velocities: vec![0.4, 0.8],
            })
        }
        fn write_positions(
            &mut self,
            _: &[u8],
            _: &ServoPositionTargets,
        ) -> Result<(), ServoError> {
            panic!("read-only source wrote targets")
        }
        fn set_torque(&mut self, _: bool) -> Result<(), ServoError> {
            panic!("read-only source set torque")
        }
        fn set_gain(&mut self, _: u16) -> Result<(), ServoError> {
            panic!("read-only source set gain")
        }
        fn reboot(&mut self, _: u8) -> Result<(), ServoError> {
            panic!("read-only source rebooted")
        }
    }

    #[test]
    fn an_in_flight_raw_sample_is_discarded_if_calibration_changes_its_coordinate_frame() {
        use std::sync::atomic::{AtomicU64, Ordering};
        let revision = Arc::new(AtomicU64::new(1));
        let during_read = revision.clone();
        let observed = revision.clone();
        let mut body = ReadOnlyBody::new(Box::new(Telemetry), vec![7, 42], move || {
            during_read.store(2, Ordering::Release);
            Ok(vec![Some(JointCalibration {
                slot_id: "left_knee".into(),
                physical_id: 7,
                direction: -1,
                zero_offset_rad: 0.5,
                min_rad: None,
                max_rad: None,
            })])
        })
        .with_mapping_revision(move || observed.load(Ordering::Acquire));
        assert!(
            body.read_positions(&[7])
                .unwrap_err()
                .to_string()
                .contains("sample discarded")
        );
        let fresh = body.read_positions(&[7]).unwrap();
        assert_eq!(fresh.positions, [-0.75]);
        assert_eq!(fresh.velocities, [-0.4]);
    }

    #[test]
    fn running_source_uses_new_id_direction_and_zero_without_reopening() {
        let template = duck_body::BodyTemplate::official().unwrap();
        let joint = JointCalibration {
            slot_id: "left_knee".into(),
            physical_id: 7,
            direction: 1,
            zero_offset_rad: 0.25,
            min_rad: None,
            max_rad: None,
        };
        let mapping = Arc::new(Mutex::new(Some(joint)));
        let cache = mapping.clone();
        let driver = ReadOnlyBody::new(Box::new(Telemetry), vec![7, 42], move || {
            let mut joints = vec![None; 15];
            joints[3] = cache.lock().unwrap().clone();
            Ok(joints)
        });
        let binding = loop_body::LoopBinding {
            body: loop_body::BodyRuntimeView::new(template.clone()).unwrap(),
            servos: loop_body::ServoBinding::all_slots(&template, servo::ServoModel::Hd1910),
            calibration: loop_body::Calibration {
                body_id: template.id,
                body_version: template.version,
                joints: template
                    .actuators
                    .iter()
                    .enumerate()
                    .map(|(index, slot)| JointCalibration {
                        slot_id: slot.id.clone(),
                        physical_id: (index + 1) as u8,
                        direction: 1,
                        zero_offset_rad: 0.0,
                        min_rad: None,
                        max_rad: None,
                    })
                    .collect(),
            },
        };
        let mut source = loop_body::ReadOnlySource::open(binding, Box::new(driver));
        source
            .start(loop_body::ReadOnlySourceConfig {
                id: "physical-test".into(),
                sample_period: Duration::from_millis(2),
                timeout: Duration::from_secs(1),
                max_age: Duration::from_secs(1),
                cache_capacity: 1,
            })
            .unwrap();
        let generation = source.generation();
        let deadline = Instant::now() + Duration::from_secs(1);
        let first = loop {
            if let Ok(sample) = source.read() {
                break sample;
            }
            assert!(Instant::now() < deadline);
            std::thread::sleep(Duration::from_millis(1));
        };
        assert_eq!(first.positions[3], 1.0);
        let mut next = mapping.lock().unwrap();
        let joint = next.as_mut().unwrap();
        joint.physical_id = 42;
        joint.direction = -1;
        joint.zero_offset_rad = 0.5;
        drop(next);
        let deadline = Instant::now() + Duration::from_secs(1);
        loop {
            let sample = source.read().unwrap();
            if sample.positions[3] == -2.0 {
                assert_eq!(sample.velocities[3], -0.8);
                assert!(sample.valid_positions[3]);
                assert!(sample.sequence > first.sequence);
                break;
            }
            assert!(Instant::now() < deadline);
            std::thread::sleep(Duration::from_millis(1));
        }
        *mapping.lock().unwrap() = None;
        let deadline = Instant::now() + Duration::from_secs(1);
        while source.read().unwrap().valid_positions[3] {
            assert!(Instant::now() < deadline);
            std::thread::sleep(Duration::from_millis(1));
        }
        assert_eq!(source.generation(), generation);
        source.stop().unwrap();
    }
}
