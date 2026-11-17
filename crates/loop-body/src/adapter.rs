//! Canonical sensor/target adapter for physical and virtual servo drivers.

use duck_body::BodyBinding;
use servo::{ServoLoopDriver, ServoPositionTargets};

use crate::{LoopError, SensorFrame};

pub(crate) struct ServoAdapter {
    driver: Box<dyn ServoLoopDriver>,
    binding: BodyBinding,
}

impl ServoAdapter {
    pub(crate) fn new(driver: Box<dyn ServoLoopDriver>, binding: BodyBinding) -> Self {
        Self { driver, binding }
    }

    pub(crate) fn read(&mut self) -> Result<SensorFrame, LoopError> {
        let ids = self.binding.physical_ids();
        let sample = self.driver.read_positions(&ids)?;
        if sample.positions.len() != ids.len() || sample.velocities.len() != ids.len() {
            return Err(LoopError::SensorLength {
                actual: sample.positions.len(),
                expected: ids.len(),
            });
        }
        Ok(SensorFrame {
            positions: self.binding.to_body_positions(&sample.positions),
            velocities: self.binding.to_body_velocities(&sample.velocities),
        })
    }

    pub(crate) fn write(&mut self, targets: &[f64]) -> Result<(), LoopError> {
        let ids = self.binding.physical_ids();
        let positions = self.binding.to_servo_positions(targets);
        self.driver
            .write_positions(&ids, &ServoPositionTargets { positions })?;
        Ok(())
    }

    pub(crate) fn set_torque(&mut self, enabled: bool) -> Result<(), LoopError> {
        self.driver.set_torque(enabled)?;
        Ok(())
    }

    pub(crate) fn set_gain(&mut self, gain: u16) -> Result<(), LoopError> {
        self.driver.set_gain(gain)?;
        Ok(())
    }
}
