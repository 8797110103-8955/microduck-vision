//! Sampling and execution boundary implemented by physical and virtual bodies.

use std::sync::{Arc, atomic::AtomicBool};
use std::time::Duration;

use servo::ServoError;

#[derive(Debug, Clone, PartialEq)]
pub struct BackendJointSample {
    pub positions: Vec<f64>,
    pub velocities: Vec<f64>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct BackendPositionTargets {
    pub positions: Vec<f64>,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BackendImuSample {
    pub gyro: [f64; 3],
    pub gravity: [f64; 3],
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GainCapability {
    Configurable,
    FixedModel,
    Unsupported,
}

pub trait ControlBackend: Send {
    /// Sensor workers install this before I/O. Check it between bounded transactions.
    fn set_read_cancel(&mut self, _cancel: Arc<AtomicBool>) {}

    /// A placeholder can have a display value without providing feedback.
    fn feedback_validity(&self, ids: &[u8]) -> Vec<bool> {
        vec![true; ids.len()]
    }

    fn read_positions(&mut self, ids: &[u8]) -> Result<BackendJointSample, ServoError>;

    fn read_imu(&mut self) -> Result<Option<BackendImuSample>, ServoError> {
        Ok(None)
    }

    fn write_positions(
        &mut self,
        ids: &[u8],
        targets: &BackendPositionTargets,
    ) -> Result<(), ServoError>;

    fn set_torque(&mut self, enabled: bool) -> Result<(), ServoError>;

    fn gain_capability(&self) -> GainCapability {
        GainCapability::Configurable
    }

    fn set_gain(&mut self, gain: u16) -> Result<(), ServoError>;

    fn reboot(&mut self, id: u8) -> Result<(), ServoError>;

    /// Advance the backend clock without implicitly submitting another target.
    fn advance_execution(
        &mut self,
        _physics_steps: u32,
        _physics_dt: Duration,
    ) -> Result<(), ServoError> {
        Ok(())
    }

    /// Reset the backend clock and pending actuator state before the next sample.
    fn reset_execution(&mut self) -> Result<(), ServoError> {
        Ok(())
    }
}
