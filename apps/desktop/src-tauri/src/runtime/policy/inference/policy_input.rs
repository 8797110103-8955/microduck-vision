//! The single deployed 61-dimensional observation layout and command encoder.

use duck_control::imu::ImuData;
use duck_control::model::NUM_JOINTS;
use duck_control::obs::{ACTION_LEN, Command, Observation};

#[derive(Debug, Clone, Copy, Default)]
pub struct PolicyInputEncoder;

impl PolicyInputEncoder {
    pub fn encode(
        &self,
        imu: &ImuData,
        positions: &[f64; NUM_JOINTS],
        velocities: &[f64; NUM_JOINTS],
        home: &[f64; NUM_JOINTS],
        last_action: &[f32; ACTION_LEN],
        command: &Command,
    ) -> Observation {
        Observation::build(imu, positions, velocities, home, last_action, command)
    }
}
