//! Shared control-loop types and validated body/servo bindings.

use std::time::Duration;

use duck_body::{BodyError, BodyTemplate, ServoParameterSet};
use serde::{Deserialize, Serialize};
use servo::ServoModel;
use thiserror::Error;

#[derive(Debug, Error, PartialEq)]
pub enum LoopError {
    #[error("body error: {0}")]
    Body(#[from] BodyError),
    #[error("servo driver error: {0}")]
    Servo(#[from] servo::ServoError),
    #[error("body has no actuator slots")]
    NoActuators,
    #[error("servo binding slots do not match body actuator slots")]
    BindingSlotsMismatch,
    #[error("calibration entries do not match body actuator slots")]
    CalibrationSlotsMismatch,
    #[error("calibration for slot {slot:?} has invalid direction {direction}")]
    InvalidDirection { slot: String, direction: i8 },
    #[error("calibration for slot {slot:?} has a non-finite zero offset")]
    NonFiniteOffset { slot: String },
    #[error("command belongs to session {command}, current session is {current}")]
    StaleSession { command: u64, current: u64 },
    #[error("target vector has length {actual}, expected {expected}")]
    TargetLength { actual: usize, expected: usize },
    #[error("target for joint {joint:?} is not finite")]
    NonFiniteTarget { joint: String },
    #[error("loop action is invalid in phase {0:?}")]
    InvalidAction(LoopPhase),
    #[error("tick duration must equal the configured period")]
    InvalidTick,
    #[error("loop gain must be greater than zero")]
    ZeroGain,
    #[error("sensor frame has length {actual}, expected {expected}")]
    SensorLength { actual: usize, expected: usize },
    #[error("policy error: {0}")]
    Policy(String),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum LoopPhase {
    Limp,
    Homing,
    Ready,
    Driving,
    Holding,
    Stopped,
    Reconnecting,
    Estop,
}

#[derive(Debug, Clone, PartialEq)]
pub struct LoopConfig {
    pub period: Duration,
    pub homing_ticks: u32,
    pub sensor_failure_limit: u32,
    pub gain: u16,
}

impl Default for LoopConfig {
    fn default() -> Self {
        Self {
            period: Duration::from_millis(20),
            homing_ticks: 5,
            sensor_failure_limit: 3,
            gain: 200,
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct BodyRuntimeView {
    template: BodyTemplate,
}

impl BodyRuntimeView {
    pub fn new(template: BodyTemplate) -> Result<Self, LoopError> {
        template.validate()?;
        if template.actuators.is_empty() {
            return Err(LoopError::NoActuators);
        }
        Ok(Self { template })
    }

    pub fn template(&self) -> &BodyTemplate {
        &self.template
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct ServoBinding {
    pub model: ServoModel,
    pub slot_ids: Vec<String>,
}

impl ServoBinding {
    pub fn all_slots(template: &BodyTemplate, model: ServoModel) -> Self {
        Self {
            model,
            slot_ids: template
                .actuators
                .iter()
                .map(|slot| slot.id.clone())
                .collect(),
        }
    }

    pub(crate) fn validate(&self, template: &BodyTemplate) -> Result<(), LoopError> {
        let expected: Vec<_> = template
            .actuators
            .iter()
            .map(|slot| slot.id.clone())
            .collect();
        if self.slot_ids != expected {
            return Err(LoopError::BindingSlotsMismatch);
        }
        let params = ServoParameterSet::bind_all(template, self.model);
        params.validate_for(template)?;
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct JointCalibration {
    pub slot_id: String,
    pub physical_id: u8,
    pub direction: i8,
    pub zero_offset_rad: f64,
}

#[derive(Debug, Clone, PartialEq)]
pub struct Calibration {
    pub joints: Vec<JointCalibration>,
}

impl Calibration {
    pub(crate) fn validate(&self, template: &BodyTemplate) -> Result<(), LoopError> {
        let expected: Vec<_> = template
            .actuators
            .iter()
            .map(|slot| slot.id.clone())
            .collect();
        let actual: Vec<_> = self
            .joints
            .iter()
            .map(|joint| joint.slot_id.clone())
            .collect();
        if actual != expected {
            return Err(LoopError::CalibrationSlotsMismatch);
        }
        for joint in &self.joints {
            if !matches!(joint.direction, -1 | 1) {
                return Err(LoopError::InvalidDirection {
                    slot: joint.slot_id.clone(),
                    direction: joint.direction,
                });
            }
            if !joint.zero_offset_rad.is_finite() {
                return Err(LoopError::NonFiniteOffset {
                    slot: joint.slot_id.clone(),
                });
            }
        }
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct LoopBinding {
    pub body: BodyRuntimeView,
    pub servos: ServoBinding,
    pub calibration: Calibration,
}

#[derive(Debug, Clone, PartialEq)]
pub struct BodyCommand {
    pub session: u64,
    pub targets: Vec<f64>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LoopAction {
    Power(bool),
    Drive(bool),
    Stop,
    Estop,
    Reconnect { session: u64 },
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct LoopStatus {
    pub phase: LoopPhase,
    pub session: u64,
    pub tick: u64,
    pub torque: bool,
    pub positions: Vec<f64>,
    pub targets: Vec<f64>,
    pub sensor_failures: u32,
    pub last_error: Option<String>,
}
