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
    pub min_rad: Option<f64>,
    pub max_rad: Option<f64>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct Calibration {
    pub joints: Vec<JointCalibration>,
}

impl Calibration {
    /// The driver order is always the body actuator order.  This is the boundary
    /// between canonical simulation/ONNX slots and machine-specific servo IDs.
    pub fn physical_ids(&self) -> Vec<u8> {
        self.joints.iter().map(|joint| joint.physical_id).collect()
    }

    pub fn to_servo_positions(&self, positions: &[f64]) -> Vec<f64> {
        positions
            .iter()
            .zip(&self.joints)
            .map(|(position, calibration)| {
                let bounded = match (calibration.min_rad, calibration.max_rad) {
                    (Some(min), Some(max)) => position.clamp(min, max),
                    (Some(min), None) => position.max(min),
                    (None, Some(max)) => position.min(max),
                    (None, None) => *position,
                };
                (bounded + calibration.zero_offset_rad) * f64::from(calibration.direction)
            })
            .collect()
    }

    pub fn to_body_positions(&self, positions: &[f64]) -> Vec<f64> {
        positions
            .iter()
            .zip(&self.joints)
            .map(|(position, calibration)| {
                *position * f64::from(calibration.direction) - calibration.zero_offset_rad
            })
            .collect()
    }

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
            if joint.min_rad.is_some_and(|value| !value.is_finite())
                || joint.max_rad.is_some_and(|value| !value.is_finite())
                || matches!((joint.min_rad, joint.max_rad), (Some(min), Some(max)) if min > max)
            {
                return Err(LoopError::NonFiniteOffset {
                    slot: joint.slot_id.clone(),
                });
            }
        }
        Ok(())
    }
}

#[cfg(test)]
mod calibration_tests {
    use super::{Calibration, JointCalibration};

    #[test]
    fn canonical_slots_translate_to_bound_machine_ids() {
        let calibration = Calibration {
            joints: vec![
                JointCalibration {
                    slot_id: "left_hip_yaw".into(),
                    physical_id: 42,
                    direction: -1,
                    zero_offset_rad: 0.2,
                    min_rad: None,
                    max_rad: None,
                },
                JointCalibration {
                    slot_id: "left_hip_roll".into(),
                    physical_id: 43,
                    direction: 1,
                    zero_offset_rad: -0.1,
                    min_rad: None,
                    max_rad: None,
                },
            ],
        };
        assert_eq!(calibration.physical_ids(), vec![42, 43]);
        let servo = calibration.to_servo_positions(&[0.5, 0.25]);
        assert!((servo[0] + 0.7).abs() < 1e-9);
        assert!((servo[1] - 0.15).abs() < 1e-9);
        let body = calibration.to_body_positions(&servo);
        assert!((body[0] - 0.5).abs() < 1e-9);
        assert!((body[1] - 0.25).abs() < 1e-9);
    }

    #[test]
    fn installation_range_clamps_servo_targets() {
        let calibration = Calibration {
            joints: vec![JointCalibration {
                slot_id: "left_hip_yaw".into(),
                physical_id: 42,
                direction: 1,
                zero_offset_rad: 0.1,
                min_rad: Some(-0.5),
                max_rad: Some(0.5),
            }],
        };
        assert_eq!(calibration.to_servo_positions(&[1.0]), vec![0.6]);
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
