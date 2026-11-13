use std::path::PathBuf;

use thiserror::Error;

use crate::model::ActuatorCapability;

#[derive(Debug, Error, Clone, PartialEq)]
pub enum BodyError {
    #[error("body template has no {field}")]
    MissingField { field: &'static str },
    #[error("body template contains duplicate {kind} {id:?}")]
    Duplicate { kind: &'static str, id: String },
    #[error(
        "body template has invalid joint order at {joint:?}: expected {expected}, got {actual}"
    )]
    JointOrder {
        joint: String,
        expected: u16,
        actual: u16,
    },
    #[error("body template has invalid range for joint {joint:?}")]
    InvalidRange { joint: String },
    #[error("actuator {actuator:?} references missing joint {joint:?}")]
    MissingJoint { actuator: String, joint: String },
    #[error("joint {joint:?} references missing actuator {actuator:?}")]
    MissingActuator { joint: String, actuator: String },
    #[error("control contract joint order does not match template order")]
    ControlOrderMismatch,
    #[error("control contract action size {actual} does not match actuator count {expected}")]
    ActionSizeMismatch { actual: usize, expected: usize },
    #[error("body asset does not exist: {0}")]
    MissingAsset(PathBuf),
    #[error("MJCF parse failed: {0}")]
    Mjcf(String),
    #[error("MJCF joint {0:?} is not present in the kinematics model")]
    MissingSceneJoint(String),
    #[error("servo slot {slot:?} has no parameter binding")]
    MissingServoBinding { slot: String },
    #[error("servo type {servo_type:?} is not allowed for slot {slot:?}")]
    ServoTypeNotAllowed { slot: String, servo_type: String },
    #[error("servo type {servo_type:?} lacks capability {capability:?} for slot {slot:?}")]
    MissingCapability {
        slot: String,
        servo_type: String,
        capability: ActuatorCapability,
    },
    #[error("body vector length is {actual}, expected {expected}")]
    VectorLength { actual: usize, expected: usize },
    #[error("target for joint {joint:?} is outside its body range")]
    TargetOutOfRange { joint: String },
    #[error("time step must be finite and greater than zero")]
    InvalidTimeStep,
    #[error("dynamics backend failed: {0}")]
    Dynamics(String),
    #[error("body template TOML parse failed: {0}")]
    Toml(String),
}
