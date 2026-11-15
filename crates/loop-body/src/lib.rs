//! Shared body control-loop boundary.
//!
//! The crate keeps the control lifecycle and policy execution product-agnostic. Physical and
//! virtual callers provide a `ServoLoopDriver`; policy files and application state stay outside.

mod body;
mod policy;
mod types;

pub use body::BodyLoop;
pub use policy::{PolicyLoop, PolicyTrace};
pub use types::{
    BodyCommand, BodyRuntimeView, Calibration, JointCalibration, LoopAction, LoopBinding,
    LoopConfig, LoopError, LoopPhase, LoopStatus, ServoBinding,
};
