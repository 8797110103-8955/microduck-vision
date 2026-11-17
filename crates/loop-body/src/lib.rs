//! Shared body control-loop boundary.
//!
//! The crate keeps the control lifecycle and policy execution product-agnostic. Physical and
//! virtual callers provide a `ServoLoopDriver`; policy files and application state stay outside.

mod adapter;
mod body;
mod lifecycle;
mod policy;
mod policy_input;
mod policy_output;
mod safety;
mod types;

pub use body::BodyLoop;
pub use policy::{PolicyConfig, PolicyDescriptor, PolicyLoop, PolicyTrace};
pub use policy_input::PolicyInputEncoder;
pub use policy_output::RawPolicyOutputDecoder;
pub use types::{
    BodyCommand, BodyRuntimeView, Calibration, JointCalibration, JointRangeState, LoopAction,
    LoopBinding, LoopConfig, LoopError, LoopPhase, LoopStatus, SensorFrame, ServoBinding,
};
