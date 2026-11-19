//! Shared body control-loop boundary.
//!
//! The crate keeps the control lifecycle and backend boundary product-agnostic. Physical and
//! virtual callers provide a `ServoLoopDriver`; controller files and application state stay outside.

mod adapter;
mod body;
mod lifecycle;
mod registry;
mod runtime;
mod safety;
mod types;

pub use adapter::{ReadOnlySource, SourceHealth};
pub use body::BodyLoop;
pub use registry::{
    BackendAccess, BackendDescriptor, BackendKind, BackendRegistry, BackendSession, ExecutionGate,
    PhysicalWritePermit,
};
pub use runtime::{
    ControlSource, ExecutionGeneration, LoopRuntimeConfig, PhysicalAccess, ReadOnlySourceConfig,
    RuntimeConfigError, RuntimeExecutor, RuntimeGeneration, RuntimeLoop, RuntimeSource,
    RuntimeTick, SourceGeneration,
};
pub use types::{
    ActuatorDelaySpec, BackendLifecycleAction, BackendState, BodyCommand, BodyRuntimeView,
    Calibration, ControlBackend, ControlTick, ControllerInput, DelayTimebase, ExecutionInterval,
    JointCalibration, JointRangeState, LoopAction, LoopBinding, LoopConfig, LoopController,
    LoopError, LoopPhase, LoopStatus, SampleTime, SensorFrame, ServoBinding,
};
