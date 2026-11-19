//! Product-agnostic servo API boundary.
//!
//! Model-specific protocol facts come from the pinned vendor implementation through the XL330
//! and Feetech adapters. Body templates and loop policy stay above this crate.

pub mod error;
pub mod hd1910;
pub mod xl330;

pub use rustypot::servo::{RegisterAccess, RegisterInfo};

pub use error::ServoError;
pub use hd1910::{FeetechProtocol, HD1910, STS3215};
pub use xl330::{
    MemoryXl330Bus, RawBus, RustypotXl330Bus, ServoDriver, XL330, XL330_CAPABILITIES,
    XL330_DEFAULT_BAUD, XL330_MODEL_NAME, XL330_MODEL_NUMBER, XL330_RESOLUTION,
    XL330_VELOCITY_RAD_PER_SEC_PER_COUNT, Xl330Driver, Xl330Protocol, decode_register,
    encode_register, position_rad_from_raw, position_raw_from_rad, register, registers,
    velocity_rad_s_from_raw,
};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum ServoModel {
    Xl330M288,
    Hd1910,
    Sts3215,
    Scs0009,
}

/// Telemetry type key. Fields are intentionally deferred until the implementation contract is
/// finalized.
pub struct ServoTelemetry;

/// Command type key. Fields are intentionally deferred until the implementation contract is
/// finalized.
pub struct ServoCommand;

/// Capability type key. Fields are intentionally deferred until the implementation contract is
/// finalized.
pub struct ServoCapabilities;

pub struct ServoDynamics;
pub struct ProtocolDescription;
pub struct ServoDefinition;
pub struct ServoIdentity;
pub struct Gain;

#[derive(Debug, Clone, PartialEq)]
pub struct ServoPositionSample {
    pub positions: Vec<f64>,
    pub velocities: Vec<f64>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct ServoPositionTargets {
    pub positions: Vec<f64>,
}

/// Optional body-sensor data owned by a servo/backend implementation.
///
/// The servo crate deliberately keeps this independent from the policy crate. A physical bus
/// may return `None` when no IMU is attached, while a simulation backend can derive the same
/// frame from its body state. The policy loop decides how the frame enters its observation.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ServoImuSample {
    pub gyro: [f64; 3],
    pub gravity: [f64; 3],
}

pub trait ServoLoopDriver: Send {
    /// Sensor workers install this before I/O. Check it between bounded transactions.
    fn set_read_cancel(&mut self, _cancel: std::sync::Arc<std::sync::atomic::AtomicBool>) {}
    /// Slots with actual telemetry. A placeholder may have a display value but is not feedback.
    fn feedback_validity(&self, ids: &[u8]) -> Vec<bool> {
        vec![true; ids.len()]
    }
    fn read_positions(&mut self, ids: &[u8]) -> Result<ServoPositionSample, ServoError>;
    fn read_imu(&mut self) -> Result<Option<ServoImuSample>, ServoError> {
        Ok(None)
    }
    fn write_positions(
        &mut self,
        ids: &[u8],
        targets: &ServoPositionTargets,
    ) -> Result<(), ServoError>;
    fn set_torque(&mut self, enabled: bool) -> Result<(), ServoError>;
    fn set_gain(&mut self, gain: u16) -> Result<(), ServoError>;
    fn reboot(&mut self, id: u8) -> Result<(), ServoError>;

    /// Advance backend-owned execution without implicitly submitting another target. Physical
    /// transports normally leave this as a no-op because their firmware owns the clock; simulated
    /// and embedded adapters override it to advance their deterministic physics/device clock.
    fn advance_execution(
        &mut self,
        _physics_steps: u32,
        _physics_dt: std::time::Duration,
    ) -> Result<(), ServoError> {
        Ok(())
    }

    /// Reset backend-owned execution state before the next observation is encoded. Physical
    /// transports may leave this as a no-op; deterministic simulators use it to clear the
    /// solver, delay queue, torque history and published snapshot synchronously.
    fn reset_execution(&mut self) -> Result<(), ServoError> {
        Ok(())
    }
}

pub trait ServoDefinitionProvider {
    fn key(&self) -> &str;
    fn capabilities(&self) -> ServoCapabilities;
    fn dynamics(&self) -> ServoDynamics;
    fn protocol(&self) -> ProtocolDescription;
}

pub trait ServoControl: Send {
    fn read(&mut self) -> Result<ServoTelemetry, ServoError>;
    fn write(&mut self, command: ServoCommand) -> Result<(), ServoError>;
    fn set_torque(&mut self, enabled: bool) -> Result<(), ServoError>;
    fn set_gain(&mut self, gain: Gain) -> Result<(), ServoError>;
    fn scan(&mut self) -> Result<Vec<ServoIdentity>, ServoError>;
}
