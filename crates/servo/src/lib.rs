//! Product-agnostic servo API boundary.
//!
//! Model-specific protocol facts come from the pinned vendor implementation through the XL330
//! and Feetech adapters. Body templates and loop policy stay above this crate.

pub mod error;
pub mod hd1910;
pub mod model;
pub mod xl330;

pub use rustypot::servo::{RegisterAccess, RegisterInfo};

pub use error::ServoError;
pub use hd1910::{FeetechProtocol, HD1910, STS3215};
pub use model::{
    ServoCapabilities, ServoCapability, ServoHardwareSpecs, ServoModel, ServoModelFacts,
};
pub use xl330::{
    MemoryXl330Bus, RawBus, RustypotXl330Bus, ServoDriver, XL330, XL330_CAPABILITIES,
    XL330_DEFAULT_BAUD, XL330_MODEL_NAME, XL330_MODEL_NUMBER, XL330_RESOLUTION,
    XL330_VELOCITY_RAD_PER_SEC_PER_COUNT, Xl330Driver, Xl330Protocol, decode_register,
    encode_register, position_rad_from_raw, position_raw_from_rad, register, registers,
    velocity_rad_s_from_raw,
};

/// Telemetry type key. Fields are intentionally deferred until the implementation contract is
/// finalized.
pub struct ServoTelemetry;

/// Command type key. Fields are intentionally deferred until the implementation contract is
/// finalized.
pub struct ServoCommand;

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

/// Device position I/O for a driver bound to a set of servo IDs.
pub trait ServoDeviceDriver: Send {
    fn read_positions(&mut self, ids: &[u8]) -> Result<ServoPositionSample, ServoError>;
    fn write_positions(
        &mut self,
        ids: &[u8],
        targets: &ServoPositionTargets,
    ) -> Result<(), ServoError>;
    fn set_torque(&mut self, enabled: bool) -> Result<(), ServoError>;
    fn set_gain(&mut self, gain: u16) -> Result<(), ServoError>;
    fn reboot(&mut self, id: u8) -> Result<(), ServoError>;
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
