//! XL330-specific protocol and driver adapters.

mod driver;
mod protocol;
mod testing;

pub use driver::{RawBus, RustypotXl330Bus, ServoDriver, Xl330Driver, write_position_target};
pub use protocol::{
    XL330, XL330_DEFAULT_BAUD, XL330_MODEL_NAME, XL330_MODEL_NUMBER, XL330_RESOLUTION,
    XL330_VELOCITY_RAD_PER_SEC_PER_COUNT, Xl330Protocol, decode_register, encode_register,
    position_rad_from_raw, position_raw_from_rad, register, registers, velocity_rad_s_from_raw,
};
pub use testing::MemoryXl330Bus;

use crate::ServoCapabilities;

pub const XL330_CAPABILITIES: ServoCapabilities = ServoCapabilities;
