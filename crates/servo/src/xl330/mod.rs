//! XL330-specific protocol and driver adapters.

mod driver;
mod protocol;
mod testing;

pub use driver::{
    RawBus, RustypotXl330Bus, ServoDriver, Xl330Driver, read_position, read_position_sample,
    set_position_gain, set_position_pid, set_torque, write_position_target,
};
pub use protocol::{
    XL330, XL330_BAUD_RATES, XL330_CURRENT_MA_PER_COUNT, XL330_DEFAULT_BAUD, XL330_KP_DIVISOR,
    XL330_MODEL_NAME, XL330_MODEL_NUMBER, XL330_PWM_LIMIT, XL330_RESOLUTION,
    XL330_VELOCITY_RAD_PER_SEC_PER_COUNT, XL330_VOLTAGE_PER_COUNT, Xl330Protocol, decode_register,
    encode_register, position_rad_from_raw, position_raw_from_rad, register, registers,
    velocity_rad_s_from_raw,
};
pub use testing::MemoryXl330Bus;

pub use crate::model::XL330_CAPABILITIES;
