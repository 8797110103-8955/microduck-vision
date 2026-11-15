//! Feetech Protocol 1 adapters. HD1910 is the product default; STS and SCS remain explicit
//! protocol families so their table differences cannot be hidden behind one fake definition.

mod driver;
mod protocol;

pub use driver::{
    FeetechDriver, FeetechRawBus, Hd1910Driver, RustypotFeetechBus, write_position_target,
};
pub use protocol::{
    FeetechProtocol, FeetechRegister, FeetechRegisterArea, HD1910, STS3215, XdatImage, baud_code,
    current_ma_from_raw, decode_frame, decode_register, decode_xdat, encode_frame, encode_register,
    encode_xdat, model_from_identity, motion_registers, position_rad_from_steps,
    position_steps_from_rad, protocol_for_model, register, registers, reset_instruction,
    temperature_c_from_raw, velocity_rad_s_from_raw, voltage_v_from_raw,
};
