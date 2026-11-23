//! Feetech Protocol 1 adapters. HD1910 is the product default; STS and SCS remain explicit
//! protocol families so their table differences cannot be hidden behind one fake definition.

mod driver;
mod protocol;

pub use driver::{
    FeetechDriver, FeetechRawBus, Hd1910Driver, RustypotFeetechBus, position_target_block,
    read_configuration, read_position, read_position_sample, send_center_calibration,
    set_position_gain, set_torque, write_position_target,
};
pub use protocol::{
    FeetechProtocol, FeetechRegister, FeetechRegisterArea, HD1910, HD1910_BAUD_RATES,
    HD1910_CURRENT_MA_PER_COUNT, HD1910_MODEL_NUMBER, HD1910_MOVE_BLOCK, HD1910_RESOLUTION,
    HD1910_VELOCITY_RAD_PER_SEC_PER_COUNT, HD1910_VELOCITY_RPM_PER_COUNT, HD1910_VOLTAGE_PER_COUNT,
    Hd1910Configuration, PositionDomain, STS3215, XdatImage, baud_code, current_ma_from_raw,
    decode_frame, decode_motion_block, decode_register, decode_xdat, encode_frame, encode_register,
    encode_xdat, model_from_identity, motion_registers, position_rad_from_steps,
    position_steps_from_rad, protocol_for_model, register, registers, reset_instruction,
    temperature_c_from_raw, velocity_rad_s_from_counts, voltage_v_from_raw,
};
