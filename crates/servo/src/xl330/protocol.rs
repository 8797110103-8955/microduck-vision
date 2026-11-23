use std::f64::consts::PI;

use rustypot::servo::RegisterInfo;
use rustypot::servo::conversion::Conversion;
use rustypot::servo::dynamixel::xl330::{DEFINITION, INFO, REGISTERS};

use crate::ServoError;

pub const XL330_MODEL_NAME: &str = "XL330M288";
pub const XL330_MODEL_NUMBER: u16 = 1200;
pub const XL330_DEFAULT_BAUD: u32 = 1_000_000;
pub const XL330_RESOLUTION: i32 = 4096;
pub const XL330_KP_DIVISOR: u32 = 256;
pub const XL330_PWM_LIMIT: u32 = 885;
pub const XL330_BAUD_RATES: [u32; 7] = [
    9_600, 57_600, 115_200, 1_000_000, 2_000_000, 3_000_000, 4_000_000,
];
pub const XL330_VOLTAGE_PER_COUNT: f64 = 0.1;
pub const XL330_CURRENT_MA_PER_COUNT: f64 = 1.0;
pub const XL330_VELOCITY_RPM_PER_COUNT: f64 = 0.229;
pub const XL330_VELOCITY_RAD_PER_SEC_PER_COUNT: f64 =
    XL330_VELOCITY_RPM_PER_COUNT * 2.0 * PI / 60.0;

pub struct Xl330Protocol;

pub const XL330: Xl330Protocol = Xl330Protocol;

pub fn registers() -> &'static [RegisterInfo] {
    REGISTERS
}

pub fn register(name: &str) -> Option<RegisterInfo> {
    DEFINITION.register(name)
}

pub fn encode_register(name: &str, value: i64) -> Result<Vec<u8>, ServoError> {
    let register = register(name).ok_or_else(|| ServoError::UnknownRegister(name.into()))?;
    register
        .encode(INFO.word_order, value)
        .map_err(|error| ServoError::RegisterOutOfRange {
            name: name.into(),
            value: match error {
                rustypot::servo::RegisterError::OutOfRange { value, .. } => value,
                _ => value,
            },
        })
}

pub fn decode_register(name: &str, bytes: &[u8]) -> Result<i64, ServoError> {
    let register = register(name).ok_or_else(|| ServoError::UnknownRegister(name.into()))?;
    if bytes.len() != register.size as usize {
        return Err(ServoError::ShortRead {
            what: "XL330 register",
            expected: register.size as usize,
            got: bytes.len(),
        });
    }
    register
        .decode(INFO.word_order, bytes)
        .map_err(|_| ServoError::InvalidRegister { name: name.into() })
}

pub fn position_rad_from_raw(raw: i32) -> f64 {
    rustypot::servo::dynamixel::xl330::AnglePosition::from_raw(raw)
}

pub fn position_raw_from_rad(rad: f64) -> Result<i32, ServoError> {
    if !rad.is_finite() {
        return Err(ServoError::NonFinitePosition(rad));
    }
    if !(-PI..=PI).contains(&rad) {
        return Err(ServoError::RegisterOutOfRange {
            name: "goal_position".into(),
            value: (rad * 1_000_000.0) as i64,
        });
    }
    Ok(rustypot::servo::dynamixel::xl330::AnglePosition::to_raw(
        rad,
    ))
}

pub fn velocity_rad_s_from_raw(raw: i32) -> f64 {
    rustypot::servo::dynamixel::xl330::conv::dxl_vel_to_rpm(raw) as f64 * 2.0 * PI / 60.0
}
