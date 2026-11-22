use rustypot::servo::RegisterInfo;
use rustypot::servo::conversion::Conversion;
use rustypot::servo::feetech::{scs0009, sts3215};
use std::f64::consts::PI;

use crate::{ServoError, ServoModel};

pub struct FeetechProtocol;
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct FeetechRegister;
pub struct FeetechRegisterArea;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct XdatImage {
    pub bytes: [u8; 51],
}

pub const HD1910: FeetechProtocol = FeetechProtocol;
pub const STS3215: FeetechProtocol = FeetechProtocol;

/// HLS present velocity (address 58), in RPM per signed count.
/// https://ftservo.github.io/ftservo-wiki/reference/parameter/memory-hls/
pub const HD1910_VELOCITY_RPM_PER_COUNT: f64 = 0.732;
pub const HD1910_VELOCITY_RAD_PER_SEC_PER_COUNT: f64 =
    HD1910_VELOCITY_RPM_PER_COUNT * 2.0 * PI / 60.0;

pub fn protocol_for_model(model: ServoModel) -> Option<FeetechProtocol> {
    matches!(
        model,
        ServoModel::Hd1910 | ServoModel::Sts3215 | ServoModel::Scs0009
    )
    .then_some(FeetechProtocol)
}

pub fn baud_code(_model: ServoModel, baud: u32) -> Option<u8> {
    [
        (1_000_000, 0),
        (500_000, 1),
        (250_000, 2),
        (128_000, 3),
        (115_200, 4),
        (76_800, 5),
        (57_600, 6),
        (38_400, 7),
    ]
    .into_iter()
    .find_map(|(rate, code)| (rate == baud).then_some(code))
}

pub fn reset_instruction(model: ServoModel) -> Option<u8> {
    match model {
        ServoModel::Scs0009 => Some(0x0A),
        ServoModel::Hd1910 | ServoModel::Sts3215 => Some(0x06),
        ServoModel::Xl330M288 => None,
    }
}

pub fn motion_registers(model: ServoModel) -> &'static [&'static str] {
    static STS: &[&str] = &[
        "acceleration",
        "goal_position",
        "pwm_open_speed",
        "goal_speed",
    ];
    static SCS: &[&str] = &["goal_position", "goal_time", "goal_speed"];
    match model {
        ServoModel::Scs0009 => SCS,
        ServoModel::Hd1910 | ServoModel::Sts3215 => STS,
        ServoModel::Xl330M288 => &[],
    }
}

pub fn position_steps_from_rad(model: ServoModel, radians: f64) -> Result<i32, ServoError> {
    let resolution = match model {
        ServoModel::Scs0009 => 1024.0,
        ServoModel::Hd1910 | ServoModel::Sts3215 => 4096.0,
        ServoModel::Xl330M288 => return Err(ServoError::UnknownRegister("model".into())),
    };
    if !radians.is_finite() {
        return Err(ServoError::NonFinitePosition(radians));
    }
    if model == ServoModel::Scs0009 && !(-PI..=PI).contains(&radians) {
        return Err(ServoError::RegisterOutOfRange {
            name: "goal_position".into(),
            value: (radians * 1_000_000.0) as i64,
        });
    }
    // HD/STS feedback is a signed continuous position, including turn count and
    // hardware homing offset. Preserve it on the way back to goal_position.
    // Installation travel limits are separate from this register encoding domain.
    let steps = (resolution * (PI + radians) / (2.0 * PI)).round();
    if !(f64::from(i32::MIN)..=f64::from(i32::MAX)).contains(&steps) {
        return Err(ServoError::RegisterOutOfRange {
            name: "goal_position".into(),
            value: steps as i64,
        });
    }
    let steps = steps as i32;
    if model != ServoModel::Scs0009 {
        encode_register("goal_position", i64::from(steps))?;
    }
    Ok(steps)
}

pub fn position_rad_from_steps(model: ServoModel, steps: i32) -> f64 {
    let resolution = match model {
        ServoModel::Scs0009 => 1024.0,
        ServoModel::Hd1910 | ServoModel::Sts3215 => 4096.0,
        ServoModel::Xl330M288 => 4096.0,
    };
    (f64::from(steps) * (2.0 * PI) / resolution) - PI
}

/// Convert signed counts after the register codec has decoded sign-magnitude once.
pub fn velocity_rad_s_from_counts(model: ServoModel, counts: i16) -> f64 {
    if model == ServoModel::Hd1910 {
        // HD uses HLS units. Reusing the STS SDK's fine scale understates actual speed
        // by ~50x even when the sign has been decoded correctly.
        return f64::from(counts) * HD1910_VELOCITY_RAD_PER_SEC_PER_COUNT;
    }
    // Vendor Velocity::from_raw consumes an unsigned wire word, not a decoded i16.
    // Casting -1 to u16 would decode it again as -32767 and invent ~50 rad/s.
    sts3215::Velocity::from_raw(counts.unsigned_abs()).copysign(f64::from(counts))
}

/// The shared HD1910 live-telemetry decode path, including register sign and units.
pub fn decode_motion_block(bytes: &[u8]) -> Result<(f64, f64), ServoError> {
    if bytes.len() != 4 {
        return Err(ServoError::ShortRead {
            what: "HD1910 motion block",
            expected: 4,
            got: bytes.len(),
        });
    }
    let position = decode_register("present_position", &bytes[..2])? as i32;
    let velocity = decode_register("present_velocity", &bytes[2..])? as i16;
    Ok((
        position_rad_from_steps(ServoModel::Hd1910, position),
        velocity_rad_s_from_counts(ServoModel::Hd1910, velocity),
    ))
}

pub fn current_ma_from_raw(_model: ServoModel, raw: i16) -> f64 {
    f64::from(raw) * 6.5
}

pub fn voltage_v_from_raw(_model: ServoModel, raw: u8) -> f64 {
    f64::from(raw) * 0.1
}

pub fn temperature_c_from_raw(_model: ServoModel, raw: u8) -> f64 {
    f64::from(raw)
}

pub fn model_from_identity(identity: [u8; 2]) -> ServoModel {
    match identity {
        [5, 4] => ServoModel::Scs0009,
        [9, 3] => ServoModel::Sts3215,
        [10, 31] => ServoModel::Hd1910,
        _ => ServoModel::Hd1910,
    }
}

pub fn registers() -> &'static [RegisterInfo] {
    sts3215::REGISTERS
}

pub fn register(name: &str) -> Option<FeetechRegister> {
    let custom = matches!(
        name,
        "pwm_open_speed"
            | "current_bias"
            | "integral_limit"
            | "v_fk"
            | "v_kgi"
            | "p_fk"
            | "e_fk"
            | "vk"
    );
    (custom
        || sts3215::DEFINITION.register(name).is_some()
        || scs0009::DEFINITION.register(name).is_some())
    .then_some(FeetechRegister)
}

pub fn encode_register(name: &str, value: i64) -> Result<Vec<u8>, ServoError> {
    if matches!(name, "model" | "model_number") {
        return u8::try_from(value).map(|value| vec![value]).map_err(|_| {
            ServoError::RegisterOutOfRange {
                name: name.into(),
                value,
            }
        });
    }
    if name == "pwm_open_speed" {
        return encode_register("goal_time", value);
    }
    if name == "present_load" {
        let magnitude = value.unsigned_abs();
        if magnitude > 0x7fff {
            return Err(ServoError::RegisterOutOfRange {
                name: name.into(),
                value,
            });
        }
        let raw = magnitude as u16 | if value < 0 { 0x8000 } else { 0 };
        return Ok(raw.to_le_bytes().to_vec());
    }
    if name == "current_bias" {
        return u16::try_from(value)
            .map(u16::to_le_bytes)
            .map(Vec::from)
            .map_err(|_| ServoError::RegisterOutOfRange {
                name: name.into(),
                value,
            });
    }
    // The FT STS/HD control table uses two independent one-byte parameters at
    // 0x18 and 0x19.  The upstream rustypot STS3215 definition predates this
    // correction and models minimum_startup_force as a u16 at 0x18, so keep
    // the HD wire representation explicit here.
    if matches!(name, "minimum_startup_force" | "integral_limit") {
        return u8::try_from(value).map(|value| vec![value]).map_err(|_| {
            ServoError::RegisterOutOfRange {
                name: name.into(),
                value,
            }
        });
    }
    if matches!(name, "v_fk" | "v_kgi" | "p_fk" | "e_fk" | "vk") {
        return u8::try_from(value).map(|value| vec![value]).map_err(|_| {
            ServoError::RegisterOutOfRange {
                name: name.into(),
                value,
            }
        });
    }
    let register = sts3215::DEFINITION
        .register(name)
        .or_else(|| scs0009::DEFINITION.register(name))
        .ok_or_else(|| ServoError::UnknownRegister(name.into()))?;
    register
        .encode(sts3215::INFO.word_order, value)
        .map_err(|_| ServoError::RegisterOutOfRange {
            name: name.into(),
            value,
        })
}

pub fn decode_register(name: &str, bytes: &[u8]) -> Result<i64, ServoError> {
    if matches!(name, "model" | "model_number") {
        if bytes.len() != 1 {
            return Err(ServoError::ShortRead {
                what: "Feetech register",
                expected: 1,
                got: bytes.len(),
            });
        }
        return bytes
            .first()
            .copied()
            .map(i64::from)
            .ok_or(ServoError::ShortRead {
                what: "Feetech register",
                expected: 1,
                got: bytes.len(),
            });
    }
    if name == "pwm_open_speed" {
        return decode_register("goal_time", bytes);
    }
    if name == "current_bias" {
        if bytes.len() != 2 {
            return Err(ServoError::ShortRead {
                what: "Feetech register",
                expected: 2,
                got: bytes.len(),
            });
        }
        return Ok(i64::from(u16::from_le_bytes([bytes[0], bytes[1]])));
    }
    if matches!(name, "minimum_startup_force" | "integral_limit") {
        if bytes.len() != 1 {
            return Err(ServoError::ShortRead {
                what: "Feetech register",
                expected: 1,
                got: bytes.len(),
            });
        }
        return bytes
            .first()
            .copied()
            .map(i64::from)
            .ok_or(ServoError::ShortRead {
                what: "Feetech register",
                expected: 1,
                got: bytes.len(),
            });
    }
    if matches!(name, "v_fk" | "v_kgi" | "p_fk" | "e_fk" | "vk") {
        if bytes.len() != 1 {
            return Err(ServoError::ShortRead {
                what: "Feetech register",
                expected: 1,
                got: bytes.len(),
            });
        }
        return bytes
            .first()
            .copied()
            .map(i64::from)
            .ok_or(ServoError::ShortRead {
                what: "Feetech register",
                expected: 1,
                got: bytes.len(),
            });
    }
    let register = sts3215::DEFINITION
        .register(name)
        .or_else(|| scs0009::DEFINITION.register(name))
        .ok_or_else(|| ServoError::UnknownRegister(name.into()))?;
    if bytes.len() != register.size as usize {
        return Err(ServoError::ShortRead {
            what: "Feetech register",
            expected: register.size as usize,
            got: bytes.len(),
        });
    }
    if name == "present_load" {
        let raw = u16::from_le_bytes([bytes[0], bytes[1]]);
        let magnitude = raw & 0x7fff;
        return Ok(if raw & 0x8000 != 0 {
            -i64::from(magnitude)
        } else {
            i64::from(magnitude)
        });
    }
    register
        .decode(sts3215::INFO.word_order, bytes)
        .map_err(|_| ServoError::InvalidRegister { name: name.into() })
}

pub fn encode_frame(id: u8, instruction: u8, parameters: &[u8]) -> Result<Vec<u8>, ServoError> {
    let length = parameters
        .len()
        .checked_add(2)
        .ok_or(ServoError::RegisterOutOfRange {
            name: "frame_length".into(),
            value: i64::MAX,
        })?;
    let length = u8::try_from(length).map_err(|_| ServoError::RegisterOutOfRange {
        name: "frame_length".into(),
        value: length as i64,
    })?;
    let checksum = !(id
        .wrapping_add(length)
        .wrapping_add(instruction)
        .wrapping_add(parameters.iter().copied().fold(0, u8::wrapping_add)));
    let mut frame = vec![0xFF, 0xFF, id, length, instruction];
    frame.extend_from_slice(parameters);
    frame.push(checksum);
    Ok(frame)
}

pub fn decode_frame(bytes: &[u8]) -> Result<(u8, u8, Vec<u8>), ServoError> {
    if bytes.len() < 6 || bytes[0..2] != [0xFF, 0xFF] {
        return Err(ServoError::InvalidRegister {
            name: "frame".into(),
        });
    }
    let length = bytes[3] as usize;
    let expected = length + 4;
    if bytes.len() != expected || length < 2 {
        return Err(ServoError::ShortRead {
            what: "Feetech frame",
            expected,
            got: bytes.len(),
        });
    }
    let checksum = !bytes[2..bytes.len() - 1]
        .iter()
        .copied()
        .fold(0, u8::wrapping_add);
    if checksum != bytes[bytes.len() - 1] {
        return Err(ServoError::InvalidRegister {
            name: "checksum".into(),
        });
    }
    Ok((bytes[2], bytes[4], bytes[5..bytes.len() - 1].to_vec()))
}

pub fn decode_xdat(bytes: &[u8]) -> Result<XdatImage, ServoError> {
    if bytes.len() != 51 {
        return Err(ServoError::ShortRead {
            what: "xdat",
            expected: 51,
            got: bytes.len(),
        });
    }
    if bytes[..2] != [0x00, 0x28] {
        return Err(ServoError::InvalidRegister {
            name: "xdat_header".into(),
        });
    }
    let mut image = [0; 51];
    image.copy_from_slice(bytes);
    Ok(XdatImage { bytes: image })
}

pub fn encode_xdat(image: XdatImage) -> Result<Vec<u8>, ServoError> {
    let mut bytes = image.bytes;
    bytes[..2].copy_from_slice(&[0x00, 0x28]);
    Ok(bytes.to_vec())
}
