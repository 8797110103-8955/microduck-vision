//! Servo operations on a connected physical bus.

use servo::hd1910::{
    FeetechRawBus, decode_register, position_rad_from_steps,
    velocity_rad_s_from_raw as hd_velocity_rad_s_from_raw,
};
use servo::xl330::{
    RawBus, decode_register as decode_xl_register, position_rad_from_raw, register as xl_register,
    velocity_rad_s_from_raw as xl_velocity_rad_s_from_raw,
};
use servo::{ServoModel, ServoPositionSample};

use crate::types::ServoIdentity;

use super::ConnectedBus;

impl ConnectedBus {
    pub(crate) fn set_gain(
        &mut self,
        model: ServoModel,
        ids: &[u8],
        gain: u16,
    ) -> Result<(), String> {
        match (model, self) {
            (ServoModel::Hd1910, Self::Hd1910(bus)) => {
                let bytes =
                    servo::hd1910::encode_register("p_coefficient", i64::from(gain.min(255)))
                        .map_err(|e| e.to_string())?;
                for id in ids {
                    bus.write_raw(*id, 21, &bytes).map_err(|e| e.to_string())?;
                }
                Ok(())
            }
            (ServoModel::Xl330M288, Self::Xl330(bus)) => {
                let register =
                    xl_register("position_p_gain").ok_or("missing XL330 P gain register")?;
                let bytes = servo::xl330::encode_register("position_p_gain", i64::from(gain))
                    .map_err(|e| e.to_string())?;
                for id in ids {
                    bus.write_raw(*id, register.addr, &bytes)
                        .map_err(|e| e.to_string())?;
                }
                Ok(())
            }
            _ => Err("connected bus protocol does not match servo model".into()),
        }
    }

    pub(crate) fn reboot(&mut self, id: u8) -> Result<(), String> {
        match self {
            Self::Hd1910(bus) => bus.reboot(id).map_err(|e| e.to_string()),
            Self::Xl330(bus) => bus.reboot(id).map_err(|e| e.to_string()),
        }
    }
    pub(crate) fn write_position(&mut self, id: u8, position: f64) -> Result<(), String> {
        if !position.is_finite() {
            return Err("目标位置必须是有限数字".into());
        }
        match self {
            Self::Hd1910(bus) => servo::hd1910::write_position_target(bus, id, position)
                .map_err(|error| error.to_string()),
            Self::Xl330(bus) => servo::xl330::write_position_target(bus, id, position)
                .map_err(|error| error.to_string()),
        }
    }

    pub(crate) fn scan(&mut self) -> Result<Vec<u8>, String> {
        match self {
            Self::Hd1910(bus) => bus
                .broadcast_scan_all()
                .map(|found| found.keys().copied().collect())
                .map_err(|error| error.to_string()),
            Self::Xl330(bus) => bus
                .broadcast_scan_all()
                .map(|found| found.keys().copied().collect())
                .map_err(|error| error.to_string()),
        }
    }

    pub(crate) fn scan_census(&mut self) -> Result<Vec<ServoIdentity>, String> {
        let model = match self {
            Self::Hd1910(_) => "hd1910",
            Self::Xl330(_) => "xl330",
        };
        Ok(self
            .scan()?
            .into_iter()
            .map(|servo_id| ServoIdentity {
                servo_id,
                model: model.into(),
            })
            .collect())
    }

    pub(crate) fn read_positions(
        &mut self,
        model: ServoModel,
        ids: &[u8],
    ) -> Result<Vec<f64>, String> {
        match (model, self) {
            (ServoModel::Hd1910, Self::Hd1910(bus)) => ids
                .iter()
                .map(|id| {
                    let bytes = bus
                        .read_raw(*id, 56, 2)
                        .map_err(|error| error.to_string())?;
                    if bytes.len() != 2 {
                        return Err("HD1910 position read returned a short frame".into());
                    }
                    let steps = decode_register("present_position", &bytes)
                        .map_err(|error| error.to_string())?;
                    Ok(position_rad_from_steps(ServoModel::Hd1910, steps as i32))
                })
                .collect(),
            (ServoModel::Xl330M288, Self::Xl330(bus)) => {
                let register = xl_register("present_position")
                    .ok_or_else(|| "missing XL330 position register".to_string())?;
                ids.iter()
                    .map(|id| {
                        let bytes = bus
                            .read_raw(*id, register.addr, register.size)
                            .map_err(|error| error.to_string())?;
                        if bytes.len() != register.size as usize {
                            return Err("XL330 position read returned a short frame".into());
                        }
                        let raw = decode_xl_register("present_position", &bytes)
                            .map_err(|error| error.to_string())?;
                        Ok(position_rad_from_raw(raw as i32))
                    })
                    .collect()
            }
            _ => Err("connected bus protocol does not match servo model".into()),
        }
    }

    pub(crate) fn read_position_samples(
        &mut self,
        model: ServoModel,
        ids: &[u8],
    ) -> Result<ServoPositionSample, String> {
        match (model, self) {
            (ServoModel::Hd1910, Self::Hd1910(bus)) => ids
                .iter()
                .map(|id| {
                    let bytes = bus
                        .read_raw(*id, 56, 4)
                        .map_err(|error| error.to_string())?;
                    if bytes.len() != 4 {
                        return Err("HD1910 motion block returned a short frame".into());
                    }
                    let position = decode_register("present_position", &bytes[..2])
                        .map_err(|error| error.to_string())?;
                    let velocity = decode_register("present_velocity", &bytes[2..])
                        .map_err(|error| error.to_string())?;
                    Ok((
                        position_rad_from_steps(model, position as i32),
                        hd_velocity_rad_s_from_raw(model, velocity as i16),
                    ))
                })
                .collect::<Result<Vec<_>, String>>()
                .map(|samples| ServoPositionSample {
                    positions: samples.iter().map(|(position, _)| *position).collect(),
                    velocities: samples.iter().map(|(_, velocity)| *velocity).collect(),
                }),
            (ServoModel::Xl330M288, Self::Xl330(bus)) => {
                let velocity = xl_register("present_velocity")
                    .ok_or_else(|| "missing XL330 velocity register".to_owned())?;
                let position = xl_register("present_position")
                    .ok_or_else(|| "missing XL330 position register".to_owned())?;
                let start = velocity.addr;
                let end = position.addr.saturating_add(position.size);
                let len = end.saturating_sub(start);
                let blocks = bus
                    .sync_read_raw(ids, start, len)
                    .map_err(|error| error.to_string())?;
                if blocks.len() != ids.len() {
                    return Err(format!(
                        "XL330 motion block count mismatch: expected {}, got {}",
                        ids.len(),
                        blocks.len()
                    ));
                }
                let mut positions = Vec::with_capacity(ids.len());
                let mut velocities = Vec::with_capacity(ids.len());
                for block in blocks {
                    if block.len() != len as usize {
                        return Err("XL330 motion block returned a short frame".into());
                    }
                    let velocity_raw =
                        decode_xl_register("present_velocity", &block[..velocity.size as usize])
                            .map_err(|error| error.to_string())?;
                    let position_start = position.addr.saturating_sub(start) as usize;
                    let position_raw = decode_xl_register(
                        "present_position",
                        &block[position_start..position_start + position.size as usize],
                    )
                    .map_err(|error| error.to_string())?;
                    velocities.push(xl_velocity_rad_s_from_raw(velocity_raw as i32));
                    positions.push(position_rad_from_raw(position_raw as i32));
                }
                Ok(ServoPositionSample {
                    positions,
                    velocities,
                })
            }
            _ => Err("connected bus protocol does not match servo model".into()),
        }
    }

    pub(crate) fn set_torque(
        &mut self,
        model: ServoModel,
        ids: &[u8],
        enabled: bool,
    ) -> Result<(), String> {
        match (model, self) {
            (ServoModel::Hd1910, Self::Hd1910(bus)) => {
                let bytes = servo::hd1910::encode_register("torque_enable", i64::from(enabled))
                    .map_err(|error| error.to_string())?;
                for id in ids {
                    bus.write_raw(*id, 40, &bytes)
                        .map_err(|error| error.to_string())?;
                }
                Ok(())
            }
            (ServoModel::Xl330M288, Self::Xl330(bus)) => {
                let register = xl_register("torque_enable")
                    .ok_or_else(|| "missing XL330 torque register".to_string())?;
                let bytes = servo::xl330::encode_register("torque_enable", i64::from(enabled))
                    .map_err(|error| error.to_string())?;
                for id in ids {
                    bus.write_raw(*id, register.addr, &bytes)
                        .map_err(|error| error.to_string())?;
                }
                Ok(())
            }
            _ => Err("connected bus protocol does not match servo model".into()),
        }
    }
}
