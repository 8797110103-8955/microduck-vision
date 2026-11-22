//! Servo operations on a connected physical bus.

use servo::hd1910::{FeetechRawBus, decode_motion_block, decode_register, position_rad_from_steps};
use servo::xl330::{
    RawBus, decode_register as decode_xl_register, position_rad_from_raw, register as xl_register,
    velocity_rad_s_from_raw as xl_velocity_rad_s_from_raw,
};
use servo::{ServoModel, ServoPositionSample};

use crate::types::ServoIdentity;

use super::ConnectedBus;

fn hd1910_configuration_snapshot(bytes: &[u8]) -> Result<serde_json::Value, String> {
    if bytes.len() != 60 {
        return Err(format!(
            "HD1910 configuration snapshot expected 60 bytes, got {}",
            bytes.len()
        ));
    }
    let word = |offset| u16::from_le_bytes([bytes[offset], bytes[offset + 1]]);
    Ok(serde_json::json!({
        "raw_bytes": bytes,
        "firmware": [bytes[0], bytes[1]],
        "model_version": [bytes[3], bytes[4]],
        "angle_limit_min_count": word(9),
        "angle_limit_max_count": word(11),
        "phase": bytes[18],
        "configured_kp": bytes[21],
        "configured_kd": bytes[22],
        "configured_ki": bytes[23],
        "protection_current_raw": word(28),
        "position_resolution_factor": bytes[30],
        "operating_mode": bytes[33],
        "torque_enable_raw": bytes[40],
        "goal_position_raw": word(42),
        "target_current_raw": word(44),
        "goal_speed_raw": word(46),
        "torque_limit_raw": word(48),
        "runtime_kp": bytes[50],
        "runtime_kd": bytes[51],
        "runtime_ki": bytes[52],
        "eeprom_lock": bytes[55],
        "present_position_raw": word(56),
        "present_velocity_raw": word(58),
        "velocity_rad_s_per_count": servo::hd1910::HD1910_VELOCITY_RAD_PER_SEC_PER_COUNT,
    }))
}

fn set_torque_for_ids(
    ids: &[u8],
    enabled: bool,
    mut write: impl FnMut(u8) -> Result<(), String>,
) -> Result<(), String> {
    let mut failures = Vec::new();
    for id in ids {
        if let Err(error) = write(*id) {
            let detail = format!("id={id}: {error}");
            if enabled {
                return Err(detail);
            }
            failures.push(detail);
        }
    }
    if failures.is_empty() {
        Ok(())
    } else {
        Err(format!("关闭力矩未确认：{}", failures.join("; ")))
    }
}

impl ConnectedBus {
    pub(crate) fn read_hd1910_configuration(
        &mut self,
        id: u8,
    ) -> Result<serde_json::Value, String> {
        let Self::Hd1910(bus) = self else {
            return Err("HD1910 configuration requires a Feetech bus".into());
        };
        let bytes = bus.read_raw(id, 0, 60).map_err(|error| error.to_string())?;
        hd1910_configuration_snapshot(&bytes)
    }

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
                    decode_motion_block(&bytes).map_err(|error| error.to_string())
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
                set_torque_for_ids(ids, enabled, |id| {
                    bus.write_raw(id, 40, &bytes)
                        .map_err(|error| error.to_string())
                })
            }
            (ServoModel::Xl330M288, Self::Xl330(bus)) => {
                let register = xl_register("torque_enable")
                    .ok_or_else(|| "missing XL330 torque register".to_string())?;
                let bytes = servo::xl330::encode_register("torque_enable", i64::from(enabled))
                    .map_err(|error| error.to_string())?;
                set_torque_for_ids(ids, enabled, |id| {
                    bus.write_raw(id, register.addr, &bytes)
                        .map_err(|error| error.to_string())
                })
            }
            _ => Err("connected bus protocol does not match servo model".into()),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::{hd1910_configuration_snapshot, set_torque_for_ids};

    #[test]
    fn configuration_snapshot_keeps_limits_and_runtime_gain_separate_from_startup_gain() {
        let mut bytes = [0_u8; 60];
        bytes[0] = 3;
        bytes[1] = 44;
        bytes[11..13].copy_from_slice(&4095_u16.to_le_bytes());
        bytes[21] = 200;
        bytes[30] = 1;
        bytes[44..46].copy_from_slice(&120_u16.to_le_bytes());
        bytes[50] = 32;
        let snapshot = hd1910_configuration_snapshot(&bytes).unwrap();
        assert_eq!(snapshot["firmware"], serde_json::json!([3, 44]));
        assert_eq!(snapshot["angle_limit_min_count"], 0);
        assert_eq!(snapshot["angle_limit_max_count"], 4095);
        assert_eq!(snapshot["configured_kp"], 200);
        assert_eq!(snapshot["runtime_kp"], 32);
        assert_eq!(snapshot["target_current_raw"], 120);
        assert!(hd1910_configuration_snapshot(&bytes[..59]).is_err());
    }

    #[test]
    fn torque_off_attempts_every_id_and_identifies_all_failures() {
        let mut attempted = Vec::new();
        let error = set_torque_for_ids(&[9, 1, 3], false, |id| {
            attempted.push(id);
            if id == 9 || id == 3 {
                Err("access denied".into())
            } else {
                Ok(())
            }
        })
        .unwrap_err();
        assert_eq!(attempted, [9, 1, 3]);
        assert!(error.contains("id=9"));
        assert!(error.contains("id=3"));
    }

    #[test]
    fn torque_on_stops_after_the_first_failed_id() {
        let mut attempted = Vec::new();
        assert!(
            set_torque_for_ids(&[9, 1, 3], true, |id| {
                attempted.push(id);
                Err("access denied".into())
            })
            .is_err()
        );
        assert_eq!(attempted, [9]);
    }
}
