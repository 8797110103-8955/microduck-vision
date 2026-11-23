//! Servo operations on a connected physical bus.

use microduck_body_runtime::PhysicalGains;
use servo::hd1910::{FeetechRawBus, Hd1910Configuration};
use servo::xl330::RawBus;
use servo::{ServoModel, ServoPositionSample};

use crate::types::ServoIdentity;

use super::ConnectedBus;

pub(crate) fn hd1910_configuration_snapshot(config: &Hd1910Configuration) -> serde_json::Value {
    serde_json::json!({
        "raw_bytes": config.bytes.as_slice(),
        "firmware": config.firmware,
        "model_version": config.model_version,
        "angle_limit_min_count": config.domain.limits[0],
        "angle_limit_max_count": config.domain.limits[1],
        "phase": config.phase,
        "configured_kp": config.configured_gains[0],
        "configured_kd": config.configured_gains[1],
        "configured_ki": config.configured_gains[2],
        "protection_current_raw": config.protection_current_raw,
        "position_resolution_factor": config.domain.resolution,
        "operating_mode": config.domain.mode,
        "torque_enable_raw": config.torque_enable_raw,
        "goal_position_raw": config.goal_position_raw,
        "target_current_raw": config.target_current_raw,
        "goal_speed_raw": config.goal_speed_raw,
        "torque_limit_raw": config.torque_limit_raw,
        "runtime_kp": config.runtime_gains[0],
        "runtime_kd": config.runtime_gains[1],
        "runtime_ki": config.runtime_gains[2],
        "eeprom_lock": config.eeprom_lock,
        "present_position_raw": config.present_position_raw,
        "present_velocity_raw": config.present_velocity_raw,
        "velocity_rad_s_per_count": servo::hd1910::HD1910_VELOCITY_RAD_PER_SEC_PER_COUNT,
    })
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
    ) -> Result<Hd1910Configuration, String> {
        let Self::Hd1910(bus) = self else {
            return Err("HD1910 configuration requires a Feetech bus".into());
        };
        servo::hd1910::read_configuration(bus, id).map_err(|error| error.to_string())
    }

    pub(crate) fn set_gain(&mut self, ids: &[u8], gains: PhysicalGains) -> Result<(), String> {
        match (gains, self) {
            (PhysicalGains::Hd1910 { p }, Self::Hd1910(bus)) => ids
                .iter()
                .try_for_each(|&id| servo::hd1910::set_position_gain(bus, id, u16::from(p))),
            (PhysicalGains::Xl330 { p, i, d }, Self::Xl330(bus)) => ids
                .iter()
                .try_for_each(|&id| servo::xl330::set_position_pid(bus, id, p, i, d)),
            _ => return Err("connected bus protocol does not match servo model".into()),
        }
        .map_err(|error| error.to_string())
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
        let positions: Result<Vec<_>, _> = match (model, self) {
            (ServoModel::Hd1910, Self::Hd1910(bus)) => ids
                .iter()
                .map(|&id| servo::hd1910::read_position(bus, id))
                .collect(),
            (ServoModel::Xl330M288, Self::Xl330(bus)) => ids
                .iter()
                .map(|&id| servo::xl330::read_position(bus, id))
                .collect(),
            _ => return Err("connected bus protocol does not match servo model".into()),
        };
        positions.map_err(|error| error.to_string())
    }

    pub(crate) fn read_position_samples(
        &mut self,
        model: ServoModel,
        ids: &[u8],
    ) -> Result<ServoPositionSample, String> {
        match (model, self) {
            (ServoModel::Hd1910, Self::Hd1910(bus)) => {
                servo::hd1910::read_position_sample(bus, ids)
            }
            (ServoModel::Xl330M288, Self::Xl330(bus)) => {
                servo::xl330::read_position_sample(bus, ids)
            }
            _ => return Err("connected bus protocol does not match servo model".into()),
        }
        .map_err(|error| error.to_string())
    }

    pub(crate) fn set_torque(
        &mut self,
        model: ServoModel,
        ids: &[u8],
        enabled: bool,
    ) -> Result<(), String> {
        match (model, self) {
            (ServoModel::Hd1910, Self::Hd1910(bus)) => set_torque_for_ids(ids, enabled, |id| {
                servo::hd1910::set_torque(bus, id, enabled).map_err(|error| error.to_string())
            }),
            (ServoModel::Xl330M288, Self::Xl330(bus)) => set_torque_for_ids(ids, enabled, |id| {
                servo::xl330::set_torque(bus, id, enabled).map_err(|error| error.to_string())
            }),
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
        let configuration = servo::hd1910::Hd1910Configuration::parse(&bytes).unwrap();
        let snapshot = hd1910_configuration_snapshot(&configuration);
        assert_eq!(snapshot["firmware"], serde_json::json!([3, 44]));
        assert_eq!(snapshot["angle_limit_min_count"], 0);
        assert_eq!(snapshot["angle_limit_max_count"], 4095);
        assert_eq!(snapshot["configured_kp"], 200);
        assert_eq!(snapshot["runtime_kp"], 32);
        assert_eq!(snapshot["target_current_raw"], 120);
        assert!(servo::hd1910::Hd1910Configuration::parse(&bytes[..59]).is_err());
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
