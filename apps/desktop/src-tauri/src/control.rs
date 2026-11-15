//! Adapters between desktop commands and the servo/loop-body crates.

use std::collections::HashMap;
use std::sync::Mutex;
use std::time::Duration;

use servo::hd1910::{
    FeetechRawBus, Hd1910Driver, RustypotFeetechBus, decode_register, position_rad_from_steps,
};
use servo::xl330::{
    RawBus, RustypotXl330Bus, Xl330Driver, decode_register as decode_xl_register,
    position_rad_from_raw, register as xl_register,
};
use servo::{
    RegisterAccess, ServoLoopDriver, ServoModel, ServoPositionSample, ServoPositionTargets,
};

/// The subset of a control-table entry needed by the register UI and raw bus
/// operations.  It is intentionally owned by the desktop boundary so model
/// specific corrections (such as the HD1910 0x18/0x19 split) do not require
/// constructing rustypot's non-exhaustive `RegisterInfo`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct RegisterDescriptor {
    pub(crate) name: &'static str,
    pub(crate) addr: u8,
    pub(crate) size: u8,
    pub(crate) access: RegisterAccess,
    pub(crate) is_default: bool,
}

use crate::types::ServoIdentity;

/// The FT reference controller allows up to 300 ms for one protocol
/// transaction.  EEPROM writes can acknowledge noticeably later than SRAM
/// writes, so the old 20 ms port timeout could report a timeout after the
/// servo had already applied the write.
const PHYSICAL_SERIAL_TIMEOUT: Duration = Duration::from_millis(300);

fn is_timeout_error(error: &str) -> bool {
    let error = error.to_ascii_lowercase();
    error.contains("timed out") || error.contains("timeout")
}

/// In-memory driver used by simulation loops.  Hardware drivers live in the
/// servo crate and are adapted to the same loop-body interface below.
pub(crate) struct MemoryDriver {
    pub(crate) positions: Vec<f64>,
    pub(crate) torque: bool,
}

pub(crate) enum ConnectedBus {
    Hd1910(RustypotFeetechBus),
    Xl330(RustypotXl330Bus),
}

pub(crate) fn open_for_model(
    model: ServoModel,
    port_name: &str,
    baud: u32,
) -> Result<ConnectedBus, String> {
    match model {
        ServoModel::Hd1910 => {
            RustypotFeetechBus::open_sts(port_name, baud, PHYSICAL_SERIAL_TIMEOUT)
                .map(ConnectedBus::Hd1910)
                .map_err(|error| error.to_string())
        }
        ServoModel::Xl330M288 => {
            RustypotXl330Bus::open(port_name, baud, PHYSICAL_SERIAL_TIMEOUT, true)
                .map(ConnectedBus::Xl330)
                .map_err(|error| error.to_string())
        }
        _ => Err("unsupported physical servo model".into()),
    }
}

pub(crate) fn try_open_feetech(port_name: &str, baud: u32) -> Result<ConnectedBus, String> {
    RustypotFeetechBus::open_sts(port_name, baud, PHYSICAL_SERIAL_TIMEOUT)
        .map(ConnectedBus::Hd1910)
        .map_err(|error| error.to_string())
}

pub(crate) fn try_open_xl330(port_name: &str, baud: u32) -> Result<ConnectedBus, String> {
    RustypotXl330Bus::open(port_name, baud, PHYSICAL_SERIAL_TIMEOUT, true)
        .map(ConnectedBus::Xl330)
        .map_err(|error| error.to_string())
}

/// Move a probed bus out of the temporary port entry before taking the lock
/// again.  Keeping this operation here avoids deadlocks in connection flows.
pub(crate) fn move_pending_bus<T>(
    buses: &Mutex<HashMap<String, T>>,
    pending_bus_key: Option<&str>,
    machine_id: &str,
) -> Result<(), String> {
    let pending_bus = if let Some(key) = pending_bus_key {
        let mut buses = buses.lock().map_err(|error| error.to_string())?;
        buses.remove(key)
    } else {
        None
    };
    if let Some(bus) = pending_bus {
        buses
            .lock()
            .map_err(|error| error.to_string())?
            .insert(machine_id.to_owned(), bus);
    }
    Ok(())
}

impl ConnectedBus {
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

    pub(crate) fn read_register(
        &mut self,
        model: ServoModel,
        servo_id: u8,
        register: RegisterDescriptor,
    ) -> Result<i64, String> {
        let bytes = match (model, self) {
            (ServoModel::Hd1910, Self::Hd1910(bus)) => bus
                .read_raw(servo_id, register.addr, register.size)
                .map_err(|error| error.to_string())?,
            (ServoModel::Xl330M288, Self::Xl330(bus)) => bus
                .read_raw(servo_id, register.addr, register.size)
                .map_err(|error| error.to_string())?,
            _ => return Err("connected bus protocol does not match servo model".into()),
        };
        if bytes.len() != register.size as usize {
            return Err(format!(
                "register {} returned {} bytes, expected {}",
                register.name,
                bytes.len(),
                register.size
            ));
        }
        match model {
            ServoModel::Hd1910 => servo::hd1910::decode_register(register.name, &bytes)
                .map_err(|error| error.to_string()),
            ServoModel::Xl330M288 => servo::xl330::decode_register(register.name, &bytes)
                .map_err(|error| error.to_string()),
            _ => Err("unsupported servo model".into()),
        }
    }

    pub(crate) fn write_register(
        &mut self,
        model: ServoModel,
        servo_id: u8,
        register: RegisterDescriptor,
        value: i64,
    ) -> Result<(), String> {
        let bytes = match model {
            ServoModel::Hd1910 => servo::hd1910::encode_register(register.name, value),
            ServoModel::Xl330M288 => servo::xl330::encode_register(register.name, value),
            _ => return Err("unsupported servo model".into()),
        }
        .map_err(|error| error.to_string())?;
        if bytes.len() != register.size as usize {
            return Err(format!(
                "register {} encoded to {} bytes, expected {}",
                register.name,
                bytes.len(),
                register.size
            ));
        }
        if matches!(model, ServoModel::Hd1910) && register.addr < 40 {
            // Feetech EPROM writes require the same unlock/write/relock sequence as the
            // reference controller.  Without this, the write may appear to succeed but only
            // affect the volatile mirror and disappear after power cycling.
            self.write_raw(model, servo_id, 55, &[0])
                .map_err(|error| format!("EEPROM unlock: {error}"))?;
            std::thread::sleep(Duration::from_millis(50));
            let write_result = self
                .write_raw(model, servo_id, register.addr, &bytes)
                .map_err(|error| format!("register write: {error}"));
            std::thread::sleep(Duration::from_millis(50));
            let mut relock_result = self
                .write_raw(model, servo_id, 55, &[1])
                .map_err(|error| format!("EEPROM lock: {error}"));
            if relock_result
                .as_ref()
                .is_err_and(|error| is_timeout_error(error))
            {
                // Some HD1910 units apply the lock byte but do not return a
                // status packet while the EEPROM protection state changes.
                // Retry once; if the second attempt has the same timeout, the
                // lock command was still sent on the wire and the preceding
                // target write is already confirmed.
                std::thread::sleep(Duration::from_millis(50));
                relock_result = self
                    .write_raw(model, servo_id, 55, &[1])
                    .map_err(|error| format!("EEPROM lock: {error}"));
            }
            return match (write_result, relock_result) {
                (Err(error), _) => Err(error),
                (Ok(()), Err(error)) if is_timeout_error(&error) => Ok(()),
                (Ok(()), Err(error)) => Err(error),
                (Ok(()), Ok(())) => Ok(()),
            };
        }
        self.write_raw(model, servo_id, register.addr, &bytes)
    }

    fn write_raw(
        &mut self,
        model: ServoModel,
        servo_id: u8,
        address: u8,
        bytes: &[u8],
    ) -> Result<(), String> {
        match (model, self) {
            (ServoModel::Hd1910, Self::Hd1910(bus)) => bus
                .write_raw(servo_id, address, bytes)
                .map_err(|error| error.to_string()),
            (ServoModel::Xl330M288, Self::Xl330(bus)) => bus
                .write_raw(servo_id, address, bytes)
                .map_err(|error| error.to_string()),
            _ => Err("connected bus protocol does not match servo model".into()),
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

    pub(crate) fn into_driver(
        self,
        model: ServoModel,
        ids: Vec<u8>,
    ) -> Result<Box<dyn ServoLoopDriver>, String> {
        match (model, self) {
            (ServoModel::Hd1910, Self::Hd1910(bus)) => {
                Ok(Box::new(Hd1910Driver::new(bus).with_ids(ids)))
            }
            (ServoModel::Xl330M288, Self::Xl330(bus)) => {
                Ok(Box::new(Xl330Driver::new(bus).with_ids(ids)))
            }
            (ServoModel::Hd1910, Self::Xl330(_)) | (ServoModel::Xl330M288, Self::Hd1910(_)) => {
                Err("connected bus protocol does not match virtual machine servo model".into())
            }
            (_, _) => Err("unsupported hardware servo model".into()),
        }
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

impl ServoLoopDriver for MemoryDriver {
    fn read_positions(&mut self, ids: &[u8]) -> Result<ServoPositionSample, servo::ServoError> {
        if self.positions.len() != ids.len() {
            self.positions.resize(ids.len(), 0.0);
        }
        Ok(ServoPositionSample {
            positions: self.positions.clone(),
            velocities: vec![0.0; ids.len()],
        })
    }

    fn write_positions(
        &mut self,
        _ids: &[u8],
        targets: &ServoPositionTargets,
    ) -> Result<(), servo::ServoError> {
        self.positions = targets.positions.clone();
        Ok(())
    }

    fn set_torque(&mut self, enabled: bool) -> Result<(), servo::ServoError> {
        self.torque = enabled;
        Ok(())
    }

    fn set_gain(&mut self, _gain: u16) -> Result<(), servo::ServoError> {
        Ok(())
    }

    fn reboot(&mut self, _id: u8) -> Result<(), servo::ServoError> {
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::{is_timeout_error, move_pending_bus};
    use std::collections::HashMap;
    use std::sync::Mutex;

    #[test]
    fn moving_pending_bus_releases_the_lock_before_reinserting() {
        let buses = Mutex::new(HashMap::from([("pending:COM7".to_owned(), 7_u8)]));

        move_pending_bus(&buses, Some("pending:COM7"), "physical-1").unwrap();

        let buses = buses.lock().unwrap();
        assert!(!buses.contains_key("pending:COM7"));
        assert_eq!(buses.get("physical-1"), Some(&7_u8));
    }

    #[test]
    fn only_timeout_errors_are_tolerated_for_final_eeprom_lock() {
        assert!(is_timeout_error(
            "EEPROM lock: bus error: Operation timed out"
        ));
        assert!(is_timeout_error("serial timeout"));
        assert!(!is_timeout_error("EEPROM lock: checksum mismatch"));
    }
}
