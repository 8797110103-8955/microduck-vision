//! Register descriptors and raw register operations for connected buses.

use std::time::Duration;

use servo::hd1910::FeetechRawBus;
use servo::xl330::RawBus;
use servo::{RegisterAccess, ServoModel};

use super::ConnectedBus;

/// The subset of a control-table entry needed by the register UI and raw bus
/// operations. It is owned by the desktop boundary so model-specific corrections
/// do not require constructing rustypot's non-exhaustive RegisterInfo.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct RegisterDescriptor {
    pub(crate) name: &'static str,
    pub(crate) addr: u8,
    pub(crate) size: u8,
    pub(crate) access: RegisterAccess,
    pub(crate) is_default: bool,
}

pub(super) fn is_timeout_error(error: &str) -> bool {
    let error = error.to_ascii_lowercase();
    error.contains("timed out") || error.contains("timeout")
}

impl ConnectedBus {
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
}
