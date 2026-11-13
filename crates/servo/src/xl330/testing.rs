use std::collections::BTreeMap;

use super::{RawBus, decode_register, encode_register, register};
use crate::ServoError;

/// In-memory Protocol 2 bus used by servo and loop tests.
///
/// Register addresses, widths, byte order and value validation are delegated to the pinned
/// rustypot XL330 definition. The fake only models bus persistence and device presence.
#[derive(Default)]
pub struct MemoryXl330Bus {
    servos: BTreeMap<u8, BTreeMap<u8, Vec<u8>>>,
}

impl MemoryXl330Bus {
    pub fn with_servos(ids: &[u8]) -> Self {
        let mut bus = Self::default();
        for &id in ids {
            bus.add_servo(id);
        }
        bus
    }

    pub fn add_servo(&mut self, id: u8) {
        let mut registers = BTreeMap::new();
        for name in [
            "torque_enable",
            "goal_position",
            "present_velocity",
            "present_position",
            "present_input_voltage",
            "present_temperature",
            "position_p_gain",
        ] {
            if let Some(info) = register(name) {
                registers.insert(info.addr, vec![0; info.size as usize]);
            }
        }
        if let Some(info) = register("present_position") {
            registers.insert(
                info.addr,
                encode_register("present_position", 2048).unwrap(),
            );
        }
        self.servos.insert(id, registers);
    }

    pub fn register(&self, id: u8, name: &str) -> i64 {
        let Some(info) = register(name) else { return 0 };
        self.servos
            .get(&id)
            .and_then(|registers| registers.get(&info.addr))
            .and_then(|bytes| decode_register(name, bytes).ok())
            .unwrap_or_default()
    }

    pub fn set_register(&mut self, id: u8, name: &str, value: i64) -> Result<(), ServoError> {
        let info = register(name).ok_or_else(|| ServoError::UnknownRegister(name.into()))?;
        let bytes = encode_register(name, value)?;
        self.write_raw(id, info.addr, &bytes)
    }

    fn registers_mut(&mut self, id: u8) -> Result<&mut BTreeMap<u8, Vec<u8>>, ServoError> {
        self.servos.get_mut(&id).ok_or(ServoError::MissingServo(id))
    }
}

impl RawBus for MemoryXl330Bus {
    fn read_raw(&mut self, id: u8, addr: u8, len: u8) -> Result<Vec<u8>, ServoError> {
        let registers = self.servos.get(&id).ok_or(ServoError::MissingServo(id))?;
        let mut bytes = Vec::with_capacity(len as usize);
        for offset in 0..len {
            let byte = registers
                .iter()
                .find_map(|(start, value)| {
                    let end = start.saturating_add(value.len() as u8);
                    (*start <= addr.saturating_add(offset) && addr.saturating_add(offset) < end)
                        .then(|| value[(addr.saturating_add(offset) - *start) as usize])
                })
                .unwrap_or(0);
            bytes.push(byte);
        }
        Ok(bytes)
    }

    fn write_raw(&mut self, id: u8, addr: u8, data: &[u8]) -> Result<(), ServoError> {
        let registers = self.registers_mut(id)?;
        registers.insert(addr, data.to_vec());
        Ok(())
    }

    fn sync_read_raw(&mut self, ids: &[u8], addr: u8, len: u8) -> Result<Vec<Vec<u8>>, ServoError> {
        ids.iter().map(|&id| self.read_raw(id, addr, len)).collect()
    }

    fn sync_write_raw(&mut self, ids: &[u8], addr: u8, data: &[Vec<u8>]) -> Result<(), ServoError> {
        if ids.len() != data.len() {
            return Err(ServoError::CountMismatch {
                what: "XL330 fake sync write",
                expected: ids.len(),
                got: data.len(),
            });
        }
        for (&id, bytes) in ids.iter().zip(data) {
            self.write_raw(id, addr, bytes)?;
        }
        Ok(())
    }

    fn ping(&mut self, id: u8) -> Result<bool, ServoError> {
        Ok(self.servos.contains_key(&id))
    }

    fn reboot(&mut self, id: u8) -> Result<(), ServoError> {
        self.servos
            .contains_key(&id)
            .then_some(())
            .ok_or(ServoError::MissingServo(id))
    }
}
