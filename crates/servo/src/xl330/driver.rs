use std::time::Duration;

use rustypot::servo::dynamixel::xl330::Xl330Controller;
use serialport::SerialPort;

use super::{
    decode_register, encode_register, position_rad_from_raw, position_raw_from_rad, register,
    velocity_rad_s_from_raw,
};
use crate::{
    ServoCommand, ServoError, ServoLoopDriver, ServoPositionSample, ServoPositionTargets,
    ServoTelemetry,
};

pub trait RawBus {
    fn read_raw(&mut self, id: u8, addr: u8, len: u8) -> Result<Vec<u8>, ServoError>;
    fn write_raw(&mut self, id: u8, addr: u8, data: &[u8]) -> Result<(), ServoError>;
    fn sync_read_raw(&mut self, ids: &[u8], addr: u8, len: u8) -> Result<Vec<Vec<u8>>, ServoError>;
    fn sync_write_raw(&mut self, ids: &[u8], addr: u8, data: &[Vec<u8>]) -> Result<(), ServoError>;
    fn ping(&mut self, id: u8) -> Result<bool, ServoError>;
    fn reboot(&mut self, id: u8) -> Result<(), ServoError>;
}

pub trait ServoDriver: Send {
    fn read_telemetry(&mut self, id: u8) -> Result<ServoTelemetry, ServoError>;
    fn read_positions(&mut self, ids: &[u8]) -> Result<Vec<f64>, ServoError>;
    fn write_position(&mut self, command: ServoCommand) -> Result<(), ServoError>;
    fn sync_write_positions(&mut self, commands: &[ServoCommand]) -> Result<(), ServoError>;
    fn set_torque(&mut self, id: u8, enabled: bool) -> Result<(), ServoError>;
    fn set_gain(&mut self, id: u8, gain: u16) -> Result<(), ServoError>;
    fn reboot(&mut self, id: u8) -> Result<(), ServoError>;
    fn ping(&mut self, id: u8) -> Result<bool, ServoError>;
}

pub struct Xl330Driver<B> {
    bus: B,
    ids: Vec<u8>,
}

impl<B> Xl330Driver<B> {
    pub fn new(bus: B) -> Self {
        Self {
            bus,
            ids: Vec::new(),
        }
    }

    pub fn with_ids(mut self, ids: impl IntoIterator<Item = u8>) -> Self {
        self.ids = ids.into_iter().collect();
        self
    }

    pub fn into_inner(self) -> B {
        self.bus
    }
}

impl<B: RawBus + Send> ServoDriver for Xl330Driver<B> {
    fn read_telemetry(&mut self, id: u8) -> Result<ServoTelemetry, ServoError> {
        self.read_position_sample(&[id])?;
        let voltage = register("present_input_voltage").expect("vendor XL330 register");
        let temperature = register("present_temperature").expect("vendor XL330 register");
        let len = temperature
            .addr
            .saturating_add(temperature.size)
            .saturating_sub(voltage.addr);
        let _ = self.bus.read_raw(id, voltage.addr, len)?;
        Ok(ServoTelemetry)
    }

    fn read_positions(&mut self, ids: &[u8]) -> Result<Vec<f64>, ServoError> {
        self.read_position_sample(ids)
            .map(|sample| sample.positions)
    }

    fn write_position(&mut self, _command: ServoCommand) -> Result<(), ServoError> {
        Err(ServoError::NotImplemented("Xl330Driver::write_position"))
    }

    fn sync_write_positions(&mut self, _commands: &[ServoCommand]) -> Result<(), ServoError> {
        Err(ServoError::NotImplemented(
            "Xl330Driver::sync_write_positions",
        ))
    }

    fn set_torque(&mut self, _id: u8, _enabled: bool) -> Result<(), ServoError> {
        let register = register("torque_enable").expect("vendor XL330 register");
        let bytes = encode_register("torque_enable", i64::from(_enabled))?;
        self.bus.write_raw(_id, register.addr, &bytes)
    }

    fn set_gain(&mut self, id: u8, gain: u16) -> Result<(), ServoError> {
        let register = register("position_p_gain").expect("vendor XL330 register");
        let bytes = encode_register("position_p_gain", i64::from(gain))?;
        self.bus.write_raw(id, register.addr, &bytes)
    }

    fn reboot(&mut self, _id: u8) -> Result<(), ServoError> {
        self.bus.reboot(_id)
    }

    fn ping(&mut self, _id: u8) -> Result<bool, ServoError> {
        self.bus.ping(_id)
    }
}

impl<B: RawBus + Send> Xl330Driver<B> {
    fn read_position_sample(&mut self, ids: &[u8]) -> Result<ServoPositionSample, ServoError> {
        let velocity = register("present_velocity").expect("vendor XL330 register");
        let position = register("present_position").expect("vendor XL330 register");
        let start = velocity.addr;
        let end = position.addr.saturating_add(position.size);
        let len = end.saturating_sub(start);
        let blocks = self.bus.sync_read_raw(ids, start, len)?;
        if blocks.len() != ids.len() {
            return Err(ServoError::CountMismatch {
                what: "XL330 sync read",
                expected: ids.len(),
                got: blocks.len(),
            });
        }
        let mut positions = Vec::with_capacity(ids.len());
        let mut velocities = Vec::with_capacity(ids.len());
        for block in blocks {
            if block.len() != len as usize {
                return Err(ServoError::ShortRead {
                    what: "XL330 motion block",
                    expected: len as usize,
                    got: block.len(),
                });
            }
            let velocity_raw =
                decode_register("present_velocity", &block[..velocity.size as usize])?;
            let position_start = position.addr.saturating_sub(start) as usize;
            let position_raw = decode_register(
                "present_position",
                &block[position_start..position_start + position.size as usize],
            )?;
            velocities.push(velocity_rad_s_from_raw(velocity_raw as i32));
            positions.push(position_rad_from_raw(position_raw as i32));
        }
        Ok(ServoPositionSample {
            positions,
            velocities,
        })
    }

    fn write_position_targets(
        &mut self,
        ids: &[u8],
        targets: &ServoPositionTargets,
    ) -> Result<(), ServoError> {
        if ids.len() != targets.positions.len() {
            return Err(ServoError::CountMismatch {
                what: "XL330 position targets",
                expected: ids.len(),
                got: targets.positions.len(),
            });
        }
        let register = register("goal_position").expect("vendor XL330 register");
        let data = targets
            .positions
            .iter()
            .map(|position| {
                position_raw_from_rad(*position)
                    .and_then(|raw| encode_register("goal_position", i64::from(raw)))
            })
            .collect::<Result<Vec<_>, _>>()?;
        self.bus.sync_write_raw(ids, register.addr, &data)
    }
}

impl<B: RawBus + Send> ServoLoopDriver for Xl330Driver<B> {
    fn read_positions(&mut self, ids: &[u8]) -> Result<ServoPositionSample, ServoError> {
        self.read_position_sample(ids)
    }

    fn write_positions(
        &mut self,
        ids: &[u8],
        targets: &ServoPositionTargets,
    ) -> Result<(), ServoError> {
        self.write_position_targets(ids, targets)
    }

    fn set_torque(&mut self, enabled: bool) -> Result<(), ServoError> {
        for id in self.ids.clone() {
            ServoDriver::set_torque(self, id, enabled)?;
        }
        Ok(())
    }

    fn set_gain(&mut self, gain: u16) -> Result<(), ServoError> {
        for id in self.ids.clone() {
            ServoDriver::set_gain(self, id, gain)?;
        }
        Ok(())
    }

    fn reboot(&mut self, id: u8) -> Result<(), ServoError> {
        ServoDriver::reboot(self, id)
    }
}

pub struct RustypotXl330Bus {
    controller: Xl330Controller,
}

impl RustypotXl330Bus {
    pub fn open(
        port: &str,
        baud: u32,
        timeout: Duration,
        fast_sync_read: bool,
    ) -> Result<Self, ServoError> {
        let serial = serialport::new(port, baud)
            .timeout(timeout)
            .open()
            .map_err(|error| ServoError::Bus(error.to_string()))?;
        Ok(Self::from_serial(serial, fast_sync_read))
    }

    pub fn from_serial(port: Box<dyn SerialPort>, fast_sync_read: bool) -> Self {
        let mut controller = Xl330Controller::new()
            .with_serial_port(port)
            .with_protocol_v2();
        controller.set_fast_sync_read(fast_sync_read);
        Self { controller }
    }
}

impl RawBus for RustypotXl330Bus {
    fn read_raw(&mut self, id: u8, addr: u8, len: u8) -> Result<Vec<u8>, ServoError> {
        self.controller
            .read_raw_data(id, addr, len)
            .map_err(|error| ServoError::Bus(error.to_string()))
    }

    fn write_raw(&mut self, id: u8, addr: u8, data: &[u8]) -> Result<(), ServoError> {
        self.controller
            .write_raw_data(id, addr, data.to_vec())
            .map_err(|error| ServoError::Bus(error.to_string()))
    }

    fn sync_read_raw(&mut self, ids: &[u8], addr: u8, len: u8) -> Result<Vec<Vec<u8>>, ServoError> {
        self.controller
            .sync_read_raw_data(ids, addr, len)
            .map_err(|error| ServoError::Bus(error.to_string()))
    }

    fn sync_write_raw(&mut self, ids: &[u8], addr: u8, data: &[Vec<u8>]) -> Result<(), ServoError> {
        self.controller
            .sync_write_raw_data(ids, addr, data)
            .map_err(|error| ServoError::Bus(error.to_string()))
    }

    fn ping(&mut self, id: u8) -> Result<bool, ServoError> {
        self.controller
            .ping(id)
            .map_err(|error| ServoError::Bus(error.to_string()))
    }

    fn reboot(&mut self, id: u8) -> Result<(), ServoError> {
        self.controller
            .reboot(id)
            .map(|_| ())
            .map_err(|error| ServoError::Bus(error.to_string()))
    }
}
