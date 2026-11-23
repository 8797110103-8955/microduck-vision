use std::collections::BTreeMap;
use std::time::Duration;

use rustypot::servo::dynamixel::xl330::Xl330Controller;
use serialport::SerialPort;

use super::{
    decode_register, encode_register, position_rad_from_raw, position_raw_from_rad, register,
    velocity_rad_s_from_raw,
};
use crate::{
    ServoCommand, ServoDeviceDriver, ServoError, ServoPositionSample, ServoPositionTargets,
    ServoTelemetry,
};

pub fn read_position(bus: &mut impl RawBus, id: u8) -> Result<f64, ServoError> {
    let position = register("present_position").expect("vendor XL330 register");
    let bytes = bus.read_raw(id, position.addr, position.size)?;
    if bytes.len() != usize::from(position.size) {
        return Err(ServoError::ShortRead {
            what: "XL330 position",
            expected: usize::from(position.size),
            got: bytes.len(),
        });
    }
    let position = decode_register("present_position", &bytes)? as i32;
    Ok(position_rad_from_raw(position))
}

pub fn set_torque(bus: &mut impl RawBus, id: u8, enabled: bool) -> Result<(), ServoError> {
    let torque = register("torque_enable").expect("vendor XL330 register");
    let bytes = encode_register("torque_enable", i64::from(enabled))?;
    bus.write_raw(id, torque.addr, &bytes)
}

pub fn set_position_gain(bus: &mut impl RawBus, id: u8, gain: u16) -> Result<(), ServoError> {
    let register = register("position_p_gain").expect("vendor XL330 register");
    let bytes = encode_register("position_p_gain", i64::from(gain))?;
    bus.write_raw(id, register.addr, &bytes)
}

/// Write the full XL330 position controller settings in P, I, D order.
pub fn set_position_pid(
    bus: &mut impl RawBus,
    id: u8,
    p: u16,
    i: u16,
    d: u16,
) -> Result<(), ServoError> {
    let registers = [
        ("position_p_gain", p),
        ("position_i_gain", i),
        ("position_d_gain", d),
    ]
    .into_iter()
    .map(|(name, value)| {
        let bytes = encode_register(name, i64::from(value))?;
        Ok((
            register(name).expect("XL330 position gain register").addr,
            bytes,
        ))
    })
    .collect::<Result<Vec<_>, ServoError>>()?;
    for (address, bytes) in registers {
        bus.write_raw(id, address, &bytes)?;
    }
    Ok(())
}

pub fn read_position_sample(
    bus: &mut impl RawBus,
    ids: &[u8],
) -> Result<ServoPositionSample, ServoError> {
    let velocity = register("present_velocity").expect("vendor XL330 register");
    let position = register("present_position").expect("vendor XL330 register");
    let start = velocity.addr;
    let end = position.addr.saturating_add(position.size);
    let len = end.saturating_sub(start);
    let blocks = bus.sync_read_raw(ids, start, len)?;
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
        let velocity_raw = decode_register("present_velocity", &block[..velocity.size as usize])?;
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

/// Send one XL330 goal-position command through the protocol's sync-write
/// path.  Keeping this at the driver boundary lets the desktop debug control
/// use the same conversion and wire format as a running loop.
pub fn write_position_target<B: RawBus>(
    bus: &mut B,
    id: u8,
    position: f64,
) -> Result<(), ServoError> {
    let register = register("goal_position").expect("vendor XL330 register");
    let raw = position_raw_from_rad(position)?;
    let data = encode_register("goal_position", i64::from(raw))?;
    bus.sync_write_raw(&[id], register.addr, &[data])
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
        read_position_sample(&mut self.bus, &[id])?;
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
        read_position_sample(&mut self.bus, ids).map(|sample| sample.positions)
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
        set_torque(&mut self.bus, _id, _enabled)
    }

    fn set_gain(&mut self, id: u8, gain: u16) -> Result<(), ServoError> {
        set_position_gain(&mut self.bus, id, gain)
    }

    fn reboot(&mut self, _id: u8) -> Result<(), ServoError> {
        self.bus.reboot(_id)
    }

    fn ping(&mut self, _id: u8) -> Result<bool, ServoError> {
        self.bus.ping(_id)
    }
}

impl<B: RawBus + Send> Xl330Driver<B> {
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

impl<B: RawBus + Send> ServoDeviceDriver for Xl330Driver<B> {
    fn read_positions(&mut self, ids: &[u8]) -> Result<ServoPositionSample, ServoError> {
        read_position_sample(&mut self.bus, ids)
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

    /// Discover every answering XL330 servo with one broadcast ping and a model read
    /// per responder. This avoids waiting once for every absent ID on the bus.
    pub fn broadcast_scan_all(&mut self) -> Result<BTreeMap<u8, u16>, ServoError> {
        self.controller
            .broadcast_scan_all()
            .map_err(|error| ServoError::Bus(error.to_string()))
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
