use std::collections::BTreeMap;
use std::time::Duration;

use rustypot::servo::feetech::scs0009::Scs0009Controller;
use rustypot::servo::feetech::sts3215::Sts3215Controller;
use serialport::SerialPort;

use super::{
    decode_register, position_rad_from_steps, position_steps_from_rad, velocity_rad_s_from_raw,
};
use crate::{
    Gain, ServoCommand, ServoError, ServoIdentity, ServoLoopDriver, ServoPositionSample,
    ServoPositionTargets, ServoTelemetry,
};

const DEFAULT_MOVE_SPEED: i64 = 1_000;

/// Send one position command using the same seven-byte motion block as the
/// FT web controller.  This is intentionally an ordinary WRITE (rather than
/// SYNC_WRITE): HD1910 firmware used by the desktop app can acknowledge a
/// sync write without moving the servo.
pub fn write_position_target<B: FeetechRawBus>(
    bus: &mut B,
    id: u8,
    position: f64,
) -> Result<(), ServoError> {
    let steps = position_steps_from_rad(crate::ServoModel::Hd1910, position)?;
    let position = super::encode_register("goal_position", i64::from(steps))?;
    let speed = super::encode_register("goal_speed", DEFAULT_MOVE_SPEED)?;
    let mut block = Vec::with_capacity(7);
    block.push(0); // acceleration: 0 means the servo's maximum
    block.extend_from_slice(&position);
    block.extend_from_slice(&[0, 0]); // PWM open speed: use speed mode
    block.extend_from_slice(&speed);
    bus.write_raw(id, 41, &block)
}

pub trait FeetechRawBus {
    fn read_raw(&mut self, id: u8, addr: u8, len: u8) -> Result<Vec<u8>, ServoError>;
    fn write_raw(&mut self, id: u8, addr: u8, data: &[u8]) -> Result<(), ServoError>;
    fn sync_read_raw(&mut self, ids: &[u8], addr: u8, len: u8) -> Result<Vec<Vec<u8>>, ServoError>;
    fn sync_write_raw(&mut self, ids: &[u8], addr: u8, data: &[Vec<u8>]) -> Result<(), ServoError>;
    fn ping(&mut self, id: u8) -> Result<bool, ServoError>;
    fn reboot(&mut self, id: u8) -> Result<(), ServoError>;
}

pub trait FeetechDriver: Send {
    fn read(&mut self, id: u8) -> Result<ServoTelemetry, ServoError>;
    fn write(&mut self, command: ServoCommand) -> Result<(), ServoError>;
    fn set_torque(&mut self, enabled: bool) -> Result<(), ServoError>;
    fn set_gain(&mut self, gain: Gain) -> Result<(), ServoError>;
    fn scan(&mut self) -> Result<Vec<ServoIdentity>, ServoError>;
    fn reboot(&mut self, id: u8) -> Result<(), ServoError>;
}

pub enum RustypotFeetechBus {
    Sts(Sts3215Controller),
    Scs(Scs0009Controller),
}

impl RustypotFeetechBus {
    pub fn open_sts(port: &str, baud: u32, timeout: Duration) -> Result<Self, ServoError> {
        let serial = serialport::new(port, baud)
            .timeout(timeout)
            .open()
            .map_err(|error| ServoError::Bus(error.to_string()))?;
        Ok(Self::Sts(
            Sts3215Controller::new()
                .with_serial_port(serial)
                .with_protocol_v1(),
        ))
    }

    pub fn open_scs(port: &str, baud: u32, timeout: Duration) -> Result<Self, ServoError> {
        let serial = serialport::new(port, baud)
            .timeout(timeout)
            .open()
            .map_err(|error| ServoError::Bus(error.to_string()))?;
        Ok(Self::Scs(
            Scs0009Controller::new()
                .with_serial_port(serial)
                .with_protocol_v1(),
        ))
    }

    pub fn from_sts_serial(port: Box<dyn SerialPort>) -> Self {
        Self::Sts(
            Sts3215Controller::new()
                .with_serial_port(port)
                .with_protocol_v1(),
        )
    }

    /// Discover every answering STS servo with one broadcast ping and a model read
    /// per responder. This avoids waiting once for every absent ID on the bus.
    pub fn broadcast_scan_all(&mut self) -> Result<BTreeMap<u8, u16>, ServoError> {
        match self {
            Self::Sts(controller) => controller
                .broadcast_scan_all()
                .map_err(|error| ServoError::Bus(error.to_string())),
            Self::Scs(_) => Err(ServoError::NotImplemented(
                "SCS broadcast scan is not supported",
            )),
        }
    }
}

impl FeetechRawBus for RustypotFeetechBus {
    fn read_raw(&mut self, id: u8, addr: u8, len: u8) -> Result<Vec<u8>, ServoError> {
        match self {
            Self::Sts(controller) => controller
                .read_raw_data(id, addr, len)
                .map_err(|error| ServoError::Bus(error.to_string())),
            Self::Scs(controller) => controller
                .read_raw_data(id, addr, len)
                .map_err(|error| ServoError::Bus(error.to_string())),
        }
    }

    fn write_raw(&mut self, id: u8, addr: u8, data: &[u8]) -> Result<(), ServoError> {
        match self {
            Self::Sts(controller) => controller
                .write_raw_data(id, addr, data.to_vec())
                .map_err(|error| ServoError::Bus(error.to_string())),
            Self::Scs(controller) => controller
                .write_raw_data(id, addr, data.to_vec())
                .map_err(|error| ServoError::Bus(error.to_string())),
        }
    }

    fn sync_read_raw(&mut self, ids: &[u8], addr: u8, len: u8) -> Result<Vec<Vec<u8>>, ServoError> {
        match self {
            Self::Sts(controller) => controller
                .sync_read_raw_data(ids, addr, len)
                .map_err(|error| ServoError::Bus(error.to_string())),
            Self::Scs(controller) => controller
                .sync_read_raw_data(ids, addr, len)
                .map_err(|error| ServoError::Bus(error.to_string())),
        }
    }

    fn sync_write_raw(&mut self, ids: &[u8], addr: u8, data: &[Vec<u8>]) -> Result<(), ServoError> {
        match self {
            Self::Sts(controller) => controller
                .sync_write_raw_data(ids, addr, data)
                .map_err(|error| ServoError::Bus(error.to_string())),
            Self::Scs(controller) => controller
                .sync_write_raw_data(ids, addr, data)
                .map_err(|error| ServoError::Bus(error.to_string())),
        }
    }

    fn ping(&mut self, id: u8) -> Result<bool, ServoError> {
        match self {
            Self::Sts(controller) => controller
                .ping(id)
                .map_err(|error| ServoError::Bus(error.to_string())),
            Self::Scs(controller) => controller
                .ping(id)
                .map_err(|error| ServoError::Bus(error.to_string())),
        }
    }

    fn reboot(&mut self, id: u8) -> Result<(), ServoError> {
        match self {
            Self::Sts(controller) => controller
                .reboot(id)
                .map(|_| ())
                .map_err(|error| ServoError::Bus(error.to_string())),
            Self::Scs(controller) => controller
                .reboot(id)
                .map(|_| ())
                .map_err(|error| ServoError::Bus(error.to_string())),
        }
    }
}

pub struct Hd1910Driver<B> {
    bus: B,
    ids: Vec<u8>,
}

impl<B> Hd1910Driver<B> {
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

impl<B: FeetechRawBus + Send> FeetechDriver for Hd1910Driver<B> {
    fn read(&mut self, id: u8) -> Result<ServoTelemetry, ServoError> {
        let _ = self.bus.read_raw(id, 56, 2)?;
        Ok(ServoTelemetry)
    }

    fn write(&mut self, _command: ServoCommand) -> Result<(), ServoError> {
        Err(ServoError::NotImplemented("Hd1910Driver::write"))
    }

    fn set_torque(&mut self, _enabled: bool) -> Result<(), ServoError> {
        let bytes = super::encode_register("torque_enable", i64::from(_enabled))?;
        for id in self.ids.clone() {
            self.bus.write_raw(id, 40, &bytes)?;
        }
        Ok(())
    }

    fn set_gain(&mut self, _gain: Gain) -> Result<(), ServoError> {
        Ok(())
    }

    fn scan(&mut self) -> Result<Vec<ServoIdentity>, ServoError> {
        let mut found = Vec::new();
        for id in 0..=253 {
            if self.bus.ping(id)? {
                found.push(ServoIdentity);
            }
        }
        Ok(found)
    }

    fn reboot(&mut self, id: u8) -> Result<(), ServoError> {
        self.bus.reboot(id)
    }
}

impl<B: FeetechRawBus + Send> Hd1910Driver<B> {
    fn read_position_sample(&mut self, ids: &[u8]) -> Result<ServoPositionSample, ServoError> {
        let mut positions = Vec::with_capacity(ids.len());
        let mut velocities = Vec::with_capacity(ids.len());
        for &id in ids {
            let block = self.bus.read_raw(id, 56, 4)?;
            if block.len() != 4 {
                return Err(ServoError::ShortRead {
                    what: "HD1910 motion block",
                    expected: 4,
                    got: block.len(),
                });
            }
            let position = decode_register("present_position", &block[..2])? as i32;
            let velocity = decode_register("present_velocity", &block[2..])? as i16;
            positions.push(position_rad_from_steps(crate::ServoModel::Hd1910, position));
            velocities.push(velocity_rad_s_from_raw(crate::ServoModel::Hd1910, velocity));
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
                what: "HD1910 position targets",
                expected: ids.len(),
                got: targets.positions.len(),
            });
        }
        // Feetech STS/HD1910 uses the same motion block as the official web
        // controller: write from acceleration (41) through goal speed (46) in
        // one ordinary WRITE transaction per servo.  The web controller does
        // not use SYNC_WRITE here; some HD1910 firmware accepts the latter
        // without moving, so keep the wire operation identical to the known
        // working implementation.  Writing only goal_position (42) also leaves
        // speed and acceleration from a previous command active on the servo.
        for (&id, position) in ids.iter().zip(&targets.positions) {
            write_position_target(&mut self.bus, id, *position)?;
        }
        Ok(())
    }
}

impl<B: FeetechRawBus + Send> ServoLoopDriver for Hd1910Driver<B> {
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
        FeetechDriver::set_torque(self, enabled)
    }

    fn set_gain(&mut self, _gain: u16) -> Result<(), ServoError> {
        for id in self.ids.clone() {
            let bytes = super::encode_register("p_coefficient", i64::from(_gain.min(255)))?;
            self.bus.write_raw(id, 21, &bytes)?;
        }
        Ok(())
    }

    fn reboot(&mut self, id: u8) -> Result<(), ServoError> {
        FeetechDriver::reboot(self, id)
    }
}
