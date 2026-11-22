use std::collections::BTreeMap;
use std::time::Duration;

use rustypot::servo::feetech::scs0009::Scs0009Controller;
use rustypot::servo::feetech::sts3215::Sts3215Controller;
use serialport::SerialPort;

use super::{decode_motion_block, position_steps_from_rad};
use crate::{
    Gain, ServoCommand, ServoError, ServoIdentity, ServoLoopDriver, ServoPositionSample,
    ServoPositionTargets, ServoTelemetry,
};

const DEFAULT_MOVE_SPEED: i64 = 1_000;

/// Send exactly one calibration request. A timeout is an uncertain result, never a reason
/// to repeat an EEPROM-changing instruction. Readback/recovery belong to the caller.
pub fn send_center_calibration<T: std::io::Read + std::io::Write + ?Sized>(
    port: &mut T,
    id: u8,
) -> Result<(), ServoError> {
    if id > 253 {
        return Err(ServoError::Bus(
            "position calibration requires a unicast ID".into(),
        ));
    }
    let packet = super::encode_frame(id, 0x0B, &[])?;
    let io_error = |error: std::io::Error| ServoError::Bus(error.to_string());
    port.write_all(&packet).map_err(io_error)?;
    port.flush().map_err(io_error)?;
    let mut response = [0; 6];
    port.read_exact(&mut response).map_err(io_error)?;
    let (response_id, error, parameters) = super::decode_frame(&response)?;
    if response_id != id || error != 0 || !parameters.is_empty() {
        return Err(ServoError::Bus(format!(
            "calibration status id={response_id} error={error} parameters={parameters:?}"
        )));
    }
    Ok(())
}

/// Send one position command using the same seven-byte motion block as the
/// FT web controller.  This is intentionally an ordinary WRITE (rather than
/// SYNC_WRITE): HD1910 firmware used by the desktop app can acknowledge a
/// sync write without moving the servo.
pub fn write_position_target<B: FeetechRawBus>(
    bus: &mut B,
    id: u8,
    position: f64,
) -> Result<(), ServoError> {
    let block = position_target_block(position)?;
    bus.write_raw(id, 41, &block)
}

pub fn position_target_block(position: f64) -> Result<[u8; 7], ServoError> {
    let steps = position_steps_from_rad(crate::ServoModel::Hd1910, position)?;
    let position = super::encode_register("goal_position", i64::from(steps))?;
    let speed = super::encode_register("goal_speed", DEFAULT_MOVE_SPEED)?;
    Ok([0, position[0], position[1], 0, 0, speed[0], speed[1]])
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
    Sts(Sts3215Controller, Option<Box<dyn SerialPort>>),
    Scs(Scs0009Controller),
}

impl RustypotFeetechBus {
    pub fn open_sts(port: &str, baud: u32, timeout: Duration) -> Result<Self, ServoError> {
        let serial = serialport::new(port, baud)
            .timeout(timeout)
            .open()
            .map_err(|error| ServoError::Bus(error.to_string()))?;
        let mut calibration_port = serial
            .try_clone()
            .map_err(|error| ServoError::Bus(error.to_string()))?;
        calibration_port
            .set_timeout(timeout)
            .map_err(|error| ServoError::Bus(error.to_string()))?;
        Ok(Self::Sts(
            Sts3215Controller::new()
                .with_serial_port(serial)
                .with_protocol_v1(),
            Some(calibration_port),
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
        let calibration_port = port.try_clone().ok();
        Self::Sts(
            Sts3215Controller::new()
                .with_serial_port(port)
                .with_protocol_v1(),
            calibration_port,
        )
    }

    /// HLS firmware >=3.43 uses instruction 0x0B, not STS torque=128.
    /// The caller must have disabled torque and must verify the resulting position/offset.
    pub fn calibrate_position_center(&mut self, id: u8) -> Result<(), ServoError> {
        let Self::Sts(_, Some(port)) = self else {
            return Err(ServoError::NotImplemented(
                "HLS position calibration transport",
            ));
        };
        port.clear(serialport::ClearBuffer::Input)
            .map_err(|error| ServoError::Bus(error.to_string()))?;
        send_center_calibration(port.as_mut(), id)
    }

    /// Discover every answering STS servo with one broadcast ping and a model read
    /// per responder. This avoids waiting once for every absent ID on the bus.
    pub fn broadcast_scan_all(&mut self) -> Result<BTreeMap<u8, u16>, ServoError> {
        match self {
            Self::Sts(controller, _) => controller
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
            Self::Sts(controller, _) => controller
                .read_raw_data(id, addr, len)
                .map_err(|error| ServoError::Bus(error.to_string())),
            Self::Scs(controller) => controller
                .read_raw_data(id, addr, len)
                .map_err(|error| ServoError::Bus(error.to_string())),
        }
    }

    fn write_raw(&mut self, id: u8, addr: u8, data: &[u8]) -> Result<(), ServoError> {
        match self {
            Self::Sts(controller, _) => controller
                .write_raw_data(id, addr, data.to_vec())
                .map_err(|error| ServoError::Bus(error.to_string())),
            Self::Scs(controller) => controller
                .write_raw_data(id, addr, data.to_vec())
                .map_err(|error| ServoError::Bus(error.to_string())),
        }
    }

    fn sync_read_raw(&mut self, ids: &[u8], addr: u8, len: u8) -> Result<Vec<Vec<u8>>, ServoError> {
        match self {
            Self::Sts(controller, _) => controller
                .sync_read_raw_data(ids, addr, len)
                .map_err(|error| ServoError::Bus(error.to_string())),
            Self::Scs(controller) => controller
                .sync_read_raw_data(ids, addr, len)
                .map_err(|error| ServoError::Bus(error.to_string())),
        }
    }

    fn sync_write_raw(&mut self, ids: &[u8], addr: u8, data: &[Vec<u8>]) -> Result<(), ServoError> {
        match self {
            Self::Sts(controller, _) => controller
                .sync_write_raw_data(ids, addr, data)
                .map_err(|error| ServoError::Bus(error.to_string())),
            Self::Scs(controller) => controller
                .sync_write_raw_data(ids, addr, data)
                .map_err(|error| ServoError::Bus(error.to_string())),
        }
    }

    fn ping(&mut self, id: u8) -> Result<bool, ServoError> {
        match self {
            Self::Sts(controller, _) => controller
                .ping(id)
                .map_err(|error| ServoError::Bus(error.to_string())),
            Self::Scs(controller) => controller
                .ping(id)
                .map_err(|error| ServoError::Bus(error.to_string())),
        }
    }

    fn reboot(&mut self, id: u8) -> Result<(), ServoError> {
        match self {
            Self::Sts(controller, _) => controller
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
            let (position, velocity) = decode_motion_block(&block)?;
            positions.push(position);
            velocities.push(velocity);
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
        let blocks = targets
            .positions
            .iter()
            .map(|position| position_target_block(*position))
            .collect::<Result<Vec<_>, _>>()?;
        for (&id, block) in ids.iter().zip(blocks) {
            self.bus.write_raw(id, 41, &block)?;
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
