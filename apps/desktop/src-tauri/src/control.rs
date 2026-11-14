//! Adapters between desktop commands and the servo/loop-body crates.

use std::collections::HashMap;
use std::sync::Mutex;
use std::time::Duration;

use servo::hd1910::{Hd1910Driver, RustypotFeetechBus};
use servo::xl330::{RustypotXl330Bus, Xl330Driver};
use servo::{ServoLoopDriver, ServoModel, ServoPositionSample, ServoPositionTargets};

use crate::types::ServoIdentity;

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
        ServoModel::Hd1910 => RustypotFeetechBus::open_sts(port_name, baud, Duration::from_millis(20))
            .map(ConnectedBus::Hd1910)
            .map_err(|error| error.to_string()),
        ServoModel::Xl330M288 => RustypotXl330Bus::open(
            port_name,
            baud,
            Duration::from_millis(20),
            true,
        )
        .map(ConnectedBus::Xl330)
        .map_err(|error| error.to_string()),
        _ => Err("unsupported physical servo model".into()),
    }
}

pub(crate) fn try_open_feetech(
    port_name: &str,
    baud: u32,
) -> Result<ConnectedBus, String> {
    RustypotFeetechBus::open_sts(port_name, baud, Duration::from_millis(20))
        .map(ConnectedBus::Hd1910)
        .map_err(|error| error.to_string())
}

pub(crate) fn try_open_xl330(
    port_name: &str,
    baud: u32,
) -> Result<ConnectedBus, String> {
    RustypotXl330Bus::open(port_name, baud, Duration::from_millis(20), true)
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
    use super::move_pending_bus;
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
}
