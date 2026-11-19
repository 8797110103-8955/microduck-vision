//! Serial transport ownership and connected-bus construction.

use std::collections::HashMap;
use std::sync::Mutex;
use std::time::Duration;

use servo::ServoModel;
use servo::hd1910::RustypotFeetechBus;
use servo::xl330::RustypotXl330Bus;

/// The FT reference controller allows up to 300 ms for one protocol transaction.
const PHYSICAL_SERIAL_TIMEOUT: Duration = Duration::from_millis(300);

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

/// Move a probed bus out of the temporary port entry before taking the lock again.
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
