//! Keeps the saved physical-machine state in sync with serial USB devices.
//!
//! `serialport::available_ports` is an enumeration API, not a notification
//! API.  A small polling worker therefore watches the port set and runs the
//! existing identity-aware reconnect flow whenever that set changes.  The
//! interval is short enough for UI feedback while keeping all serial I/O off
//! the Tauri command thread.

use std::collections::BTreeSet;
use std::sync::{
    Arc,
    atomic::{AtomicBool, Ordering},
};
use std::thread::{self, JoinHandle};
use std::time::Duration;

use tauri::{AppHandle, Wry};

use crate::application::devices;

const POLL_INTERVAL: Duration = Duration::from_millis(500);

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord)]
struct PortKey {
    port_name: String,
    vid: Option<u16>,
    pid: Option<u16>,
    adapter_serial: Option<String>,
}

fn port_snapshot() -> Option<BTreeSet<PortKey>> {
    let ports = serialport::available_ports().ok()?;
    Some(
        ports
            .into_iter()
            .map(|port| match port.port_type {
                serialport::SerialPortType::UsbPort(info) => PortKey {
                    port_name: port.port_name,
                    vid: Some(info.vid),
                    pid: Some(info.pid),
                    adapter_serial: info.serial_number,
                },
                _ => PortKey {
                    port_name: port.port_name,
                    vid: None,
                    pid: None,
                    adapter_serial: None,
                },
            })
            .collect(),
    )
}

pub(crate) struct DeviceWatcher {
    stop: Arc<AtomicBool>,
    worker: Option<JoinHandle<()>>,
}

impl DeviceWatcher {
    pub(crate) fn start(app: AppHandle<Wry>) -> Self {
        let stop = Arc::new(AtomicBool::new(false));
        let worker_stop = Arc::clone(&stop);
        let worker = thread::Builder::new()
            .name("microduck-usb-watch".into())
            .spawn(move || {
                let mut previous = port_snapshot();
                // Reconcile persisted machines once at startup. This covers a
                // USB device that was already connected before the app opened.
                let _ = devices::refresh_physical_machines(&app);
                while !worker_stop.load(Ordering::Relaxed) {
                    thread::sleep(POLL_INTERVAL);
                    let Some(current) = port_snapshot() else {
                        continue;
                    };
                    if previous.as_ref() != Some(&current) {
                        previous = Some(current);
                        let _ = devices::refresh_physical_machines(&app);
                    }
                }
            })
            .expect("failed to start USB watcher");
        Self {
            stop,
            worker: Some(worker),
        }
    }
}

impl Drop for DeviceWatcher {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(worker) = self.worker.take() {
            let _ = worker.join();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn port_key_distinguishes_usb_adapter_changes() {
        let mut before = BTreeSet::new();
        before.insert(PortKey {
            port_name: "COM3".into(),
            vid: Some(0x0403),
            pid: Some(0x6015),
            adapter_serial: Some("A".into()),
        });
        let mut after = before.clone();
        after.remove(&PortKey {
            port_name: "COM3".into(),
            vid: Some(0x0403),
            pid: Some(0x6015),
            adapter_serial: Some("A".into()),
        });
        after.insert(PortKey {
            port_name: "COM4".into(),
            vid: Some(0x0403),
            pid: Some(0x6015),
            adapter_serial: Some("B".into()),
        });
        assert_ne!(before, after);
    }
}
