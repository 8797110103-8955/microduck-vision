//! Physical-machine application services.
//!
//! This module owns identity-aware port reconciliation and reconnect orchestration.  Tauri
//! commands and lifecycle workers may call these services, but the services do not depend on
//! command handlers or UI DTOs.

use tauri::{AppHandle, Emitter, Manager, Wry};

use crate::backend::control::open_for_model;
use crate::backend::physical;
use crate::helpers::{model, model_key};
use crate::state::DesktopState;
use crate::types::{PhysicalMachine, ReconnectHint, SerialPortCandidate, VirtualMachine};

pub(crate) fn virtual_machines(state: &DesktopState) -> Result<Vec<VirtualMachine>, String> {
    state
        .virtual_machines
        .lock()
        .map(|machines| machines.clone())
        .map_err(|error| error.to_string())
}

pub(crate) fn physical_machines(state: &DesktopState) -> Result<Vec<PhysicalMachine>, String> {
    state
        .physical_machines
        .lock()
        .map(|machines| machines.clone())
        .map_err(|error| error.to_string())
}

pub(crate) fn auto_mount_enabled(state: &DesktopState) -> Result<bool, String> {
    state
        .auto_mount_enabled
        .lock()
        .map(|enabled| *enabled)
        .map_err(|error| error.to_string())
}

pub(crate) fn set_auto_mount_enabled(state: &DesktopState, enabled: bool) -> Result<bool, String> {
    *state
        .auto_mount_enabled
        .lock()
        .map_err(|error| error.to_string())? = enabled;
    state.persist()?;
    Ok(enabled)
}

pub(crate) fn auto_reconnect_enabled(state: &DesktopState) -> Result<bool, String> {
    state
        .auto_reconnect_enabled
        .lock()
        .map(|enabled| *enabled)
        .map_err(|error| error.to_string())
}

pub(crate) fn set_auto_reconnect_enabled(
    state: &DesktopState,
    enabled: bool,
) -> Result<bool, String> {
    *state
        .auto_reconnect_enabled
        .lock()
        .map_err(|error| error.to_string())? = enabled;
    state.persist()?;
    Ok(enabled)
}

pub(crate) fn open_logs(state: &DesktopState) -> Result<(), String> {
    let file = crate::logging::current_log_file(state.config_dir.as_deref());
    if let Some(parent) = file.parent() {
        std::fs::create_dir_all(parent).map_err(|error| error.to_string())?;
    }
    if !file.exists() {
        std::fs::File::create(&file).map_err(|error| error.to_string())?;
    }
    #[cfg(target_os = "windows")]
    let mut command = {
        let mut command = std::process::Command::new("cmd");
        command.args(["/C", "start", ""]).arg(&file);
        command
    };
    #[cfg(target_os = "macos")]
    let mut command = std::process::Command::new("open");
    #[cfg(all(unix, not(target_os = "macos")))]
    let mut command = std::process::Command::new("xdg-open");
    #[cfg(not(target_os = "windows"))]
    command.arg(&file);
    command
        .spawn()
        .map(|_| ())
        .map_err(|error| format!("打开日志文件失败：{error}"))
}

pub(crate) fn ports() -> Result<Vec<SerialPortCandidate>, String> {
    serialport::available_ports()
        .map_err(|error| error.to_string())
        .map(|ports| {
            ports
                .into_iter()
                .map(|port| match port.port_type {
                    serialport::SerialPortType::UsbPort(info) => SerialPortCandidate {
                        port_name: port.port_name,
                        vid: Some(info.vid),
                        pid: Some(info.pid),
                        adapter_serial: info.serial_number,
                        product: info.product,
                    },
                    _ => SerialPortCandidate {
                        port_name: port.port_name,
                        vid: None,
                        pid: None,
                        adapter_serial: None,
                        product: None,
                    },
                })
                .collect()
        })
}

pub(crate) fn refresh_physical_machines(app: &AppHandle<Wry>) -> Result<(), String> {
    let state = app.state::<DesktopState>();
    let ports = ports()?;
    let candidates = ports
        .iter()
        .map(|port| ReconnectHint {
            port_name: port.port_name.clone(),
            vid: port.vid,
            pid: port.pid,
            adapter_serial: port.adapter_serial.clone(),
            product: port.product.clone(),
        })
        .collect::<Vec<_>>();
    let auto_reconnect = *state
        .auto_reconnect_enabled
        .lock()
        .map_err(|error| error.to_string())?;
    let machines = state
        .physical_machines
        .lock()
        .map_err(|error| error.to_string())?
        .clone();

    for machine in machines {
        let hint = machine.reconnect_hint.clone().or_else(|| {
            (!machine.usb_port.trim().is_empty()).then(|| ReconnectHint {
                port_name: machine.usb_port.clone(),
                vid: None,
                pid: None,
                adapter_serial: None,
                product: None,
            })
        });
        let Some(hint) = hint else { continue };
        let matches = physical::reconnect_candidates(&hint, &candidates);
        if matches.len() != 1 {
            mark_offline(state.inner(), &machine.id)?;
            continue;
        }
        if auto_reconnect
            && auto_reconnect_one(state.inner(), machine.id.clone(), 1_000_000).is_err()
        {
            // A USB adapter may enumerate while the servo bus is absent or belongs to another
            // machine. Do not leave stale online state in that case.
            mark_offline(state.inner(), &machine.id)?;
        }
    }
    state.persist()?;
    let _ = app.emit("physical-machines-changed", ());
    Ok(())
}

pub(crate) fn auto_reconnect(
    state: &DesktopState,
    id: String,
    baud: u32,
) -> Result<PhysicalMachine, String> {
    auto_reconnect_one(state, id, baud)
}

fn auto_reconnect_one(
    state: &DesktopState,
    id: String,
    baud: u32,
) -> Result<PhysicalMachine, String> {
    let _transition = state
        .execution_transition
        .lock()
        .map_err(|error| error.to_string())?;
    let machine_snapshot = state
        .physical_machines
        .lock()
        .map_err(|error| error.to_string())?
        .iter()
        .find(|machine| machine.id == id)
        .cloned()
        .ok_or_else(|| "unknown physical machine".to_owned())?;
    // The active sensor/control session owns this port. USB refresh must not open a second
    // handle or replace its connection underneath the registered backend.
    if state
        .loop_physical_machine_id
        .lock()
        .map_err(|error| error.to_string())?
        .as_deref()
        == Some(id.as_str())
        || state
            .connected_buses
            .lock()
            .map_err(|error| error.to_string())?
            .contains_key(&id)
    {
        return Ok(machine_snapshot);
    }
    let saved_hint = machine_snapshot
        .reconnect_hint
        .clone()
        .ok_or_else(|| "physical machine has no reconnect hint".to_owned())?;
    let candidates = ports()?
        .into_iter()
        .map(|port| ReconnectHint {
            port_name: port.port_name,
            vid: port.vid,
            pid: port.pid,
            adapter_serial: port.adapter_serial,
            product: port.product,
        })
        .collect::<Vec<_>>();
    let matches = physical::reconnect_candidates(&saved_hint, &candidates);
    if matches.len() != 1 {
        return Err(if matches.is_empty() {
            "reconnect candidate not found or ambiguous".into()
        } else {
            "reconnect candidate is ambiguous".into()
        });
    }
    let reconnect_hint = matches[0].clone();
    let model_key = machine_snapshot
        .census
        .first()
        .map(|servo| servo.model.trim().to_ascii_lowercase())
        .map(|model| model_key(&model))
        .transpose()?
        .unwrap_or(duck_body::ServoModelKey::Hd1910);
    let mut bus = open_for_model(model(&model_key), &reconnect_hint.port_name, baud)
        .map_err(|_| "reconnect open failed".to_owned())?;
    let census = bus
        .scan_census()
        .map_err(|_| "reconnect census read failed".to_owned())?;
    let canonical = physical::canonicalize_census(&census)
        .map_err(|_| "reconnect census is incomplete".to_owned())?;
    if !physical::same_census(&machine_snapshot.census, &canonical) {
        return Err("reconnect census does not match saved physical machine".into());
    }
    let machine_code =
        physical::census_md5(&canonical).map_err(|_| "reconnect census is invalid".to_owned())?;
    state
        .connected_buses
        .lock()
        .map_err(|error| error.to_string())?
        .insert(id.clone(), bus);
    let mut machines = state
        .physical_machines
        .lock()
        .map_err(|error| error.to_string())?;
    let machine = machines
        .iter_mut()
        .find(|machine| machine.id == id)
        .ok_or_else(|| "unknown physical machine".to_owned())?;
    machine.machine_code = machine_code;
    machine.census = canonical;
    machine.reconnect_hint = Some(reconnect_hint.clone());
    machine.usb_port = reconnect_hint.port_name;
    machine.online_servo_ids = machine.census.iter().map(|servo| servo.servo_id).collect();
    machine.connected = true;
    machine.last_seen = Some("刚刚".into());
    let result = machine.clone();
    drop(machines);
    state.persist()?;
    Ok(result)
}

fn mark_offline(state: &DesktopState, id: &str) -> Result<(), String> {
    crate::control::detach_physical_connection(state, id)?;
    state
        .connected_buses
        .lock()
        .map_err(|error| error.to_string())?
        .remove(id);
    let mut machines = state
        .physical_machines
        .lock()
        .map_err(|error| error.to_string())?;
    if let Some(machine) = machines.iter_mut().find(|machine| machine.id == id) {
        machine.connected = false;
        machine.online_servo_ids.clear();
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::mark_offline;
    use crate::state::DesktopState;
    use crate::types::PhysicalMachine;

    #[test]
    fn marking_a_machine_offline_clears_stale_online_state() {
        let state = DesktopState::default();
        state
            .physical_machines
            .lock()
            .expect("physical machines lock")
            .push(PhysicalMachine {
                id: "physical-device-test".into(),
                name: "设备测试".into(),
                usb_port: "COM-test".into(),
                machine_code: "test".into(),
                census: vec![],
                reconnect_hint: None,
                virtual_machine_id: None,
                online_servo_ids: vec![1, 2],
                connected: true,
                last_seen: Some("刚刚".into()),
                torque_enabled: false,
                joint_directions: vec![],
            });

        mark_offline(&state, "physical-device-test").expect("offline transition");
        let machine = state
            .physical_machines
            .lock()
            .expect("physical machines lock")
            .iter()
            .find(|machine| machine.id == "physical-device-test")
            .cloned()
            .expect("offline machine");
        assert!(!machine.connected);
        assert!(machine.online_servo_ids.is_empty());
    }
}
