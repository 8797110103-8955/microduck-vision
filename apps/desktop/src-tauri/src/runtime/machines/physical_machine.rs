//! Physical-machine application service.
//!
//! This service owns persisted machine lifecycle and binding orchestration. Serial ownership and
//! device-specific behavior remain in backend modules; Tauri command files only adapt arguments.

use crate::types::{ExecutionKindKey, ServoModelKey};
use microduck_body_runtime::Selection;

use crate::platform::helpers::{default_physical_model, model, model_key};
use crate::runtime::adapters::physical_identity as physical_backend;
use crate::runtime::adapters::servo_bus::{
    ConnectedBus, move_pending_bus, open_for_model, try_open_feetech, try_open_xl330,
};
use crate::runtime::session::state::DesktopState;
use crate::types::{
    DetectedPhysicalMachine, DiagnosticCode, MatchResult, PhysicalMachine,
    PhysicalMachineDetectInput, PhysicalMachineInput, PhysicalMachineProbeInput,
    PhysicalMachineReconnectInput, PhysicalMachineSaveInput, ReconnectHint, ServoIdentity,
};
pub(crate) fn physical_machine_save(
    state: &DesktopState,
    input: PhysicalMachineSaveInput,
) -> Result<PhysicalMachine, String> {
    let mut machines = state.physical_machines.lock().map_err(|e| e.to_string())?;
    let (md5, result) =
        physical_backend::match_saved(&machines, &input.census).map_err(|e| e.to_string())?;
    let MatchResult::New = result else {
        return Err(match result {
            MatchResult::Unique {
                physical_machine_id,
            } => format!("physical machine already saved: {physical_machine_id}"),
            MatchResult::IdentityConflict => "physical machine identity conflict".into(),
            MatchResult::IncompleteCensus => "incomplete servo census".into(),
            MatchResult::New => unreachable!(),
        });
    };
    let hint = input.reconnect_hint;
    let census = physical_backend::canonicalize_census(&input.census).map_err(|e| e.to_string())?;
    let servo_model = physical_backend::census_model(&census).ok();
    let machine = PhysicalMachine {
        id: crate::platform::helpers::new_machine_id("physical"),
        name: input
            .name
            .and_then(|name| (!name.trim().is_empty()).then(|| name.trim().to_owned()))
            .unwrap_or_else(|| md5.clone()),
        usb_port: hint
            .as_ref()
            .map(|hint| hint.port_name.clone())
            .unwrap_or_default(),
        machine_code: md5,
        execution_kind: ExecutionKindKey::Physical,
        model_kind: servo_model.map(default_physical_model),
        servo_model,
        census,
        reconnect_hint: hint.clone(),
        virtual_machine_id: None,
        online_servo_ids: input.census.iter().map(|servo| servo.servo_id).collect(),
        connected: true,
        last_seen: Some("刚刚".into()),
        torque_enabled: false,
        joint_directions: Vec::new(),
        joint_bindings: Vec::new(),
    };
    let pending_bus_key = hint
        .as_ref()
        .map(|hint| format!("pending:{}", hint.port_name));
    machines.push(machine.clone());
    drop(machines);
    move_pending_bus(
        &state.connected_buses,
        pending_bus_key.as_deref(),
        &machine.id,
    )?;
    state.persist()?;
    Ok(machine)
}

pub(crate) fn physical_servo_model(machine: &PhysicalMachine) -> Result<servo::ServoModel, String> {
    Ok(physical_selection(machine)?.servo_model)
}

pub(crate) fn physical_selection(machine: &PhysicalMachine) -> Result<Selection, String> {
    if machine.execution_kind != ExecutionKindKey::Physical {
        return Err("physical machines require physical execution".into());
    }
    let detected = physical_backend::census_model(&machine.census)?;
    let saved = machine.servo_model.ok_or("真机未保存舵机型号")?;
    if detected != saved {
        return Err("真机保存的型号与舵机清单不一致".into());
    }
    crate::runtime::machines::machine_resolution::model_selection(
        machine.execution_kind,
        machine.model_kind.ok_or("真机未保存执行模型")?,
        saved,
    )
}

pub(crate) fn physical_machine_disconnect(
    state: &DesktopState,
    id: String,
) -> Result<PhysicalMachine, String> {
    crate::runtime::session::loop_service::detach_physical_connection(state, &id)?;
    state
        .connected_buses
        .lock()
        .map_err(|e| e.to_string())?
        .remove(&id);
    let mut machines = state.physical_machines.lock().map_err(|e| e.to_string())?;
    let machine = machines
        .iter_mut()
        .find(|machine| machine.id == id)
        .ok_or_else(|| "unknown physical machine".to_owned())?;
    machine.connected = false;
    machine.online_servo_ids.clear();
    let result = machine.clone();
    drop(machines);
    state.persist()?;
    Ok(result)
}

pub(crate) fn physical_machine_reconnect(
    state: &DesktopState,
    input: PhysicalMachineReconnectInput,
) -> Result<PhysicalMachine, String> {
    let canonical =
        physical_backend::canonicalize_census(&input.census).map_err(|error| error.to_string())?;
    let mut machines = state.physical_machines.lock().map_err(|e| e.to_string())?;
    let machine = machines
        .iter_mut()
        .find(|machine| machine.id == input.id)
        .ok_or_else(|| "unknown physical machine".to_owned())?;
    if !physical_backend::same_census(&machine.census, &canonical) {
        machine.connected = false;
        return Err("reconnect census does not match saved physical machine".into());
    }
    machine.machine_code =
        physical_backend::census_md5(&canonical).map_err(|error| error.to_string())?;
    machine.census = canonical;
    machine.reconnect_hint = input.reconnect_hint;
    if let Some(hint) = &machine.reconnect_hint {
        machine.usb_port = hint.port_name.clone();
    }
    machine.online_servo_ids = machine.census.iter().map(|servo| servo.servo_id).collect();
    machine.connected = true;
    machine.last_seen = Some("刚刚".into());
    let result = machine.clone();
    drop(machines);
    state.persist()?;
    Ok(result)
}

pub(crate) fn physical_machine_create(
    state: &DesktopState,
    input: PhysicalMachineInput,
) -> Result<PhysicalMachine, String> {
    let mut machines = state.physical_machines.lock().map_err(|e| e.to_string())?;
    let name = if input.name.trim().is_empty() {
        "未命名实机".to_owned()
    } else {
        input.name.trim().to_owned()
    };
    let reconnect_hint = (!input.usb_port.trim().is_empty()).then(|| ReconnectHint {
        port_name: input.usb_port.clone(),
        vid: None,
        pid: None,
        adapter_serial: None,
        product: None,
    });
    let machine = PhysicalMachine {
        id: crate::platform::helpers::new_machine_id("physical"),
        name,
        usb_port: input.usb_port,
        machine_code: input.machine_code,
        execution_kind: ExecutionKindKey::Physical,
        model_kind: None,
        servo_model: None,
        census: Vec::new(),
        reconnect_hint,
        virtual_machine_id: None,
        online_servo_ids: Vec::new(),
        connected: false,
        last_seen: None,
        torque_enabled: false,
        joint_directions: Vec::new(),
        joint_bindings: Vec::new(),
    };
    machines.push(machine.clone());
    drop(machines);
    state.persist()?;
    Ok(machine)
}

pub(crate) fn physical_machine_connect(
    state: &DesktopState,
    id: String,
) -> Result<PhysicalMachine, String> {
    let _transition = state
        .execution_transition
        .lock()
        .map_err(|error| error.to_string())?;
    let machine_snapshot = state
        .physical_machines
        .lock()
        .map_err(|e| e.to_string())?
        .iter()
        .find(|machine| machine.id == id)
        .cloned()
        .ok_or_else(|| "unknown physical machine".to_string())?;
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
    let model_key = machine_snapshot
        .census
        .first()
        .map(|servo| servo.model.trim().to_ascii_lowercase())
        .map(|model| model_key(&model))
        .transpose()?
        .or_else(|| {
            machine_snapshot
                .virtual_machine_id
                .as_ref()
                .and_then(|virtual_id| {
                    state
                        .virtual_machines
                        .lock()
                        .ok()?
                        .iter()
                        .find(|machine| &machine.id == virtual_id)
                        .map(|machine| machine.servo_model)
                })
        })
        .unwrap_or(ServoModelKey::Hd1910);
    let bus = open_for_model(model(&model_key), &machine_snapshot.usb_port, 1_000_000)?;
    state
        .connected_buses
        .lock()
        .map_err(|e| e.to_string())?
        .insert(id.clone(), bus);
    let mut machines = state.physical_machines.lock().map_err(|e| e.to_string())?;
    let machine = machines
        .iter_mut()
        .find(|machine| machine.id == id)
        .ok_or_else(|| "unknown physical machine".to_string())?;
    machine.connected = true;
    Ok(machine.clone())
}

pub(crate) fn physical_machine_scan(
    state: &DesktopState,
    id: String,
) -> Result<PhysicalMachine, String> {
    let online = crate::runtime::machines::physical_io::with_bus(
        state,
        &id,
        loop_body::BackendAccess::ReadOnly,
        |bus| {
            bus.ok_or_else(|| "physical machine is not connected".to_owned())?
                .scan()
        },
    );
    let online = match online {
        Ok(online) => online,
        Err(error) => {
            crate::runtime::session::loop_service::detach_physical_connection(state, &id)?;
            state
                .connected_buses
                .lock()
                .map_err(|e| e.to_string())?
                .remove(&id);
            if let Ok(mut machines) = state.physical_machines.lock()
                && let Some(machine) = machines.iter_mut().find(|machine| machine.id == id)
            {
                machine.connected = false;
                machine.online_servo_ids.clear();
            }
            state.persist()?;
            let reconnect_enabled = *state
                .auto_reconnect_enabled
                .lock()
                .map_err(|e| e.to_string())?;
            if reconnect_enabled {
                return crate::runtime::machines::devices::auto_reconnect(state, id, 1_000_000);
            }
            return Err(error);
        }
    };
    let mut machines = state.physical_machines.lock().map_err(|e| e.to_string())?;
    let machine = machines
        .iter_mut()
        .find(|machine| machine.id == id)
        .ok_or_else(|| "unknown physical machine".to_string())?;
    machine.online_servo_ids = online;
    Ok(machine.clone())
}

pub(crate) fn physical_machine_mount(
    state: &DesktopState,
    id: String,
    virtual_machine_id: String,
) -> Result<PhysicalMachine, String> {
    let _transition = state
        .execution_transition
        .lock()
        .map_err(|e| e.to_string())?;
    crate::runtime::machines::encoder_calibration::ensure_ready(state, &id)?;
    let mut virtual_machines = state.virtual_machines.lock().map_err(|e| e.to_string())?;
    if !virtual_machines
        .iter()
        .any(|machine| machine.id == virtual_machine_id)
    {
        return Err("unknown virtual machine".into());
    }
    for machine in &mut *virtual_machines {
        machine.referenced_by.retain(|reference| reference != &id);
    }
    if let Some(machine) = virtual_machines
        .iter_mut()
        .find(|machine| machine.id == virtual_machine_id)
    {
        machine.referenced_by.push(id.clone());
    }
    drop(virtual_machines);
    let mut machines = state.physical_machines.lock().map_err(|e| e.to_string())?;
    let machine = machines
        .iter_mut()
        .find(|machine| machine.id == id)
        .ok_or_else(|| "unknown physical machine".to_string())?;
    machine.virtual_machine_id = Some(virtual_machine_id);
    let result = machine.clone();
    drop(machines);
    state.persist()?;
    Ok(result)
}

pub(crate) fn physical_machine_delete(state: &DesktopState, id: String) -> Result<(), String> {
    crate::runtime::machines::encoder_calibration::ensure_ready(state, &id)?;
    crate::runtime::session::loop_service::detach_physical_connection(state, &id)?;
    let _transition = state
        .execution_transition
        .lock()
        .map_err(|e| e.to_string())?;
    crate::runtime::machines::encoder_calibration::ensure_ready(state, &id)?;
    state
        .connected_buses
        .lock()
        .map_err(|e| e.to_string())?
        .remove(&id);
    state
        .physical_machines
        .lock()
        .map_err(|e| e.to_string())?
        .retain(|machine| machine.id != id);
    let mut virtual_machines = state.virtual_machines.lock().map_err(|e| e.to_string())?;
    for machine in &mut *virtual_machines {
        machine.referenced_by.retain(|reference| reference != &id);
    }
    drop(virtual_machines);
    state.persist()?;
    Ok(())
}

pub(crate) fn physical_machine_rename(
    state: &DesktopState,
    id: String,
    name: String,
) -> Result<PhysicalMachine, String> {
    let name = name.trim();
    if name.is_empty() {
        return Err("physical machine name cannot be empty".into());
    }
    let mut machines = state.physical_machines.lock().map_err(|e| e.to_string())?;
    let machine = machines
        .iter_mut()
        .find(|machine| machine.id == id)
        .ok_or_else(|| "unknown physical machine".to_string())?;
    machine.name = name.to_owned();
    let result = machine.clone();
    drop(machines);
    state.persist()?;
    Ok(result)
}
pub(crate) fn commit_probed_bus(
    state: &DesktopState,
    port_name: String,
    bus: ConnectedBus,
    census: Vec<ServoIdentity>,
    reconnect_hint: Option<ReconnectHint>,
) -> Result<DetectedPhysicalMachine, String> {
    let detected = apply_physical_machine_detect(
        state,
        PhysicalMachineDetectInput {
            census,
            reconnect_hint,
        },
    )?;
    let key = match &detected.match_result {
        MatchResult::Unique {
            physical_machine_id,
        } => physical_machine_id.clone(),
        _ => format!("pending:{port_name}"),
    };
    state
        .connected_buses
        .lock()
        .map_err(|error| error.to_string())?
        .insert(key, bus);
    Ok(detected)
}

pub(crate) fn physical_machine_probe(
    state: &DesktopState,
    input: PhysicalMachineProbeInput,
) -> Result<DetectedPhysicalMachine, String> {
    let reconnect_hint = input.reconnect_hint.clone();
    let mut diagnostics = Vec::new();
    let open_hd = try_open_feetech(&input.port_name, input.baud);
    if let Ok(bus) = open_hd {
        let mut bus = bus;
        match bus.scan_census() {
            Ok(census) if !census.is_empty() => {
                return commit_probed_bus(
                    state,
                    input.port_name.clone(),
                    bus,
                    census,
                    reconnect_hint,
                );
            }
            Ok(_) => diagnostics.push(DiagnosticCode::IncompleteCensus),
            Err(_) => diagnostics.push(DiagnosticCode::Timeout),
        }
    } else {
        diagnostics.push(DiagnosticCode::OpenFailed);
    }
    let bus = match try_open_xl330(&input.port_name, input.baud) {
        Ok(bus) => bus,
        Err(_) => {
            diagnostics.push(DiagnosticCode::ProtocolMismatch);
            return Ok(physical_backend::failed_probe(reconnect_hint, diagnostics));
        }
    };
    let mut bus = bus;
    let census = match bus.scan_census() {
        Ok(census) => census,
        Err(_) => {
            diagnostics.push(DiagnosticCode::Timeout);
            return Ok(physical_backend::failed_probe(reconnect_hint, diagnostics));
        }
    };
    if census.is_empty() {
        return Ok(physical_backend::failed_probe(reconnect_hint, diagnostics));
    }
    commit_probed_bus(state, input.port_name, bus, census, reconnect_hint)
}

pub(crate) fn apply_physical_machine_detect(
    state: &DesktopState,
    input: PhysicalMachineDetectInput,
) -> Result<DetectedPhysicalMachine, String> {
    let mut machines = state.physical_machines.lock().map_err(|e| e.to_string())?;
    let detected = physical_backend::detect(&machines, input.census, input.reconnect_hint);
    if let MatchResult::Unique {
        physical_machine_id,
    } = &detected.match_result
        && let Some(machine) = machines
            .iter_mut()
            .find(|machine| &machine.id == physical_machine_id)
    {
        machine.connected = true;
        if let Some(md5) = &detected.md5 {
            machine.machine_code = md5.clone();
        }
        machine.last_seen = Some("刚刚".into());
        machine.reconnect_hint = detected.reconnect_hint.clone();
        if let Some(hint) = &machine.reconnect_hint {
            machine.usb_port = hint.port_name.clone();
        }
        machine.online_servo_ids = detected.census.iter().map(|servo| servo.servo_id).collect();
    }
    let matched_machine = match &detected.match_result {
        MatchResult::Unique {
            physical_machine_id,
        } => machines
            .iter()
            .find(|machine| &machine.id == physical_machine_id)
            .cloned(),
        _ => None,
    };
    drop(machines);
    let auto_mount = *state
        .auto_mount_enabled
        .lock()
        .map_err(|error| error.to_string())?;
    let startup_virtual_machine_id = state
        .virtual_machines
        .lock()
        .map_err(|error| error.to_string())?
        .iter()
        .find(|machine| machine.startup_selected)
        .map(|machine| machine.id.clone());
    if let (Some(machine), Some(startup_id)) = (matched_machine, startup_virtual_machine_id)
        && physical_backend::can_auto_mount(&detected, &machine, Some(&startup_id), auto_mount)
    {
        let mut virtual_machines = state
            .virtual_machines
            .lock()
            .map_err(|error| error.to_string())?;
        let mut physical_machines = state
            .physical_machines
            .lock()
            .map_err(|error| error.to_string())?;
        for virtual_machine in &mut *virtual_machines {
            virtual_machine
                .referenced_by
                .retain(|reference| reference != &machine.id);
        }
        if let Some(virtual_machine) = virtual_machines
            .iter_mut()
            .find(|virtual_machine| virtual_machine.id == startup_id)
        {
            virtual_machine.referenced_by.push(machine.id.clone());
        }
        if let Some(saved) = physical_machines
            .iter_mut()
            .find(|saved| saved.id == machine.id)
        {
            saved.virtual_machine_id = Some(startup_id);
        }
    }
    state.persist()?;
    Ok(detected)
}

pub(crate) fn physical_machine_detect(
    state: &DesktopState,
    input: PhysicalMachineDetectInput,
) -> Result<DetectedPhysicalMachine, String> {
    apply_physical_machine_detect(state, input)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn physical_machine_service_creates_and_renames_without_hardware() {
        let state = DesktopState::new(None);
        let machine = physical_machine_create(
            &state,
            PhysicalMachineInput {
                name: "测试实机".into(),
                usb_port: "COM-test".into(),
                machine_code: "pending".into(),
            },
        )
        .expect("physical machine should be created");
        assert!(!machine.connected);
        assert_eq!(machine.usb_port, "COM-test");

        let renamed = physical_machine_rename(&state, machine.id, "新名称".into())
            .expect("physical machine should be renamed");
        assert_eq!(renamed.name, "新名称");
    }
}
