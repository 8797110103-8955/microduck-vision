//! Tauri command layer for the desktop client.
//!
//! These commands coordinate user-facing state and persistence.  Hardware and
//! loop-body adapters stay in [`crate::control`], while storage lives in
//! [`crate::state`].

use duck_body::{BodyTemplate, ServoCatalogEntry, ServoModelKey};
use loop_body::{
    BodyRuntimeView, Calibration, JointCalibration, LoopAction as BodyLoopAction, LoopBinding,
    LoopConfig, PolicyLoop, ServoBinding,
};
use servo::ServoLoopDriver;
use tauri::State;

use crate::control::{ConnectedBus, MemoryDriver, move_pending_bus, open_for_model, try_open_feetech, try_open_xl330};
use crate::helpers::{model, model_key};
use crate::physical;
use crate::policy::{self, PolicyEntry};
use crate::simulation::PreviewPolicyLoop;
use crate::state::DesktopState;
use crate::types::{
    ActionKind, ActionRecord, ActionSource, DetectedPhysicalMachine, DiagnosticCode,
    LoopAction as FrontendLoopAction, LoopStatus as FrontendLoopStatus, MatchResult,
    PhysicalMachine, ReconnectHint, SceneBody as FrontendSceneBody,
    SceneFrame as FrontendSceneFrame, SceneGeom as FrontendSceneGeom,
    SceneInfo as FrontendSceneInfo, SceneJoint as FrontendSceneJoint,
    SceneModel as FrontendSceneModel, SceneSite as FrontendSceneSite,
    SerialPortCandidate, ServoIdentity, VirtualMachine,
};
use crate::{
    ActionInput, LoopOpenInput, PhysicalMachineDetectInput, PhysicalMachineInput,
    PhysicalMachineProbeInput, PhysicalMachineReconnectInput, PhysicalMachineSaveInput,
    VirtualMachineInput,
};
#[tauri::command]
#[specta::specta]
pub(crate) fn servo_catalog() -> Vec<ServoCatalogEntry> {
    duck_body::servo_catalog()
}

pub(crate) fn scene_model() -> Result<FrontendSceneInfo, String> {
    let scene = duck_body::OfficialScene::load().map_err(|error| error.to_string())?;
    let model = scene.model();
    Ok(FrontendSceneInfo {
        model: FrontendSceneModel {
            bodies: model
                .bodies
                .iter()
                .map(|body| FrontendSceneBody {
                    name: body.name.clone(),
                    geoms: body
                        .geoms
                        .iter()
                        .map(|geom| FrontendSceneGeom {
                            mesh: geom.mesh.clone(),
                            pos: geom.pos,
                            quat: geom.quat,
                            rgba: geom.rgba,
                        })
                        .collect(),
                })
                .collect(),
            joints: model
                .joints
                .iter()
                .map(|joint| FrontendSceneJoint {
                    name: joint.name.clone(),
                    body: joint.body,
                })
                .collect(),
            trunk_height: model.trunk_height,
            head_camera: model.head_camera.as_ref().map(|site| FrontendSceneSite {
                body: site.body,
                pos: site.pos,
            }),
        },
        mesh_dir: scene.mesh_dir().to_string_lossy().into_owned(),
    })
}

#[tauri::command]
#[specta::specta]
pub(crate) fn official_scene_model() -> Result<FrontendSceneInfo, String> {
    scene_model()
}

#[tauri::command]
#[specta::specta]
pub(crate) fn official_scene_frame(positions: Vec<f64>) -> Result<FrontendSceneFrame, String> {
    let scene = duck_body::OfficialScene::load().map_err(|error| error.to_string())?;
    let frame = scene.frame(&positions).map_err(|error| error.to_string())?;
    Ok(FrontendSceneFrame {
        bodies: frame.bodies,
        markers: frame.markers,
    })
}

#[tauri::command]
#[specta::specta]
pub(crate) fn virtual_machines(state: State<'_, DesktopState>) -> Result<Vec<VirtualMachine>, String> {
    state
        .virtual_machines
        .lock()
        .map(|machines| machines.clone())
        .map_err(|e| e.to_string())
}

#[tauri::command]
#[specta::specta]
pub(crate) fn physical_machines(state: State<'_, DesktopState>) -> Result<Vec<PhysicalMachine>, String> {
    state
        .physical_machines
        .lock()
        .map(|machines| machines.clone())
        .map_err(|e| e.to_string())
}

#[tauri::command]
#[specta::specta]
pub(crate) fn auto_mount_enabled(state: State<'_, DesktopState>) -> Result<bool, String> {
    state
        .auto_mount_enabled
        .lock()
        .map(|enabled| *enabled)
        .map_err(|error| error.to_string())
}

#[tauri::command]
#[specta::specta]
pub(crate) fn set_auto_mount_enabled(state: State<'_, DesktopState>, enabled: bool) -> Result<bool, String> {
    *state
        .auto_mount_enabled
        .lock()
        .map_err(|error| error.to_string())? = enabled;
    state.persist()?;
    Ok(enabled)
}

#[tauri::command]
#[specta::specta]
pub(crate) fn auto_reconnect_enabled(state: State<'_, DesktopState>) -> Result<bool, String> {
    state
        .auto_reconnect_enabled
        .lock()
        .map(|enabled| *enabled)
        .map_err(|error| error.to_string())
}

#[tauri::command]
#[specta::specta]
pub(crate) fn set_auto_reconnect_enabled(
    state: State<'_, DesktopState>,
    enabled: bool,
) -> Result<bool, String> {
    *state
        .auto_reconnect_enabled
        .lock()
        .map_err(|error| error.to_string())? = enabled;
    state.persist()?;
    Ok(enabled)
}

#[tauri::command]
#[specta::specta]
pub(crate) fn physical_ports() -> Result<Vec<SerialPortCandidate>, String> {
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

#[tauri::command]
#[specta::specta]
pub(crate) fn physical_baud_choices() -> Vec<u32> {
    vec![1_000_000, 576_000, 115_200, 57_600]
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

#[tauri::command]
#[specta::specta]
pub(crate) fn physical_machine_probe(
    state: State<'_, DesktopState>,
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
                    &state,
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
            return Ok(physical::failed_probe(reconnect_hint, diagnostics));
        }
    };
    let mut bus = bus;
    let census = match bus.scan_census() {
        Ok(census) => census,
        Err(_) => {
            diagnostics.push(DiagnosticCode::Timeout);
            return Ok(physical::failed_probe(reconnect_hint, diagnostics));
        }
    };
    if census.is_empty() {
        return Ok(physical::failed_probe(reconnect_hint, diagnostics));
    }
    commit_probed_bus(&state, input.port_name, bus, census, reconnect_hint)
}

pub(crate) fn apply_physical_machine_detect(
    state: &DesktopState,
    input: PhysicalMachineDetectInput,
) -> Result<DetectedPhysicalMachine, String> {
    let mut machines = state.physical_machines.lock().map_err(|e| e.to_string())?;
    let detected = physical::detect(&machines, input.census, input.reconnect_hint);
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
        && physical::can_auto_mount(&detected, &machine, Some(&startup_id), auto_mount)
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

#[tauri::command]
#[specta::specta]
pub(crate) fn physical_machine_detect(
    state: State<'_, DesktopState>,
    input: PhysicalMachineDetectInput,
) -> Result<DetectedPhysicalMachine, String> {
    apply_physical_machine_detect(&state, input)
}

#[tauri::command]
#[specta::specta]
pub(crate) fn physical_machine_save(
    state: State<'_, DesktopState>,
    input: PhysicalMachineSaveInput,
) -> Result<PhysicalMachine, String> {
    let mut machines = state.physical_machines.lock().map_err(|e| e.to_string())?;
    let (md5, result) =
        physical::match_saved(&machines, &input.census).map_err(|e| e.to_string())?;
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
    let machine = PhysicalMachine {
        id: format!("physical-{}", machines.len() + 1),
        name: input
            .name
            .and_then(|name| (!name.trim().is_empty()).then(|| name.trim().to_owned()))
            .unwrap_or_else(|| md5.clone()),
        usb_port: hint
            .as_ref()
            .map(|hint| hint.port_name.clone())
            .unwrap_or_default(),
        machine_code: md5,
        census: physical::canonicalize_census(&input.census).map_err(|e| e.to_string())?,
        reconnect_hint: hint.clone(),
        virtual_machine_id: None,
        online_servo_ids: input.census.iter().map(|servo| servo.servo_id).collect(),
        connected: true,
        last_seen: Some("刚刚".into()),
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

#[tauri::command]
#[specta::specta]
pub(crate) fn physical_machine_disconnect(
    state: State<'_, DesktopState>,
    id: String,
) -> Result<PhysicalMachine, String> {
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
    let result = machine.clone();
    drop(machines);
    state.persist()?;
    Ok(result)
}

#[tauri::command]
#[specta::specta]
pub(crate) fn physical_machine_reconnect(
    state: State<'_, DesktopState>,
    input: PhysicalMachineReconnectInput,
) -> Result<PhysicalMachine, String> {
    let canonical =
        physical::canonicalize_census(&input.census).map_err(|error| error.to_string())?;
    let mut machines = state.physical_machines.lock().map_err(|e| e.to_string())?;
    let machine = machines
        .iter_mut()
        .find(|machine| machine.id == input.id)
        .ok_or_else(|| "unknown physical machine".to_owned())?;
    if !physical::same_census(&machine.census, &canonical) {
        machine.connected = false;
        return Err("reconnect census does not match saved physical machine".into());
    }
    machine.machine_code = physical::census_md5(&canonical).map_err(|error| error.to_string())?;
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

#[tauri::command]
#[specta::specta]
pub(crate) fn physical_machine_auto_reconnect(
    state: State<'_, DesktopState>,
    id: String,
    baud: u32,
) -> Result<PhysicalMachine, String> {
    let machine_snapshot = state
        .physical_machines
        .lock()
        .map_err(|error| error.to_string())?
        .iter()
        .find(|machine| machine.id == id)
        .cloned()
        .ok_or_else(|| "unknown physical machine".to_owned())?;
    let saved_hint = machine_snapshot
        .reconnect_hint
        .clone()
        .ok_or_else(|| "physical machine has no reconnect hint".to_owned())?;
    let candidates = physical_ports()?
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
        .unwrap_or(ServoModelKey::Hd1910);
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

#[tauri::command]
#[specta::specta]
pub(crate) fn physical_machine_create(
    state: State<'_, DesktopState>,
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
        id: format!("physical-{}", machines.len() + 1),
        name,
        usb_port: input.usb_port,
        machine_code: input.machine_code,
        census: Vec::new(),
        reconnect_hint,
        virtual_machine_id: None,
        online_servo_ids: Vec::new(),
        connected: false,
        last_seen: None,
    };
    machines.push(machine.clone());
    drop(machines);
    state.persist()?;
    Ok(machine)
}

#[tauri::command]
#[specta::specta]
pub(crate) fn physical_machine_connect(
    state: State<'_, DesktopState>,
    id: String,
) -> Result<PhysicalMachine, String> {
    let machine_snapshot = state
        .physical_machines
        .lock()
        .map_err(|e| e.to_string())?
        .iter()
        .find(|machine| machine.id == id)
        .cloned()
        .ok_or_else(|| "unknown physical machine".to_string())?;
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

#[tauri::command]
#[specta::specta]
pub(crate) fn physical_machine_scan(
    state: State<'_, DesktopState>,
    id: String,
) -> Result<PhysicalMachine, String> {
    let online = {
        let mut buses = state.connected_buses.lock().map_err(|e| e.to_string())?;
        let bus = buses
            .get_mut(&id)
            .ok_or_else(|| "physical machine is not connected".to_string())?;
        bus.scan()
    };
    let online = match online {
        Ok(online) => online,
        Err(error) => {
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
                return physical_machine_auto_reconnect(state, id, 1_000_000);
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

#[tauri::command]
#[specta::specta]
pub(crate) fn physical_machine_mount(
    state: State<'_, DesktopState>,
    id: String,
    virtual_machine_id: String,
) -> Result<PhysicalMachine, String> {
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

#[tauri::command]
#[specta::specta]
pub(crate) fn physical_machine_delete(state: State<'_, DesktopState>, id: String) -> Result<(), String> {
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

#[tauri::command]
#[specta::specta]
pub(crate) fn physical_machine_rename(
    state: State<'_, DesktopState>,
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

#[tauri::command]
#[specta::specta]
pub(crate) fn virtual_machine_create(
    state: State<'_, DesktopState>,
    input: VirtualMachineInput,
) -> Result<VirtualMachine, String> {
    let servo_model = model_key(&input.servo_model)?;
    let mut machines = state.virtual_machines.lock().map_err(|e| e.to_string())?;
    let name = if input.name.trim().is_empty() {
        format!("机器 {}", machines.len())
    } else {
        input.name.trim().to_owned()
    };
    let machine = VirtualMachine {
        id: format!("virtual-{}", machines.len() + 1),
        name,
        body_id: input.body_id,
        servo_model,
        simulation: input.simulation,
        startup_selected: false,
        referenced_by: Vec::new(),
    };
    machines.push(machine.clone());
    drop(machines);
    state.persist()?;
    Ok(machine)
}

#[tauri::command]
#[specta::specta]
pub(crate) fn virtual_machine_set_startup(state: State<'_, DesktopState>, id: String) -> Result<(), String> {
    let mut machines = state.virtual_machines.lock().map_err(|e| e.to_string())?;
    if !machines.iter().any(|machine| machine.id == id) {
        return Err(format!("unknown virtual machine: {id}"));
    }
    for machine in &mut *machines {
        machine.startup_selected = machine.id == id;
    }
    drop(machines);
    state.persist()?;
    Ok(())
}

#[tauri::command]
#[specta::specta]
pub(crate) fn virtual_machine_delete(state: State<'_, DesktopState>, id: String) -> Result<(), String> {
    let mut machines = state.virtual_machines.lock().map_err(|e| e.to_string())?;
    let Some(machine) = machines.iter().find(|machine| machine.id == id) else {
        return Err(format!("unknown virtual machine: {id}"));
    };
    if machine.startup_selected || !machine.referenced_by.is_empty() {
        return Err("cannot delete selected or referenced virtual machine".into());
    }
    machines.retain(|machine| machine.id != id);
    drop(machines);
    state.persist()?;
    Ok(())
}

#[tauri::command]
#[specta::specta]
pub(crate) fn virtual_machine_rename(
    state: State<'_, DesktopState>,
    id: String,
    name: String,
) -> Result<VirtualMachine, String> {
    let name = name.trim();
    if name.is_empty() {
        return Err("virtual machine name cannot be empty".into());
    }
    let mut machines = state.virtual_machines.lock().map_err(|e| e.to_string())?;
    let machine = machines
        .iter_mut()
        .find(|machine| machine.id == id)
        .ok_or_else(|| format!("unknown virtual machine: {id}"))?;
    machine.name = name.to_string();
    let result = machine.clone();
    drop(machines);
    state.persist()?;
    Ok(result)
}

#[tauri::command]
#[specta::specta]
pub(crate) fn actions(state: State<'_, DesktopState>) -> Result<Vec<ActionRecord>, String> {
    state
        .actions
        .lock()
        .map(|actions| actions.clone())
        .map_err(|e| e.to_string())
}

#[tauri::command]
#[specta::specta]
pub(crate) fn action_create(
    state: State<'_, DesktopState>,
    input: ActionInput,
) -> Result<ActionRecord, String> {
    let name = input.name.trim();
    if name.is_empty() {
        return Err("action name cannot be empty".into());
    }
    let mut actions = state.actions.lock().map_err(|e| e.to_string())?;
    let kind = match input.kind.as_str() {
        "pose" => ActionKind::Pose,
        "sequence" => ActionKind::Sequence,
        "policy" => ActionKind::Policy,
        _ => return Err("unknown action kind".into()),
    };
    let action = ActionRecord {
        id: format!("action-{}", actions.len() + 1),
        name: name.to_string(),
        kind,
        updated_at: "刚刚".into(),
        source: ActionSource::User,
    };
    actions.push(action.clone());
    Ok(action)
}

#[tauri::command]
#[specta::specta]
pub(crate) fn action_delete(state: State<'_, DesktopState>, id: String) -> Result<(), String> {
    let mut actions = state.actions.lock().map_err(|e| e.to_string())?;
    if actions
        .iter()
        .any(|action| action.id == id && action.source == ActionSource::Builtin)
    {
        return Err("built-in action cannot be deleted".into());
    }
    actions.retain(|action| action.id != id);
    Ok(())
}

#[tauri::command]
#[specta::specta]
pub(crate) fn policy_list(state: State<'_, DesktopState>) -> Result<Vec<PolicyEntry>, String> {
    policy::list(state.config_dir.as_deref())
}

#[tauri::command]
#[specta::specta]
pub(crate) fn policy_download(
    state: State<'_, DesktopState>,
    id: String,
) -> Result<PolicyEntry, String> {
    policy::download(state.config_dir.as_deref(), &id)
}

#[tauri::command]
#[specta::specta]
pub(crate) fn policy_select(
    state: State<'_, DesktopState>,
    id: String,
) -> Result<PolicyEntry, String> {
    policy::select(state.config_dir.as_deref(), &id)
}

#[tauri::command]
#[specta::specta]
pub(crate) fn loop_open(
    state: State<'_, DesktopState>,
    input: LoopOpenInput,
) -> Result<FrontendLoopStatus, String> {
    let LoopOpenInput {
        virtual_machine_id,
        servo_ids,
        physical_machine_id: requested_physical_id,
        policy_id,
    } = input;
    let machine = state
        .virtual_machines
        .lock()
        .map_err(|e| e.to_string())?
        .iter()
        .find(|machine| machine.id == virtual_machine_id)
        .cloned()
        .ok_or_else(|| "unknown virtual machine".to_string())?;
    let template = BodyTemplate::official().map_err(|e| e.to_string())?;
    if servo_ids.len() != template.actuators.len() {
        return Err(format!("expected {} servo ids", template.actuators.len()));
    }
    let model = model(&machine.servo_model);
    let policy_path = if let Some(policy_id) = policy_id {
        let selected = policy::list(state.config_dir.as_deref())?
            .into_iter()
            .find(|entry| entry.id == policy_id && entry.cached && entry.selected)
            .ok_or_else(|| "selected ONNX policy is not downloaded".to_owned())?;
        Some(policy::selected_path(state.config_dir.as_deref()).ok_or_else(|| {
            format!("selected ONNX policy is unavailable: {}", selected.id)
        })?)
    } else {
        None
    };
    let calibration = Calibration {
        joints: template
            .actuators
            .iter()
            .zip(servo_ids.iter().copied())
            .map(|(slot, id)| JointCalibration {
                slot_id: slot.id.clone(),
                physical_id: id,
                direction: 1,
                zero_offset_rad: 0.0,
            })
            .collect(),
    };
    let binding = LoopBinding {
        body: BodyRuntimeView::new(template.clone()).map_err(|e| e.to_string())?,
        servos: ServoBinding::all_slots(&template, model),
        calibration,
    };
    let physical_id = if let Some(requested_id) = requested_physical_id {
        let physical = state
            .physical_machines
            .lock()
            .map_err(|e| e.to_string())?
            .iter()
            .find(|physical| physical.id == requested_id)
            .cloned()
            .ok_or_else(|| "unknown physical machine".to_string())?;
        if !physical.connected
            || physical
                .virtual_machine_id
                .as_deref()
                .is_some_and(|id| id != machine.id)
        {
            return Err("physical machine is not available for this virtual machine".into());
        }
        Some(physical.id)
    } else {
        state
            .physical_machines
            .lock()
            .map_err(|e| e.to_string())?
            .iter()
            .find(|physical| {
                physical.virtual_machine_id.as_deref() == Some(&machine.id) && physical.connected
            })
            .map(|physical| physical.id.clone())
    };
    let driver: Box<dyn ServoLoopDriver> = if let Some(physical_id) = physical_id {
        let bus = state
            .connected_buses
            .lock()
            .map_err(|e| e.to_string())?
            .remove(&physical_id)
            .ok_or_else(|| "physical machine has no connected bus".to_string())?;
        bus.into_driver(model, servo_ids)?
    } else {
        Box::new(MemoryDriver {
            positions: vec![0.0; template.joints.len()],
            torque: false,
        })
    };
    let mut body_loop = PolicyLoop::open(
        binding,
        driver,
        LoopConfig::default(),
    )
    .map_err(|e| e.to_string())?;
    if let Some(path) = policy_path {
        body_loop.select_policy(path).map_err(|e| e.to_string())?;
    }
    let status = FrontendLoopStatus::from(body_loop.status());
    *state.loop_state.lock().map_err(|e| e.to_string())? = Some(body_loop);
    Ok(status)
}

#[tauri::command]
#[specta::specta]
pub(crate) fn loop_command(state: State<'_, DesktopState>, targets: Vec<f64>) -> Result<(), String> {
    let mut guard = state.loop_state.lock().map_err(|e| e.to_string())?;
    let loop_state = guard.as_mut().ok_or_else(|| "loop is closed".to_string())?;
    let session = loop_state.status().session;
    loop_state
        .set_command(loop_body::BodyCommand { session, targets })
        .map_err(|e| e.to_string())
}

#[tauri::command]
#[specta::specta]
pub(crate) fn loop_action(
    state: State<'_, DesktopState>,
    action: FrontendLoopAction,
) -> Result<FrontendLoopStatus, String> {
    let action = match action {
        FrontendLoopAction::Power { on } => BodyLoopAction::Power(on),
        FrontendLoopAction::Drive { on } => BodyLoopAction::Drive(on),
        FrontendLoopAction::Stop => BodyLoopAction::Stop,
        FrontendLoopAction::Estop => BodyLoopAction::Estop,
        FrontendLoopAction::Reconnect { session } => BodyLoopAction::Reconnect { session },
    };
    let mut guard = state.loop_state.lock().map_err(|e| e.to_string())?;
    let loop_state = guard.as_mut().ok_or_else(|| "loop is closed".to_string())?;
    loop_state.send(action).map_err(|e| e.to_string())?;
    Ok(FrontendLoopStatus::from(loop_state.status()))
}

#[tauri::command]
#[specta::specta]
pub(crate) fn loop_tick(state: State<'_, DesktopState>) -> Result<FrontendLoopStatus, String> {
    let mut guard = state.loop_state.lock().map_err(|e| e.to_string())?;
    let loop_state = guard.as_mut().ok_or_else(|| "loop is closed".to_string())?;
    loop_state
        .tick(LoopConfig::default().period)
        .map_err(|e| e.to_string())?;
    Ok(FrontendLoopStatus::from(loop_state.status()))
}

#[tauri::command]
#[specta::specta]
pub(crate) fn loop_status(state: State<'_, DesktopState>) -> Result<Option<FrontendLoopStatus>, String> {
    Ok(state
        .loop_state
        .lock()
        .map_err(|e| e.to_string())?
        .as_ref()
        .map(|body_loop| FrontendLoopStatus::from(body_loop.status())))
}

#[tauri::command]
#[specta::specta]
pub(crate) fn loop_close(state: State<'_, DesktopState>) -> Result<(), String> {
    *state.loop_state.lock().map_err(|e| e.to_string())? = None;
    Ok(())
}

#[tauri::command]
#[specta::specta]
pub(crate) fn preview_loop_open(
    state: State<'_, DesktopState>,
    virtual_machine_id: String,
) -> Result<FrontendLoopStatus, String> {
    let machine = state
        .virtual_machines
        .lock()
        .map_err(|e| e.to_string())?
        .iter()
        .find(|machine| machine.id == virtual_machine_id)
        .cloned()
        .ok_or_else(|| "unknown virtual machine".to_string())?;
    let preview = PreviewPolicyLoop::open(model(&machine.servo_model))?;
    let status = preview.status();
    *state.preview_loop.lock().map_err(|e| e.to_string())? = Some(preview);
    Ok(status)
}

#[tauri::command]
#[specta::specta]
pub(crate) fn preview_loop_policy(
    state: State<'_, DesktopState>,
    policy_id: String,
) -> Result<FrontendLoopStatus, String> {
    let selected = policy::list(state.config_dir.as_deref())?
        .into_iter()
        .find(|entry| entry.id == policy_id && entry.cached && entry.selected)
        .ok_or_else(|| "selected ONNX policy is not downloaded".to_owned())?;
    let path = policy::selected_path(state.config_dir.as_deref())
        .ok_or_else(|| format!("selected ONNX policy is unavailable: {}", selected.id))?;
    let mut guard = state.preview_loop.lock().map_err(|e| e.to_string())?;
    let preview = guard
        .as_mut()
        .ok_or_else(|| "preview loop is closed".to_owned())?;
    preview.select_policy(&path, policy_id)?;
    Ok(preview.status())
}

#[tauri::command]
#[specta::specta]
pub(crate) fn preview_loop_tick(
    state: State<'_, DesktopState>,
) -> Result<FrontendLoopStatus, String> {
    let mut guard = state.preview_loop.lock().map_err(|e| e.to_string())?;
    guard
        .as_mut()
        .ok_or_else(|| "preview loop is closed".to_owned())?
        .tick()
}

#[tauri::command]
#[specta::specta]
pub(crate) fn preview_loop_close(state: State<'_, DesktopState>) -> Result<(), String> {
    *state.preview_loop.lock().map_err(|e| e.to_string())? = None;
    Ok(())
}
