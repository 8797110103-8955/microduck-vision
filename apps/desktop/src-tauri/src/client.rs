//! Tauri command layer for the desktop client.
//!
//! These commands coordinate user-facing state and persistence.  Hardware and
//! loop-body adapters stay in [`crate::control`], while storage lives in
//! [`crate::state`].

use std::collections::{HashMap, HashSet};
use std::process::Command as ProcessCommand;
use std::sync::OnceLock;

use duck_body::{BodyTemplate, ServoModelKey};
use duck_control::obs::Command;
use loop_body::{
    BodyRuntimeView, Calibration, JointCalibration, LoopAction as BodyLoopAction, LoopBinding,
    LoopConfig, PolicyLoop, ServoBinding,
};
use servo::ServoLoopDriver;
use tauri::{AppHandle, Emitter, Manager, State, Wry};

use crate::control::{
    ConnectedBus, MemoryDriver, RegisterDescriptor, move_pending_bus, open_for_model,
    try_open_feetech, try_open_xl330,
};
use crate::helpers::{model, model_key};
use crate::logging::{
    LoggedServoDriver, append_control_event, append_policy_trace, begin_control_loop,
};
use crate::physical;
use crate::policy;
use crate::simulation::PreviewPolicyLoop;
use crate::state::DesktopState;
use crate::types::{
    ActionInput, ActionKind, ActionRecord, ActionSource, DetectedPhysicalMachine, DiagnosticCode,
    JointRange, LoopAction as FrontendLoopAction, LoopCommandInput, LoopOpenInput,
    LoopStatus as FrontendLoopStatus, MatchResult, PhysicalMachine, PhysicalMachineBindingInput,
    PhysicalMachineDetectInput, PhysicalMachineInput, PhysicalMachineProbeInput,
    PhysicalMachineReconnectInput, PhysicalMachineRegisterWriteInput, PhysicalMachineSaveInput,
    PhysicalServoRegister, PolicyEntry, PreviewInput, ReconnectHint,
    SceneBody as FrontendSceneBody, SceneFrame as FrontendSceneFrame,
    SceneGeom as FrontendSceneGeom, SceneInfo as FrontendSceneInfo,
    SceneJoint as FrontendSceneJoint, SceneModel as FrontendSceneModel,
    SceneSite as FrontendSceneSite, SerialPortCandidate, ServoCatalogEntry, ServoIdentity,
    ServoJointBinding, VirtualMachine, VirtualMachineBindingInput, VirtualMachineInput,
};

fn register_table(model: servo::ServoModel) -> Vec<RegisterDescriptor> {
    let registers = match model {
        servo::ServoModel::Hd1910 => servo::hd1910::registers(),
        servo::ServoModel::Xl330M288 => servo::xl330::registers(),
        _ => &[],
    };
    let mut addresses = HashSet::new();
    let mut table = Vec::new();
    for register in registers {
        if model == servo::ServoModel::Hd1910 && register.addr == 3 {
            if !addresses.insert(3) {
                continue;
            }
            // FT exposes the servo model as two one-byte version fields.
            // rustypot's shared STS definition exposes a u16 alias at 0x03.
            table.extend([
                RegisterDescriptor {
                    name: "model",
                    addr: 3,
                    size: 1,
                    access: servo::RegisterAccess::Read,
                    is_default: false,
                },
                RegisterDescriptor {
                    name: "model_number",
                    addr: 4,
                    size: 1,
                    access: servo::RegisterAccess::Read,
                    is_default: false,
                },
            ]);
            addresses.insert(4);
            continue;
        }
        // rustypot's shared STS table has the old two-byte definition at 0x18
        // and the old factory meaning at 0x53. HD1910 uses the FT table here.
        if model == servo::ServoModel::Hd1910 && matches!(register.addr, 24 | 83) {
            continue;
        }
        if model == servo::ServoModel::Hd1910 && register.addr == 44 {
            table.push(RegisterDescriptor {
                name: "pwm_open_speed",
                addr: 44,
                size: 2,
                access: servo::RegisterAccess::ReadWrite,
                is_default: false,
            });
            addresses.insert(44);
            continue;
        }
        if !addresses.insert(register.addr) {
            continue;
        }
        table.push(RegisterDescriptor {
            name: register.name,
            addr: register.addr,
            size: register.size,
            access: register.access,
            is_default: model == servo::ServoModel::Hd1910 && register.addr >= 80,
        });
        if model == servo::ServoModel::Hd1910 && register.addr == 69 {
            table.extend([
                RegisterDescriptor {
                    name: "current_bias",
                    addr: 73,
                    size: 2,
                    access: servo::RegisterAccess::Read,
                    is_default: false,
                },
                RegisterDescriptor {
                    name: "v_fk",
                    addr: 77,
                    size: 1,
                    access: servo::RegisterAccess::Read,
                    is_default: true,
                },
                RegisterDescriptor {
                    name: "v_kgi",
                    addr: 78,
                    size: 1,
                    access: servo::RegisterAccess::Read,
                    is_default: true,
                },
                RegisterDescriptor {
                    name: "p_fk",
                    addr: 79,
                    size: 1,
                    access: servo::RegisterAccess::Read,
                    is_default: true,
                },
            ]);
            addresses.extend([73, 77, 78, 79]);
        }
        if model == servo::ServoModel::Hd1910 && register.addr == 82 {
            // 0x52/0x53 are eFk/Vk on HD1910, rather than the shared STS
            // velocity-unit/minimum-speed pair.
            table.pop();
            addresses.remove(&82);
            table.extend([
                RegisterDescriptor {
                    name: "e_fk",
                    addr: 82,
                    size: 1,
                    access: servo::RegisterAccess::Read,
                    is_default: true,
                },
                RegisterDescriptor {
                    name: "vk",
                    addr: 83,
                    size: 1,
                    access: servo::RegisterAccess::Read,
                    is_default: true,
                },
            ]);
            addresses.extend([82, 83]);
        }
        if model == servo::ServoModel::Hd1910 && register.addr == 23 {
            table.push(RegisterDescriptor {
                name: "minimum_startup_force",
                addr: 24,
                size: 1,
                access: servo::RegisterAccess::ReadWrite,
                is_default: false,
            });
            table.push(RegisterDescriptor {
                name: "integral_limit",
                addr: 25,
                size: 1,
                access: servo::RegisterAccess::ReadWrite,
                is_default: false,
            });
            addresses.extend([24, 25]);
        }
    }
    table
}

fn register_model(name: &str) -> Result<servo::ServoModel, String> {
    match model_key(&name.trim().to_ascii_lowercase())? {
        ServoModelKey::Hd1910 => Ok(servo::ServoModel::Hd1910),
        ServoModelKey::Xl330 => Ok(servo::ServoModel::Xl330M288),
    }
}

fn register_is_writable(register: RegisterDescriptor) -> bool {
    !register.is_default && !matches!(register.access, servo::RegisterAccess::Read)
}

fn binding_conflict_error(bindings: &[ServoJointBinding]) -> Option<String> {
    let mut by_id = HashMap::<u8, Vec<&str>>::new();
    for binding in bindings {
        by_id
            .entry(binding.physical_id)
            .or_default()
            .push(binding.slot_id.as_str());
    }
    let mut conflicts = by_id
        .into_iter()
        .filter(|(_, slots)| slots.len() > 1)
        .map(|(id, mut slots)| {
            slots.sort_unstable();
            format!("{id} → {}", slots.join(","))
        })
        .collect::<Vec<_>>();
    conflicts.sort_unstable();
    (!conflicts.is_empty()).then(|| format!("绑定冲突：{}", conflicts.join("；")))
}

fn official_servo_bindings(census: &[ServoIdentity]) -> Vec<ServoJointBinding> {
    duck_control::model::JOINT_NAMES
        .iter()
        .copied()
        .zip(duck_control::model::JOINT_IDS)
        .filter(|(_, id)| census.iter().any(|servo| servo.servo_id == *id))
        .map(|(slot_id, physical_id)| ServoJointBinding {
            slot_id: slot_id.to_owned(),
            physical_id,
            direction: 1,
            zero_offset_rad: 0.0,
            min_rad: None,
            max_rad: None,
        })
        .collect()
}

#[tauri::command]
#[specta::specta]
pub(crate) fn servo_catalog() -> Vec<ServoCatalogEntry> {
    duck_body::servo_catalog()
}

#[tauri::command]
#[specta::specta]
pub(crate) fn official_joint_slots() -> Vec<String> {
    duck_control::model::JOINT_NAMES
        .iter()
        .map(|name| (*name).to_owned())
        .collect()
}

#[tauri::command]
#[specta::specta]
pub(crate) fn official_joint_ranges() -> Result<Vec<JointRange>, String> {
    let template = BodyTemplate::official().map_err(|error| error.to_string())?;
    Ok(template
        .actuators
        .iter()
        .map(|slot| {
            let range = template
                .joint_range(&slot.joint)
                .unwrap_or([-std::f64::consts::PI, std::f64::consts::PI]);
            JointRange {
                slot_id: slot.id.clone(),
                min_rad: range[0],
                max_rad: range[1],
            }
        })
        .collect())
}

fn official_scene() -> Result<&'static duck_body::OfficialScene, String> {
    static SCENE: OnceLock<Result<duck_body::OfficialScene, String>> = OnceLock::new();
    SCENE
        .get_or_init(|| duck_body::OfficialScene::load().map_err(|error| error.to_string()))
        .as_ref()
        .map_err(Clone::clone)
}

pub(crate) fn scene_model() -> Result<FrontendSceneInfo, String> {
    let scene = official_scene()?;
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
    let scene = official_scene()?;
    let frame = scene.frame(&positions).map_err(|error| error.to_string())?;
    Ok(FrontendSceneFrame {
        bodies: frame.bodies,
        markers: frame.markers,
    })
}

#[tauri::command]
#[specta::specta]
pub(crate) fn virtual_machines(
    state: State<'_, DesktopState>,
) -> Result<Vec<VirtualMachine>, String> {
    state
        .virtual_machines
        .lock()
        .map(|machines| machines.clone())
        .map_err(|e| e.to_string())
}

#[tauri::command]
#[specta::specta]
pub(crate) fn physical_machines(
    state: State<'_, DesktopState>,
) -> Result<Vec<PhysicalMachine>, String> {
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
pub(crate) fn set_auto_mount_enabled(
    state: State<'_, DesktopState>,
    enabled: bool,
) -> Result<bool, String> {
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

/// Reconcile saved physical machines with the ports currently exposed by the
/// operating system.  This is called by the USB watcher after a port set
/// change, and deliberately does not hold the machine lock while opening a
/// serial bus or scanning servos.
pub(crate) fn refresh_physical_machines(app: &AppHandle<Wry>) -> Result<(), String> {
    let state = app.state::<DesktopState>();
    let ports = physical_ports()?;
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
            mark_physical_machine_offline(&state, &machine.id)?;
            continue;
        }
        if auto_reconnect
            && physical_machine_auto_reconnect(state.clone(), machine.id.clone(), 1_000_000)
                .is_err()
        {
            // The adapter can still enumerate while the servo bus is
            // absent or belongs to another machine. Do not leave stale
            // "online" state in that case.
            mark_physical_machine_offline(&state, &machine.id)?;
        }
    }
    state.persist()?;
    let _ = app.emit("physical-machines-changed", ());
    Ok(())
}

fn mark_physical_machine_offline(state: &DesktopState, id: &str) -> Result<(), String> {
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
        torque_enabled: false,
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
        torque_enabled: false,
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
pub(crate) fn physical_machine_registers(
    state: State<'_, DesktopState>,
    id: String,
) -> Result<Vec<PhysicalServoRegister>, String> {
    let machine = state
        .physical_machines
        .lock()
        .map_err(|error| error.to_string())?
        .iter()
        .find(|machine| machine.id == id)
        .cloned()
        .ok_or_else(|| "unknown physical machine".to_string())?;
    let mut buses = state
        .connected_buses
        .lock()
        .map_err(|error| error.to_string())?;
    let mut bus = buses.get_mut(&id);
    let mut result = Vec::new();
    for servo_identity in &machine.census {
        let model = register_model(&servo_identity.model)?;
        for register in register_table(model) {
            let writable = register_is_writable(register);
            let value = if matches!(register.access, servo::RegisterAccess::Write) {
                None
            } else {
                bus.as_mut().and_then(|bus| {
                    bus.read_register(model, servo_identity.servo_id, register)
                        .ok()
                        .and_then(|value| i32::try_from(value).ok())
                })
            };
            result.push(PhysicalServoRegister {
                servo_id: servo_identity.servo_id,
                address: register.addr,
                name: register.name.to_owned(),
                size: register.size,
                default: register.is_default,
                writable,
                value,
            });
        }
    }
    Ok(result)
}

#[tauri::command]
#[specta::specta]
pub(crate) fn physical_machine_read_register(
    state: State<'_, DesktopState>,
    id: String,
    servo_id: u8,
    name: String,
) -> Result<PhysicalServoRegister, String> {
    let machine = state
        .physical_machines
        .lock()
        .map_err(|error| error.to_string())?
        .iter()
        .find(|machine| machine.id == id)
        .cloned()
        .ok_or_else(|| "unknown physical machine".to_string())?;
    let servo_identity = machine
        .census
        .iter()
        .find(|servo| servo.servo_id == servo_id)
        .ok_or_else(|| format!("servo {servo_id} is not in the machine census"))?;
    let model = register_model(&servo_identity.model)?;
    let register = register_table(model)
        .into_iter()
        .find(|register| register.name == name)
        .ok_or_else(|| format!("unknown register {name}"))?;
    let value = if matches!(register.access, servo::RegisterAccess::Write) {
        None
    } else {
        Some(
            state
                .connected_buses
                .lock()
                .map_err(|error| error.to_string())?
                .get_mut(&id)
                .ok_or_else(|| "physical machine is not connected".to_owned())?
                .read_register(model, servo_id, register)?,
        )
    };
    Ok(PhysicalServoRegister {
        servo_id,
        address: register.addr,
        name: register.name.to_owned(),
        size: register.size,
        default: register.is_default,
        writable: register_is_writable(register),
        value: value.and_then(|value| i32::try_from(value).ok()),
    })
}

#[tauri::command]
#[specta::specta]
pub(crate) fn physical_machine_write_register(
    state: State<'_, DesktopState>,
    input: PhysicalMachineRegisterWriteInput,
) -> Result<PhysicalServoRegister, String> {
    let machine = state
        .physical_machines
        .lock()
        .map_err(|error| error.to_string())?
        .iter()
        .find(|machine| machine.id == input.physical_machine_id)
        .cloned()
        .ok_or_else(|| "unknown physical machine".to_string())?;
    let servo_identity = machine
        .census
        .iter()
        .find(|servo| servo.servo_id == input.servo_id)
        .ok_or_else(|| format!("servo {} is not in the machine census", input.servo_id))?;
    let model = register_model(&servo_identity.model)?;
    let register = register_table(model)
        .into_iter()
        .find(|register| register.name == input.name)
        .ok_or_else(|| format!("unknown register {}", input.name))?;
    let new_servo_id = if register.addr == 5 {
        Some(
            u8::try_from(input.value)
                .map_err(|_| "舵机 ID 必须是 0 到 255 之间的整数".to_owned())?,
        )
    } else {
        None
    };
    if let Some(new_servo_id) = new_servo_id
        && new_servo_id != input.servo_id
        && machine
            .census
            .iter()
            .any(|servo| servo.servo_id == new_servo_id)
    {
        return Err(format!("舵机 ID {new_servo_id} 已经存在"));
    }
    if !register_is_writable(register) {
        if register.is_default {
            return Err(format!(
                "register {} is a default value and read-only",
                register.name
            ));
        }
        return Err(format!("register {} is read-only", register.name));
    }
    {
        let mut buses = state
            .connected_buses
            .lock()
            .map_err(|error| error.to_string())?;
        let bus = buses
            .get_mut(&input.physical_machine_id)
            .ok_or_else(|| "physical machine is not connected".to_string())?;
        bus.write_register(model, input.servo_id, register, i64::from(input.value))?;
    }
    if let Some(new_servo_id) = new_servo_id {
        let virtual_machine_id = {
            let mut machines = state
                .physical_machines
                .lock()
                .map_err(|error| error.to_string())?;
            let machine = machines
                .iter_mut()
                .find(|machine| machine.id == input.physical_machine_id)
                .ok_or_else(|| "unknown physical machine".to_owned())?;
            let old_servo_id = input.servo_id;
            update_servo_id_cache(machine, old_servo_id, new_servo_id)?;
            machine.virtual_machine_id.clone()
        };
        if let Some(virtual_machine_id) = virtual_machine_id {
            let mut virtual_machines = state
                .virtual_machines
                .lock()
                .map_err(|error| error.to_string())?;
            if let Some(virtual_machine) = virtual_machines
                .iter_mut()
                .find(|virtual_machine| virtual_machine.id == virtual_machine_id)
            {
                for binding in &mut virtual_machine.joint_bindings {
                    if binding.physical_id == input.servo_id {
                        binding.physical_id = new_servo_id;
                    }
                }
            }
        }
        state.persist()?;
    }
    // The write status packet already confirms the command.  Do not issue an
    // immediate read-back here: EEPROM writes may still be committing after
    // the lock byte is restored, and that extra read can time out even though
    // the requested value was accepted by the servo.
    let value = Some(input.value);
    Ok(PhysicalServoRegister {
        servo_id: new_servo_id.unwrap_or(input.servo_id),
        address: register.addr,
        name: register.name.to_owned(),
        size: register.size,
        default: register.is_default,
        writable: true,
        value,
    })
}

/// Keep all persisted views of a physical servo ID coherent after a successful
/// write to register 5.  This is deliberately separate from the serial command
/// so it can be checked without hardware and cannot silently drift when the
/// census representation changes.
fn update_servo_id_cache(
    machine: &mut PhysicalMachine,
    old_servo_id: u8,
    new_servo_id: u8,
) -> Result<(), String> {
    if old_servo_id == new_servo_id {
        return Ok(());
    }
    if !machine
        .census
        .iter()
        .any(|servo| servo.servo_id == old_servo_id)
    {
        return Err(format!("servo {old_servo_id} is not in the machine census"));
    }
    if machine
        .census
        .iter()
        .any(|servo| servo.servo_id == new_servo_id)
    {
        return Err(format!("舵机 ID {new_servo_id} 已经存在"));
    }
    for servo in &mut machine.census {
        if servo.servo_id == old_servo_id {
            servo.servo_id = new_servo_id;
        }
    }
    machine.census =
        physical::canonicalize_census(&machine.census).map_err(|error| error.to_string())?;
    machine.machine_code =
        physical::census_md5(&machine.census).map_err(|error| error.to_string())?;
    for id in &mut machine.online_servo_ids {
        if *id == old_servo_id {
            *id = new_servo_id;
        }
    }
    Ok(())
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
pub(crate) fn physical_machine_delete(
    state: State<'_, DesktopState>,
    id: String,
) -> Result<(), String> {
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
pub(crate) fn physical_machine_bind(
    state: State<'_, DesktopState>,
    input: PhysicalMachineBindingInput,
) -> Result<PhysicalMachine, String> {
    if !duck_control::model::JOINT_NAMES.contains(&input.slot_id.as_str()) {
        return Err(format!("unknown joint slot: {}", input.slot_id));
    }
    let physical = state
        .physical_machines
        .lock()
        .map_err(|e| e.to_string())?
        .iter()
        .find(|machine| machine.id == input.physical_machine_id)
        .ok_or_else(|| "unknown physical machine".to_owned())?
        .clone();
    if let Some(physical_id) = input.physical_id
        && !physical
            .census
            .iter()
            .any(|servo| servo.servo_id == physical_id)
    {
        return Err(format!("servo {physical_id} is not in the machine census"));
    }
    let virtual_id = input
        .virtual_machine_id
        .or_else(|| physical.virtual_machine_id.clone());
    let Some(virtual_id) = virtual_id else {
        return Err("实机尚未关联模拟机，绑定关系必须保存到模拟机".into());
    };
    let mut virtual_machines = state.virtual_machines.lock().map_err(|e| e.to_string())?;
    let virtual_machine = virtual_machines
        .iter_mut()
        .find(|machine| machine.id == virtual_id)
        .ok_or_else(|| "unknown virtual machine".to_owned())?;
    if let Some(physical_id) = input.physical_id {
        if let Some(binding) = virtual_machine
            .joint_bindings
            .iter_mut()
            .find(|binding| binding.slot_id == input.slot_id)
        {
            binding.physical_id = physical_id;
        } else {
            virtual_machine.joint_bindings.push(ServoJointBinding {
                slot_id: input.slot_id.clone(),
                physical_id,
                direction: 1,
                zero_offset_rad: 0.0,
                min_rad: None,
                max_rad: None,
            });
        }
    } else {
        virtual_machine
            .joint_bindings
            .retain(|binding| binding.slot_id != input.slot_id);
    }
    virtual_machine.joint_bindings.sort_by_key(|binding| {
        duck_control::model::joint_index(&binding.slot_id).unwrap_or(usize::MAX)
    });
    drop(virtual_machines);
    state.persist()?;
    Ok(physical)
}

/// Store a body-to-servo mapping on the virtual machine itself.  This path is
/// used while no physical machine is available, so it deliberately does not
/// validate the ID against a hardware census.  The same mapping is consumed by
/// the simulation loop and becomes the source of truth when hardware is later
/// connected.
#[tauri::command]
#[specta::specta]
pub(crate) fn virtual_machine_bind(
    state: State<'_, DesktopState>,
    input: VirtualMachineBindingInput,
) -> Result<VirtualMachine, String> {
    if !duck_control::model::JOINT_NAMES.contains(&input.slot_id.as_str()) {
        return Err(format!("unknown joint slot: {}", input.slot_id));
    }
    let mut virtual_machines = state.virtual_machines.lock().map_err(|e| e.to_string())?;
    let machine = virtual_machines
        .iter_mut()
        .find(|machine| machine.id == input.virtual_machine_id)
        .ok_or_else(|| "unknown virtual machine".to_owned())?;
    if let Some(physical_id) = input.physical_id {
        if let Some(binding) = machine
            .joint_bindings
            .iter_mut()
            .find(|binding| binding.slot_id == input.slot_id)
        {
            binding.physical_id = physical_id;
        } else {
            machine.joint_bindings.push(ServoJointBinding {
                slot_id: input.slot_id,
                physical_id,
                direction: 1,
                zero_offset_rad: 0.0,
                min_rad: None,
                max_rad: None,
            });
        }
    } else {
        machine
            .joint_bindings
            .retain(|binding| binding.slot_id != input.slot_id);
    }
    machine.joint_bindings.sort_by_key(|binding| {
        duck_control::model::joint_index(&binding.slot_id).unwrap_or(usize::MAX)
    });
    let result = machine.clone();
    drop(virtual_machines);
    state.persist()?;
    Ok(result)
}

#[tauri::command]
#[specta::specta]
pub(crate) fn virtual_machine_validate_bindings(
    state: State<'_, DesktopState>,
    id: String,
) -> Result<VirtualMachine, String> {
    let mut virtual_machines = state.virtual_machines.lock().map_err(|e| e.to_string())?;
    let machine = virtual_machines
        .iter_mut()
        .find(|machine| machine.id == id)
        .ok_or_else(|| "unknown virtual machine".to_owned())?;
    let mut conflicting_ids = Vec::new();
    for (index, binding) in machine.joint_bindings.iter().enumerate() {
        if machine.joint_bindings[index + 1..]
            .iter()
            .any(|other| other.physical_id == binding.physical_id)
        {
            conflicting_ids.push(binding.physical_id);
        }
    }
    machine
        .joint_bindings
        .retain(|binding| !conflicting_ids.contains(&binding.physical_id));
    let result = machine.clone();
    drop(virtual_machines);
    if !conflicting_ids.is_empty() {
        state.persist()?;
    }
    Ok(result)
}

#[tauri::command]
#[specta::specta]
pub(crate) fn physical_machine_validate_bindings(
    state: State<'_, DesktopState>,
    id: String,
) -> Result<PhysicalMachine, String> {
    let physical = state
        .physical_machines
        .lock()
        .map_err(|e| e.to_string())?
        .iter()
        .find(|machine| machine.id == id)
        .cloned()
        .ok_or_else(|| "unknown physical machine".to_owned())?;
    let Some(virtual_id) = physical.virtual_machine_id.clone() else {
        return Ok(physical);
    };
    let mut virtual_machines = state.virtual_machines.lock().map_err(|e| e.to_string())?;
    let virtual_machine = virtual_machines
        .iter_mut()
        .find(|machine| machine.id == virtual_id)
        .ok_or_else(|| "关联的模拟机不存在".to_owned())?;
    let mut bindings = virtual_machine.joint_bindings.clone();
    let mut conflicting_ids = Vec::new();
    for (index, binding) in bindings.iter().enumerate() {
        if bindings[index + 1..]
            .iter()
            .any(|other| other.physical_id == binding.physical_id)
        {
            conflicting_ids.push(binding.physical_id);
        }
    }
    if !conflicting_ids.is_empty() {
        bindings.retain(|binding| !conflicting_ids.contains(&binding.physical_id));
    }
    virtual_machine.joint_bindings = bindings;
    drop(virtual_machines);
    if !conflicting_ids.is_empty() {
        state.persist()?;
    }
    Ok(physical)
}

#[tauri::command]
#[specta::specta]
pub(crate) fn physical_machine_set_pose_zero(
    state: State<'_, DesktopState>,
    id: String,
    virtual_machine_id: Option<String>,
) -> Result<PhysicalMachine, String> {
    if state
        .loop_state
        .lock()
        .map_err(|error| error.to_string())?
        .is_some()
    {
        return Err("控制环运行时不能绑定姿态零位".into());
    }
    let machine = {
        let machines = state
            .physical_machines
            .lock()
            .map_err(|error| error.to_string())?;
        machines
            .iter()
            .find(|machine| machine.id == id)
            .cloned()
            .ok_or_else(|| "unknown physical machine".to_owned())?
    };
    let virtual_id = virtual_machine_id
        .as_deref()
        .or(machine.virtual_machine_id.as_deref())
        .ok_or_else(|| "实机尚未关联模拟机".to_owned())?
        .to_owned();
    let (model, bindings) = {
        let virtual_machine = state
            .virtual_machines
            .lock()
            .map_err(|error| error.to_string())?
            .iter()
            .find(|item| item.id == virtual_id)
            .cloned()
            .ok_or_else(|| "关联的模拟机不存在".to_owned())?;
        let bindings = if virtual_machine.joint_bindings.is_empty() {
            official_servo_bindings(&machine.census)
        } else {
            virtual_machine.joint_bindings
        };
        let model = model(&virtual_machine.servo_model);
        (model, bindings)
    };
    if let Some(error) = binding_conflict_error(&bindings) {
        return Err(error);
    }
    let ids = bindings
        .iter()
        .map(|binding| binding.physical_id)
        .collect::<Vec<_>>();
    let positions = state
        .connected_buses
        .lock()
        .map_err(|error| error.to_string())?
        .get_mut(&id)
        .ok_or_else(|| "实机没有可用连接".to_owned())?
        .read_positions(model, &ids)?;
    let zeroes = bindings
        .iter()
        .zip(positions)
        .map(|(binding, position)| (binding.slot_id.clone(), position))
        .collect::<Vec<_>>();
    let mut virtual_machines = state
        .virtual_machines
        .lock()
        .map_err(|error| error.to_string())?;
    let virtual_machine = virtual_machines
        .iter_mut()
        .find(|machine| machine.id == virtual_id)
        .ok_or_else(|| "关联的模拟机不存在".to_owned())?;
    if virtual_machine.joint_bindings.is_empty() {
        virtual_machine.joint_bindings = bindings;
    }
    for (binding, position) in &zeroes {
        if let Some(current_binding) = virtual_machine
            .joint_bindings
            .iter_mut()
            .find(|item| item.slot_id == *binding)
        {
            current_binding.zero_offset_rad = *position;
        }
    }
    drop(virtual_machines);
    state.persist()?;
    Ok(machine)
}

#[tauri::command]
#[specta::specta]
pub(crate) fn physical_machine_pose(
    state: State<'_, DesktopState>,
    id: String,
    virtual_machine_id: Option<String>,
) -> Result<Vec<f64>, String> {
    let template = BodyTemplate::official().map_err(|error| error.to_string())?;
    let (model, bindings) = {
        let machines = state
            .physical_machines
            .lock()
            .map_err(|error| error.to_string())?;
        let machine = machines
            .iter()
            .find(|machine| machine.id == id)
            .cloned()
            .ok_or_else(|| "unknown physical machine".to_owned())?;
        let virtual_id = virtual_machine_id
            .as_deref()
            .or(machine.virtual_machine_id.as_deref())
            .ok_or_else(|| "实机尚未关联模拟机".to_owned())?;
        let model = state
            .virtual_machines
            .lock()
            .map_err(|error| error.to_string())?
            .iter()
            .find(|item| item.id == virtual_id)
            .map(|item| model(&item.servo_model))
            .ok_or_else(|| "关联的模拟机不存在".to_owned())?;
        let virtual_bindings = state
            .virtual_machines
            .lock()
            .map_err(|error| error.to_string())?
            .iter()
            .find(|item| item.id == virtual_id)
            .map(|item| item.joint_bindings.clone())
            .unwrap_or_default();
        let bindings = if !virtual_bindings.is_empty() {
            virtual_bindings
        } else {
            official_servo_bindings(&machine.census)
        };
        (model, bindings)
    };
    if let Some(error) = binding_conflict_error(&bindings) {
        return Err(error);
    }
    let mut ids = Vec::with_capacity(template.actuators.len());
    let calibration = template
        .actuators
        .iter()
        .map(|slot| {
            let binding = bindings
                .iter()
                .find(|binding| binding.slot_id == slot.id)
                .ok_or_else(|| format!("实机尚未绑定肢体槽位 {}", slot.id))?;
            ids.push(binding.physical_id);
            Ok(JointCalibration {
                slot_id: slot.id.clone(),
                physical_id: binding.physical_id,
                direction: binding.direction,
                zero_offset_rad: binding.zero_offset_rad,
                min_rad: binding.min_rad,
                max_rad: binding.max_rad,
            })
        })
        .collect::<Result<Vec<_>, String>>()?;
    let positions = state
        .connected_buses
        .lock()
        .map_err(|error| error.to_string())?
        .get_mut(&id)
        .ok_or_else(|| "实机没有可用连接".to_owned())?
        .read_positions(model, &ids)?;
    Ok(Calibration {
        joints: calibration,
    }
    .to_body_positions(&positions))
}

#[tauri::command]
#[specta::specta]
pub(crate) fn physical_machine_set_torque(
    state: State<'_, DesktopState>,
    id: String,
    enabled: bool,
    virtual_machine_id: Option<String>,
) -> Result<PhysicalMachine, String> {
    if state
        .loop_state
        .lock()
        .map_err(|error| error.to_string())?
        .is_some()
    {
        return Err("控制环运行时不能切换全部力矩".into());
    }
    let (model, ids) = {
        let machines = state
            .physical_machines
            .lock()
            .map_err(|error| error.to_string())?;
        let machine = machines
            .iter()
            .find(|machine| machine.id == id)
            .ok_or_else(|| "unknown physical machine".to_owned())?;
        let virtual_id = virtual_machine_id
            .as_deref()
            .or(machine.virtual_machine_id.as_deref())
            .ok_or_else(|| "实机尚未关联模拟机".to_owned())?;
        let model = state
            .virtual_machines
            .lock()
            .map_err(|error| error.to_string())?
            .iter()
            .find(|item| item.id == virtual_id)
            .map(|item| model(&item.servo_model))
            .ok_or_else(|| "关联的模拟机不存在".to_owned())?;
        (
            model,
            machine
                .census
                .iter()
                .map(|servo| servo.servo_id)
                .collect::<Vec<_>>(),
        )
    };
    state
        .connected_buses
        .lock()
        .map_err(|error| error.to_string())?
        .get_mut(&id)
        .ok_or_else(|| "实机没有可用连接".to_owned())?
        .set_torque(model, &ids, enabled)?;
    let mut machines = state
        .physical_machines
        .lock()
        .map_err(|error| error.to_string())?;
    let machine = machines
        .iter_mut()
        .find(|machine| machine.id == id)
        .ok_or_else(|| "unknown physical machine".to_owned())?;
    machine.torque_enabled = enabled;
    let result = machine.clone();
    drop(machines);
    Ok(result)
}

/// Write one already-calibrated servo position.  Binding lookup, range
/// clamping, zero offsets and torque orchestration belong to the debug UI;
/// this command is deliberately only the desktop-to-bus write boundary.
#[tauri::command]
#[specta::specta]
pub(crate) fn physical_machine_write_position(
    state: State<'_, DesktopState>,
    id: String,
    servo_id: u8,
    position_rad: f64,
) -> Result<PhysicalMachine, String> {
    if !position_rad.is_finite() {
        return Err("目标位置必须是有限数字".into());
    }
    if state
        .loop_state
        .lock()
        .map_err(|error| error.to_string())?
        .is_some()
    {
        return Err("控制环运行时请通过控制环发送目标".into());
    }

    let machine = state
        .physical_machines
        .lock()
        .map_err(|error| error.to_string())?
        .iter()
        .find(|machine| machine.id == id)
        .cloned()
        .ok_or_else(|| "unknown physical machine".to_owned())?;
    if !machine.connected {
        return Err("实机没有可用连接".into());
    }
    if !machine.online_servo_ids.contains(&servo_id)
        && !machine.census.iter().any(|servo| servo.servo_id == servo_id)
    {
        return Err(format!("实机舵机 {servo_id} 当前不在线"));
    }
    let mut buses = state
        .connected_buses
        .lock()
        .map_err(|error| error.to_string())?;
    let bus = buses
        .get_mut(&id)
        .ok_or_else(|| "实机没有可用连接".to_owned())?;
    bus.write_position(servo_id, position_rad)?;
    drop(buses);
    Ok(machine)
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
        joint_bindings: Vec::new(),
    };
    machines.push(machine.clone());
    drop(machines);
    state.persist()?;
    Ok(machine)
}

#[tauri::command]
#[specta::specta]
pub(crate) fn virtual_machine_set_startup(
    state: State<'_, DesktopState>,
    id: String,
) -> Result<(), String> {
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
pub(crate) fn virtual_machine_set_world_position(
    state: State<'_, DesktopState>,
    id: String,
    position: [f64; 3],
) -> Result<VirtualMachine, String> {
    if position.iter().any(|value| !value.is_finite()) {
        return Err("世界位置必须是有限数字".into());
    }
    let mut machines = state.virtual_machines.lock().map_err(|e| e.to_string())?;
    let machine = machines
        .iter_mut()
        .find(|machine| machine.id == id)
        .ok_or_else(|| format!("unknown virtual machine: {id}"))?;
    machine.simulation.world_position = position;
    let result = machine.clone();
    drop(machines);
    state.persist()?;
    Ok(result)
}

#[tauri::command]
#[specta::specta]
pub(crate) fn virtual_machine_delete(
    state: State<'_, DesktopState>,
    id: String,
) -> Result<(), String> {
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
pub(crate) fn policy_open_folder(state: State<'_, DesktopState>) -> Result<(), String> {
    let folder = std::path::absolute(policy::cache_root(state.config_dir.as_deref()))
        .map_err(|error| format!("解析策略缓存目录失败：{error}"))?;
    std::fs::create_dir_all(&folder).map_err(|error| error.to_string())?;
    #[cfg(target_os = "windows")]
    let mut command = ProcessCommand::new("explorer");
    #[cfg(target_os = "macos")]
    let mut command = ProcessCommand::new("open");
    #[cfg(all(unix, not(target_os = "macos")))]
    let mut command = ProcessCommand::new("xdg-open");
    command
        .arg(folder)
        .spawn()
        .map(|_| ())
        .map_err(|error| format!("打开策略缓存目录失败：{error}"))
}

#[tauri::command]
#[specta::specta]
pub(crate) fn open_logs(state: State<'_, DesktopState>) -> Result<(), String> {
    let folder = state
        .config_dir
        .as_deref()
        .and_then(std::path::Path::parent)
        .map(|root| root.join("logs"))
        .unwrap_or_else(|| std::env::temp_dir().join("microduck-vision").join("logs"));
    std::fs::create_dir_all(&folder).map_err(|error| error.to_string())?;
    #[cfg(target_os = "windows")]
    let mut command = ProcessCommand::new("explorer");
    #[cfg(target_os = "macos")]
    let mut command = ProcessCommand::new("open");
    #[cfg(all(unix, not(target_os = "macos")))]
    let mut command = ProcessCommand::new("xdg-open");
    command
        .arg(folder)
        .spawn()
        .map(|_| ())
        .map_err(|error| format!("打开日志目录失败：{error}"))
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
    if let Some(error) = binding_conflict_error(&machine.joint_bindings) {
        return Err(error);
    }
    let template = BodyTemplate::official().map_err(|e| e.to_string())?;
    let model = model(&machine.servo_model);
    let policy_path = if let Some(policy_id) = policy_id {
        let selected = policy::list(state.config_dir.as_deref())?
            .into_iter()
            .find(|entry| entry.id == policy_id && entry.cached && entry.selected)
            .ok_or_else(|| "selected ONNX policy is not downloaded".to_owned())?;
        Some(
            policy::selected_path(state.config_dir.as_deref())
                .ok_or_else(|| format!("selected ONNX policy is unavailable: {}", selected.id))?,
        )
    } else {
        None
    };
    let physical = if let Some(requested_id) = requested_physical_id {
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
        Some(physical)
    } else {
        state
            .physical_machines
            .lock()
            .map_err(|e| e.to_string())?
            .iter()
            .find(|physical| {
                physical.virtual_machine_id.as_deref() == Some(&machine.id) && physical.connected
            })
            .cloned()
    };
    let physical_id = physical.as_ref().map(|machine| machine.id.clone());
    let physical_log_context = physical
        .as_ref()
        .map(|machine| (machine.id.clone(), machine.name.clone()));
    let bound_servo_ids = if let Some(physical) = physical.as_ref() {
        let bindings = if !machine.joint_bindings.is_empty() {
            machine.joint_bindings.clone()
        } else {
            official_servo_bindings(&physical.census)
        };
        let mut ids = Vec::with_capacity(template.actuators.len());
        for slot in &template.actuators {
            let binding = bindings
                .iter()
                .find(|binding| binding.slot_id == slot.id)
                .ok_or_else(|| format!("实机尚未绑定肢体槽位 {}", slot.id))?;
            if ids.contains(&binding.physical_id) {
                return Err(format!("实机舵机 {} 被重复绑定", binding.physical_id));
            }
            ids.push(binding.physical_id);
        }
        ids
    } else {
        if servo_ids.len() != template.actuators.len() {
            return Err(format!("expected {} servo ids", template.actuators.len()));
        }
        if machine.joint_bindings.is_empty() {
            servo_ids.clone()
        } else {
            let mut ids = Vec::with_capacity(template.actuators.len());
            for (index, slot) in template.actuators.iter().enumerate() {
                let id = machine
                    .joint_bindings
                    .iter()
                    .find(|binding| binding.slot_id == slot.id)
                    .map(|binding| binding.physical_id)
                    .unwrap_or(servo_ids[index]);
                if ids.contains(&id) {
                    return Err(format!("模拟机舵机 {} 被重复绑定", id));
                }
                ids.push(id);
            }
            ids
        }
    };
    let saved_bindings = if !machine.joint_bindings.is_empty() {
        machine.joint_bindings.clone()
    } else if let Some(physical_machine) = physical.as_ref() {
        official_servo_bindings(&physical_machine.census)
    } else {
        Vec::new()
    };
    let calibration = Calibration {
        joints: template
            .actuators
            .iter()
            .zip(bound_servo_ids.iter().copied())
            .map(|(slot, id)| {
                let saved = saved_bindings
                    .iter()
                    .find(|binding| binding.slot_id == slot.id && binding.physical_id == id);
                JointCalibration {
                    slot_id: slot.id.clone(),
                    physical_id: id,
                    direction: saved.map(|binding| binding.direction).unwrap_or(1),
                    zero_offset_rad: saved.map(|binding| binding.zero_offset_rad).unwrap_or(0.0),
                    min_rad: saved.and_then(|binding| binding.min_rad),
                    max_rad: saved.and_then(|binding| binding.max_rad),
                }
            })
            .collect(),
    };
    let binding_summary = calibration
        .joints
        .iter()
        .map(|joint| {
            format!(
                "{}->{} dir={} offset={}",
                joint.slot_id, joint.physical_id, joint.direction, joint.zero_offset_rad
            )
        })
        .collect::<Vec<_>>()
        .join(" | ");
    let binding = LoopBinding {
        body: BodyRuntimeView::new(template.clone()).map_err(|e| e.to_string())?,
        servos: ServoBinding::all_slots(&template, model),
        calibration: calibration.clone(),
    };
    let has_physical_driver = physical_id.is_some();
    let log_machine_id = physical_log_context
        .as_ref()
        .map(|(machine_id, _)| machine_id.as_str())
        .unwrap_or(machine.id.as_str());
    let log_machine_name = physical_log_context
        .as_ref()
        .map(|(_, machine_name)| machine_name.as_str())
        .unwrap_or(machine.name.as_str());
    let log_handle = begin_control_loop(
        state.config_dir.as_deref(),
        log_machine_id,
        log_machine_name,
        &binding_summary,
    );
    let driver: Box<dyn ServoLoopDriver> = if let Some(physical_id) = physical_id {
        let bus = state
            .connected_buses
            .lock()
            .map_err(|e| e.to_string())?
            .remove(&physical_id)
            .ok_or_else(|| "physical machine has no connected bus".to_string())?;
        let driver = bus.into_driver(model, bound_servo_ids.clone())?;
        if let Some(handle) = log_handle {
            Box::new(LoggedServoDriver::new(
                driver,
                handle,
                log_machine_id.to_owned(),
            ))
        } else {
            driver
        }
    } else {
        let driver: Box<dyn ServoLoopDriver> = Box::new(MemoryDriver {
            positions: vec![0.0; template.joints.len()],
            torque: false,
        });
        if let Some(handle) = log_handle {
            Box::new(LoggedServoDriver::new(
                driver,
                handle,
                log_machine_id.to_owned(),
            ))
        } else {
            driver
        }
    };
    let mut body_loop =
        PolicyLoop::open(binding, driver, LoopConfig::default()).map_err(|e| e.to_string())?;
    if let Some(path) = policy_path {
        append_control_event(
            state.config_dir.as_deref(),
            format!("policy_selected path={:?}", path),
        );
        body_loop.select_policy(path).map_err(|e| e.to_string())?;
    } else {
        append_control_event(state.config_dir.as_deref(), "policy_selected none");
    }
    let mut motion = state.motion.lock().map_err(|e| e.to_string())?;
    motion.reset();
    // Servo buses expose joint feedback only.  Keep a command estimate for simulation, but do
    // not present it as physical odometry until a separate sensor provider is connected.
    motion.set_estimate_from_command(!has_physical_driver);
    drop(motion);
    let mut status = FrontendLoopStatus::from(body_loop.status());
    status.world = state.motion.lock().map_err(|e| e.to_string())?.world();
    *state.loop_state.lock().map_err(|e| e.to_string())? = Some(body_loop);
    Ok(status)
}

#[tauri::command]
#[specta::specta]
pub(crate) fn loop_command(
    state: State<'_, DesktopState>,
    targets: Vec<f64>,
) -> Result<(), String> {
    let mut guard = state.loop_state.lock().map_err(|e| e.to_string())?;
    let loop_state = guard.as_mut().ok_or_else(|| "loop is closed".to_string())?;
    let session = loop_state.status().session;
    append_control_event(
        state.config_dir.as_deref(),
        format!("loop_command session={} body_targets={targets:?}", session),
    );
    loop_state
        .set_command(loop_body::BodyCommand { session, targets })
        .map_err(|e| e.to_string())
}

#[tauri::command]
#[specta::specta]
pub(crate) fn loop_policy_command(
    state: State<'_, DesktopState>,
    input: LoopCommandInput,
) -> Result<(), String> {
    let mut guard = state.loop_state.lock().map_err(|e| e.to_string())?;
    let loop_state = guard.as_mut().ok_or_else(|| "loop is closed".to_string())?;
    loop_state.set_policy_command(Command {
        twist: input.twist,
        head: input.head,
        body: duck_control::obs::BodyPose {
            z: input.body[0],
            roll: input.body[1],
            pitch: input.body[2],
        },
    });
    state
        .motion
        .lock()
        .map_err(|e| e.to_string())?
        .set_command(input.twist);
    Ok(())
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
    append_control_event(
        state.config_dir.as_deref(),
        format!("loop_action action={action:?}"),
    );
    loop_state.send(action).map_err(|e| e.to_string())?;
    if matches!(
        action,
        BodyLoopAction::Power(false)
            | BodyLoopAction::Drive(false)
            | BodyLoopAction::Stop
            | BodyLoopAction::Estop
    ) {
        state
            .motion
            .lock()
            .map_err(|e| e.to_string())?
            .set_command([0.0; 3]);
    }
    let mut status = FrontendLoopStatus::from(loop_state.status());
    status.world = state.motion.lock().map_err(|e| e.to_string())?.world();
    Ok(status)
}

#[tauri::command]
#[specta::specta]
pub(crate) fn loop_tick(state: State<'_, DesktopState>) -> Result<FrontendLoopStatus, String> {
    let mut guard = state.loop_state.lock().map_err(|e| e.to_string())?;
    let loop_state = guard.as_mut().ok_or_else(|| "loop is closed".to_string())?;
    loop_state
        .tick(LoopConfig::default().period)
        .map_err(|e| e.to_string())?;
    if let Some(trace) = loop_state.policy_trace() {
        append_policy_trace(state.config_dir.as_deref(), loop_state.status().tick, trace);
    }
    let driving = loop_state.status().phase == loop_body::LoopPhase::Driving;
    let world = state
        .motion
        .lock()
        .map_err(|e| e.to_string())?
        .tick(driving, LoopConfig::default().period);
    let mut status = FrontendLoopStatus::from(loop_state.status());
    status.world = world;
    Ok(status)
}

#[tauri::command]
#[specta::specta]
pub(crate) fn loop_status(
    state: State<'_, DesktopState>,
) -> Result<Option<FrontendLoopStatus>, String> {
    let mut status = state
        .loop_state
        .lock()
        .map_err(|e| e.to_string())?
        .as_ref()
        .map(|body_loop| FrontendLoopStatus::from(body_loop.status()));
    if let Some(status) = &mut status {
        status.world = state.motion.lock().map_err(|e| e.to_string())?.world();
    }
    Ok(status)
}

#[tauri::command]
#[specta::specta]
pub(crate) fn loop_close(state: State<'_, DesktopState>) -> Result<(), String> {
    append_control_event(state.config_dir.as_deref(), "loop_close");
    *state.loop_state.lock().map_err(|e| e.to_string())? = None;
    state.motion.lock().map_err(|e| e.to_string())?.reset();
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
    let preview = PreviewPolicyLoop::open(
        model(&machine.servo_model),
        state.config_dir.as_deref(),
        &machine.id,
        &machine.name,
    )?;
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
    preview.select_policy(&path, selected.command)?;
    Ok(preview.status())
}

#[tauri::command]
#[specta::specta]
pub(crate) fn preview_loop_command(
    state: State<'_, DesktopState>,
    input: PreviewInput,
) -> Result<FrontendLoopStatus, String> {
    let mut guard = state.preview_loop.lock().map_err(|e| e.to_string())?;
    let preview = guard
        .as_mut()
        .ok_or_else(|| "preview loop is closed".to_owned())?;
    preview.set_input(input);
    Ok(preview.status())
}

#[tauri::command]
#[specta::specta]
pub(crate) fn preview_loop_reset_world(
    state: State<'_, DesktopState>,
) -> Result<FrontendLoopStatus, String> {
    let mut guard = state.preview_loop.lock().map_err(|e| e.to_string())?;
    let preview = guard
        .as_mut()
        .ok_or_else(|| "preview loop is closed".to_owned())?;
    preview.reset_world();
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
    append_control_event(state.config_dir.as_deref(), "preview_loop_close");
    *state.preview_loop.lock().map_err(|e| e.to_string())? = None;
    Ok(())
}

#[cfg(test)]
mod register_table_tests {
    use super::{
        binding_conflict_error, register_is_writable, register_table, update_servo_id_cache,
    };
    use crate::physical;
    use crate::types::{PhysicalMachine, ServoIdentity, ServoJointBinding};
    use servo::ServoModel;

    #[test]
    fn hd_table_contains_one_version_pair_and_default_rows_are_read_only() {
        let table = register_table(ServoModel::Hd1910);
        assert_eq!(table.iter().filter(|row| row.addr == 3).count(), 1);
        assert_eq!(table.iter().filter(|row| row.addr == 4).count(), 1);
        assert_eq!(
            table.iter().find(|row| row.addr == 3).map(|row| row.name),
            Some("model")
        );
        assert_eq!(
            table.iter().find(|row| row.addr == 4).map(|row| row.name),
            Some("model_number")
        );
        assert_eq!(
            table.iter().find(|row| row.addr == 44).map(|row| row.name),
            Some("pwm_open_speed")
        );
        assert_eq!(
            table.iter().find(|row| row.addr == 24).map(|row| row.size),
            Some(1)
        );
        assert_eq!(
            table.iter().find(|row| row.addr == 25).map(|row| row.size),
            Some(1)
        );
        assert_eq!(
            table.iter().find(|row| row.addr == 44).map(|row| row.size),
            Some(2)
        );
        for address in 80..=86 {
            let row = table
                .iter()
                .find(|row| row.addr == address)
                .expect("HD1910 default row");
            assert!(row.is_default);
            assert!(!register_is_writable(*row));
        }
    }

    #[test]
    fn changing_a_servo_id_updates_census_and_online_ids() {
        let mut machine = PhysicalMachine {
            id: "physical-1".into(),
            name: "duck".into(),
            usb_port: "COM1".into(),
            machine_code: String::new(),
            census: vec![
                ServoIdentity {
                    servo_id: 5,
                    model: "hd1910".into(),
                },
                ServoIdentity {
                    servo_id: 2,
                    model: "hd1910".into(),
                },
            ],
            reconnect_hint: None,
            virtual_machine_id: Some("virtual-1".into()),
            online_servo_ids: vec![5, 2],
            connected: true,
            last_seen: None,
            torque_enabled: false,
        };

        update_servo_id_cache(&mut machine, 5, 7).unwrap();
        assert_eq!(
            machine
                .census
                .iter()
                .map(|servo| servo.servo_id)
                .collect::<Vec<_>>(),
            vec![2, 7]
        );
        assert_eq!(machine.online_servo_ids, vec![7, 2]);
        assert_eq!(
            machine.machine_code,
            physical::census_md5(&machine.census).unwrap()
        );
    }

    #[test]
    fn changing_a_servo_id_rejects_missing_or_duplicate_ids_without_mutating() {
        let mut machine = PhysicalMachine {
            id: "physical-1".into(),
            name: "duck".into(),
            usb_port: "COM1".into(),
            machine_code: String::new(),
            census: vec![
                ServoIdentity {
                    servo_id: 1,
                    model: "hd1910".into(),
                },
                ServoIdentity {
                    servo_id: 2,
                    model: "hd1910".into(),
                },
            ],
            reconnect_hint: None,
            virtual_machine_id: None,
            online_servo_ids: vec![1, 2],
            connected: true,
            last_seen: None,
            torque_enabled: false,
        };
        let before = machine.clone();
        assert!(update_servo_id_cache(&mut machine, 9, 3).is_err());
        assert_eq!(machine.census, before.census);
        assert_eq!(machine.online_servo_ids, before.online_servo_ids);
        assert_eq!(machine.machine_code, before.machine_code);
        assert!(update_servo_id_cache(&mut machine, 1, 2).is_err());
        assert_eq!(machine.census, before.census);
        assert_eq!(machine.online_servo_ids, before.online_servo_ids);
        assert_eq!(machine.machine_code, before.machine_code);
    }

    #[test]
    fn duplicate_virtual_bindings_are_rejected_before_hardware_use() {
        let bindings = vec![
            ServoJointBinding {
                slot_id: "left_hip_yaw".into(),
                physical_id: 42,
                direction: 1,
                zero_offset_rad: 0.0,
                min_rad: None,
                max_rad: None,
            },
            ServoJointBinding {
                slot_id: "right_hip_yaw".into(),
                physical_id: 42,
                direction: 1,
                zero_offset_rad: 0.0,
                min_rad: None,
                max_rad: None,
            },
        ];
        assert_eq!(
            binding_conflict_error(&bindings).as_deref(),
            Some("绑定冲突：42 → left_hip_yaw,right_hip_yaw")
        );
    }
}

#[cfg(test)]
mod joint_binding_tests {
    use super::{ServoIdentity, official_servo_bindings};

    #[test]
    fn official_census_is_ordered_by_body_slots() {
        let census = [
            ServoIdentity {
                servo_id: 14,
                model: "hd1910".into(),
            },
            ServoIdentity {
                servo_id: 20,
                model: "hd1910".into(),
            },
            ServoIdentity {
                servo_id: 10,
                model: "hd1910".into(),
            },
        ];
        let bindings = official_servo_bindings(&census);
        assert_eq!(
            bindings
                .iter()
                .map(|binding| binding.slot_id.as_str())
                .collect::<Vec<_>>(),
            ["left_hip_yaw", "right_hip_yaw", "right_ankle",]
        );
        assert_eq!(
            bindings
                .iter()
                .map(|binding| binding.physical_id)
                .collect::<Vec<_>>(),
            [20, 10, 14]
        );
    }
}
