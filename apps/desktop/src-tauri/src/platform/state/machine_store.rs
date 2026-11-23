//! Persisted machine settings and recovery records.

use std::path::Path;

use duck_body::JointCalibration;
use serde::{Deserialize, Serialize};

use crate::platform::helpers::default_simulation;
use crate::runtime::adapters::encoder_calibration::EncoderSnapshot;
use crate::runtime::adapters::physical_identity;
use crate::types::{
    ExecutionKindKey, ModelKindKey, PhysicalMachine, ReconnectHint, ServoIdentity,
    ServoJointCalibration, ServoJointDirection, ServoModelKey, VirtualMachine,
};

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub(crate) struct PendingCalibration {
    pub(crate) physical_id: String,
    pub(crate) operation_id: String,
    pub(crate) joint: JointCalibration,
    pub(crate) before: EncoderSnapshot,
    pub(crate) body_angle_rad: f64,
    pub(crate) new_zero_offset_rad: Option<f64>,
    pub(crate) after: Option<EncoderSnapshot>,
}

fn read_config<T: serde::de::DeserializeOwned>(
    directory: Option<&Path>,
    name: &str,
) -> Result<Option<T>, String> {
    let Some(directory) = directory else {
        return Ok(None);
    };
    let path = directory.join(name);
    match std::fs::read_to_string(&path) {
        Ok(text) => toml::from_str(&text)
            .map(Some)
            .map_err(|error| format!("{}: {error}", path.display())),
        Err(error) if error.kind() == std::io::ErrorKind::NotFound => Ok(None),
        Err(error) => Err(format!("{}: {error}", path.display())),
    }
}

pub(crate) fn load_pending_calibrations(
    config: Option<&Path>,
) -> Result<Vec<PendingCalibration>, String> {
    let Some(config) = config else {
        return Ok(Vec::new());
    };
    match std::fs::read(config.join("encoder-calibration.json")) {
        Ok(bytes) => serde_json::from_slice(&bytes)
            .map_err(|error| format!("read encoder calibration recovery: {error}")),
        Err(error) if error.kind() == std::io::ErrorKind::NotFound => Ok(Vec::new()),
        Err(error) => Err(format!("read encoder calibration recovery: {error}")),
    }
}

#[derive(Debug, Clone, Default, Deserialize, Serialize)]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
struct PersistedMachineTable {
    #[serde(default)]
    default: Option<String>,
    #[serde(default = "default_auto_mount")]
    auto_mount: bool,
    #[serde(default = "default_auto_reconnect")]
    auto_reconnect: bool,
    #[serde(rename = "machine", default)]
    virtual_machines: Vec<VirtualMachine>,
}

fn default_auto_mount() -> bool {
    true
}

fn default_auto_reconnect() -> bool {
    true
}

#[derive(Debug, Clone, Default, Deserialize, Serialize)]
pub(crate) struct PersistedRobotTable {
    #[serde(rename = "robot", default)]
    pub(crate) physical_machines: Vec<PersistedPhysicalMachine>,
}

#[derive(Debug, Clone, Deserialize, Serialize)]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
pub(crate) struct PersistedPhysicalMachine {
    id: String,
    name: String,
    execution_kind: ExecutionKindKey,
    model_kind: Option<ModelKindKey>,
    servo_model: Option<ServoModelKey>,
    census: Vec<PersistedServoIdentity>,
    reconnect_hint: Option<PersistedReconnectHint>,
    virtual_machine_id: Option<String>,
    #[serde(default)]
    joint_directions: Vec<ServoJointDirection>,
    #[serde(default)]
    joint_bindings: Vec<ServoJointCalibration>,
}

#[derive(Debug, Clone, Deserialize, Serialize)]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
struct PersistedServoIdentity {
    servo_id: u8,
    model: String,
}

#[derive(Debug, Clone, Deserialize, Serialize)]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
struct PersistedReconnectHint {
    port_name: String,
    vid: Option<u16>,
    pid: Option<u16>,
    adapter_serial: Option<String>,
    product: Option<String>,
}

impl From<&PhysicalMachine> for PersistedPhysicalMachine {
    fn from(machine: &PhysicalMachine) -> Self {
        Self {
            id: machine.id.clone(),
            name: machine.name.clone(),
            execution_kind: machine.execution_kind,
            model_kind: machine.model_kind,
            servo_model: machine.servo_model,
            census: machine
                .census
                .iter()
                .map(|servo| PersistedServoIdentity {
                    servo_id: servo.servo_id,
                    model: servo.model.clone(),
                })
                .collect(),
            reconnect_hint: machine
                .reconnect_hint
                .as_ref()
                .map(|hint| PersistedReconnectHint {
                    port_name: hint.port_name.clone(),
                    vid: hint.vid,
                    pid: hint.pid,
                    adapter_serial: hint.adapter_serial.clone(),
                    product: hint.product.clone(),
                }),
            virtual_machine_id: machine.virtual_machine_id.clone(),
            joint_directions: machine.joint_directions.clone(),
            joint_bindings: machine.joint_bindings.clone(),
        }
    }
}

impl From<PersistedPhysicalMachine> for PhysicalMachine {
    fn from(machine: PersistedPhysicalMachine) -> Self {
        let reconnect_hint = machine.reconnect_hint.map(|hint| ReconnectHint {
            port_name: hint.port_name,
            vid: hint.vid,
            pid: hint.pid,
            adapter_serial: hint.adapter_serial,
            product: hint.product,
        });
        let census = machine
            .census
            .into_iter()
            .map(|servo| ServoIdentity {
                servo_id: servo.servo_id,
                model: servo.model,
            })
            .collect::<Vec<_>>();
        let machine_code = physical_identity::census_md5(&census).unwrap_or_default();
        let name = if machine.name.trim().is_empty() {
            machine_code.clone()
        } else {
            machine.name
        };
        Self {
            id: machine.id,
            name,
            execution_kind: machine.execution_kind,
            model_kind: machine.model_kind,
            servo_model: machine.servo_model,
            usb_port: reconnect_hint
                .as_ref()
                .map(|hint| hint.port_name.clone())
                .unwrap_or_default(),
            machine_code,
            census,
            reconnect_hint,
            virtual_machine_id: machine.virtual_machine_id,
            online_servo_ids: Vec::new(),
            connected: false,
            last_seen: None,
            torque_enabled: false,
            joint_directions: machine
                .joint_directions
                .into_iter()
                .filter(|binding| matches!(binding.direction, -1 | 1))
                .collect(),
            joint_bindings: machine.joint_bindings,
        }
    }
}

pub(crate) struct LoadedMachines {
    pub(crate) virtual_machines: Vec<VirtualMachine>,
    pub(crate) physical_machines: Vec<PhysicalMachine>,
    pub(crate) auto_mount_enabled: bool,
    pub(crate) auto_reconnect_enabled: bool,
}

pub(crate) fn load_machines(config_dir: Option<&Path>) -> Result<LoadedMachines, String> {
    let persisted_machines = read_config::<PersistedMachineTable>(config_dir, "machines.toml")?;
    let persisted_robots = read_config::<PersistedRobotTable>(config_dir, "robots.toml")?;
    let mut virtual_machines = persisted_machines
        .as_ref()
        .map(|config| config.virtual_machines.clone())
        .unwrap_or_else(|| {
            vec![VirtualMachine {
                id: crate::platform::helpers::new_machine_id("virtual"),
                name: "机器 0".into(),
                body_id: "official".into(),
                execution_kind: ExecutionKindKey::Simulation,
                model_kind: ModelKindKey::Hd1910Compatible,
                servo_model: ServoModelKey::Hd1910,
                simulation: default_simulation(ServoModelKey::Hd1910),
                startup_selected: true,
                referenced_by: Vec::new(),
                joint_bindings: crate::platform::helpers::default_joint_bindings(),
            }]
        });
    let auto_mount_enabled = persisted_machines
        .as_ref()
        .map(|config| config.auto_mount)
        .unwrap_or_else(default_auto_mount);
    let auto_reconnect_enabled = persisted_machines
        .as_ref()
        .map(|config| config.auto_reconnect)
        .unwrap_or_else(default_auto_reconnect);
    if let Some(default) = persisted_machines
        .as_ref()
        .and_then(|config| config.default.clone())
    {
        for machine in &mut virtual_machines {
            machine.startup_selected = machine.id == default;
        }
    }
    let physical_machines: Vec<PhysicalMachine> = persisted_robots
        .map(|config| config.physical_machines)
        .unwrap_or_default()
        .into_iter()
        .map(Into::into)
        .collect();
    Ok(LoadedMachines {
        virtual_machines,
        physical_machines,
        auto_mount_enabled,
        auto_reconnect_enabled,
    })
}

pub(crate) fn save_virtual_machines(
    dir: &Path,
    virtual_machines: Vec<VirtualMachine>,
    auto_mount: bool,
    auto_reconnect: bool,
) -> Result<(), String> {
    std::fs::create_dir_all(dir)
        .map_err(|error| format!("create configuration directory {}: {error}", dir.display()))?;
    let default = virtual_machines
        .iter()
        .find(|machine| machine.startup_selected)
        .map(|machine| machine.id.clone());
    let machines = toml::to_string_pretty(&PersistedMachineTable {
        default,
        auto_mount,
        auto_reconnect,
        virtual_machines,
    })
    .map_err(|error| error.to_string())?;
    let path = dir.join("machines.toml");
    std::fs::write(&path, machines).map_err(|error| {
        format!(
            "write virtual machine configuration {}: {error}",
            path.display()
        )
    })
}

pub(crate) fn save_physical_machines(
    dir: &Path,
    machines: &[PhysicalMachine],
) -> Result<(), String> {
    let robots = toml::to_string_pretty(&PersistedRobotTable {
        physical_machines: machines.iter().map(Into::into).collect(),
    })
    .map_err(|e| e.to_string())?;
    std::fs::create_dir_all(dir)
        .map_err(|error| format!("create configuration directory {}: {error}", dir.display()))?;
    let path = dir.join("robots.toml");
    let temporary = dir.join("robots.tmp.toml");
    let write = (|| -> std::io::Result<()> {
        use std::io::Write;
        let mut file = std::fs::File::create(&temporary)?;
        file.write_all(robots.as_bytes())?;
        file.sync_all()?;
        drop(file);
        std::fs::rename(&temporary, &path)
    })();
    write.map_err(|error| {
        format!(
            "write physical machine configuration {}: {error}",
            path.display()
        )
    })
}

pub(crate) fn save_pending_calibrations(
    config: &Path,
    entries: &[PendingCalibration],
) -> Result<(), String> {
    use std::io::Write;
    std::fs::create_dir_all(config).map_err(|e| e.to_string())?;
    let temporary = config.join("encoder-calibration.tmp.json");
    let mut file = std::fs::File::create(&temporary).map_err(|e| e.to_string())?;
    file.write_all(&serde_json::to_vec_pretty(entries).map_err(|e| e.to_string())?)
        .map_err(|e| e.to_string())?;
    file.sync_all().map_err(|e| e.to_string())?;
    drop(file);
    std::fs::rename(temporary, config.join("encoder-calibration.json"))
        .map_err(|e| e.to_string())?;
    Ok(())
}
