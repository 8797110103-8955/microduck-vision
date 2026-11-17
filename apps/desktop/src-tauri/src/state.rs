//! Client-owned runtime state and TOML persistence.

use std::collections::HashMap;
use std::path::PathBuf;
use std::process::Child;
use std::sync::{Arc, Mutex};

use duck_body::ServoModelKey;
use loop_body::PolicyLoop;
use serde::{Deserialize, Serialize};
use tauri::{AppHandle, Emitter, Wry};

use crate::control::ConnectedBus;
use crate::helpers::default_simulation;
use crate::motion::MotionTracker;
use crate::physical;
use crate::simulation::PreviewPolicyLoop;
use crate::types::{
    ActionKind, ActionRecord, ActionSource, PhysicalMachine, ReconnectHint, ServoIdentity,
    ServoJointDirection, TrainingProgress, VirtualMachine,
};

#[derive(Debug, Clone, Default, Deserialize, Serialize)]
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
struct PersistedRobotTable {
    #[serde(rename = "robot", default)]
    physical_machines: Vec<PersistedPhysicalMachine>,
}

#[derive(Debug, Clone, Deserialize, Serialize)]
#[serde(rename_all = "snake_case")]
struct PersistedPhysicalMachine {
    id: String,
    name: String,
    census: Vec<PersistedServoIdentity>,
    reconnect_hint: Option<PersistedReconnectHint>,
    virtual_machine_id: Option<String>,
    #[serde(default)]
    joint_directions: Vec<ServoJointDirection>,
}

#[derive(Debug, Clone, Deserialize, Serialize)]
#[serde(rename_all = "snake_case")]
struct PersistedServoIdentity {
    servo_id: u8,
    model: String,
}

#[derive(Debug, Clone, Deserialize, Serialize)]
#[serde(rename_all = "snake_case")]
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
        let machine_code = physical::census_md5(&census).unwrap_or_default();
        let name = if machine.name.trim().is_empty() {
            machine_code.clone()
        } else {
            machine.name
        };
        Self {
            id: machine.id,
            name,
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
        }
    }
}

pub(crate) struct DesktopState {
    pub(crate) config_dir: Option<PathBuf>,
    pub(crate) auto_mount_enabled: Mutex<bool>,
    pub(crate) auto_reconnect_enabled: Mutex<bool>,
    pub(crate) virtual_machines: Mutex<Vec<VirtualMachine>>,
    pub(crate) physical_machines: Mutex<Vec<PhysicalMachine>>,
    pub(crate) loop_state: Mutex<Option<PolicyLoop>>,
    pub(crate) motion: Mutex<MotionTracker>,
    /// The preview loop is always virtual and never owns a connected servo bus.
    pub(crate) preview_loop: Mutex<Option<PreviewPolicyLoop>>,
    pub(crate) connected_buses: Mutex<HashMap<String, ConnectedBus>>,
    pub(crate) actions: Mutex<Vec<ActionRecord>>,
    /// The optional local trainer is owned by the desktop process so its
    /// output can continue to reach the UI while the command itself returns.
    pub(crate) training_process: Mutex<Option<Arc<Mutex<Child>>>>,
    pub(crate) training_progress: Mutex<Option<TrainingProgress>>,
    pub(crate) training_stop_requested: Mutex<bool>,
    pub(crate) app_handle: Mutex<Option<AppHandle<Wry>>>,
}

impl Default for DesktopState {
    fn default() -> Self {
        Self::new(None)
    }
}

impl DesktopState {
    pub(crate) fn new(config_dir: Option<PathBuf>) -> Self {
        let persisted_machines = config_dir.as_ref().and_then(|dir| {
            std::fs::read_to_string(dir.join("machines.toml"))
                .ok()
                .and_then(|text| toml::from_str::<PersistedMachineTable>(&text).ok())
        });
        let persisted_robots = config_dir.as_ref().and_then(|dir| {
            std::fs::read_to_string(dir.join("robots.toml"))
                .ok()
                .and_then(|text| toml::from_str::<PersistedRobotTable>(&text).ok())
        });
        let mut virtual_machines = persisted_machines
            .as_ref()
            .map(|config| config.virtual_machines.clone())
            .unwrap_or_else(|| {
                vec![VirtualMachine {
                    id: "official-hd1910".into(),
                    name: "机器 0".into(),
                    body_id: "official".into(),
                    servo_model: ServoModelKey::Hd1910,
                    simulation: default_simulation(ServoModelKey::Hd1910),
                    startup_selected: true,
                    referenced_by: Vec::new(),
                    joint_bindings: Vec::new(),
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
        Self::rebuild_references(&mut virtual_machines, &physical_machines);
        Self {
            config_dir,
            auto_mount_enabled: Mutex::new(auto_mount_enabled),
            auto_reconnect_enabled: Mutex::new(auto_reconnect_enabled),
            virtual_machines: Mutex::new(virtual_machines),
            physical_machines: Mutex::new(physical_machines),
            loop_state: Mutex::new(None),
            motion: Mutex::new(MotionTracker::default()),
            preview_loop: Mutex::new(None),
            connected_buses: Mutex::new(HashMap::new()),
            actions: Mutex::new(vec![
                ActionRecord {
                    id: "stand".into(),
                    name: "站立".into(),
                    kind: ActionKind::Pose,
                    updated_at: "内置".into(),
                    source: ActionSource::Builtin,
                },
                ActionRecord {
                    id: "sit".into(),
                    name: "坐下".into(),
                    kind: ActionKind::Pose,
                    updated_at: "内置".into(),
                    source: ActionSource::Builtin,
                },
            ]),
            training_process: Mutex::new(None),
            training_progress: Mutex::new(None),
            training_stop_requested: Mutex::new(false),
            app_handle: Mutex::new(None),
        }
    }

    pub(crate) fn set_app_handle(&self, app: AppHandle<Wry>) {
        if let Ok(mut current) = self.app_handle.lock() {
            *current = Some(app);
        }
    }

    pub(crate) fn emit_joint_range_states(&self, states: &[crate::types::JointRangeState]) {
        if let Ok(handle) = self.app_handle.lock()
            && let Some(app) = handle.as_ref()
        {
            let _ = app.emit("joint-range-state", states);
        }
    }

    pub(crate) fn load(config_dir: PathBuf) -> Self {
        Self::new(Some(config_dir.join("config")))
    }

    fn rebuild_references(
        virtual_machines: &mut [VirtualMachine],
        physical_machines: &[PhysicalMachine],
    ) {
        for machine in virtual_machines.iter_mut() {
            machine.referenced_by.clear();
        }
        for physical in physical_machines {
            if let Some(virtual_id) = &physical.virtual_machine_id
                && let Some(virtual_machine) = virtual_machines
                    .iter_mut()
                    .find(|machine| &machine.id == virtual_id)
            {
                virtual_machine.referenced_by.push(physical.id.clone());
            }
        }
    }

    pub(crate) fn persist(&self) -> Result<(), String> {
        let Some(dir) = &self.config_dir else {
            return Ok(());
        };
        let virtual_machines = self
            .virtual_machines
            .lock()
            .map_err(|error| error.to_string())?
            .clone();
        let physical_machines = self
            .physical_machines
            .lock()
            .map_err(|error| error.to_string())?
            .clone();
        let auto_mount = *self
            .auto_mount_enabled
            .lock()
            .map_err(|error| error.to_string())?;
        let auto_reconnect = *self
            .auto_reconnect_enabled
            .lock()
            .map_err(|error| error.to_string())?;
        std::fs::create_dir_all(dir).map_err(|error| error.to_string())?;
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
        let robots = toml::to_string_pretty(&PersistedRobotTable {
            physical_machines: physical_machines.iter().map(Into::into).collect(),
        })
        .map_err(|error| error.to_string())?;
        std::fs::write(dir.join("machines.toml"), machines).map_err(|error| error.to_string())?;
        std::fs::write(dir.join("robots.toml"), robots).map_err(|error| error.to_string())
    }
}

#[cfg(test)]
mod tests {
    use super::DesktopState;
    use crate::types::{PhysicalMachine, ReconnectHint, ServoIdentity, ServoJointDirection};

    fn servo(id: u8, model: &str) -> ServoIdentity {
        ServoIdentity {
            servo_id: id,
            model: model.into(),
        }
    }

    #[test]
    fn system_config_round_trips_physical_weak_reference() {
        let config_dir = std::env::temp_dir().join(format!(
            "microduck-vision-system-config-{}",
            std::process::id()
        ));
        let _ = std::fs::remove_dir_all(&config_dir);
        let state = DesktopState::new(Some(config_dir.clone()));
        state
            .physical_machines
            .lock()
            .unwrap()
            .push(PhysicalMachine {
                id: "physical-1".into(),
                name: "鸭子".into(),
                usb_port: "COM7".into(),
                machine_code: "census-md5".into(),
                census: vec![servo(1, "hd1910")],
                reconnect_hint: Some(ReconnectHint {
                    port_name: "COM7".into(),
                    vid: Some(1),
                    pid: Some(2),
                    adapter_serial: Some("adapter-1".into()),
                    product: Some("USB bridge".into()),
                }),
                virtual_machine_id: Some("official-hd1910".into()),
                online_servo_ids: vec![1],
                connected: true,
                last_seen: Some("刚刚".into()),
                torque_enabled: false,
                joint_directions: vec![ServoJointDirection {
                    slot_id: "left_hip_yaw".into(),
                    direction: -1,
                }],
            });
        state.virtual_machines.lock().unwrap()[0]
            .simulation
            .world_position = [1.25, -0.4, 2.0];
        state.persist().unwrap();

        let loaded = DesktopState::new(Some(config_dir.clone()));
        let physical = &loaded.physical_machines.lock().unwrap()[0];
        assert!(!physical.connected);
        assert_eq!(
            physical
                .reconnect_hint
                .as_ref()
                .unwrap()
                .adapter_serial
                .as_deref(),
            Some("adapter-1")
        );
        assert_eq!(
            physical.virtual_machine_id.as_deref(),
            Some("official-hd1910")
        );
        assert_eq!(physical.joint_directions[0].direction, -1);
        assert_eq!(
            loaded.virtual_machines.lock().unwrap()[0].referenced_by,
            vec!["physical-1"]
        );
        assert_eq!(
            loaded.virtual_machines.lock().unwrap()[0]
                .simulation
                .world_position,
            [1.25, -0.4, 2.0]
        );
        assert!(config_dir.join("machines.toml").is_file());
        let machines_text = std::fs::read_to_string(config_dir.join("machines.toml")).unwrap();
        assert!(machines_text.contains("auto_mount = true"));
        assert!(machines_text.contains("auto_reconnect = true"));
        let robots_text = std::fs::read_to_string(config_dir.join("robots.toml")).unwrap();
        assert!(robots_text.contains("reconnect_hint"));
        assert!(robots_text.contains("joint_directions"));
        assert!(!robots_text.contains("machine_code"));
        assert!(!robots_text.contains("connected"));
        let _ = std::fs::remove_dir_all(config_dir);
    }
}
