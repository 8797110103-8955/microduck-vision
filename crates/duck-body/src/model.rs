use std::collections::BTreeSet;
use std::path::PathBuf;

use serde::{Deserialize, Serialize};

use super::{BodyError, OFFICIAL_BODY_ID, OFFICIAL_BODY_NAME, OFFICIAL_MJCF};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum BodyStatus {
    Official,
    Experimental,
    Custom,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ActuatorCapability {
    PositionControl,
    PresentPosition,
    PresentVelocity,
    TorqueEnable,
    SetGain,
    PresentCurrent,
    PresentVoltage,
    PresentTemperature,
    Reboot,
    FastSyncRead,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct BodyJoint {
    pub name: String,
    pub order: u16,
    pub range_rad: [f64; 2],
    pub scene_joint: Option<String>,
    pub axis: [f64; 3],
    pub actuator: Option<String>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ActuatorSlot {
    pub id: String,
    pub joint: String,
    pub required_capabilities: Vec<ActuatorCapability>,
    pub suggested_servo_type: Option<String>,
    pub allowed_servo_types: Vec<String>,
    pub mounted_mass_g: Option<f64>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct BodyAssets {
    pub display_mjcf: PathBuf,
    pub physics_scene: Option<PathBuf>,
    pub physics_robot: Option<PathBuf>,
    pub mesh_dir: Option<PathBuf>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct BodyLink {
    pub name: String,
    pub mass_g: f64,
    pub inertia: Option<[f64; 6]>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct BodyPhysical {
    pub mass_g: Option<f64>,
    pub links: Vec<BodyLink>,
    pub actuators_mass_g: Option<f64>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct BodyControlContract {
    pub joint_order: Vec<String>,
    pub position_control: bool,
    pub action_size: usize,
    pub observation_joint_count: usize,
    pub policy_keys: Vec<String>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct BodyTemplate {
    pub id: String,
    pub name: String,
    pub version: u32,
    pub status: BodyStatus,
    pub joints: Vec<BodyJoint>,
    pub actuators: Vec<ActuatorSlot>,
    pub assets: BodyAssets,
    pub physical: BodyPhysical,
    pub control: BodyControlContract,
}

impl BodyTemplate {
    pub fn official() -> Result<Self, BodyError> {
        let model = kinematics::Model::parse(OFFICIAL_MJCF)
            .map_err(|error| BodyError::Mjcf(error.to_string()))?;
        let scene_names: BTreeSet<&str> = model.joint_names().collect();
        let mut joints = Vec::with_capacity(duck_ipc_proto::JOINT_NAMES.len());
        for (order, name) in duck_ipc_proto::JOINT_NAMES.iter().enumerate() {
            let scene_joint = if scene_names.contains(name) {
                Some((*name).to_owned())
            } else if *name == "mouth" {
                None
            } else {
                return Err(BodyError::MissingSceneJoint((*name).to_owned()));
            };
            let range_rad = match model.joint_index(name) {
                Some(index) => model
                    .joint_range(index)
                    .map(|(lo, hi)| [lo, hi])
                    .unwrap_or([-std::f64::consts::PI, std::f64::consts::PI]),
                None if *name == "mouth" => [-5.0_f64.to_radians(), 30.0_f64.to_radians()],
                None => return Err(BodyError::MissingSceneJoint((*name).to_owned())),
            };
            joints.push(BodyJoint {
                name: (*name).to_owned(),
                order: order as u16,
                range_rad,
                scene_joint,
                axis: [0.0, 0.0, 1.0],
                actuator: Some((*name).to_owned()),
            });
        }
        let required = vec![
            ActuatorCapability::PositionControl,
            ActuatorCapability::PresentPosition,
            ActuatorCapability::PresentVelocity,
            ActuatorCapability::TorqueEnable,
        ];
        let actuators = joints
            .iter()
            .map(|joint| ActuatorSlot {
                id: joint.name.clone(),
                joint: joint.name.clone(),
                required_capabilities: required.clone(),
                suggested_servo_type: Some("hd1910".to_owned()),
                allowed_servo_types: vec!["hd1910".to_owned(), "xl330".to_owned()],
                mounted_mass_g: None,
            })
            .collect();
        let physical = parse_body_physical(OFFICIAL_MJCF)?;
        let template = Self {
            id: OFFICIAL_BODY_ID.to_owned(),
            name: OFFICIAL_BODY_NAME.to_owned(),
            version: 1,
            status: BodyStatus::Official,
            joints,
            actuators,
            assets: BodyAssets {
                display_mjcf: PathBuf::from(env!("CARGO_MANIFEST_DIR")).join(
                    "../../vendor/microduck_rl/src/mjlab_microduck/robot/microduck/robot_walk.xml",
                ),
                physics_scene: Some(PathBuf::from(env!("CARGO_MANIFEST_DIR")).join(
                    "../../vendor/microduck_rl/src/mjlab_microduck/robot/microduck/scene.xml",
                )),
                physics_robot: Some(PathBuf::from(env!("CARGO_MANIFEST_DIR")).join(
                    "../../vendor/microduck_rl/src/mjlab_microduck/robot/microduck/robot_allcollisions.xml",
                )),
                mesh_dir: Some(PathBuf::from(env!("CARGO_MANIFEST_DIR")).join(
                    "../../vendor/microduck_rl/src/mjlab_microduck/robot/microduck/assets",
                )),
            },
            physical,
            control: BodyControlContract {
                joint_order: duck_ipc_proto::JOINT_NAMES
                    .iter()
                    .map(|s| (*s).to_owned())
                    .collect(),
                position_control: true,
                action_size: duck_ipc_proto::JOINT_NAMES.len() - 1,
                observation_joint_count: duck_ipc_proto::JOINT_NAMES.len(),
                policy_keys: vec!["alpha".to_owned()],
            },
        };
        template.validate()?;
        Ok(template)
    }

    pub fn from_mjcf(
        id: impl Into<String>,
        name: impl Into<String>,
        xml: &str,
        assets: BodyAssets,
    ) -> Result<Self, BodyError> {
        let model =
            kinematics::Model::parse(xml).map_err(|error| BodyError::Mjcf(error.to_string()))?;
        let joints: Vec<BodyJoint> = model
            .joint_names()
            .enumerate()
            .map(|(order, name)| BodyJoint {
                name: name.to_owned(),
                order: order as u16,
                range_rad: model
                    .joint_range(order)
                    .map(|(lo, hi)| [lo, hi])
                    .unwrap_or([-std::f64::consts::PI, std::f64::consts::PI]),
                scene_joint: Some(name.to_owned()),
                axis: [0.0, 0.0, 1.0],
                actuator: Some(name.to_owned()),
            })
            .collect();
        let actuators = joints
            .iter()
            .map(|joint| ActuatorSlot {
                id: joint.name.clone(),
                joint: joint.name.clone(),
                required_capabilities: vec![
                    ActuatorCapability::PositionControl,
                    ActuatorCapability::PresentPosition,
                ],
                suggested_servo_type: Some("hd1910".to_owned()),
                allowed_servo_types: vec!["hd1910".to_owned(), "xl330".to_owned()],
                mounted_mass_g: None,
            })
            .collect();
        let joint_order = joints
            .iter()
            .map(|joint| joint.name.clone())
            .collect::<Vec<_>>();
        let template = Self {
            id: id.into(),
            name: name.into(),
            version: 1,
            status: BodyStatus::Custom,
            joints,
            actuators,
            assets,
            physical: parse_body_physical(xml)?,
            control: BodyControlContract {
                action_size: joint_order.len(),
                observation_joint_count: joint_order.len(),
                joint_order,
                position_control: true,
                policy_keys: Vec::new(),
            },
        };
        template.validate()?;
        Ok(template)
    }

    pub fn from_toml(text: &str) -> Result<Self, BodyError> {
        toml::from_str(text).map_err(|error| BodyError::Toml(error.to_string()))
    }

    pub fn validate(&self) -> Result<(), BodyError> {
        if self.id.trim().is_empty() {
            return Err(BodyError::MissingField { field: "id" });
        }
        if self.name.trim().is_empty() {
            return Err(BodyError::MissingField { field: "name" });
        }
        if self.assets.display_mjcf.as_os_str().is_empty() {
            return Err(BodyError::MissingField {
                field: "display_mjcf",
            });
        }
        let mut names = BTreeSet::new();
        for (expected, joint) in self.joints.iter().enumerate() {
            if joint.name.trim().is_empty() {
                return Err(BodyError::MissingField {
                    field: "joint.name",
                });
            }
            if !names.insert(joint.name.clone()) {
                return Err(BodyError::Duplicate {
                    kind: "joint",
                    id: joint.name.clone(),
                });
            }
            if joint.order != expected as u16 {
                return Err(BodyError::JointOrder {
                    joint: joint.name.clone(),
                    expected: expected as u16,
                    actual: joint.order,
                });
            }
            if !joint.range_rad[0].is_finite()
                || !joint.range_rad[1].is_finite()
                || joint.range_rad[0] > joint.range_rad[1]
            {
                return Err(BodyError::InvalidRange {
                    joint: joint.name.clone(),
                });
            }
            if let Some(actuator) = &joint.actuator
                && !self.actuators.iter().any(|slot| slot.id == *actuator)
            {
                return Err(BodyError::MissingActuator {
                    joint: joint.name.clone(),
                    actuator: actuator.clone(),
                });
            }
        }
        let joint_names: Vec<_> = self.joints.iter().map(|joint| joint.name.clone()).collect();
        if self.control.joint_order != joint_names {
            return Err(BodyError::ControlOrderMismatch);
        }
        if self.control.action_size > self.actuators.len() {
            return Err(BodyError::ActionSizeMismatch {
                actual: self.control.action_size,
                expected: self.actuators.len(),
            });
        }
        let mut actuator_ids = BTreeSet::new();
        for slot in &self.actuators {
            if slot.id.trim().is_empty() {
                return Err(BodyError::MissingField {
                    field: "actuator.id",
                });
            }
            if !actuator_ids.insert(slot.id.clone()) {
                return Err(BodyError::Duplicate {
                    kind: "actuator",
                    id: slot.id.clone(),
                });
            }
            if !names.contains(&slot.joint) {
                return Err(BodyError::MissingJoint {
                    actuator: slot.id.clone(),
                    joint: slot.joint.clone(),
                });
            }
            if slot
                .mounted_mass_g
                .is_some_and(|mass| !mass.is_finite() || mass < 0.0)
            {
                return Err(BodyError::InvalidRange {
                    joint: slot.joint.clone(),
                });
            }
        }
        Ok(())
    }

    pub fn validate_assets(&self) -> Result<(), BodyError> {
        for path in [
            Some(&self.assets.display_mjcf),
            self.assets.physics_scene.as_ref(),
            self.assets.physics_robot.as_ref(),
            self.assets.mesh_dir.as_ref(),
        ]
        .into_iter()
        .flatten()
        {
            if !path.exists() {
                return Err(BodyError::MissingAsset(path.clone()));
            }
        }
        Ok(())
    }

    pub fn joint_index(&self, name: &str) -> Option<usize> {
        self.joints.iter().position(|joint| joint.name == name)
    }

    pub fn joint_range(&self, name: &str) -> Option<[f64; 2]> {
        self.joints
            .iter()
            .find(|joint| joint.name == name)
            .map(|joint| joint.range_rad)
    }
}

fn parse_body_physical(xml: &str) -> Result<BodyPhysical, BodyError> {
    let doc =
        roxmltree::Document::parse(xml).map_err(|error| BodyError::Mjcf(error.to_string()))?;
    let mut links = Vec::new();
    for body in doc.descendants().filter(|node| node.has_tag_name("body")) {
        let Some(inertial) = body.children().find(|node| node.has_tag_name("inertial")) else {
            continue;
        };
        let Some(mass_text) = inertial.attribute("mass") else {
            continue;
        };
        let mass_kg = mass_text
            .parse::<f64>()
            .map_err(|_| BodyError::Mjcf(format!("invalid inertial mass {mass_text:?}")))?;
        let inertia = inertial.attribute("fullinertia").and_then(|text| {
            let values: Vec<f64> = text
                .split_whitespace()
                .filter_map(|value| value.parse().ok())
                .collect();
            (values.len() == 6).then(|| {
                [
                    values[0], values[1], values[2], values[3], values[4], values[5],
                ]
            })
        });
        links.push(BodyLink {
            name: body.attribute("name").unwrap_or_default().to_owned(),
            mass_g: mass_kg * 1000.0,
            inertia,
        });
    }
    let mass_g = (!links.is_empty()).then(|| links.iter().map(|link| link.mass_g).sum());
    Ok(BodyPhysical {
        mass_g,
        links,
        actuators_mass_g: None,
    })
}
