use std::collections::{BTreeMap, BTreeSet};

use serde::{Deserialize, Serialize};

use crate::{ActuatorCapability, AssetResolver, BodyAssets, BodyError};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum BodyStatus {
    Official,
    Experimental,
    Custom,
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
    /// Separately accounted mounting mass. Link inertials exclude this value.
    pub mounted_mass_g: Option<f64>,
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
    /// True when link inertials already account for the installed actuators.
    pub includes_actuator_mass: bool,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct BodyControlContract {
    pub joint_order: Vec<String>,
    pub position_control: bool,
    /// Canonical reference pose used by observations and target post-processing.
    #[serde(default)]
    pub home_pose: Vec<f64>,
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
    /// Named poses read from the body's scene, such as STAND and SIT. They are
    /// reference facts only; skills remain policy-driven at runtime.
    #[serde(default)]
    pub reference_poses: BTreeMap<String, Vec<f64>>,
}

impl BodyTemplate {
    pub fn official(assets: BodyAssets, resolver: &dyn AssetResolver) -> Result<Self, BodyError> {
        crate::official::load_template(assets, resolver)
    }

    pub fn from_mjcf(
        id: impl Into<String>,
        name: impl Into<String>,
        xml: &str,
        assets: BodyAssets,
    ) -> Result<Self, BodyError> {
        let model =
            kinematics::Model::parse(xml).map_err(|error| BodyError::Mjcf(error.to_string()))?;
        let axes = parse_joint_axes(xml)?;
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
                axis: axes.get(name).copied().unwrap_or([0.0, 0.0, 1.0]),
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
                suggested_servo_type: None,
                allowed_servo_types: Vec::new(),
                mounted_mass_g: None,
            })
            .collect();
        let joint_order = joints
            .iter()
            .map(|joint| joint.name.clone())
            .collect::<Vec<_>>();
        let joint_count = joint_order.len();
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
                joint_order,
                position_control: true,
                home_pose: vec![0.0; joint_count],
            },
            reference_poses: BTreeMap::new(),
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
        if self.control.home_pose.len() != self.joints.len()
            || self
                .control
                .home_pose
                .iter()
                .any(|value| !value.is_finite())
        {
            return Err(BodyError::InvalidHomePose);
        }
        for (name, pose) in &self.reference_poses {
            if pose.len() != self.joints.len() || pose.iter().any(|value| !value.is_finite()) {
                return Err(BodyError::InvalidReferencePose { name: name.clone() });
            }
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

    pub fn validate_assets(&self, resolver: &dyn AssetResolver) -> Result<(), BodyError> {
        self.assets.validate(resolver)
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
        includes_actuator_mass: false,
    })
}

fn parse_joint_axes(xml: &str) -> Result<BTreeMap<String, [f64; 3]>, BodyError> {
    let doc =
        roxmltree::Document::parse(xml).map_err(|error| BodyError::Mjcf(error.to_string()))?;
    let mut axes = BTreeMap::new();
    for joint in doc.descendants().filter(|node| node.has_tag_name("joint")) {
        let Some(name) = joint.attribute("name") else {
            continue;
        };
        let axis = joint
            .attribute("axis")
            .unwrap_or("0 0 1")
            .split_whitespace()
            .map(|value| {
                value
                    .parse::<f64>()
                    .map_err(|_| BodyError::Mjcf(format!("invalid joint axis {value:?}")))
            })
            .collect::<Result<Vec<_>, _>>()?;
        if axis.len() != 3 {
            return Err(BodyError::Mjcf(format!(
                "joint {name:?} axis must have three components"
            )));
        }
        let norm = (axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]).sqrt();
        if !norm.is_finite() || norm <= f64::EPSILON {
            return Err(BodyError::Mjcf(format!("joint {name:?} has a zero axis")));
        }
        axes.insert(
            name.to_owned(),
            [axis[0] / norm, axis[1] / norm, axis[2] / norm],
        );
    }
    Ok(axes)
}
