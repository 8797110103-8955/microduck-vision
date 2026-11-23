//! Adapt the pinned MicroDuck MJCF to the upstream canonical joint order.

use std::collections::BTreeMap;
use std::path::Path;

use duck_ipc_proto::JOINT_NAMES;

use crate::assets::{read_xml, visit_mjcf};
use crate::{
    ActuatorCapability, ActuatorSlot, AssetResolver, BodyAssets, BodyError, BodyJoint, BodyStatus,
    BodyTemplate, OFFICIAL_BODY_ID, OFFICIAL_BODY_NAME,
};

pub fn official_assets(directory: &Path) -> BodyAssets {
    BodyAssets {
        display_mjcf: directory.join("robot_walk.xml"),
        pose_scene: Some(directory.join("scene.xml")),
    }
}

pub(crate) fn load_template(
    assets: BodyAssets,
    resolver: &dyn AssetResolver,
) -> Result<BodyTemplate, BodyError> {
    let xml = read_xml(resolver, &assets.display_mjcf)?;
    let pose_source = assets.pose_scene.clone().ok_or(BodyError::MissingField {
        field: "pose_scene",
    })?;
    let mut body = BodyTemplate::from_mjcf(OFFICIAL_BODY_ID, OFFICIAL_BODY_NAME, &xml, assets)?;
    body.status = BodyStatus::Official;
    body.physical.includes_actuator_mass = true;
    body.joints = JOINT_NAMES
        .iter()
        .enumerate()
        .map(|(index, &name)| {
            let mut joint = if name == "mouth" {
                BodyJoint {
                    name: name.to_owned(),
                    order: 0,
                    range_rad: [-5.0_f64.to_radians(), 30.0_f64.to_radians()],
                    scene_joint: None,
                    axis: [0.0, 0.0, 1.0],
                    actuator: Some(name.to_owned()),
                }
            } else {
                body.joints
                    .iter()
                    .find(|joint| joint.name == name)
                    .cloned()
                    .ok_or_else(|| BodyError::MissingSceneJoint(name.to_owned()))?
            };
            joint.order = index as u16;
            Ok(joint)
        })
        .collect::<Result<_, BodyError>>()?;
    body.actuators = body
        .joints
        .iter()
        .map(|joint| ActuatorSlot {
            id: joint.name.clone(),
            joint: joint.name.clone(),
            required_capabilities: vec![
                ActuatorCapability::PositionControl,
                ActuatorCapability::PresentPosition,
                ActuatorCapability::PresentVelocity,
                ActuatorCapability::TorqueEnable,
            ],
            suggested_servo_type: Some("hd1910".to_owned()),
            allowed_servo_types: vec!["hd1910".to_owned(), "xl330".to_owned()],
            mounted_mass_g: None,
        })
        .collect();
    body.control.joint_order = JOINT_NAMES.iter().map(|name| (*name).to_owned()).collect();
    body.reference_poses = parse_scene_poses(resolver, &pose_source, &body.joints)?;
    body.control.home_pose = body.reference_poses.get("STAND").cloned().ok_or_else(|| {
        BodyError::InvalidReferencePose {
            name: "STAND".to_owned(),
        }
    })?;
    body.validate()?;
    body.validate_assets(resolver)?;
    Ok(body)
}

fn parse_scene_poses(
    resolver: &dyn AssetResolver,
    source: &Path,
    joints: &[BodyJoint],
) -> Result<BTreeMap<String, Vec<f64>>, BodyError> {
    let mut width = 0;
    let mut scalar_indices = BTreeMap::new();
    let mut keyframes = Vec::new();
    visit_mjcf(resolver, source, &mut |_source, node| {
        if node
            .parent()
            .is_some_and(|parent| parent.has_tag_name("body"))
        {
            let size = match node.tag_name().name() {
                "freejoint" => 7,
                "joint" => match node.attribute("type").unwrap_or("hinge") {
                    "free" => 7,
                    "ball" => 4,
                    "slide" | "hinge" => 1,
                    kind => {
                        return Err(BodyError::Mjcf(format!("unsupported joint type {kind:?}")));
                    }
                },
                _ => 0,
            };
            if size == 1
                && let Some(name) = node.attribute("name")
                && scalar_indices.insert(name.to_owned(), width).is_some()
            {
                return Err(BodyError::Duplicate {
                    kind: "scene joint",
                    id: name.to_owned(),
                });
            }
            width += size;
        }
        if node.has_tag_name("key")
            && let (Some(name), Some(qpos)) = (node.attribute("name"), node.attribute("qpos"))
        {
            keyframes.push((name.to_owned(), qpos.to_owned()));
        }
        Ok(())
    })?;
    let indices = joints
        .iter()
        .map(|joint| {
            joint
                .scene_joint
                .as_ref()
                .map(|name| {
                    scalar_indices
                        .get(name)
                        .copied()
                        .ok_or_else(|| BodyError::MissingSceneJoint(name.clone()))
                })
                .transpose()
        })
        .collect::<Result<Vec<_>, _>>()?;
    keyframes
        .into_iter()
        .map(|(name, qpos)| {
            let values = qpos
                .split_whitespace()
                .map(str::parse::<f64>)
                .collect::<Result<Vec<_>, _>>()
                .map_err(|error| BodyError::Mjcf(format!("keyframe {name}: {error}")))?;
            if values.len() != width || values.iter().any(|value| !value.is_finite()) {
                return Err(BodyError::InvalidReferencePose { name });
            }
            let pose = indices
                .iter()
                .map(|index| index.map_or(0.0, |i| values[i]))
                .collect();
            Ok((name, pose))
        })
        .collect()
}

/// XL330 mesh occurrences at the installed motor output axes.
pub(crate) fn motor_mount_for(name: &str) -> Option<(&'static str, &'static str, usize)> {
    let (body, ordinal) = match name {
        "left_hip_yaw" => ("trunk_base", 0),
        "right_hip_yaw" => ("trunk_base", 1),
        "left_hip_pitch" => ("upper_leg_left", 0),
        "left_knee" => ("upper_leg_left", 1),
        "left_ankle" => ("leg", 0),
        "head_yaw" => ("neck_pitch", 0),
        "head_roll" => ("jaw_soft", 0),
        "mouth" => ("jaw_soft", 1),
        "right_hip_roll" => ("bearing_roll", 0),
        "right_hip_pitch" => ("upper_leg_right", 0),
        "right_knee" => ("upper_leg_right", 1),
        "right_ankle" => ("leg_2", 0),
        // The left hip roll, neck pitch and head pitch use their link origins.
        _ => return None,
    };
    Some((body, "xl330", ordinal))
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::path::PathBuf;

    #[test]
    fn keyframe_mapping_uses_scene_joint_names_and_qpos_widths() {
        let files = BTreeMap::from([
            (
                PathBuf::from("fixture/scene.xml"),
                br#"<mujoco>
                    <include file="robot.xml"/>
                    <keyframe><key name="STAND" qpos="0 0 0 1 0 0 0 -0.25 1 0 0 0 0.75 0.5"/></keyframe>
                </mujoco>"#.to_vec(),
            ),
            (
                PathBuf::from("fixture/robot.xml"),
                br#"<mujoco><worldbody><body>
                    <freejoint name="base"/>
                    <body><joint name="right"/></body>
                    <body><joint name="auxiliary" type="ball"/></body>
                    <body><joint name="passive"/><joint name="left"/></body>
                </body></worldbody></mujoco>"#.to_vec(),
            ),
        ]);
        let joints = [Some("left"), None, Some("right")].map(|name| BodyJoint {
            name: name.unwrap_or("mouth").into(),
            order: 0,
            range_rad: [-1.0, 1.0],
            scene_joint: name.map(str::to_owned),
            axis: [0.0, 0.0, 1.0],
            actuator: None,
        });
        let poses = parse_scene_poses(&files, Path::new("fixture/scene.xml"), &joints).unwrap();
        assert_eq!(poses["STAND"], [0.5, 0.0, -0.25]);
    }
}
