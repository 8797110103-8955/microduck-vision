//! Official Microduck display scene loaded from the vendor MJCF.
//!
//! This stays in `duck-body` because the scene is a virtual body asset. It never opens a
//! serial port or receives a real servo driver. The desktop adapter only serializes these
//! values for the WebView's official duck preview.

use std::collections::HashMap;
use std::path::{Path, PathBuf};

use duck_ipc_proto::JOINT_NAMES;
use kinematics::Model;
use serde::{Deserialize, Serialize};
use thiserror::Error;

#[derive(Debug, Error, Clone, PartialEq)]
pub enum SceneError {
    #[error("scene file cannot be read: {path}: {detail}")]
    Io { path: String, detail: String },
    #[error("scene MJCF cannot be parsed: {path}: {detail}")]
    Mjcf { path: String, detail: String },
    #[error("scene has no visual mesh")]
    NoMesh,
    #[error("scene positions contain {actual} values, expected {expected}")]
    PositionLength { actual: usize, expected: usize },
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct SceneGeom {
    pub mesh: String,
    pub pos: [f64; 3],
    pub quat: [f64; 4],
    pub rgba: [f32; 4],
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct SceneBody {
    pub name: String,
    pub geoms: Vec<SceneGeom>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct SceneJoint {
    pub name: String,
    pub body: u32,
}

/// A point fixed to a body, in that body's frame.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct SceneSite {
    pub body: u32,
    pub pos: [f64; 3],
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct SceneModel {
    pub bodies: Vec<SceneBody>,
    pub joints: Vec<SceneJoint>,
    pub trunk_height: f64,
    /// The head camera (the duck's eyes), if the MJCF declares one.
    pub head_camera: Option<SceneSite>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct SceneFrame {
    pub bodies: Vec<[f64; 7]>,
    /// Motor-axis markers in canonical [`JOINT_NAMES`] order, including the
    /// mouth actuator which is not an articulated kinematic joint.
    pub markers: Vec<[f64; 3]>,
}

pub struct OfficialScene {
    kin: Model,
    model: SceneModel,
    mesh_dir: PathBuf,
    angle_index: Vec<Option<usize>>,
    /// The visual model places every XL330 at the servo's output axis.  Keep
    /// the mount body and local point alongside the scene so marker positions
    /// follow the same transforms as the meshes instead of approximating an
    /// actuator with its child link origin.
    marker_mounts: Vec<Option<(u32, [f64; 3])>>,
    marker_bodies: Vec<Option<u32>>,
}

impl OfficialScene {
    pub fn load() -> Result<Self, SceneError> {
        let path = Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../vendor/microduck_rl/src/mjlab_microduck/robot/microduck/robot_walk.xml");
        Self::from_path(&path)
    }

    pub fn from_path(path: &Path) -> Result<Self, SceneError> {
        let path_text = path.display().to_string();
        let xml = std::fs::read_to_string(path).map_err(|error| SceneError::Io {
            path: path_text.clone(),
            detail: error.to_string(),
        })?;
        let kin = Model::parse(&xml).map_err(|error| SceneError::Mjcf {
            path: path_text.clone(),
            detail: error.to_string(),
        })?;
        let doc = roxmltree::Document::parse(&xml).map_err(|error| SceneError::Mjcf {
            path: path_text.clone(),
            detail: error.to_string(),
        })?;
        let mesh_dir = path.parent().unwrap_or(Path::new(".")).join(
            doc.descendants()
                .find(|node| node.has_tag_name("compiler"))
                .and_then(|node| node.attribute("meshdir"))
                .unwrap_or("."),
        );
        let mesh_dir = mesh_dir.canonicalize().unwrap_or(mesh_dir);
        let meshes: HashMap<String, String> = doc
            .descendants()
            .filter(|node| node.has_tag_name("mesh"))
            .filter_map(|node| {
                let file = node.attribute("file")?.to_owned();
                let name = node
                    .attribute("name")
                    .map(str::to_owned)
                    .unwrap_or_else(|| {
                        file.rsplit_once('.')
                            .map_or(file.clone(), |(stem, _)| stem.to_owned())
                    });
                Some((name, file))
            })
            .collect();
        let materials: HashMap<String, [f32; 4]> = doc
            .descendants()
            .filter(|node| node.has_tag_name("material"))
            .filter_map(|node| {
                Some((
                    node.attribute("name")?.to_owned(),
                    parse_rgba(node.attribute("rgba")?)?,
                ))
            })
            .collect();
        let body_nodes: HashMap<&str, roxmltree::Node<'_, '_>> = doc
            .descendants()
            .filter(|node| node.has_tag_name("body"))
            .filter_map(|node| Some((node.attribute("name")?, node)))
            .collect();
        let mut bodies = Vec::new();
        let mut joint_body = HashMap::new();
        let mut motor_mounts: HashMap<String, Vec<[f64; 3]>> = HashMap::new();
        let mut head_camera = None;
        for (index, name) in kin.body_names().enumerate() {
            let mut geoms = Vec::new();
            if let Some(node) = body_nodes.get(name) {
                for child in node.children().filter(|child| child.is_element()) {
                    if child.has_tag_name("joint")
                        && let Some(joint) = child.attribute("name")
                    {
                        joint_body.insert(joint.to_owned(), index);
                    }
                    if child.has_tag_name("camera")
                        && child.attribute("name") == Some("head_camera")
                    {
                        head_camera = Some(SceneSite {
                            body: index as u32,
                            pos: parse_vec(child.attribute("pos"), [0.0; 3]),
                        });
                    }
                    if !child.has_tag_name("geom") || child.attribute("class") == Some("collision")
                    {
                        continue;
                    }
                    let Some(mesh) = child.attribute("mesh").and_then(|mesh| meshes.get(mesh))
                    else {
                        continue;
                    };
                    let pos = parse_vec(child.attribute("pos"), [0.0; 3]);
                    if child.attribute("mesh") == Some("xl330") {
                        motor_mounts.entry(name.to_owned()).or_default().push(pos);
                    }
                    geoms.push(SceneGeom {
                        mesh: mesh.clone(),
                        pos,
                        quat: parse_vec(child.attribute("quat"), [1.0, 0.0, 0.0, 0.0]),
                        rgba: child
                            .attribute("material")
                            .and_then(|material| materials.get(material).copied())
                            .or_else(|| child.attribute("rgba").and_then(parse_rgba))
                            .unwrap_or([0.8, 0.8, 0.8, 1.0]),
                    });
                }
            }
            bodies.push(SceneBody {
                name: name.to_owned(),
                geoms,
            });
        }
        let joints = JOINT_NAMES
            .iter()
            .filter_map(|name| {
                Some(SceneJoint {
                    name: (*name).to_owned(),
                    body: *joint_body.get(*name)? as u32,
                })
            })
            .collect::<Vec<_>>();
        let body_index = kin
            .body_names()
            .enumerate()
            .map(|(index, name)| (name, index as u32))
            .collect::<HashMap<_, _>>();
        let marker_mounts = JOINT_NAMES
            .iter()
            .map(|name| {
                // These three anchors are the visual calibration references:
                // their link origins are already confirmed to sit on the
                // rendered motor axes.  The remaining motors use the exact
                // XL330 mount in the MJCF.
                if matches!(*name, "left_hip_roll" | "neck_pitch" | "head_pitch") {
                    return None;
                }
                let (body_name, ordinal) = motor_mount_for(name)?;
                let body = *body_index.get(body_name)?;
                let pos = *motor_mounts.get(body_name)?.get(ordinal)?;
                Some((body, pos))
            })
            .collect();
        let marker_bodies = JOINT_NAMES
            .iter()
            .map(|name| joint_body.get(*name).copied().map(|index| index as u32))
            .collect();
        let angle_index = JOINT_NAMES
            .iter()
            .map(|name| kin.joint_index(name))
            .collect();
        Ok(Self {
            model: SceneModel {
                bodies,
                joints,
                trunk_height: kin.trunk_height_m(),
                head_camera,
            },
            mesh_dir,
            kin,
            angle_index,
            marker_mounts,
            marker_bodies,
        })
    }

    pub fn model(&self) -> &SceneModel {
        &self.model
    }

    pub fn mesh_dir(&self) -> &Path {
        &self.mesh_dir
    }

    pub fn frame(&self, positions: &[f64]) -> Result<SceneFrame, SceneError> {
        if positions.len() != JOINT_NAMES.len() {
            return Err(SceneError::PositionLength {
                actual: positions.len(),
                expected: JOINT_NAMES.len(),
            });
        }
        let mut angles = vec![0.0; self.kin.num_joints()];
        for (index, slot) in self.angle_index.iter().enumerate() {
            if let Some(kin_index) = slot {
                angles[*kin_index] = positions[index];
            }
        }
        let poses = self.kin.body_poses(&angles);
        let bodies = poses
            .iter()
            .map(|pose| {
                let q = pose.quat.wxyz();
                [
                    pose.pos[0],
                    pose.pos[1],
                    pose.pos[2],
                    q[0],
                    q[1],
                    q[2],
                    q[3],
                ]
            })
            .collect();
        let markers = self
            .marker_mounts
            .iter()
            .zip(&self.marker_bodies)
            .map(|(mount, body)| {
                mount
                    .map(|(body, pos)| poses[body as usize].transform_point(pos))
                    .or_else(|| body.map(|body| poses[body as usize].pos))
                    .unwrap_or([0.0; 3])
            })
            .collect();
        Ok(SceneFrame { bodies, markers })
    }
}

/// Maps a canonical joint to the XL330 geom mounted at that motor's output
/// axis.  A few bodies carry two motors, so the ordinal is within that body,
/// not the global geom order.
fn motor_mount_for(name: &str) -> Option<(&'static str, usize)> {
    Some(match name {
        "left_hip_yaw" => ("trunk_base", 0),
        "right_hip_yaw" => ("trunk_base", 1),
        "left_hip_roll" => ("yaw2roll", 0),
        "left_hip_pitch" => ("upper_leg_left", 0),
        "left_knee" => ("upper_leg_left", 1),
        "left_ankle" => ("leg", 0),
        "neck_pitch" => ("neck", 0),
        "head_pitch" => ("neck", 1),
        "head_yaw" => ("neck_pitch", 0),
        "head_roll" => ("jaw_soft", 0),
        "mouth" => ("jaw_soft", 1),
        "right_hip_roll" => ("bearing_roll", 0),
        "right_hip_pitch" => ("upper_leg_right", 0),
        "right_knee" => ("upper_leg_right", 1),
        "right_ankle" => ("leg_2", 0),
        _ => return None,
    })
}

fn parse_vec<const N: usize>(text: Option<&str>, default: [f64; N]) -> [f64; N] {
    let Some(text) = text else { return default };
    text.split_whitespace()
        .filter_map(|value| value.parse().ok())
        .collect::<Vec<f64>>()
        .try_into()
        .unwrap_or(default)
}

fn parse_rgba(text: &str) -> Option<[f32; 4]> {
    text.split_whitespace()
        .filter_map(|value| value.parse().ok())
        .collect::<Vec<f32>>()
        .try_into()
        .ok()
}
