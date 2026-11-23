//! Body display geometry and forward kinematics from a caller-supplied MJCF.

use std::collections::HashMap;
use std::path::{Path, PathBuf};

use kinematics::Model;
use serde::{Deserialize, Serialize};
use thiserror::Error;

use crate::assets::{mesh_directory, read_xml};
use crate::{AssetResolver, BodyError, BodyTemplate};

#[derive(Debug, Error, Clone, PartialEq)]
pub enum SceneError {
    #[error(transparent)]
    Asset(#[from] BodyError),
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
    /// Motor-axis markers in canonical body joint order.
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
    pub fn from_body(
        template: &BodyTemplate,
        resolver: &dyn AssetResolver,
    ) -> Result<Self, SceneError> {
        Self::parse(template, resolver, |_| None)
    }

    pub fn for_official_body(
        template: &BodyTemplate,
        resolver: &dyn AssetResolver,
    ) -> Result<Self, SceneError> {
        Self::parse(template, resolver, crate::official::motor_mount_for)
    }

    fn parse(
        template: &BodyTemplate,
        resolver: &dyn AssetResolver,
        motor_mount: impl Fn(&str) -> Option<(&'static str, &'static str, usize)>,
    ) -> Result<Self, SceneError> {
        let path = &template.assets.display_mjcf;
        let path_text = path.display().to_string();
        let xml = read_xml(resolver, path)?;
        let kin = Model::parse(&xml).map_err(|error| SceneError::Mjcf {
            path: path_text.clone(),
            detail: error.to_string(),
        })?;
        let doc = roxmltree::Document::parse(&xml).map_err(|error| SceneError::Mjcf {
            path: path_text.clone(),
            detail: error.to_string(),
        })?;
        let mesh_dir = mesh_directory(path, &doc);
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
        for file in meshes.values() {
            resolver.read(&mesh_dir.join(file))?;
        }
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
        let marker_bodies: Vec<_> = template
            .joints
            .iter()
            .map(|joint| {
                let name = joint.scene_joint.as_ref()?;
                joint_body.get(name).map(|&index| index as u32)
            })
            .collect();
        let joints = template
            .joints
            .iter()
            .zip(&marker_bodies)
            .filter_map(|(joint, body)| {
                Some(SceneJoint {
                    name: joint.name.clone(),
                    body: (*body)?,
                })
            })
            .collect();
        let body_index = kin
            .body_names()
            .enumerate()
            .map(|(index, name)| (name, index as u32))
            .collect::<HashMap<_, _>>();
        let marker_mounts = template
            .joints
            .iter()
            .map(|joint| {
                let (name, mesh, ordinal) = motor_mount(&joint.name)?;
                let body = *body_index.get(name)?;
                let geom = body_nodes
                    .get(name)?
                    .children()
                    .filter(|child| {
                        child.has_tag_name("geom")
                            && child.attribute("class") != Some("collision")
                            && child.attribute("mesh") == Some(mesh)
                    })
                    .nth(ordinal)?;
                Some((body, parse_vec(geom.attribute("pos"), [0.0; 3])))
            })
            .collect();
        let angle_index = template
            .joints
            .iter()
            .map(|joint| {
                let Some(name) = &joint.scene_joint else {
                    return Ok(None);
                };
                kin.joint_index(name)
                    .map(Some)
                    .ok_or_else(|| BodyError::MissingSceneJoint(name.clone()).into())
            })
            .collect::<Result<Vec<_>, SceneError>>()?;
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
        if positions.len() != self.angle_index.len() {
            return Err(SceneError::PositionLength {
                actual: positions.len(),
                expected: self.angle_index.len(),
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
