//! Official body and scene projection commands.
//!
//! Scene assets are read through `duck-body`; no runtime state or backend is mutated here.

use std::sync::OnceLock;

use duck_body::BodyTemplate;

use crate::types::{
    JointRange, SceneBody as FrontendSceneBody, SceneFrame as FrontendSceneFrame,
    SceneGeom as FrontendSceneGeom, SceneInfo as FrontendSceneInfo,
    SceneJoint as FrontendSceneJoint, SceneModel as FrontendSceneModel,
    SceneSite as FrontendSceneSite, ServoCatalogEntry,
};

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
        .get_or_init(|| {
            let directory =
                crate::backend::mujoco::resource_directory().map_err(|error| error.to_string())?;
            duck_body::OfficialScene::from_path(&directory.join("robot_walk.xml"))
                .map_err(|error| error.to_string())
        })
        .as_ref()
        .map_err(Clone::clone)
}

pub(crate) fn scene_model() -> Result<FrontendSceneInfo, String> {
    let scene = official_scene()?;
    let model = scene.model();
    let template = BodyTemplate::official().map_err(|error| error.to_string())?;
    let default_frame = scene
        .frame(&template.control.home_pose)
        .map_err(|error| error.to_string())?;
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
        default_frame: FrontendSceneFrame {
            bodies: default_frame.bodies,
            markers: default_frame.markers,
        },
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
