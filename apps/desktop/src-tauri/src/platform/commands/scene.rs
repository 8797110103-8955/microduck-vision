//! Tauri commands for the official body and scene.

use crate::runtime::scene;
use crate::types::{JointRange, SceneFrame, SceneInfo, ServoCatalogEntry};

#[tauri::command]
#[specta::specta]
pub(crate) fn servo_catalog() -> Vec<ServoCatalogEntry> {
    crate::runtime::machines::servo_catalog::entries()
}

#[tauri::command]
#[specta::specta]
pub(crate) fn official_joint_slots() -> Vec<String> {
    scene::official_joint_slots()
}

#[tauri::command]
#[specta::specta]
pub(crate) fn official_joint_ranges() -> Result<Vec<JointRange>, String> {
    scene::official_joint_ranges()
}

#[tauri::command]
#[specta::specta]
pub(crate) fn official_scene_model() -> Result<SceneInfo, String> {
    scene::scene_model()
}

#[tauri::command]
#[specta::specta]
pub(crate) fn official_scene_frame(positions: Vec<f64>) -> Result<SceneFrame, String> {
    scene::scene_frame(&positions)
}
