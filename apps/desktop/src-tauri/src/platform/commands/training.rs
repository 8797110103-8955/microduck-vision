//! Tauri adapters for training process orchestration.

use tauri::{Manager, State};

use crate::runtime::session::state::DesktopState;
use crate::training::process as service;
use crate::types::{TeacherEvaluationInput, TrainingProgress, TrainingStartInput};

#[tauri::command]
#[specta::specta]
pub(crate) async fn training_start(
    state: State<'_, DesktopState>,
    input: TrainingStartInput,
) -> Result<TrainingProgress, String> {
    let app = state
        .app_handle
        .lock()
        .map_err(|error| error.to_string())?
        .clone()
        .ok_or("训练事件通道尚未初始化")?;
    tauri::async_runtime::spawn_blocking(move || {
        service::start(app.state::<DesktopState>().inner(), input)
    })
    .await
    .map_err(|error| error.to_string())?
}

#[tauri::command]
#[specta::specta]
pub(crate) async fn training_evaluate_teacher(
    state: State<'_, DesktopState>,
    input: TeacherEvaluationInput,
) -> Result<TrainingProgress, String> {
    let app = state
        .app_handle
        .lock()
        .map_err(|error| error.to_string())?
        .clone()
        .ok_or("训练事件通道尚未初始化")?;
    tauri::async_runtime::spawn_blocking(move || {
        service::start_teacher_evaluation(app.state::<DesktopState>().inner(), input)
    })
    .await
    .map_err(|error| error.to_string())?
}

#[tauri::command]
#[specta::specta]
pub(crate) fn training_stop(state: State<'_, DesktopState>) -> Result<(), String> {
    service::stop(state.inner())
}

#[tauri::command]
#[specta::specta]
pub(crate) fn training_status(
    state: State<'_, DesktopState>,
) -> Result<Option<TrainingProgress>, String> {
    service::status(state.inner())
}
