//! Tauri adapters for training process orchestration.

use tauri::{Manager, State};

use crate::runtime::session::state::DesktopState;
use crate::training::process as service;
use crate::types::{
    TaskConfiguration, TeacherEvaluationInput, TrainingPreview, TrainingPreviewInput,
    TrainingProgress, TrainingStartInput,
};

#[tauri::command]
#[specta::specta]
pub(crate) async fn training_task_configuration(
    state: State<'_, DesktopState>,
    task_id: String,
) -> Result<TaskConfiguration, String> {
    let config_dir = state.config_dir.clone();
    tauri::async_runtime::spawn_blocking(move || {
        microduck_training::launch::task_configuration(
            &microduck_training::bundle::project()?,
            config_dir.as_deref(),
            &task_id,
        )
    })
    .await
    .map_err(|error| error.to_string())?
}

#[tauri::command]
#[specta::specta]
pub(crate) async fn training_tasks(state: State<'_, DesktopState>) -> Result<Vec<String>, String> {
    let config_dir = state.config_dir.clone();
    tauri::async_runtime::spawn_blocking(move || {
        let project = microduck_training::bundle::project()?;
        microduck_training::launch::registered_tasks(&project, config_dir.as_deref())
    })
    .await
    .map_err(|error| error.to_string())?
}

#[tauri::command]
#[specta::specta]
pub(crate) async fn training_rounds(
    state: State<'_, DesktopState>,
) -> Result<Vec<crate::types::RoundSummary>, String> {
    let config = state.config_dir.clone();
    tauri::async_runtime::spawn_blocking(move || crate::training::rounds::list(config.as_deref()))
        .await
        .map_err(|e| e.to_string())?
}

#[tauri::command]
#[specta::specta]
pub(crate) async fn training_publication_targets(
    state: State<'_, DesktopState>,
) -> Result<Vec<crate::types::PublicationTarget>, String> {
    let config = state.config_dir.clone();
    tauri::async_runtime::spawn_blocking(move || {
        crate::training::rounds::publication_targets(config.as_deref())
    })
    .await
    .map_err(|error| error.to_string())
}

#[tauri::command]
#[specta::specta]
pub(crate) async fn training_publish(
    state: State<'_, DesktopState>,
    run_id: String,
    checkpoint_id: String,
    name: String,
    append_to: Option<String>,
) -> Result<crate::types::PolicyEntry, String> {
    let app = state
        .app_handle
        .lock()
        .map_err(|e| e.to_string())?
        .clone()
        .ok_or("策略通道尚未初始化")?;
    tauri::async_runtime::spawn_blocking(move || {
        crate::training::rounds::publish(
            app.state::<DesktopState>().inner(),
            &run_id,
            &checkpoint_id,
            &name,
            append_to.as_deref(),
        )
    })
    .await
    .map_err(|e| e.to_string())?
}

#[tauri::command]
#[specta::specta]
pub(crate) async fn training_preview(
    state: State<'_, DesktopState>,
    input: TrainingPreviewInput,
) -> Result<TrainingPreview, String> {
    let machine = crate::runtime::machines::machine_resolution::virtual_machine(
        state.inner(),
        &input.virtual_machine_id,
    )?;
    let config = state.config_dir.clone();
    tauri::async_runtime::spawn_blocking(move || {
        crate::training::preview::run(config.as_deref(), &machine, &input)
    })
    .await
    .map_err(|error| error.to_string())?
}

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

#[tauri::command]
#[specta::specta]
pub(crate) fn training_open_result(
    state: State<'_, DesktopState>,
    run_id: String,
) -> Result<(), String> {
    let directory = service::result_directory(state.inner(), &run_id)?;
    tauri_plugin_opener::open_path(directory, None::<&str>).map_err(|error| error.to_string())
}
