//! Tauri adapters for training process orchestration.

use tauri::State;

use crate::application::training as service;
use crate::state::DesktopState;
use crate::types::{TrainingProgress, TrainingStartInput};

#[tauri::command]
#[specta::specta]
pub(crate) fn training_start(
    state: State<'_, DesktopState>,
    input: TrainingStartInput,
) -> Result<TrainingProgress, String> {
    service::start(state.inner(), input)
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
