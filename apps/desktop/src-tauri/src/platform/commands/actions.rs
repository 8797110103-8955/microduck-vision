//! Tauri adapters for the user action catalog.

use tauri::State;

use crate::runtime::session::actions as service;
use crate::runtime::session::state::DesktopState;
use crate::types::{ActionInput, ActionRecord};

#[tauri::command]
#[specta::specta]
pub(crate) fn actions(state: State<'_, DesktopState>) -> Result<Vec<ActionRecord>, String> {
    service::list(state.inner())
}

#[tauri::command]
#[specta::specta]
pub(crate) fn action_create(
    state: State<'_, DesktopState>,
    input: ActionInput,
) -> Result<ActionRecord, String> {
    service::create(state.inner(), input)
}

#[tauri::command]
#[specta::specta]
pub(crate) fn action_delete(state: State<'_, DesktopState>, id: String) -> Result<(), String> {
    service::delete(state.inner(), id)
}
