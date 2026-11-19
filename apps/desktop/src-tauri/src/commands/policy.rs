//! Policy management commands.
//!
//! Policy files and manifests are owned by [`crate::application::policy`]; this module only
//! exposes that service to the frontend and opens the cache folder when requested.

use tauri::State;

use crate::application::policy;
use crate::state::DesktopState;
use crate::types::PolicyEntry;

#[tauri::command]
#[specta::specta]
pub(crate) fn policy_list(state: State<'_, DesktopState>) -> Result<Vec<PolicyEntry>, String> {
    policy::list(state.config_dir.as_deref())
}

#[tauri::command]
#[specta::specta]
pub(crate) fn policy_download(
    state: State<'_, DesktopState>,
    id: String,
) -> Result<PolicyEntry, String> {
    policy::download(state.config_dir.as_deref(), &id)
}

#[tauri::command]
#[specta::specta]
pub(crate) fn policy_select(
    state: State<'_, DesktopState>,
    id: String,
) -> Result<PolicyEntry, String> {
    policy::select(state.config_dir.as_deref(), &id)
}

#[tauri::command]
#[specta::specta]
pub(crate) fn policy_open_folder(state: State<'_, DesktopState>) -> Result<(), String> {
    policy::open_folder(state.config_dir.as_deref())
}
