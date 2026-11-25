//! Policy management commands.
//!
//! Policy files and manifests are owned by [`crate::runtime::policy::catalog`]; this module only
//! exposes that service to the frontend and opens the cache folder when requested.

use tauri::State;

use crate::runtime::policy::catalog as policy;
use crate::runtime::session::state::DesktopState;
use crate::types::PolicyEntry;

#[tauri::command]
#[specta::specta]
pub(crate) fn policy_list(state: State<'_, DesktopState>) -> Result<Vec<PolicyEntry>, String> {
    policy::list(state.config_dir.as_deref())
}

#[tauri::command]
#[specta::specta]
pub(crate) async fn policy_refresh(
    state: State<'_, DesktopState>,
) -> Result<Vec<PolicyEntry>, String> {
    let config = state.config_dir.clone();
    tauri::async_runtime::spawn_blocking(move || policy::refresh_list(config.as_deref()))
        .await
        .map_err(|error| error.to_string())?
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
    let _transition = state
        .execution_transition
        .lock()
        .map_err(|e| e.to_string())?;
    if state.recorder.is_active() {
        return Err("请先停止采集，再切换 ONNX".into());
    }
    policy::select(state.config_dir.as_deref(), &id)
}

#[tauri::command]
#[specta::specta]
pub(crate) fn policy_open_folder(state: State<'_, DesktopState>) -> Result<(), String> {
    policy::open_folder(state.config_dir.as_deref())
}
