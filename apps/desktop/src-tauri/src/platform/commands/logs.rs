//! Logs and recording operations exposed to the settings page.

use crate::runtime::session::state::DesktopState;
use crate::types::{
    RecordingComparison, RecordingExport, RecordingInfo, RecordingPreviewFrame, RecordingWindow,
};
use tauri::State;

#[tauri::command]
#[specta::specta]
pub(crate) async fn recording_list(
    state: State<'_, DesktopState>,
) -> Result<Vec<RecordingInfo>, String> {
    let config = state.config_dir.clone();
    let recorder = std::sync::Arc::clone(&state.recorder);
    tauri::async_runtime::spawn_blocking(move || {
        crate::runtime::recordings::list(config.as_deref(), &recorder)
    })
    .await
    .map_err(|e| e.to_string())?
}

#[tauri::command]
#[specta::specta]
pub(crate) async fn recording_start(
    state: State<'_, DesktopState>,
    name: String,
) -> Result<RecordingInfo, String> {
    let status = crate::runtime::session::loop_service::status(&state)?.ok_or("请先启动控制环")?;
    let metadata = {
        let runtime = state.runtime_loop.lock().map_err(|e| e.to_string())?;
        let runtime = runtime.as_ref().ok_or("控制环已关闭")?;
        serde_json::json!({
            "runtime_id":status.runtime_generation,"execution_generation":status.execution_generation,
            "runtime":runtime.recording_metadata(),
            "virtual_machines":*state.virtual_machines.lock().map_err(|e| e.to_string())?,
            "physical_machines":*state.physical_machines.lock().map_err(|e| e.to_string())?,
            "body":duck_body::BodyContractSnapshot::from_profile(
                crate::runtime::machines::body_catalog::official()?
            ),
            "product_version":env!("CARGO_PKG_VERSION"),
        })
    };
    let config = state.config_dir.clone();
    let recorder = std::sync::Arc::clone(&state.recorder);
    let expected = status.runtime_generation;
    let info = tauri::async_runtime::spawn_blocking(move || {
        recorder.start(config.as_deref(), &status, name, metadata)
    })
    .await
    .map_err(|e| e.to_string())??;
    if state.current_runtime_generation() != expected
        || state
            .runtime_loop
            .lock()
            .map_err(|e| e.to_string())?
            .is_none()
    {
        state.recorder.stop("runtime_changed")?;
        return Err("采集启动时控制环已变化，请重新启动采集".into());
    }
    Ok(info)
}

#[tauri::command]
#[specta::specta]
pub(crate) async fn recording_stop(
    state: State<'_, DesktopState>,
) -> Result<Option<RecordingInfo>, String> {
    let recorder = std::sync::Arc::clone(&state.recorder);
    tauri::async_runtime::spawn_blocking(move || recorder.stop("stopped"))
        .await
        .map_err(|e| e.to_string())?
}

#[tauri::command]
#[specta::specta]
pub(crate) async fn recording_frames(
    state: State<'_, DesktopState>,
    window: RecordingWindow,
) -> Result<Vec<RecordingPreviewFrame>, String> {
    let config = state.config_dir.clone();
    tauri::async_runtime::spawn_blocking(move || {
        crate::runtime::recordings::preview_frames(config.as_deref(), &window)
    })
    .await
    .map_err(|e| e.to_string())?
}

#[tauri::command]
#[specta::specta]
pub(crate) fn frontend_diagnostic(state: State<'_, DesktopState>, message: String) {
    crate::platform::logging::frontend_diagnostic(state.config_dir.as_deref(), message);
}

#[tauri::command]
#[specta::specta]
pub(crate) async fn recording_compare(
    state: State<'_, DesktopState>,
    left: RecordingWindow,
    right: RecordingWindow,
) -> Result<RecordingComparison, String> {
    let config = state.config_dir.clone();
    tauri::async_runtime::spawn_blocking(move || {
        crate::runtime::recordings::compare(config.as_deref(), left, right)
    })
    .await
    .map_err(|e| e.to_string())?
}

#[tauri::command]
#[specta::specta]
pub(crate) async fn recording_export(
    state: State<'_, DesktopState>,
    window: RecordingWindow,
    kind: String,
) -> Result<RecordingExport, String> {
    let config = state.config_dir.clone();
    tauri::async_runtime::spawn_blocking(move || {
        crate::runtime::recordings::export(config.as_deref(), window, &kind)
    })
    .await
    .map_err(|e| e.to_string())?
}

#[tauri::command]
#[specta::specta]
pub(crate) async fn diagnostic_tail(state: State<'_, DesktopState>) -> Result<Vec<String>, String> {
    let path = crate::platform::logging::current_log_file(state.config_dir.as_deref());
    tauri::async_runtime::spawn_blocking(move || {
        crate::platform::logging::flush();
        use std::io::{Read, Seek, SeekFrom};
        if !path.exists() {
            return Ok(Vec::new());
        }
        let mut file = std::fs::File::open(path).map_err(|e| e.to_string())?;
        let size = file.metadata().map_err(|e| e.to_string())?.len();
        let offset = size.saturating_sub(262_144);
        file.seek(SeekFrom::Start(offset))
            .map_err(|e| e.to_string())?;
        let mut bytes = Vec::new();
        file.read_to_end(&mut bytes).map_err(|e| e.to_string())?;
        let text = String::from_utf8_lossy(&bytes);
        Ok(text
            .lines()
            .skip(usize::from(offset > 0))
            .map(str::to_owned)
            .collect::<Vec<_>>()
            .into_iter()
            .rev()
            .take(200)
            .collect())
    })
    .await
    .map_err(|e| e.to_string())?
}
