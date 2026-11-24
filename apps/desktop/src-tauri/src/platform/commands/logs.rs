//! Logs and recording operations exposed to the settings page.

use crate::runtime::session::state::DesktopState;
use crate::types::{
    PreviewInput, RecordingComparison, RecordingExport, RecordingInfo, RecordingPreviewFrame,
    RecordingWindow,
};
use tauri::State;

#[tauri::command]
#[specta::specta]
pub(crate) fn open_logs(state: State<'_, DesktopState>) -> Result<(), String> {
    let directory = crate::platform::logging::log_directory(state.config_dir.as_deref());
    std::fs::create_dir_all(&directory).map_err(|error| format!("创建日志文件夹失败：{error}"))?;
    tauri_plugin_opener::open_path(&directory, None::<&str>)
        .map_err(|error| format!("打开日志文件夹失败：{error}"))
}

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
pub(crate) fn recording_start(
    state: State<'_, DesktopState>,
    virtual_machine_id: String,
    policy_id: String,
    input: PreviewInput,
) -> Result<crate::types::LoopStatus, String> {
    crate::runtime::session::loop_service::start_recording(
        state.inner(),
        virtual_machine_id,
        policy_id,
        input,
    )
}

#[tauri::command]
#[specta::specta]
pub(crate) fn recording_stop(
    state: State<'_, DesktopState>,
    generation: u32,
) -> Result<(), String> {
    crate::runtime::session::loop_service::stop_recording(state.inner(), u64::from(generation))
}

#[tauri::command]
#[specta::specta]
pub(crate) fn recording_reveal(
    state: State<'_, DesktopState>,
    id: String,
    motion: bool,
) -> Result<(), String> {
    crate::runtime::recordings::reveal_file(state.config_dir.as_deref(), &id, motion)
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
