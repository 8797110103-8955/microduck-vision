//! Tauri adapters for runtime configuration and generation-bound execution commands.

use crate::application::{loop_service, preview};
use crate::control;
use crate::state::DesktopState;
use crate::types::{LoopStatus, RuntimeCloseScope, RuntimeCommand, RuntimeOpenInput};
use tauri::State;

#[tauri::command]
#[specta::specta]
pub(crate) fn runtime_open(
    state: State<'_, DesktopState>,
    input: RuntimeOpenInput,
) -> Result<LoopStatus, String> {
    loop_service::start(state.inner(), input)
}

#[tauri::command]
#[specta::specta]
pub(crate) fn runtime_reconfigure(
    state: State<'_, DesktopState>,
    generation: u32,
    config_revision: u32,
    input: RuntimeOpenInput,
) -> Result<LoopStatus, String> {
    loop_service::reconfigure(
        state.inner(),
        u64::from(generation),
        u64::from(config_revision),
        input,
    )
}

#[tauri::command]
#[specta::specta]
pub(crate) fn runtime_command(
    state: State<'_, DesktopState>,
    generation: u32,
    execution_generation: u32,
    sequence: u32,
    request: RuntimeCommand,
) -> Result<LoopStatus, String> {
    loop_service::command(
        state.inner(),
        u64::from(generation),
        u64::from(execution_generation),
        u64::from(sequence),
        request,
    )
}

#[tauri::command]
#[specta::specta]
pub(crate) fn loop_status(state: State<'_, DesktopState>) -> Result<Option<LoopStatus>, String> {
    control::status(state.inner())
}

#[tauri::command]
#[specta::specta]
pub(crate) fn runtime_close(
    state: State<'_, DesktopState>,
    generation: u32,
    scope: RuntimeCloseScope,
) -> Result<(), String> {
    loop_service::close(state.inner(), u64::from(generation), scope)
}

#[tauri::command]
#[specta::specta]
pub(crate) fn preview_machine_select(
    state: State<'_, DesktopState>,
    virtual_machine_id: String,
) -> Result<(), String> {
    preview::select_machine(state.inner(), virtual_machine_id)
}
