//! Virtual-machine Tauri commands.
//!
//! These handlers keep the frontend command names stable and delegate persisted state changes to
//! the application service.

use super::*;
use crate::application::binding as binding_service;
use crate::application::virtual_machine as service;
use crate::types::VirtualJointBindingInput;

#[tauri::command]
#[specta::specta]
pub(crate) fn virtual_machine_bindings(
    state: State<'_, DesktopState>,
    virtual_machine_id: String,
    bindings: Vec<VirtualJointBindingInput>,
) -> Result<VirtualMachine, String> {
    binding_service::virtual_machine_bindings(state.inner(), virtual_machine_id, bindings)
}

#[tauri::command]
#[specta::specta]
pub(crate) fn virtual_machine_bind(
    state: State<'_, DesktopState>,
    input: VirtualMachineBindingInput,
) -> Result<VirtualMachine, String> {
    binding_service::virtual_machine_bind(state.inner(), input)
}

#[tauri::command]
#[specta::specta]
pub(crate) fn virtual_machine_validate_bindings(
    state: State<'_, DesktopState>,
    id: String,
) -> Result<VirtualMachine, String> {
    binding_service::virtual_machine_validate_bindings(state.inner(), id)
}

#[tauri::command]
#[specta::specta]
pub(crate) fn virtual_machine_create(
    state: State<'_, DesktopState>,
    input: VirtualMachineInput,
) -> Result<VirtualMachine, String> {
    service::virtual_machine_create(state.inner(), input)
}

#[tauri::command]
#[specta::specta]
pub(crate) fn virtual_machine_set_startup(
    state: State<'_, DesktopState>,
    id: String,
) -> Result<(), String> {
    service::virtual_machine_set_startup(state.inner(), id)
}

#[tauri::command]
#[specta::specta]
pub(crate) fn virtual_machine_set_world_position(
    state: State<'_, DesktopState>,
    id: String,
    position: [f64; 3],
) -> Result<VirtualMachine, String> {
    service::virtual_machine_set_world_position(state.inner(), id, position)
}

#[tauri::command]
#[specta::specta]
pub(crate) fn virtual_machine_delete(
    state: State<'_, DesktopState>,
    id: String,
) -> Result<(), String> {
    service::virtual_machine_delete(state.inner(), id)
}

#[tauri::command]
#[specta::specta]
pub(crate) fn virtual_machine_rename(
    state: State<'_, DesktopState>,
    id: String,
    name: String,
) -> Result<VirtualMachine, String> {
    service::virtual_machine_rename(state.inner(), id, name)
}
