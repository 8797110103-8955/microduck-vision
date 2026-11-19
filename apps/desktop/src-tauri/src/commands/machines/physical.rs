//! Physical-machine Tauri commands.
//!
//! These handlers preserve frontend command names and delegate lifecycle, binding, debug, and
//! register orchestration to application services. They only adapt Tauri arguments.

use super::*;
use crate::application::binding as binding_service;
use crate::application::physical_machine as service;

#[tauri::command]
#[specta::specta]
pub(crate) fn physical_ports() -> Result<Vec<SerialPortCandidate>, String> {
    crate::application::devices::ports()
}

#[tauri::command]
#[specta::specta]
pub(crate) fn physical_baud_choices() -> Vec<u32> {
    vec![1_000_000, 576_000, 115_200, 57_600]
}

#[tauri::command]
#[specta::specta]
pub(crate) fn physical_machine_probe(
    state: State<'_, DesktopState>,
    input: PhysicalMachineProbeInput,
) -> Result<DetectedPhysicalMachine, String> {
    service::physical_machine_probe(state.inner(), input)
}

#[tauri::command]
#[specta::specta]
pub(crate) fn physical_machine_detect(
    state: State<'_, DesktopState>,
    input: PhysicalMachineDetectInput,
) -> Result<DetectedPhysicalMachine, String> {
    service::physical_machine_detect(state.inner(), input)
}

#[tauri::command]
#[specta::specta]
pub(crate) fn physical_machine_save(
    state: State<'_, DesktopState>,
    input: PhysicalMachineSaveInput,
) -> Result<PhysicalMachine, String> {
    service::physical_machine_save(state.inner(), input)
}

#[tauri::command]
#[specta::specta]
pub(crate) fn physical_machine_disconnect(
    state: State<'_, DesktopState>,
    id: String,
) -> Result<PhysicalMachine, String> {
    service::physical_machine_disconnect(state.inner(), id)
}

#[tauri::command]
#[specta::specta]
pub(crate) fn physical_machine_reconnect(
    state: State<'_, DesktopState>,
    input: PhysicalMachineReconnectInput,
) -> Result<PhysicalMachine, String> {
    service::physical_machine_reconnect(state.inner(), input)
}

#[tauri::command]
#[specta::specta]
pub(crate) fn physical_machine_auto_reconnect(
    state: State<'_, DesktopState>,
    id: String,
    baud: u32,
) -> Result<PhysicalMachine, String> {
    crate::application::devices::auto_reconnect(state.inner(), id, baud)
}

#[tauri::command]
#[specta::specta]
pub(crate) fn physical_machine_create(
    state: State<'_, DesktopState>,
    input: PhysicalMachineInput,
) -> Result<PhysicalMachine, String> {
    service::physical_machine_create(state.inner(), input)
}

#[tauri::command]
#[specta::specta]
pub(crate) fn physical_machine_connect(
    state: State<'_, DesktopState>,
    id: String,
) -> Result<PhysicalMachine, String> {
    service::physical_machine_connect(state.inner(), id)
}

#[tauri::command]
#[specta::specta]
pub(crate) fn physical_machine_scan(
    state: State<'_, DesktopState>,
    id: String,
) -> Result<PhysicalMachine, String> {
    service::physical_machine_scan(state.inner(), id)
}

#[tauri::command]
#[specta::specta]
pub(crate) fn physical_machine_mount(
    state: State<'_, DesktopState>,
    id: String,
    virtual_machine_id: String,
) -> Result<PhysicalMachine, String> {
    service::physical_machine_mount(state.inner(), id, virtual_machine_id)
}

#[tauri::command]
#[specta::specta]
pub(crate) fn physical_machine_delete(
    state: State<'_, DesktopState>,
    id: String,
) -> Result<(), String> {
    service::physical_machine_delete(state.inner(), id)
}

#[tauri::command]
#[specta::specta]
pub(crate) fn physical_machine_rename(
    state: State<'_, DesktopState>,
    id: String,
    name: String,
) -> Result<PhysicalMachine, String> {
    service::physical_machine_rename(state.inner(), id, name)
}

#[tauri::command]
#[specta::specta]
pub(crate) fn physical_machine_bind(
    state: State<'_, DesktopState>,
    input: PhysicalMachineBindingInput,
) -> Result<PhysicalMachine, String> {
    binding_service::physical_machine_bind(state.inner(), input)
}

#[tauri::command]
#[specta::specta]
pub(crate) fn physical_machine_validate_bindings(
    state: State<'_, DesktopState>,
    id: String,
) -> Result<PhysicalMachine, String> {
    binding_service::physical_machine_validate_bindings(state.inner(), id)
}

#[tauri::command]
#[specta::specta]
pub(crate) fn physical_machine_set_pose_zero(
    state: State<'_, DesktopState>,
    id: String,
    virtual_machine_id: Option<String>,
) -> Result<PhysicalMachine, String> {
    crate::application::debug_machine::set_pose_zero(state.inner(), id, virtual_machine_id)
}

#[tauri::command]
#[specta::specta]
pub(crate) fn physical_machine_pose(
    state: State<'_, DesktopState>,
    id: String,
    virtual_machine_id: Option<String>,
) -> Result<Vec<f64>, String> {
    crate::application::debug_machine::pose(state.inner(), id, virtual_machine_id)
}

#[tauri::command]
#[specta::specta]
pub(crate) fn physical_machine_set_torque(
    state: State<'_, DesktopState>,
    id: String,
    enabled: bool,
    virtual_machine_id: Option<String>,
) -> Result<PhysicalMachine, String> {
    crate::application::debug_machine::set_torque_state(
        state.inner(),
        id,
        enabled,
        virtual_machine_id,
    )
}

#[tauri::command]
#[specta::specta]
pub(crate) fn physical_machine_write_position(
    state: State<'_, DesktopState>,
    id: String,
    servo_id: u8,
    position_rad: f64,
) -> Result<PhysicalMachine, String> {
    crate::application::debug_machine::write_servo_position(
        state.inner(),
        id,
        servo_id,
        position_rad,
    )
}
