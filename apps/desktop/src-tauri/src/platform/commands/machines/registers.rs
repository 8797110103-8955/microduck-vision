//! Servo register Tauri commands.
//!
//! Register reads and writes are delegated to the register application service.

use super::*;
use crate::runtime::machines::registers as service;
#[tauri::command]
#[specta::specta]
pub(crate) fn physical_machine_registers(
    state: State<'_, DesktopState>,
    id: String,
) -> Result<Vec<PhysicalServoRegister>, String> {
    service::physical_machine_registers(state.inner(), id)
}

#[tauri::command]
#[specta::specta]
pub(crate) fn physical_machine_read_register(
    state: State<'_, DesktopState>,
    id: String,
    servo_id: u8,
    name: String,
) -> Result<PhysicalServoRegister, String> {
    service::physical_machine_read_register(state.inner(), id, servo_id, name)
}

#[tauri::command]
#[specta::specta]
pub(crate) fn physical_machine_write_register(
    state: State<'_, DesktopState>,
    input: PhysicalMachineRegisterWriteInput,
) -> Result<PhysicalServoRegister, String> {
    service::physical_machine_write_register(state.inner(), input)
}
