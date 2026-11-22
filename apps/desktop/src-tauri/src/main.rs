#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]

mod application;
mod backend;
mod commands;
mod control;
mod helpers;
mod lifecycle;
mod logging;
mod recordings;
mod state;
pub use state::types;

pub use types::{
    ActionInput, LoopOpenInput, PhysicalMachineBindingInput, PhysicalMachineDetectInput,
    PhysicalMachineInput, PhysicalMachineProbeInput, PhysicalMachineReconnectInput,
    PhysicalMachineRegisterWriteInput, PhysicalMachineSaveInput, ReconnectHint, ServoIdentity,
    SimulationParameters, VirtualMachineBindingInput, VirtualMachineInput,
};

fn main() {
    let builder = commands::command_builder::<tauri::Wry>();
    let app = tauri::Builder::default()
        .setup(lifecycle::setup)
        .plugin(
            tauri_plugin_snap_layout::init()
                .button_id("snap-btn")
                .build(),
        )
        .plugin(tauri_plugin_opener::init())
        .invoke_handler(builder.invoke_handler())
        .build(tauri::generate_context!())
        .expect("error while building Microduck");
    app.run(lifecycle::handle_event);
}
