use tauri::Manager;
mod client;
mod commands;
mod control;
mod device_watcher;
mod helpers;
mod logging;
mod motion;
pub mod physical;
mod policy;
#[allow(dead_code)]
mod simulation;
mod state;
pub mod types;

use state::DesktopState;
pub use types::{
    ActionInput, LoopOpenInput, PhysicalMachineBindingInput, PhysicalMachineDetectInput,
    PhysicalMachineInput, PhysicalMachineProbeInput, PhysicalMachineReconnectInput,
    PhysicalMachineRegisterWriteInput, PhysicalMachineSaveInput, ReconnectHint, ServoIdentity,
    SimulationParameters, VirtualMachineBindingInput, VirtualMachineInput,
};

pub fn run() {
    let builder = commands::command_builder::<tauri::Wry>();
    tauri::Builder::default()
        .setup(|app| {
            let config_dir = app
                .path()
                .app_config_dir()
                .map_err(|error| std::io::Error::other(error.to_string()))?;
            let state = DesktopState::load(config_dir);
            state.set_app_handle(app.handle().clone());
            app.manage(state);
            app.manage(device_watcher::DeviceWatcher::start(app.handle().clone()));
            Ok(())
        })
        .plugin(
            tauri_plugin_snap_layout::init()
                .button_id("snap-btn")
                .build(),
        )
        .plugin(tauri_plugin_opener::init())
        .invoke_handler(builder.invoke_handler())
        .run(tauri::generate_context!())
        .expect("error while running Microduck");
}
