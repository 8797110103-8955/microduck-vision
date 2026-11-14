use serde::Deserialize;
use specta_typescript::Typescript;
use tauri::Manager;
use tauri_specta::{Builder, collect_commands};
pub mod physical;
pub mod types;
mod client;
mod control;
mod helpers;
mod policy;
#[allow(dead_code)]
mod simulation;
mod state;

use state::DesktopState;
use types::{ReconnectHint, ServoIdentity, SimulationParameters};

#[derive(Debug, Clone, Deserialize, specta::Type)]
#[serde(rename_all = "camelCase")]
pub struct ActionInput {
    pub name: String,
    pub kind: String,
}

#[derive(Debug, Clone, Deserialize, specta::Type)]
#[serde(rename_all = "camelCase")]
pub struct VirtualMachineInput {
    pub name: String,
    pub body_id: String,
    pub servo_model: String,
    pub simulation: SimulationParameters,
}

#[derive(Debug, Clone, Deserialize, specta::Type)]
#[serde(rename_all = "camelCase")]
pub struct PhysicalMachineInput {
    pub name: String,
    pub usb_port: String,
    pub machine_code: String,
}

#[derive(Debug, Clone, Deserialize, specta::Type)]
#[serde(rename_all = "camelCase")]
pub struct PhysicalMachineDetectInput {
    pub census: Vec<ServoIdentity>,
    pub reconnect_hint: Option<ReconnectHint>,
}

#[derive(Debug, Clone, Deserialize, specta::Type)]
#[serde(rename_all = "camelCase")]
pub struct PhysicalMachineSaveInput {
    pub census: Vec<ServoIdentity>,
    pub reconnect_hint: Option<ReconnectHint>,
    pub name: Option<String>,
}

#[derive(Debug, Clone, Deserialize, specta::Type)]
#[serde(rename_all = "camelCase")]
pub struct PhysicalMachineProbeInput {
    pub port_name: String,
    pub baud: u32,
    pub reconnect_hint: Option<ReconnectHint>,
}

#[derive(Debug, Clone, Deserialize, specta::Type)]
#[serde(rename_all = "camelCase")]
pub struct PhysicalMachineReconnectInput {
    pub id: String,
    pub census: Vec<ServoIdentity>,
    pub reconnect_hint: Option<ReconnectHint>,
}

#[derive(Debug, Clone, Deserialize, specta::Type)]
#[serde(rename_all = "camelCase")]
pub struct LoopOpenInput {
    pub virtual_machine_id: String,
    pub servo_ids: Vec<u8>,
    #[serde(default)]
    pub physical_machine_id: Option<String>,
    #[serde(default)]
    pub policy_id: Option<String>,
}

pub fn command_builder() -> Builder<tauri::Wry> {
    Builder::<tauri::Wry>::new()
        .disable_serde_phases()
        .commands(collect_commands![
            client::servo_catalog,
            client::official_scene_model,
            client::official_scene_frame,
            client::virtual_machines,
            client::physical_machines,
            client::auto_mount_enabled,
            client::set_auto_mount_enabled,
            client::auto_reconnect_enabled,
            client::set_auto_reconnect_enabled,
            client::physical_ports,
            client::physical_baud_choices,
            client::physical_machine_probe,
            client::physical_machine_detect,
            client::physical_machine_save,
            client::physical_machine_disconnect,
            client::physical_machine_reconnect,
            client::physical_machine_auto_reconnect,
            client::physical_machine_create,
            client::physical_machine_connect,
            client::physical_machine_scan,
            client::physical_machine_mount,
            client::physical_machine_delete,
            client::physical_machine_rename,
            client::virtual_machine_create,
            client::virtual_machine_set_startup,
            client::virtual_machine_delete,
            client::virtual_machine_rename,
            client::actions,
            client::action_create,
            client::action_delete,
            client::policy_list,
            client::policy_download,
            client::policy_select,
            client::loop_open,
            client::loop_command,
            client::loop_action,
            client::loop_tick,
            client::loop_status,
            client::loop_close,
            client::preview_loop_open,
            client::preview_loop_policy,
            client::preview_loop_tick,
            client::preview_loop_close,
        ])
}

pub fn export_command_bindings() {
    let output = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("..")
        .join("generated/types.ts");
    command_builder()
        .export(Typescript::default(), output)
        .expect("failed to export Tauri command bindings");
}

pub fn run() {
    let builder = command_builder();
    tauri::Builder::default()
        .setup(|app| {
            let config_dir = app
                .path()
                .app_config_dir()
                .map_err(|error| std::io::Error::other(error.to_string()))?;
            app.manage(DesktopState::load(config_dir));
            Ok(())
        })
        .plugin(
            tauri_plugin_snap_layout::init()
                .button_id("snap-btn")
                .build(),
        )
        .invoke_handler(builder.invoke_handler())
        .run(tauri::generate_context!())
        .expect("error while running Microduck");
}
