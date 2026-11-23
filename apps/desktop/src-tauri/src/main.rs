#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]

mod platform;
mod runtime;
mod training;

pub use platform::state::types;
use platform::{commands, lifecycle};

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

#[cfg(test)]
mod test_support {
    pub(crate) use crate::runtime::policy::catalog::{
        cached_policy_selections, list as policy_list,
    };
    pub(crate) use crate::runtime::session::contract::register_simulation;

    pub(crate) fn simulation_selection(
        model_kind: microduck_body_runtime::ModelKind,
    ) -> microduck_body_runtime::Selection {
        microduck_body_runtime::Selection {
            execution_kind: microduck_body_runtime::ExecutionKind::Simulation,
            model_kind,
            servo_model: model_kind.servo_model(),
        }
    }
}
