//! Tauri command registration and thin frontend boundary.
//!
//! Command handlers validate inputs and delegate to application/control services. Backend and
//! persistence implementation belongs in their respective modules.

use tauri_specta::{Builder, collect_commands};

pub(crate) mod actions;
pub(crate) mod logs;
pub(crate) mod loop_control;
pub(crate) mod machines;
pub(crate) mod policy;
pub(crate) mod scene;
pub(crate) mod training;

pub(crate) fn command_builder<R: tauri::Runtime>() -> Builder<R> {
    Builder::<R>::new()
        .disable_serde_phases()
        .typ::<crate::types::TrainingFrame>()
        .commands(collect_commands![
            logs::recording_list,
            logs::recording_start,
            logs::recording_stop,
            logs::recording_frames,
            logs::recording_compare,
            logs::recording_export,
            logs::diagnostic_tail,
            logs::frontend_diagnostic,
            scene::servo_catalog,
            scene::official_joint_slots,
            scene::official_joint_ranges,
            scene::official_scene_model,
            scene::official_scene_frame,
            machines::virtual_machines,
            machines::physical_machines,
            machines::auto_mount_enabled,
            machines::set_auto_mount_enabled,
            machines::auto_reconnect_enabled,
            machines::set_auto_reconnect_enabled,
            machines::physical::physical_ports,
            machines::physical::physical_baud_choices,
            machines::physical::physical_machine_probe,
            machines::physical::physical_machine_detect,
            machines::physical::physical_machine_save,
            machines::physical::physical_machine_disconnect,
            machines::physical::physical_machine_reconnect,
            machines::physical::physical_machine_auto_reconnect,
            machines::physical::physical_machine_create,
            machines::physical::physical_machine_connect,
            machines::physical::physical_machine_scan,
            machines::registers::physical_machine_registers,
            machines::registers::physical_machine_read_register,
            machines::registers::physical_machine_write_register,
            machines::physical::physical_machine_mount,
            machines::physical::physical_machine_delete,
            machines::physical::physical_machine_rename,
            machines::physical::physical_machine_bind,
            machines::virtual_machine::virtual_machine_bind,
            machines::virtual_machine::virtual_machine_bindings,
            machines::virtual_machine::virtual_machine_validate_bindings,
            machines::physical::physical_machine_validate_bindings,
            machines::physical::physical_machine_set_pose_zero,
            machines::physical::physical_machine_center_encoders,
            machines::physical::physical_machine_pose,
            machines::physical::physical_machine_set_torque,
            machines::physical::physical_machine_write_position,
            machines::virtual_machine::virtual_machine_create,
            machines::virtual_machine::virtual_machine_set_startup,
            machines::virtual_machine::virtual_machine_set_world_position,
            machines::virtual_machine::virtual_machine_delete,
            machines::virtual_machine::virtual_machine_rename,
            actions::actions,
            actions::action_create,
            actions::action_delete,
            policy::policy_list,
            policy::policy_download,
            policy::policy_select,
            policy::policy_open_folder,
            training::training_start,
            training::training_evaluate_teacher,
            training::training_stop,
            training::training_status,
            machines::open_logs,
            loop_control::runtime_open,
            loop_control::runtime_reconfigure,
            loop_control::runtime_select_body,
            loop_control::preview_machine_select,
            loop_control::runtime_command,
            loop_control::loop_status,
            loop_control::runtime_close,
        ])
}
