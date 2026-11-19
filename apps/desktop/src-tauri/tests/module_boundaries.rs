use std::path::Path;

fn rust_files(root: &Path) -> Vec<std::path::PathBuf> {
    if root.is_file() {
        return vec![root.to_owned()];
    }
    let mut files = Vec::new();
    let mut pending = vec![root.to_owned()];
    while let Some(path) = pending.pop() {
        let entries = std::fs::read_dir(&path).expect("module directory should be readable");
        for entry in entries {
            let entry = entry.expect("module directory entry should be readable");
            let path = entry.path();
            if path.is_dir() {
                pending.push(path);
            } else if path.extension().is_some_and(|extension| extension == "rs") {
                files.push(path);
            }
        }
    }
    files
}

fn assert_tree_does_not_import(root: &Path, forbidden: &[&str]) {
    for file in rust_files(root) {
        let source = std::fs::read_to_string(&file).expect("module source should be readable");
        for import in forbidden {
            assert!(
                !source.contains(import),
                "{} must not import {import}",
                file.display()
            );
        }
    }
}

#[test]
fn src_tauri_has_one_entry_for_each_runtime_boundary() {
    let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("src");
    for directory in ["application", "backend", "commands", "lifecycle", "state"] {
        assert!(
            root.join(directory).is_dir(),
            "missing src-tauri module boundary: {directory}"
        );
    }
    assert!(root.join("control.rs").is_file());
    assert!(root.join("helpers.rs").is_file());
    assert!(root.join("logging.rs").is_file());
    assert!(root.join("backend/physical_backend.rs").is_file());
    assert!(root.join("backend/control/mod.rs").is_file());
    assert!(root.join("backend/control/transport.rs").is_file());
    assert!(root.join("backend/control/registers.rs").is_file());
    assert!(root.join("backend/control/driver.rs").is_file());
    assert!(!root.join("backend/control.rs").exists());
    assert!(root.join("application/loop_service.rs").is_file());
    assert!(root.join("application/binding.rs").is_file());
    assert!(root.join("application/debug_machine.rs").is_file());
    assert!(root.join("application/preview.rs").is_file());
    assert!(root.join("application/registers.rs").is_file());
    assert!(root.join("application/actions.rs").is_file());
    assert!(root.join("application/training.rs").is_file());
    assert!(root.join("state/motion.rs").is_file());
    assert!(!root.join("application/motion.rs").exists());
    assert!(root.join("commands/machines/physical.rs").is_file());
    assert!(root.join("commands/machines/registers.rs").is_file());
    assert!(root.join("commands/machines/virtual_machine.rs").is_file());

    // The source root contains one application entrypoint and the explicitly allowed small
    // single-file modules. Keep old monolithic implementations absent so future features land in
    // the owning boundary instead.
    for legacy_module in [
        "lib.rs",
        "client.rs",
        "commands.rs",
        "device_watcher.rs",
        "motion.rs",
        "physical.rs",
        "policy.rs",
        "simulation.rs",
        "state.rs",
        "types.rs",
    ] {
        assert!(
            !root.join(legacy_module).exists(),
            "legacy implementation module remains at src root: {legacy_module}"
        );
    }
}

#[test]
fn module_guide_keeps_the_dependency_direction_visible() {
    let guide = std::fs::read_to_string(Path::new(env!("CARGO_MANIFEST_DIR")).join("README.md"))
        .expect("src-tauri module guide should be present");
    assert!(guide.contains("commands -> application/control -> backend/state"));
    assert!(guide.contains("Keep serial") && guide.contains("actuator dynamics out"));
}

#[test]
fn lifecycle_and_command_boundaries_do_not_regress() {
    let source_root = Path::new(env!("CARGO_MANIFEST_DIR")).join("src");
    let watcher = std::fs::read_to_string(source_root.join("lifecycle/device_watcher.rs"))
        .expect("device watcher should be readable");
    assert!(watcher.contains("crate::application::devices"));
    assert!(!watcher.contains("crate::commands"));

    let commands = std::fs::read_to_string(source_root.join("commands/mod.rs"))
        .expect("command registration should be readable");
    for domain in ["actions::", "policy::", "loop_control::"] {
        assert!(
            commands.contains(domain),
            "missing command domain: {domain}"
        );
    }
    assert!(commands.contains("machines::physical::physical_machine_connect"));
    assert!(commands.contains("training::training_start"));
    assert!(!commands.contains("client::physical_machine_connect"));
    assert!(!commands.contains("client::loop_tick"));
    assert!(!commands.contains("client::policy_list"));

    assert!(!source_root.join("commands/client.rs").exists());

    let machines = std::fs::read_to_string(source_root.join("commands/machines.rs"))
        .expect("machine commands should be readable");
    assert!(!machines.contains("pub(crate) fn loop_open"));
    assert!(!machines.contains("open_backend_driver(") && !machines.contains("PolicyLoop::open("));
    assert!(machines.contains("application::devices as service"));
    assert!(!machines.contains("state.virtual_machines.lock()"));
    assert!(!machines.contains("ProcessCommand"));

    let loop_commands = std::fs::read_to_string(source_root.join("commands/loop_control.rs"))
        .expect("loop commands should be readable");
    assert!(loop_commands.contains("loop_service::start"));
    assert!(loop_commands.contains("pub(crate) fn runtime_open"));
    assert!(loop_commands.contains("pub(crate) fn runtime_close"));

    let action_commands = std::fs::read_to_string(source_root.join("commands/actions.rs"))
        .expect("action commands should be readable");
    assert!(action_commands.contains("application::actions as service"));
    assert!(!action_commands.contains("state.actions.lock()"));
    assert!(!action_commands.contains("ActionKind"));

    let training_commands = std::fs::read_to_string(source_root.join("commands/training.rs"))
        .expect("training commands should be readable");
    assert!(training_commands.contains("application::training as service"));
    assert!(!training_commands.contains("ProcessCommand"));
    assert!(!training_commands.contains("training_process.lock()"));
    assert!(!training_commands.contains("parse_training_line"));

    let policy_commands = std::fs::read_to_string(source_root.join("commands/policy.rs"))
        .expect("policy commands should be readable");
    assert!(policy_commands.contains("policy::open_folder"));
    assert!(!policy_commands.contains("ProcessCommand"));

    let loop_service = std::fs::read_to_string(source_root.join("application/loop_service.rs"))
        .expect("loop application service should be readable");
    assert!(loop_service.contains("pub(crate) fn open"));
    assert!(loop_service.contains("control::open_backend_driver"));
    assert!(loop_service.contains("control::open_loop"));
    assert!(!loop_service.contains("tauri::command"));

    let state = std::fs::read_to_string(source_root.join("state/mod.rs"))
        .expect("state module should be readable");
    assert!(state.contains("mod motion"));
    assert!(!state.contains("crate::application"));

    let control = std::fs::read_to_string(source_root.join("backend/control/mod.rs"))
        .expect("backend control facade should be readable");
    assert!(control.contains("mod transport"));
    assert!(control.contains("mod registers"));
    assert!(control.contains("mod driver"));

    let virtual_commands =
        std::fs::read_to_string(source_root.join("commands/machines/virtual_machine.rs"))
            .expect("virtual-machine commands should be readable");
    assert!(virtual_commands.contains("application::virtual_machine as service"));
    assert!(!virtual_commands.contains("state.virtual_machines.lock()"));

    let physical_commands =
        std::fs::read_to_string(source_root.join("commands/machines/physical.rs"))
            .expect("physical-machine commands should be readable");
    assert!(physical_commands.contains("application::physical_machine as service"));
    assert!(physical_commands.contains("service::physical_machine_create"));
    assert!(physical_commands.contains("service::physical_machine_probe"));
    assert!(
        !physical_commands.contains("try_open_feetech")
            && !physical_commands.contains("try_open_xl330")
    );
    assert!(physical_commands.contains("application::debug_machine"));
    assert!(!physical_commands.contains("connected_buses.lock()"));

    let register_commands =
        std::fs::read_to_string(source_root.join("commands/machines/registers.rs"))
            .expect("register commands should be readable");
    assert!(register_commands.contains("application::registers as service"));
    assert!(!register_commands.contains("connected_buses.lock()"));

    let preview_commands = std::fs::read_to_string(source_root.join("commands/loop_control.rs"))
        .expect("loop commands should be readable");
    assert!(preview_commands.contains("application::{loop_service, preview}"));
    assert!(!preview_commands.contains("PreviewPolicyLoop"));
    assert!(preview_commands.contains("pub(crate) fn runtime_command"));
    for removed in [
        "fn loop_tick",
        "fn loop_command",
        "fn loop_action",
        "fn preview_loop_tick",
        "fn preview_loop_policy",
        "fn preview_loop_command",
    ] {
        assert!(
            !preview_commands.contains(removed),
            "legacy mutation entry remains: {removed}"
        );
    }

    for service in [
        "application/binding.rs",
        "application/debug_machine.rs",
        "application/physical_machine.rs",
        "application/preview.rs",
        "application/virtual_machine.rs",
        "application/actions.rs",
        "application/training.rs",
    ] {
        let source = std::fs::read_to_string(source_root.join(service))
            .expect("application service should be readable");
        assert!(
            !source.contains("tauri::command"),
            "service leaked Tauri command: {service}"
        );
    }
    assert!(
        std::fs::read_to_string(source_root.join("commands/machines/physical.rs"))
            .expect("physical commands should be readable")
            .contains("application::binding as binding_service")
    );

    assert_tree_does_not_import(&source_root.join("application"), &["crate::commands"]);
    assert_tree_does_not_import(
        &source_root.join("backend"),
        &["crate::application", "crate::commands"],
    );
    assert_tree_does_not_import(
        &source_root.join("control.rs"),
        &["crate::application", "crate::commands"],
    );
    assert_tree_does_not_import(
        &source_root.join("helpers.rs"),
        &[
            "crate::application",
            "crate::backend",
            "crate::commands",
            "crate::control",
            "crate::lifecycle",
            "crate::logging",
            "crate::state",
        ],
    );
    assert_tree_does_not_import(&source_root.join("lifecycle"), &["crate::commands"]);
    assert_tree_does_not_import(
        &source_root.join("logging.rs"),
        &["crate::application", "crate::commands"],
    );
    assert_tree_does_not_import(
        &source_root.join("state"),
        &[
            "crate::application",
            "crate::commands",
            "crate::control",
            "crate::lifecycle",
        ],
    );
}
