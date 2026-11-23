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
    for directory in [
        "platform/commands",
        "platform/lifecycle",
        "platform/state",
        "runtime/adapters",
        "runtime/machines",
        "runtime/policy",
        "runtime/session",
        "training",
    ] {
        assert!(
            root.join(directory).is_dir(),
            "missing src-tauri module boundary: {directory}"
        );
    }
    assert!(root.join("platform/helpers.rs").is_file());
    assert!(root.join("platform/logging.rs").is_file());
    assert!(root.join("runtime/adapters/physical_backend.rs").is_file());
    assert!(root.join("runtime/adapters/servo_bus/mod.rs").is_file());
    assert!(
        root.join("runtime/adapters/servo_bus/transport.rs")
            .is_file()
    );
    assert!(
        root.join("runtime/adapters/servo_bus/registers.rs")
            .is_file()
    );
    assert!(root.join("runtime/adapters/servo_bus/driver.rs").is_file());
    assert!(!root.join("runtime/adapters/servo_bus.rs").exists());
    assert!(root.join("runtime/session/loop_service.rs").is_file());
    assert!(root.join("runtime/session/state.rs").is_file());
    assert!(root.join("runtime/session/execution.rs").is_file());
    assert!(root.join("runtime/session/contract.rs").is_file());
    assert!(root.join("platform/lifecycle/runtime_events.rs").is_file());
    assert!(root.join("runtime/scene.rs").is_file());
    assert!(root.join("runtime/machines/binding.rs").is_file());
    assert!(root.join("runtime/machines/debug_machine.rs").is_file());
    assert!(root.join("runtime/session/preview.rs").is_file());
    assert!(root.join("runtime/machines/registers.rs").is_file());
    assert!(root.join("runtime/session/actions.rs").is_file());
    assert!(root.join("training/process.rs").is_file());
    assert!(root.join("platform/state/motion.rs").is_file());
    assert!(!root.join("runtime/session/motion.rs").exists());
    assert!(
        root.join("platform/commands/machines/physical.rs")
            .is_file()
    );
    assert!(
        root.join("platform/commands/machines/registers.rs")
            .is_file()
    );
    assert!(
        root.join("platform/commands/machines/virtual_machine.rs")
            .is_file()
    );

    // Keep implementations in their owning module and leave main.rs at the source root.
    for legacy_module in [
        "application",
        "backend",
        "commands",
        "lifecycle",
        "state",
        "lib.rs",
        "control.rs",
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
fn lifecycle_and_command_boundaries_do_not_regress() {
    let source_root = Path::new(env!("CARGO_MANIFEST_DIR")).join("src");
    let watcher = std::fs::read_to_string(source_root.join("platform/lifecycle/device_watcher.rs"))
        .expect("device watcher should be readable");
    assert!(watcher.contains("crate::runtime::machines::devices"));
    assert!(!watcher.contains("crate::platform::commands"));

    let commands = std::fs::read_to_string(source_root.join("platform/commands/mod.rs"))
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

    assert!(!source_root.join("platform/commands/client.rs").exists());

    let machines = std::fs::read_to_string(source_root.join("platform/commands/machines.rs"))
        .expect("machine commands should be readable");
    assert!(!machines.contains("pub(crate) fn loop_open"));
    assert!(!machines.contains("open_backend_driver(") && !machines.contains("PolicyLoop::open("));
    assert!(machines.contains("runtime::machines::devices as service"));
    assert!(!machines.contains("state.virtual_machines.lock()"));
    assert!(!machines.contains("ProcessCommand"));

    let loop_commands =
        std::fs::read_to_string(source_root.join("platform/commands/loop_control.rs"))
            .expect("loop commands should be readable");
    assert!(loop_commands.contains("loop_service::start"));
    assert!(loop_commands.contains("pub(crate) fn runtime_open"));
    assert!(loop_commands.contains("pub(crate) fn runtime_close"));

    let action_commands = std::fs::read_to_string(source_root.join("platform/commands/actions.rs"))
        .expect("action commands should be readable");
    assert!(action_commands.contains("runtime::session::actions as service"));
    assert!(!action_commands.contains("state.actions.lock()"));
    assert!(!action_commands.contains("ActionKind"));

    let training_commands =
        std::fs::read_to_string(source_root.join("platform/commands/training.rs"))
            .expect("training commands should be readable");
    assert!(training_commands.contains("training::process as service"));
    assert!(!training_commands.contains("ProcessCommand"));
    assert!(!training_commands.contains("training_process.lock()"));
    assert!(!training_commands.contains("parse_training_line"));

    let policy_commands = std::fs::read_to_string(source_root.join("platform/commands/policy.rs"))
        .expect("policy commands should be readable");
    assert!(policy_commands.contains("policy::open_folder"));
    assert!(!policy_commands.contains("ProcessCommand"));

    let loop_service = std::fs::read_to_string(source_root.join("runtime/session/loop_service.rs"))
        .expect("loop application service should be readable");
    assert!(loop_service.contains("pub(crate) fn open"));
    assert!(loop_service.contains("physical_backend::{PhysicalOpenContext, open_backend_driver}"));
    assert!(loop_service.contains("open_backend_driver("));
    assert!(loop_service.contains("open_loop(OpenLoopInput {"));
    assert!(!loop_service.contains("tauri::command"));

    let state = std::fs::read_to_string(source_root.join("platform/state/mod.rs"))
        .expect("state module should be readable");
    assert!(state.contains("mod motion"));
    assert!(!state.contains("crate::runtime::session"));
    assert!(!state.contains("RuntimeInstance"));
    assert!(
        !source_root
            .join("runtime/policy/inference/execution.rs")
            .exists()
    );

    let control = std::fs::read_to_string(source_root.join("runtime/adapters/servo_bus/mod.rs"))
        .expect("backend control facade should be readable");
    assert!(control.contains("mod transport"));
    assert!(control.contains("mod registers"));
    assert!(control.contains("mod driver"));

    let virtual_commands =
        std::fs::read_to_string(source_root.join("platform/commands/machines/virtual_machine.rs"))
            .expect("virtual-machine commands should be readable");
    assert!(virtual_commands.contains("runtime::machines::virtual_machine as service"));
    assert!(!virtual_commands.contains("state.virtual_machines.lock()"));

    let physical_commands =
        std::fs::read_to_string(source_root.join("platform/commands/machines/physical.rs"))
            .expect("physical-machine commands should be readable");
    assert!(physical_commands.contains("runtime::machines::physical_machine as service"));
    assert!(physical_commands.contains("service::physical_machine_create"));
    assert!(physical_commands.contains("service::physical_machine_probe"));
    assert!(
        !physical_commands.contains("try_open_feetech")
            && !physical_commands.contains("try_open_xl330")
    );
    assert!(physical_commands.contains("runtime::machines::debug_machine"));
    assert!(!physical_commands.contains("connected_buses.lock()"));

    let register_commands =
        std::fs::read_to_string(source_root.join("platform/commands/machines/registers.rs"))
            .expect("register commands should be readable");
    assert!(register_commands.contains("runtime::machines::registers as service"));
    assert!(!register_commands.contains("connected_buses.lock()"));

    let preview_commands =
        std::fs::read_to_string(source_root.join("platform/commands/loop_control.rs"))
            .expect("loop commands should be readable");
    assert!(preview_commands.contains("runtime::session::{loop_service, preview}"));
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
        "runtime/machines/binding.rs",
        "runtime/machines/debug_machine.rs",
        "runtime/machines/physical_machine.rs",
        "runtime/session/preview.rs",
        "runtime/machines/virtual_machine.rs",
        "runtime/session/actions.rs",
        "training/process.rs",
    ] {
        let source = std::fs::read_to_string(source_root.join(service))
            .expect("application service should be readable");
        assert!(
            !source.contains("tauri::command"),
            "service leaked Tauri command: {service}"
        );
    }
    assert!(
        std::fs::read_to_string(source_root.join("platform/commands/machines/physical.rs"))
            .expect("physical commands should be readable")
            .contains("runtime::machines::binding as binding_service")
    );

    let services = [
        "runtime/machines",
        "runtime/policy/catalog.rs",
        "runtime/scene.rs",
        "runtime/session",
        "training",
    ];
    for service in services {
        assert_tree_does_not_import(&source_root.join(service), &["crate::platform::commands"]);
    }
    let service_imports = [
        "crate::runtime::machines",
        "crate::runtime::policy::catalog",
        "crate::runtime::scene",
        "crate::runtime::session",
        "crate::training",
    ];
    for module in [
        "runtime/adapters",
        "runtime/policy/inference",
        "platform/logging.rs",
    ] {
        let root = source_root.join(module);
        assert_tree_does_not_import(&root, &service_imports);
        assert_tree_does_not_import(&root, &["crate::platform::commands"]);
    }
    assert_tree_does_not_import(
        &source_root.join("platform/helpers.rs"),
        &[
            "crate::runtime::adapters",
            "crate::runtime::policy",
            "crate::platform::commands",
            "crate::control",
            "crate::platform::lifecycle",
            "crate::platform::logging",
            "crate::platform::state",
        ],
    );
    assert_tree_does_not_import(&source_root.join("platform/helpers.rs"), &service_imports);
    assert_tree_does_not_import(
        &source_root.join("platform/lifecycle"),
        &["crate::platform::commands"],
    );
    assert_tree_does_not_import(&source_root.join("platform/state"), &service_imports);
    assert_tree_does_not_import(
        &source_root.join("platform/state"),
        &[
            "crate::platform::commands",
            "crate::control",
            "crate::platform::lifecycle",
        ],
    );
}
