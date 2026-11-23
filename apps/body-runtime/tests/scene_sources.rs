use std::path::Path;

use duck_body::AssetResolver;
use microduck_body_runtime::models::assets::{OfficialAssets, display_scene, load_official_body};
use microduck_body_runtime::models::resolve::defaults;
use microduck_body_runtime::{ExecutionKind, ModelKind, Selection, resolve_simulation};

fn selection(model_kind: ModelKind) -> Selection {
    Selection {
        execution_kind: ExecutionKind::Simulation,
        model_kind,
        servo_model: model_kind.servo_model(),
    }
}

#[test]
fn embedded_and_pinned_file_assets_have_the_same_relative_ids_and_digests() {
    let root = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../vendor/microduck_rl/src/mjlab_microduck/robot/microduck");
    let file = OfficialAssets::Files(&root);
    let embedded = OfficialAssets::Embedded;
    let file_body = load_official_body(&file).unwrap();
    let body = load_official_body(&embedded).unwrap();
    assert_eq!(
        file_body.compatibility_snapshot(),
        body.compatibility_snapshot()
    );
    let file_display = display_scene(&file_body, &file).unwrap();
    let embedded_display = display_scene(&body, &embedded).unwrap();
    assert_eq!(file_display.model(), embedded_display.model());
    assert_eq!(
        file_display
            .frame(&body.template().control.home_pose)
            .unwrap(),
        embedded_display
            .frame(&body.template().control.home_pose)
            .unwrap(),
    );
    for model in [
        ModelKind::Xl330,
        ModelKind::Hd1910,
        ModelKind::Hd1910Compatible,
    ] {
        let from_file =
            resolve_simulation(selection(model), &body, &file, defaults(model)).unwrap();
        let from_memory =
            resolve_simulation(selection(model), &body, &embedded, defaults(model)).unwrap();
        assert_eq!(from_file, from_memory);
        let profile = from_memory.simulation.unwrap();
        assert!(
            profile
                .source_assets
                .iter()
                .any(|asset| asset.path == profile.scenario.robot_xml)
        );
        assert!(
            profile
                .source_assets
                .iter()
                .any(|asset| asset.path.starts_with("assets/") && asset.path.ends_with(".stl"))
        );
        assert_eq!(
            embedded
                .read(Path::new(profile.scenario.scene_xml.unwrap()))
                .unwrap()
                .as_ref(),
            std::fs::read(root.join(profile.scenario.scene_xml.unwrap())).unwrap(),
        );
    }
}

#[cfg(feature = "native")]
#[test]
fn every_default_scene_compiles_from_embedded_bytes_with_mujoco() {
    use microduck_body_runtime::simulation::mujoco::{CompiledScene, runtime_version};

    let source = OfficialAssets::Embedded;
    let body = load_official_body(&source).unwrap();
    assert_eq!(runtime_version(), 3010000);
    let mut scenes = Vec::new();
    for model in [
        ModelKind::Xl330,
        ModelKind::Hd1910,
        ModelKind::Hd1910Compatible,
    ] {
        let profile =
            resolve_simulation(selection(model), &body, &source, defaults(model)).unwrap();
        scenes.push(CompiledScene::compile(profile.simulation.as_ref().unwrap(), &source).unwrap());
    }
    assert_eq!(scenes.len(), 3);
}

#[cfg(feature = "native")]
#[test]
fn pinned_files_and_embedded_bytes_produce_equal_independent_physics_snapshots() {
    use std::time::Duration;

    use loop_body::{BackendPositionTargets, ControlBackend};
    use microduck_body_runtime::simulation::SnapshotHandle;
    use microduck_body_runtime::simulation::mujoco::NativeSimulation;

    let root = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../vendor/microduck_rl/src/mjlab_microduck/robot/microduck");
    let embedded = OfficialAssets::Embedded;
    let file = OfficialAssets::Files(&root);
    let body = load_official_body(&embedded).unwrap();
    for model in [
        ModelKind::Xl330,
        ModelKind::Hd1910,
        ModelKind::Hd1910Compatible,
    ] {
        let profile =
            resolve_simulation(selection(model), &body, &embedded, defaults(model)).unwrap();
        let from_files = SnapshotHandle::default();
        let from_bytes = SnapshotHandle::default();
        let mut file_backend =
            NativeSimulation::open(body.template(), &profile, &file, from_files.clone()).unwrap();
        let mut memory_backend =
            NativeSimulation::open(body.template(), &profile, &embedded, from_bytes.clone())
                .unwrap();
        for backend in [&mut file_backend, &mut memory_backend] {
            backend.set_torque(true).unwrap();
            backend
                .write_positions(
                    &[8, 9],
                    &BackendPositionTargets {
                        positions: vec![0.1, 0.2],
                    },
                )
                .unwrap();
        }
        let step = Duration::from_millis(5);
        file_backend.advance_execution(4, step).unwrap();
        assert_eq!(
            from_bytes
                .lock()
                .unwrap()
                .state
                .as_ref()
                .unwrap()
                .physics_step,
            0
        );
        memory_backend.advance_execution(4, step).unwrap();
        for _ in 0..4 {
            file_backend.advance_execution(4, step).unwrap();
            memory_backend.advance_execution(4, step).unwrap();
        }
        let file_frame = from_files.lock().unwrap();
        let memory_frame = from_bytes.lock().unwrap();
        assert_eq!(
            serde_json::to_value(file_frame.state.as_ref().unwrap()).unwrap(),
            serde_json::to_value(memory_frame.state.as_ref().unwrap()).unwrap(),
        );
    }
}
