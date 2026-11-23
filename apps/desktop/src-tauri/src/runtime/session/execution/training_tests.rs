use crate::runtime::policy::catalog as policy;
use crate::runtime::session::{preview, state::DesktopState};
use crate::types::{ModelKindKey, PreviewInput, ServoModelKey};
use duck_control::model::{MOUTH_INDEX, NUM_JOINTS};
use serde_json::json;
use std::path::PathBuf;

#[test]
#[ignore = "requires the three local RL ONNX artifacts, a teacher model, and ONNX Runtime"]
fn local_training_artifacts_drive_selected_native_bodies() {
    let root = PathBuf::from(
        std::env::var_os("MICRODUCK_TRAINING_PREVIEW_FIXTURE")
            .expect("local training artifact fixture"),
    );
    let config = root.join("config");
    let state = DesktopState::new(Some(config.clone()));
    let base = state.virtual_machines.lock().unwrap()[0].clone();
    let mut outputs = Vec::new();
    for (run, model, expected_model) in [
        ("rl-xl330", ModelKindKey::Xl330, "xl330"),
        ("rl-hd1910", ModelKindKey::Hd1910, "hd1910"),
        (
            "rl-hd1910-compatible",
            ModelKindKey::Hd1910Compatible,
            "hd1910-compatible",
        ),
        (
            "rl-xl330",
            ModelKindKey::Hd1910Compatible,
            "hd1910-compatible",
        ),
        ("teacher", ModelKindKey::Hd1910, "hd1910"),
    ] {
        let mut machine = base.clone();
        machine.id = format!("training-preview-{expected_model}");
        machine.model_kind = model;
        machine.servo_model = if model == ModelKindKey::Xl330 {
            ServoModelKey::Xl330
        } else {
            ServoModelKey::Hd1910
        };
        machine.simulation = crate::platform::helpers::default_simulation(machine.servo_model);
        let mut execution = preview::open_body(&state, &machine).unwrap();
        let id = format!("training:{run}");
        policy::select(Some(&config), &id).unwrap();
        assert!(
            policy::list(Some(&config))
                .unwrap()
                .iter()
                .any(|entry| entry.id == id && entry.selected && entry.cached)
        );
        let selection = policy::selected_training_policy(Some(&config), &id).unwrap();
        let home = selection.training.as_ref().unwrap().spec.home;
        let scale = selection.config.action_scale;
        execution.select_training_policy(selection.clone()).unwrap();
        execution.set_input(PreviewInput {
            twist: [0.15, 0.0, 0.0],
            head: [0.0; 4],
            body: [0.0; 3],
            target_distance_m: None,
        });
        let status = execution.tick().unwrap();
        assert!(status.simulation.unwrap().physics_step > 0);
        let trace = execution.runner().policy_trace().unwrap();
        for (channel, slot) in (0..NUM_JOINTS)
            .filter(|slot| *slot != MOUTH_INDEX)
            .enumerate()
        {
            assert_eq!(
                trace.observation[6 + channel],
                (trace.backend_state.positions[slot] - home[slot]) as f32
            );
            assert_eq!(
                trace.raw_targets.as_ref().unwrap()[slot],
                home[slot] + scale * f64::from(trace.action[channel])
            );
        }
        let metadata = execution.recording_metadata();
        assert_eq!(
            metadata["simulation_execution"]["model_kind"],
            expected_model
        );
        assert_eq!(metadata["training_policy_source"]["id"], id);
        assert_eq!(metadata["training_policy_home_rad"], json!(home));
        outputs.push(json!({ "path": selection.path, "run": run, "preview_model": expected_model, "observation": trace.observation, "actions": trace.action }));
    }

    for (run, expected) in [
        ("invalid-obs-order", "观测项"),
        ("invalid-path", "相对 ONNX 路径"),
        ("invalid-digest", "文件摘要"),
    ] {
        let error = policy::select(Some(&config), &format!("training:{run}")).unwrap_err();
        assert!(error.contains(expected), "{error}");
    }
    assert!(policy::selected_training_policy(Some(&config), "training:teacher").is_ok());
    let mut execution = preview::open_body(&state, &base).unwrap();
    for (run, expected) in [
        ("invalid-digest", "文件摘要"),
        ("invalid-graph", "张量合同"),
    ] {
        let id = format!("training:{run}");
        let selection = crate::training::policy_artifacts::resolve(
            &root.join("training/runs").join(run),
            id,
            crate::runtime::machines::body_catalog::official().unwrap(),
        )
        .unwrap();
        let error = execution.select_training_policy(selection).unwrap_err();
        assert!(error.contains(expected), "{error}");
    }
    std::fs::write(
        root.join("native-traces.json"),
        serde_json::to_vec_pretty(&outputs).unwrap(),
    )
    .unwrap();
}
