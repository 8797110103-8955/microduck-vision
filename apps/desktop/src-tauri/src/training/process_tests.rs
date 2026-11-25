use super::*;

#[test]
fn persisted_training_progress_restores_results_without_claiming_an_orphan_is_running() {
    let root = crate::platform::helpers::test_directory().join(new_training_run_id());
    let config = root.join("config");
    let state = DesktopState::new(Some(config.clone()));
    let id = new_training_run_id();
    let mut input = TrainingStartInput {
        initialization: None,
        replace_existing_experiment: false,
        deepmimic: None,
        program: "uv".into(),
        args: vec![
            "run".into(),
            "train".into(),
            "Mjlab-Velocity-Flat-MicroDuck".into(),
        ],
        virtual_machine_id: state.virtual_machines.lock().unwrap()[0].id.clone(),
        policy_id: None,
        experiment_id: None,
        recipe: "RL".into(),
        teacher_motion_provenance: "local".into(),
        dataset_path: None,
    };
    let directory = crate::training::rounds::allocate(Some(&config), &mut input, &id).unwrap();
    let mut saved = initial_progress(
        id.clone(),
        &directory,
        training_metadata(&state, &input, &id).unwrap(),
        "saved result".into(),
    );
    saved.phase = TrainingPhase::Finished;
    persist_training_progress(&saved);
    assert_eq!(
        status(&state).unwrap().unwrap().phase,
        TrainingPhase::Finished
    );
    assert_eq!(
        result_directory(&state, &id).unwrap(),
        directory.canonicalize().unwrap()
    );
    for invalid in ["../outside", "run-../outside", "C:/outside", "run-missing"] {
        assert!(result_directory(&state, invalid).is_err());
    }
    saved.phase = TrainingPhase::Running;
    persist_training_progress(&saved);
    let reopened = DesktopState::new(Some(config.clone()));
    let recovered = status(&reopened).unwrap().unwrap();
    assert_eq!(recovered.phase, TrainingPhase::Failed);
    assert!(recovered.message.contains("连接已中断"));
    assert_eq!(recovered.result_dir, saved.result_dir);
    saved.result_dir = root.to_string_lossy().into_owned();
    std::fs::write(
        directory.join("progress.json"),
        serde_json::to_vec(&saved).unwrap(),
    )
    .unwrap();
    assert!(status(&DesktopState::new(Some(config))).unwrap().is_none());
}

#[test]
fn a_progress_reader_keeps_its_snapshot_across_updates() {
    let run_id = new_training_run_id();
    let directory = crate::platform::helpers::test_directory().join(&run_id);
    let mut progress = initial_progress(
        run_id.clone(),
        &directory,
        TrainingRunMetadata {
            deepmimic: None,
            experiment_id: run_id,
            body_template_id: "official".into(),
            body_version: 1,
            body_source_hash: "fixture".into(),
            policy_id: None,
            policy_layout: TrainingPolicyLayout {
                obs_len: 61,
                action_len: 14,
                control_hz: 50,
                command_encoding: None,
            },
            servo_profile: crate::types::ServoModelKey::Hd1910,
            recipe: "RL".into(),
            teacher_motion_provenance: "local".into(),
        },
        "starting the pinned task".into(),
    );
    let path = directory.join("progress.json");
    persist_training_progress(&progress);
    let reader = std::fs::File::open(&path).unwrap();
    let previous = serde_json::to_value(&progress).unwrap();
    progress.phase = TrainingPhase::Running;
    progress.step = Some(18);
    progress.message = "running".into();
    persist_training_progress(&progress);

    let opened: serde_json::Value = serde_json::from_reader(reader).unwrap();
    let latest: serde_json::Value = serde_json::from_slice(&std::fs::read(path).unwrap()).unwrap();
    assert_eq!(opened, previous);
    assert_eq!(latest, serde_json::to_value(&progress).unwrap());
}

#[test]
fn accepts_only_ordered_frames_of_the_resolved_training_body() {
    let source = TrainingFrameSource {
        model_kind: ModelKindKey::Hd1910Compatible,
        execution_fingerprint: "execution-id".into(),
        active_slots: vec![true, false],
        body_count: 1,
        scene: None,
    };
    let sample = serde_json::json!({
        "runId": "run-b", "sequence": 8, "environmentIndex": 0,
        "modelKind": "hd1910-compatible", "executionFingerprint": "execution-id",
        "positions": [0.3, null],
        "trunkPosition": [0.1, 0.2, 0.125], "trunkQuaternion": [1.0, 0.0, 0.0, 0.0],
        "frame": {"bodies": [[0.1, 0.2, 0.125, 1.0, 0.0, 0.0, 0.0]], "markers": []}
    });
    let decoded = source.decode(&sample.to_string(), "run-b", 7).unwrap();
    assert_eq!(decoded.positions, [Some(0.3), None]);
    assert!(source.decode(&sample.to_string(), "run-a", 0).is_none());
    assert!(source.decode(&sample.to_string(), "run-b", 8).is_none());
    for (field, value) in [
        ("environmentIndex", serde_json::json!(1)),
        ("positions", serde_json::json!([0.3, 0.0])),
        ("modelKind", serde_json::json!("hd1910")),
        ("executionFingerprint", serde_json::json!("another-body")),
        ("trunkPosition", serde_json::json!([0.0, 0.0, 0.0])),
    ] {
        let mut invalid = sample.clone();
        invalid[field] = value;
        assert!(
            source.decode(&invalid.to_string(), "run-b", 7).is_none(),
            "{field}"
        );
    }
}
