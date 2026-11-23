use super::*;

#[test]
fn a_progress_reader_keeps_its_snapshot_across_updates() {
    let run_id = new_training_run_id();
    let directory = crate::platform::helpers::test_directory().join(&run_id);
    let mut progress = initial_progress(
        run_id.clone(),
        &directory,
        TrainingRunMetadata {
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
#[ignore = "requires uv, the pinned MJLab environment, and a recorded-policy teacher artifact"]
fn teacher_evaluation_uses_the_selected_body_and_preserves_the_student() {
    let fixture = PathBuf::from(
        std::env::var_os("MICRODUCK_TRAINING_PREVIEW_FIXTURE").expect("teacher fixture"),
    );
    let state = DesktopState::new(Some(fixture.join("config")));
    let teacher = fixture.join("training/runs/teacher");
    let original_model = std::fs::read(teacher.join("student.onnx")).unwrap();
    let original_manifest = std::fs::read(teacher.join("artifact.json")).unwrap();
    let upstream = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../../vendor/microduck_rl")
        .canonicalize()
        .unwrap();
    for (model, servo, label, invalid_digest) in [
        (
            ModelKindKey::Xl330,
            crate::types::ServoModelKey::Xl330,
            "xl330",
            false,
        ),
        (
            ModelKindKey::Hd1910,
            crate::types::ServoModelKey::Hd1910,
            "hd1910",
            false,
        ),
        (
            ModelKindKey::Hd1910Compatible,
            crate::types::ServoModelKey::Hd1910,
            "hd1910-compatible",
            false,
        ),
        (
            ModelKindKey::Xl330,
            crate::types::ServoModelKey::Xl330,
            "xl330",
            true,
        ),
    ] {
        let machine_id = {
            let mut machines = state.virtual_machines.lock().unwrap();
            machines[0].model_kind = model;
            machines[0].servo_model = servo;
            machines[0].simulation = crate::platform::helpers::default_simulation(servo);
            machines[0].id.clone()
        };
        let run_id = new_training_run_id();
        let result = crate::platform::helpers::test_directory().join(&run_id);
        std::fs::create_dir_all(&result).unwrap();
        let input = TeacherEvaluationInput {
            working_dir: upstream.to_string_lossy().into_owned(),
            virtual_machine_id: machine_id,
            teacher_policy_id: "training:teacher".into(),
            task_id: "Mjlab-Velocity-Flat-MicroDuck".into(),
            steps: 32,
            seed: 11,
        };
        let (mut command, preview, metadata) =
            prepare_teacher_evaluation(&state, &input, &run_id, &result).unwrap();
        assert_eq!(metadata.policy_id.as_deref(), Some("training:teacher"));
        let spec = result.join("teacher-evaluation-spec.json");
        let mut spec_json: serde_json::Value =
            serde_json::from_slice(&std::fs::read(&spec).unwrap()).unwrap();
        if invalid_digest {
            spec_json["onnx_sha256"] = "0".repeat(64).into();
            std::fs::write(&spec, spec_json.to_string()).unwrap();
        }
        let process = command
            .env("MICRODUCK_TRAINING_RUN_ID", &run_id)
            .output()
            .unwrap();
        std::fs::write(result.join("stdout.log"), &process.stdout).unwrap();
        std::fs::write(result.join("stderr.log"), &process.stderr).unwrap();
        assert_ne!(
            process.status.success(),
            invalid_digest,
            "{}",
            result.display()
        );
        let report: serde_json::Value =
            serde_json::from_slice(&std::fs::read(result.join("evaluation.json")).unwrap())
                .unwrap();
        assert_eq!(report["model_kind"], label);
        assert_eq!(report["seed"], input.seed);
        assert_eq!(report["policy_home_pose_rad"], spec_json["home_pose_rad"]);
        assert_eq!(report["onnx_sha256"], spec_json["onnx_sha256"]);
        if invalid_digest {
            assert_eq!(report["status"], "failed");
            assert!(report["error"].as_str().unwrap().contains("digest changed"));
        } else {
            assert_eq!(report["status"], "finished");
            assert_eq!(report["completed_steps"], input.steps);
            assert!(report["reward_sum"].as_f64().unwrap().is_finite());
            let stdout = String::from_utf8(process.stdout).unwrap();
            let frame = stdout
                .lines()
                .find_map(|line| line.strip_prefix("MICRODUCK_TRAINING_FRAME "))
                .unwrap();
            let preview = preview.unwrap();
            assert_eq!(
                report["execution_fingerprint"],
                preview.execution_fingerprint
            );
            assert!(preview.decode(frame, &run_id, 0).is_some());
        }
        assert!(!result.join("artifact.json").exists());
        println!(
            "{label} evaluation {} {}",
            report["status"],
            result.display()
        );
    }
    assert_eq!(
        std::fs::read(teacher.join("student.onnx")).unwrap(),
        original_model
    );
    assert_eq!(
        std::fs::read(teacher.join("artifact.json")).unwrap(),
        original_manifest
    );
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

#[test]
#[ignore = "requires uv and the pinned MJLab/Python environment"]
fn desktop_training_emits_frames_for_the_task_roller_meshes() {
    let state = DesktopState::new(None);
    let virtual_machine_id = {
        let mut machines = state.virtual_machines.lock().unwrap();
        machines[0].model_kind = ModelKindKey::Xl330;
        machines[0].servo_model = crate::types::ServoModelKey::Xl330;
        machines[0].id.clone()
    };
    let source = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../../vendor/microduck_rl")
        .canonicalize()
        .unwrap();
    for task in [
        "Mjlab-Velocity-Flat-MicroDuck-Rollers",
        "Mjlab-Velocity-Flat-Backlash-MicroDuck-Rollers",
    ] {
        let run_id = new_training_run_id();
        let result = crate::platform::helpers::test_directory().join(&run_id);
        std::fs::create_dir_all(&result).unwrap();
        let input = TrainingStartInput {
            program: "uv".into(),
            args: [
                "run",
                "train",
                task,
                "--num-envs",
                "1",
                "--agent.max-iterations",
                "1",
            ]
            .map(str::to_owned)
            .to_vec(),
            working_dir: source.to_string_lossy().into_owned(),
            virtual_machine_id: virtual_machine_id.clone(),
            policy_id: None,
            experiment_id: None,
            recipe: "RL".into(),
            teacher_motion_provenance: "local".into(),
            dataset_path: None,
        };
        let (mut launch, preview) = prepare_rl_command(&state, &input, &source, &result).unwrap();
        let preview = preview.expect("task-specific preview scene");
        let scene = preview.scene.as_ref().unwrap();
        assert!(
            scene
                .model
                .bodies
                .iter()
                .any(|body| body.name == "ankle_l_v1")
        );
        assert!(
            scene
                .model
                .bodies
                .iter()
                .flat_map(|body| &body.geoms)
                .any(|geom| geom.mesh == "tire.stl")
        );
        let process = launch
            .env("MICRODUCK_TRAINING_RUN_ID", &run_id)
            .output()
            .unwrap();
        std::fs::write(result.join("stdout.log"), &process.stdout).unwrap();
        std::fs::write(result.join("stderr.log"), &process.stderr).unwrap();
        assert!(
            process.status.success(),
            "{task} logs at {}",
            result.display()
        );
        let stdout = String::from_utf8(process.stdout).unwrap();
        let mut sequence = 0;
        for line in stdout
            .lines()
            .filter_map(|line| line.strip_prefix("MICRODUCK_TRAINING_FRAME "))
        {
            let frame = preview
                .decode(line, &run_id, sequence)
                .expect("live frame matches the resolved scene");
            sequence = frame.sequence;
        }
        assert!(sequence > 0, "{task} produced no frames");
        println!(
            "{task}: frames through sequence {sequence}, evidence at {}",
            result.display()
        );
    }
}

#[test]
#[ignore = "requires uv and the pinned MJLab/Python environment"]
fn desktop_prepares_real_mjlab_task_before_launching_the_packaged_adapter() {
    let state = DesktopState::new(None);
    let virtual_machine_id = state.virtual_machines.lock().unwrap()[0].id.clone();
    let source = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../../vendor/microduck_rl")
        .canonicalize()
        .unwrap();
    let result = crate::platform::helpers::test_directory().join(new_training_run_id());
    std::fs::create_dir_all(&result).unwrap();
    let input = TrainingStartInput {
        program: "uv".into(),
        args: [
            "run",
            "train",
            "Mjlab-Velocity-Flat-MicroDuck",
            "--num-envs",
            "1",
        ]
        .into_iter()
        .map(str::to_owned)
        .collect(),
        working_dir: source.to_string_lossy().into_owned(),
        virtual_machine_id,
        policy_id: None,
        experiment_id: None,
        recipe: "RL".into(),
        teacher_motion_provenance: "local".into(),
        dataset_path: None,
    };
    let (launch, _) = prepare_rl_command(&state, &input, &source, &result).unwrap();
    assert!(launch.get_args().any(|arg| arg == "--profile"));
    assert_eq!(launch.get_current_dir(), Some(result.as_path()));
    assert!(
        result
            .join("body-adapters/hd1910_compatible/mjlab_body.py")
            .is_file()
    );
    let export: serde_json::Value =
        serde_json::from_slice(&std::fs::read(result.join("simulation-profile.json")).unwrap())
            .unwrap();
    assert_eq!(export["profile"]["model_kind"], "hd1910-compatible");
    assert_eq!(
        export["profile"]["simulation"]["scenario"]["robot_xml"],
        "robot_walk.xml"
    );
    assert!(export["profile"]["simulation"]["scenario"]["scene_xml"].is_null());
    assert_eq!(export["task"]["policy_layout"]["action_len"], 14);
    assert_eq!(export["task"]["assembly"]["num_envs"], 1);
    assert!(std::fs::read_dir(&result).unwrap().all(|entry| {
        entry
            .unwrap()
            .path()
            .extension()
            .is_none_or(|extension| extension != "xml")
    }));
}
