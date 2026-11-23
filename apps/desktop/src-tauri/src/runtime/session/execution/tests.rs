use super::{PolicyCache, PolicyExecution, PreviewPolicySelection, has_reached_target};
use crate::runtime::policy::inference::{PolicyConfig, PolicyDescriptor};
use crate::test_support::simulation_selection;
use crate::types::ServoModelKey;
use microduck_body_runtime::ModelKind;
use servo::ServoModel;
use std::time::{SystemTime, UNIX_EPOCH};

fn test_cache(config_dir: Option<&std::path::Path>) -> PolicyCache<'_> {
    PolicyCache {
        config_dir,
        resolve: crate::test_support::cached_policy_selections,
    }
}

#[test]
fn idle_sampling_and_reopening_preserve_the_body_and_scene_placement() {
    let placements = std::sync::Arc::new(
        crate::platform::state::world_placements::WorldPlacements::load(None).unwrap(),
    );
    placements.remember(
        "test-duck",
        super::WorldState {
            position: [1.2, -0.4],
            yaw: 0.8,
            distance_m: 2.5,
        },
    );
    let mut registry = loop_body::BackendRegistry::default();
    let open = |registry: &mut loop_body::BackendRegistry| {
        let mut execution = PolicyExecution::open(
            simulation_selection(ModelKind::Xl330),
            crate::platform::helpers::default_simulation(ServoModelKey::Xl330),
            test_cache(None),
            "test-duck",
            "test",
            registry,
            &crate::runtime::adapters::body_assets::test_template(),
        )
        .unwrap();
        execution.restore_placement(placements.clone());
        execution
    };
    let mut execution = open(&mut registry);
    let initial = execution.status();
    for _ in 0..50 {
        execution.tick().unwrap();
    }
    let settled = execution.status();
    assert_eq!(
        initial.simulation.unwrap().trunk_position,
        settled.simulation.unwrap().trunk_position
    );
    assert_eq!(settled.world.position, initial.world.position);
    assert_eq!(settled.world.yaw, initial.world.yaw);
    assert_eq!(settled.world.distance_m, initial.world.distance_m);
    execution.release_body(&mut registry, "test-duck").unwrap();
    let mut reopened = open(&mut registry);
    assert_eq!(reopened.status().world.position, initial.world.position);
    reopened.tick().unwrap();
    assert_eq!(reopened.status().world.position, initial.world.position);
    assert_eq!(reopened.status().world.distance_m, initial.world.distance_m);
    reopened.reset_world().unwrap();
    assert_eq!(reopened.status().world.position, [0.0; 2]);
    reopened.release_body(&mut registry, "test-duck").unwrap();
    assert_eq!(open(&mut registry).status().world.position, [0.0; 2]);
}

#[test]
fn a_new_distance_request_uses_its_own_origin_without_resetting_world_coordinates() {
    let mut registry = loop_body::BackendRegistry::default();
    let mut preview = PolicyExecution::open(
        simulation_selection(ModelKind::Xl330),
        crate::platform::helpers::default_simulation(ServoModelKey::Xl330),
        test_cache(None),
        "distance-test",
        "test",
        &mut registry,
        &crate::runtime::adapters::body_assets::test_template(),
    )
    .unwrap();
    preview.world.distance_m = 2.0;
    let input = super::PreviewInput {
        twist: [0.15, 0.0, 0.0],
        head: [0.0; 4],
        body: [0.0; 3],
        target_distance_m: Some(1.0),
    };
    preview.set_input(input);
    assert_eq!(preview.effective_twist(), [0.15, 0.0, 0.0]);
    assert_eq!(preview.world.distance_m, 2.0);
    preview.world.distance_m = 3.1;
    assert_eq!(preview.effective_twist(), [0.0; 3]);
    preview.set_input(input);
    assert_eq!(
        preview.effective_twist(),
        [0.0; 3],
        "repeated input cannot restart a completed distance request"
    );
    preview.set_input(super::PreviewInput {
        twist: [0.0; 3],
        ..input
    });
    preview.set_input(input);
    assert_eq!(preview.effective_twist(), [0.15, 0.0, 0.0]);
    assert_eq!(preview.world.distance_m, 3.1);
}

#[test]
#[ignore = "requires cached official policies and ONNX Runtime"]
fn velstand_walks_from_cold_start_and_handoff_without_changing_motor_parameters() {
    let cache = std::path::PathBuf::from(std::env::var_os("MICRODUCK_POLICY_CACHE").unwrap());
    let root = crate::platform::helpers::test_directory().join(
        crate::platform::helpers::new_machine_id("walking-cold-start"),
    );
    let official = root.join("policies/official");
    std::fs::create_dir_all(&official).unwrap();
    for entry in std::fs::read_dir(cache.join("official")).unwrap() {
        let entry = entry.unwrap();
        if entry
            .path()
            .extension()
            .is_some_and(|extension| extension == "onnx")
            || entry.file_name() == "manifest.json"
        {
            std::fs::copy(entry.path(), official.join(entry.file_name())).unwrap();
        }
    }
    for (kind, key) in [
        (ModelKind::Xl330, ServoModelKey::Xl330),
        (ModelKind::Hd1910Compatible, ServoModelKey::Hd1910),
    ] {
        let model = kind.servo_model();
        for id in ["velstand"] {
            for previous in [None, Some("ball-kick-left")] {
                let mut registry = loop_body::BackendRegistry::default();
                let mut preview = PolicyExecution::open(
                    simulation_selection(kind),
                    if model == ServoModel::Xl330M288 {
                        crate::types::SimulationParameters {
                            damping: 0.049268,
                            friction_loss: 0.004937,
                            armature: 0.001889,
                            kp: 0.536635,
                            force_range: 0.966805,
                            ..crate::platform::helpers::default_simulation(key)
                        }
                    } else {
                        crate::platform::helpers::default_simulation(key)
                    },
                    test_cache(Some(&root.join("config"))),
                    "cold-start-test",
                    "test",
                    &mut registry,
                    &crate::runtime::adapters::body_assets::test_template(),
                )
                .unwrap();
                // Desktop starts its clock before the frontend selects a policy.
                for _ in 0..50 {
                    preview.tick().unwrap();
                }
                assert_eq!(preview.status().simulation.unwrap().physics_step, 0);
                if let Some(previous) = previous {
                    preview.select_policy(previous).unwrap();
                    for _ in 0..10 {
                        preview.tick().unwrap();
                    }
                }
                preview.select_policy(id).unwrap();
                preview.set_input(super::PreviewInput {
                    twist: [0.15, 0.0, 0.0],
                    head: [0.0; 4],
                    body: [0.0; 3],
                    target_distance_m: None,
                });
                let start = preview.status().simulation.unwrap().trunk_position;
                let mut low = start[2];
                let mut peak_speed: f64 = 0.0;
                for _ in 0..250 {
                    let status = preview.tick().unwrap();
                    let simulation = status.simulation.unwrap();
                    low = low.min(simulation.trunk_position[2]);
                    peak_speed = peak_speed.max(simulation.base_velocity[0].abs());
                    assert_eq!(
                        preview.runner().policy_trace().unwrap().command.twist,
                        [0.15, 0.0, 0.0]
                    );
                }
                let end = preview.status().simulation.unwrap().trunk_position;
                println!(
                    "COLD_START {model:?}/{id} previous={previous:?} delta={:?} low={low} peak_speed={peak_speed}",
                    [end[0] - start[0], end[1] - start[1], end[2] - start[2]]
                );
                assert!(
                    end[0] - start[0] > 0.18,
                    "{model:?}: cold start and handoff must both walk"
                );
                assert!(low > 0.10, "{model:?}: walking must keep the body upright");
            }
        }
    }
    crate::platform::logging::flush();
    std::fs::remove_dir_all(root).unwrap();
}

#[test]
#[ignore = "requires cached official policies and ONNX Runtime"]
fn cached_walking_policies_execute_forward_commands() {
    let cache = std::path::PathBuf::from(std::env::var_os("MICRODUCK_POLICY_CACHE").unwrap());
    let root = crate::platform::helpers::test_directory().join(
        crate::platform::helpers::new_machine_id("walking-regression"),
    );
    let official = root.join("policies/official");
    std::fs::create_dir_all(&official).unwrap();
    for entry in std::fs::read_dir(cache.join("official")).unwrap() {
        let entry = entry.unwrap();
        if entry
            .path()
            .extension()
            .is_some_and(|extension| extension == "onnx")
            || entry.file_name() == "manifest.json"
        {
            std::fs::copy(entry.path(), official.join(entry.file_name())).unwrap();
        }
    }
    for (kind, key) in [
        (ModelKind::Xl330, ServoModelKey::Xl330),
        (ModelKind::Hd1910Compatible, ServoModelKey::Hd1910),
    ] {
        let model = kind.servo_model();
        for id in ["alpha-walking", "velstand"] {
            let mut registry = loop_body::BackendRegistry::default();
            let mut preview = PolicyExecution::open(
                simulation_selection(kind),
                crate::platform::helpers::default_simulation(key),
                test_cache(Some(&root.join("config"))),
                "walking-test",
                "test",
                &mut registry,
                &crate::runtime::adapters::body_assets::test_template(),
            )
            .unwrap();
            preview.select_policy("ball-kick-left").unwrap();
            for _ in 0..10 {
                preview.tick().unwrap();
            }
            preview.world.distance_m = 2.0;
            preview.select_policy(id).unwrap();
            for speed in [0.15, 0.3] {
                if speed != 0.15 {
                    preview.reset_world().unwrap();
                }

                preview.set_input(super::PreviewInput {
                    twist: [speed, 0.0, 0.0],
                    head: [0.0; 4],
                    body: [0.0; 3],
                    target_distance_m: Some(1.0),
                });
                let mut low = f64::INFINITY;
                let mut n = 0;
                for _ in 0..250 {
                    let reached = preview.target_distance_reached();
                    let status = preview.tick().unwrap();
                    low = low.min(status.simulation.unwrap().trunk_position[2]);
                    if let Some(trace) = preview.runner().policy_trace() {
                        assert_eq!(
                            trace.command.twist,
                            if reached { [0.0; 3] } else { [speed, 0.0, 0.0] },
                            "only this walk's actual distance can stop its command"
                        );
                        n += 1;
                    }
                }
                let status = preview.status();
                let position = status.simulation.unwrap().trunk_position;
                assert!((status.world.position[0] - position[0]).abs() < 1e-10);
                assert!((status.world.position[1] - position[1]).abs() < 1e-10);
                let trace = preview.runner().policy_trace().unwrap();
                println!(
                    "{model:?}/{id} vx={speed}: position={position:?}, low={low}, traces={n}, twist={:?}, gravity={:?}",
                    trace.command.twist, trace.imu.gravity
                );
                assert_eq!(n, 250);
                if model == ServoModel::Xl330M288 || id == "velstand" || speed >= 0.3 {
                    assert!(
                        position[0] > 0.15,
                        "walking policy must execute forward motion"
                    );
                }
            }
        }
    }
    crate::platform::logging::flush();
    std::fs::remove_dir_all(root).unwrap();
}

#[test]
#[ignore = "requires cached official policies and ONNX Runtime"]
fn warmed_policies_run_until_explicit_handoff_from_the_current_body_pose() {
    let cache = std::path::PathBuf::from(std::env::var_os("MICRODUCK_POLICY_CACHE").unwrap());
    let root = crate::platform::helpers::test_directory()
        .join(crate::platform::helpers::new_machine_id("policy-handoff"));
    let official = root.join("policies/official");
    std::fs::create_dir_all(&official).unwrap();
    for entry in std::fs::read_dir(cache.join("official")).unwrap() {
        let entry = entry.unwrap();
        if entry
            .path()
            .extension()
            .is_some_and(|extension| extension == "onnx")
            || entry.file_name() == "manifest.json"
        {
            std::fs::copy(entry.path(), official.join(entry.file_name())).unwrap();
        }
    }
    let config = root.join("config");
    let count = crate::test_support::policy_list(Some(&config))
        .unwrap()
        .iter()
        .filter(|entry| entry.cached)
        .count();
    let mut registry = loop_body::BackendRegistry::default();
    let mut preview = PolicyExecution::open(
        simulation_selection(ModelKind::Xl330),
        crate::platform::helpers::default_simulation(ServoModelKey::Xl330),
        test_cache(Some(&config)),
        "handoff-test",
        "test",
        &mut registry,
        &crate::runtime::adapters::body_assets::test_template(),
    )
    .unwrap();
    assert_eq!(preview.policies.len(), count);
    assert!(count >= 10);
    assert_eq!(preview.status().tick, 0, "warm-up cannot advance the body");
    // An explicit handoff must work entirely from the warm session bank.
    for entry in std::fs::read_dir(&official).unwrap() {
        let entry = entry.unwrap();
        if entry
            .path()
            .extension()
            .is_some_and(|extension| extension == "onnx")
        {
            std::fs::remove_file(entry.path()).unwrap();
        }
    }
    preview.select_policy("ball-kick-left").unwrap();
    for _ in 0..80 {
        preview.tick().unwrap();
        assert!(preview.loop_state.policy_trace().is_some());
        assert_eq!(
            preview.active_policy.as_ref().unwrap().descriptor.id,
            "ball-kick-left"
        );
    }
    // The manifest's 0.5s window has passed; there is no automatic stop or stand policy.
    assert_eq!(preview.policy_ticks, 80);
    let before = preview.status();
    preview.select_policy("alpha-stand").unwrap();
    let after = preview.status();
    assert_eq!(before.world.position, after.world.position);
    assert_eq!(before.world.yaw, after.world.yaw);
    assert_eq!(before.world.distance_m, after.world.distance_m);
    assert_eq!(before.positions, after.positions);
    assert_eq!(before.targets, after.targets);
    assert_eq!(before.tick, after.tick);
    assert_eq!(before.session, after.session);
    assert_eq!(
        before.simulation.as_ref().unwrap().physics_step,
        after.simulation.as_ref().unwrap().physics_step
    );
    assert!(after.torque);
    preview.tick().unwrap();
    let trace = preview.loop_state.policy_trace().unwrap();
    let home = crate::runtime::adapters::body_assets::test_template()
        .control
        .home_pose;
    let offsets = std::array::from_fn(|index| trace.backend_state.positions[index] - home[index]);

    let expected = duck_control::obs::Observation::gather_action(&offsets);
    for (actual, expected) in trace.observation[34..48].iter().zip(expected) {
        assert!(
            (actual - expected).abs() < 1e-5,
            "handoff history must describe the current pose: actual={actual}, expected={expected}"
        );
    }
    assert_eq!(preview.status().tick, before.tick + 1);
    preview.select_policy("ball-kick-right").unwrap();
    preview.tick().unwrap();
    assert_eq!(
        preview.active_policy.as_ref().unwrap().descriptor.id,
        "ball-kick-right"
    );
    preview.action(loop_body::LoopAction::Drive(false)).unwrap();
    let paused = preview.status();
    for _ in 0..3 {
        preview.tick().unwrap();
        assert!(
            preview.runner().policy_trace().is_none(),
            "sampling must not repeat a previous ONNX trace"
        );
    }
    assert_eq!(preview.status().tick, paused.tick + 3);
    assert_eq!(
        preview.status().simulation.unwrap().physics_step,
        paused.simulation.unwrap().physics_step
    );
    preview.action(loop_body::LoopAction::Drive(true)).unwrap();
    preview.tick().unwrap();
    assert!(preview.runner().policy_trace().is_some());
    println!(
        "Prewarmed {count} policies; ran kick for 80 ticks; explicit handoffs preserved pose, time and inference."
    );
    drop(preview);
    crate::platform::logging::flush();
    std::fs::remove_dir_all(root).unwrap();
}

#[test]
fn preview_loop_uses_shared_policy_loop_without_hardware() {
    let mut registry = loop_body::BackendRegistry::default();
    let mut loop_state = PolicyExecution::open(
        simulation_selection(ModelKind::Xl330),
        crate::platform::helpers::default_simulation(ServoModelKey::Xl330),
        test_cache(None),
        "test-machine",
        "测试机器",
        &mut registry,
        &crate::runtime::adapters::body_assets::test_template(),
    )
    .unwrap();
    assert_eq!(loop_state.status().phase, crate::types::LoopPhase::Ready);
    loop_state.tick().unwrap();
    assert_eq!(
        loop_state.status().simulation.unwrap().physics_step,
        0,
        "sampling without a policy must preserve the simulation's initial state"
    );
    let status = loop_state.status();
    assert_eq!(status.phase, crate::types::LoopPhase::Ready);
    assert_eq!(status.positions.len(), 15);
    assert_eq!(status.targets.len(), 15);
    assert_eq!(
        status.positions,
        crate::runtime::adapters::body_assets::test_template()
            .control
            .home_pose
    );
    assert_eq!(status.sensor_failures, 0);
    for _ in 0..20 {
        loop_state.tick().unwrap();
    }
    assert_eq!(loop_state.status().simulation.as_ref().unwrap().time, 0.0);
    assert_eq!(loop_state.status().tick, 21);
    loop_state.reset_world().unwrap();
    let reset = loop_state.status();
    assert_eq!(reset.simulation.as_ref().unwrap().physics_step, 0);
    assert_eq!(reset.simulation.as_ref().unwrap().time, 0.0);
    assert_eq!(
        reset.positions,
        crate::runtime::adapters::body_assets::test_template()
            .control
            .home_pose
    );
    loop_state.tick().unwrap();
    assert_eq!(loop_state.status().simulation.unwrap().physics_step, 0);
}

#[test]
fn preview_loop_logs_virtual_servo_io() {
    let suffix = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap()
        .as_nanos();
    let root =
        crate::platform::helpers::test_directory().join(format!("microduck-preview-log-{suffix}"));
    let config_dir = root.join("config");
    let mut registry = loop_body::BackendRegistry::default();
    let mut loop_state = PolicyExecution::open(
        simulation_selection(ModelKind::Xl330),
        crate::platform::helpers::default_simulation(ServoModelKey::Xl330),
        test_cache(Some(&config_dir)),
        "virtual-1",
        "模拟机",
        &mut registry,
        &crate::runtime::adapters::body_assets::test_template(),
    )
    .unwrap();
    loop_state.tick().unwrap();
    let log = std::fs::read_to_string(crate::platform::logging::current_log_file(Some(
        &config_dir,
    )))
    .unwrap();
    assert!(log.contains("loop_open machine_id=virtual-1"));
    assert!(log.contains("resolved_contract body_id=official"));
    assert!(log.contains("backend_kind=simulation"));
    assert!(log.contains("contract_hash="));
    assert!(log.contains("physics_dt=0.005"));
    assert!(log.contains("physics_substeps=4"));
    assert!(log.contains("delay_timebase=physics_step"));
    assert!(log.contains("servo_read id=1"));
    assert!(!log.contains("servo_write_request"));
    let _ = std::fs::remove_dir_all(root);
}

#[test]
#[ignore = "requires the official ONNX artifact and a dynamic ONNX Runtime"]
fn hd1910_velstand_walks_after_standing() {
    let path = std::env::var_os("MICRODUCK_OFFICIAL_ONNX").unwrap();
    for idle_ticks in [2, 3, 10, 50, 100, 200] {
        let mut registry = loop_body::BackendRegistry::default();
        let mut preview = PolicyExecution::open(
            simulation_selection(ModelKind::Hd1910Compatible),
            crate::platform::helpers::default_simulation(ServoModelKey::Hd1910),
            test_cache(None),
            "hd1910-velstand",
            "HD1910 velstand",
            &mut registry,
            &crate::runtime::adapters::body_assets::test_template(),
        )
        .unwrap();
        preview
            .activate_policy(PreviewPolicySelection {
                path: path.clone().into(),
                command: None,
                descriptor: PolicyDescriptor {
                    id: "velstand".into(),
                    body_id: "official".into(),
                    body_version: 1,
                    obs_len: 61,
                    action_len: 14,
                    control_hz: 50,
                    command_encoding: None,
                    recurrent: None,
                    action_scale: None,
                    head_lowpass: None,
                    legs_lowpass: None,
                    joint_velocity_delay_ticks: 1,
                },
                config: PolicyConfig::default(),
                episodic: false,
                duration: None,
                training: None,
            })
            .unwrap();

        for _ in 0..idle_ticks {
            preview.tick().unwrap();
        }
        let start = preview.status().simulation.unwrap().trunk_position;
        assert!(
            start[0].hypot(start[1]) < 0.04,
            "drifted at zero command: {start:?}"
        );
        preview.set_input(super::PreviewInput {
            twist: [0.15, 0.0, 0.0],
            head: [0.0; 4],
            body: [0.0; 3],
            target_distance_m: Some(1.0),
        });
        let mut lowest = f64::INFINITY;
        for _ in 0..250 {
            let state = preview.tick().unwrap().simulation.unwrap();
            lowest = lowest.min(state.trunk_position[2]);
        }
        let distance = preview.status().simulation.unwrap().trunk_position[0] - start[0];
        println!(
            "HD1910: idle_ticks={idle_ticks} forward_m={distance:.3} lowest_trunk_m={lowest:.3}"
        );
        assert!(
            distance > 0.18,
            "failed to walk after {idle_ticks} idle ticks: {distance} m"
        );
        assert!(lowest > 0.10, "trunk fell below 0.10 m");
    }
}

/// Export a product-side trace against the same pinned artifact used by the CPU MuJoCo
/// reference runner. This is intentionally ignored because the desktop test environment
/// does not ship ONNX Runtime or the artifact; run it manually with
/// `MICRODUCK_OFFICIAL_ONNX` and `ORT_DYLIB_PATH` set to produce a comparable trace.
#[test]
#[ignore = "requires the official ONNX artifact and a dynamic ONNX Runtime"]
fn official_velstand_native_parity_for_300_ticks() {
    let policy_path = std::env::var("MICRODUCK_OFFICIAL_ONNX")
        .expect("MICRODUCK_OFFICIAL_ONNX must point to the pinned velstand.onnx");
    use sha2::{Digest, Sha256};
    let policy_sha256 = format!("{:x}", Sha256::digest(std::fs::read(&policy_path).unwrap()));
    let output_path =
        std::env::var_os("MICRODUCK_OFFICIAL_PRODUCT_TRACE").map(std::path::PathBuf::from);
    let mut registry = loop_body::BackendRegistry::default();
    let mut preview = PolicyExecution::open(
        simulation_selection(ModelKind::Xl330),
        crate::platform::helpers::default_simulation(ServoModelKey::Xl330),
        test_cache(None),
        "official-velstand-trace",
        "official velstand trace",
        &mut registry,
        &crate::runtime::adapters::body_assets::test_template(),
    )
    .unwrap();
    preview.set_input(super::PreviewInput {
        twist: [0.2, 0.0, 0.0],
        head: [0.0; 4],
        body: [0.0; 3],
        target_distance_m: None,
    });
    preview
        .activate_policy(PreviewPolicySelection {
            path: policy_path.into(),
            command: None,
            descriptor: PolicyDescriptor {
                id: "velstand".into(),
                body_id: "official".into(),
                body_version: 1,
                obs_len: 61,
                action_len: 14,
                control_hz: 50,
                command_encoding: None,
                recurrent: Some(false),
                action_scale: Some(1.0),
                head_lowpass: None,
                legs_lowpass: None,
                joint_velocity_delay_ticks: 0,
            },
            config: PolicyConfig::default(),
            episodic: false,
            duration: None,
            training: None,
        })
        .unwrap();

    let mut ticks = Vec::new();
    let mut recorded_frames = Vec::new();
    for tick in 0..300 {
        preview.set_input(super::PreviewInput {
            twist: [if (75..225).contains(&tick) { 0.2 } else { 0.0 }, 0.0, 0.0],
            head: [0.0; 4],
            body: [0.0; 3],
            target_distance_m: None,
        });
        preview.tick().unwrap();
        let trace = preview
            .loop_state
            .policy_trace()
            .cloned()
            .expect("active policy must emit a trace");
        let sample = preview
            .simulation_body
            .as_ref()
            .unwrap()
            .snapshot
            .lock()
            .unwrap()
            .state
            .clone()
            .unwrap();
        recorded_frames.push(crate::types::RecordingFrame {
            recording_id: "official-velstand-reference".into(),
            duck_id: "official-velstand-trace".into(),
            segment_id: 1,
            episode_id: 1,
            elapsed_s: (tick + 1) as f64 * super::PERIOD.as_secs_f64(),
            control_started_s: tick as f64 * super::PERIOD.as_secs_f64(),
            status: preview.status(),
            policy: Some(crate::types::RecordingPolicy::from(&trace)),
        });
        ticks.push(serde_json::json!({
            "tick": tick,
            "observation": trace.observation,
            "action": trace.action,
            "raw_targets": trace.raw_targets,
            "canonical_target": trace.body_targets,
            "backend_positions": trace.backend_state.positions,
            "backend_velocities": trace.backend_state.velocities,
            "imu_gyro": trace.imu.gyro,
            "imu_gravity": trace.imu.gravity,
            "execution_positions": trace.execution_positions,
            "simulation": sample,
        }));
    }
    let reference_path = std::env::var("MICRODUCK_OFFICIAL_REFERENCE_TRACE")
        .expect("set MICRODUCK_OFFICIAL_REFERENCE_TRACE to the pinned CPU trace");
    let reference: serde_json::Value =
        serde_json::from_slice(&std::fs::read(reference_path).unwrap()).unwrap();
    assert_eq!(
        reference["policy_sha256"].as_str().unwrap(),
        policy_sha256,
        "ONNX must match the reference artifact"
    );
    use duck_body::AssetResolver;
    let official = microduck_body_runtime::models::assets::OfficialAssets::Embedded;
    for (path, hash) in reference["asset_sha256"].as_object().unwrap() {
        assert_eq!(
            format!(
                "{:x}",
                Sha256::digest(official.read(std::path::Path::new(path)).unwrap())
            ),
            hash.as_str().unwrap(),
            "reference asset changed: {path}"
        );
    }
    let expected = reference["case"]["frames"].as_array().unwrap();
    assert_eq!(ticks.len(), expected.len());
    let mut maxima = std::collections::BTreeMap::<String, f64>::new();
    let mut first_failure = None;
    for (tick, (actual, expected)) in ticks.iter().zip(expected).enumerate() {
        assert_eq!(
            actual["simulation"]["contacts"], expected["contacts"],
            "contact pairs at tick {tick}"
        );
        for (field, tolerance) in reference["tolerances"].as_object().unwrap() {
            let value = match field.as_str() {
                "target" => &actual["canonical_target"],
                "positions" => &actual["backend_positions"],
                "velocities" => &actual["backend_velocities"],
                "gyro" => &actual["imu_gyro"],
                "gravity" => &actual["imu_gravity"],
                "observation" | "action" => &actual[field],
                "motor_positions" => &actual["simulation"]["motorPositions"],
                "passive_positions" => &actual["simulation"]["passivePositions"],
                "trunk_position" => &actual["simulation"]["trunkPosition"],
                "trunk_quaternion" => &actual["simulation"]["trunkQuaternion"],
                "base_velocity" => &actual["simulation"]["baseVelocity"],
                "time" => &actual["simulation"]["time"],
                _ => panic!("unknown reference field {field}"),
            };
            // Feedback and IMU in PolicyTrace belong to the pre-execution sample of this
            // tick. The physics reference stores post-step fields; compare to prior frame.
            let feedback = matches!(
                field.as_str(),
                "positions" | "velocities" | "gyro" | "gravity"
            );
            if feedback && tick == 0 {
                continue;
            }
            let expected = if feedback {
                &reference["case"]["frames"][tick - 1][field]
            } else {
                &expected[field]
            };
            let error = if field == "time" {
                (value.as_f64().unwrap() - expected.as_f64().unwrap()).abs()
            } else {
                let value = value.as_array().unwrap();
                let expected = expected.as_array().unwrap();
                assert_eq!(value.len(), expected.len(), "field={field} tick={tick}");
                let direct = value
                    .iter()
                    .zip(expected)
                    .map(|(a, e)| (a.as_f64().unwrap() - e.as_f64().unwrap()).abs())
                    .fold(0.0, f64::max);
                if field == "trunk_quaternion" {
                    direct.min(
                        value
                            .iter()
                            .zip(expected)
                            .map(|(a, e)| (a.as_f64().unwrap() + e.as_f64().unwrap()).abs())
                            .fold(0.0, f64::max),
                    )
                } else {
                    direct
                }
            };
            let max = maxima.entry(field.clone()).or_default();
            *max = max.max(error);
            if error > tolerance.as_f64().unwrap() && first_failure.is_none() {
                first_failure = Some(format!("tick={tick} field={field} error={error}"));
            }
        }
    }
    let report = serde_json::json!({"ticks":ticks.len(),"max_abs_error":maxima,"first_failure":first_failure});
    crate::runtime::recordings::report_reference(&report);
    if let Some(path) = output_path {
        crate::runtime::recordings::write_trace(
            &path,
            &recorded_frames,
            preview.recording_metadata(),
        )
        .unwrap();
    }
    assert!(first_failure.is_none(), "{report}");
}

#[test]
fn preview_target_distance_only_finishes_for_positive_finite_targets() {
    assert!(!has_reached_target(None, 10.0));
    assert!(!has_reached_target(Some(0.0), 10.0));
    assert!(!has_reached_target(Some(f64::NAN), 10.0));
    assert!(!has_reached_target(Some(2.0), 1.99));
    assert!(has_reached_target(Some(2.0), 2.0));
}
