use super::{PolicyCache, PolicyExecution, has_reached_target};
use crate::test_support::simulation_selection;
use crate::types::ServoModelKey;
use microduck_body_runtime::ModelKind;
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

/// Export a product-side trace against the same pinned artifact used by the CPU MuJoCo
/// reference runner. This is intentionally ignored because the desktop test environment
/// does not ship ONNX Runtime or the artifact; run it manually with
/// `MICRODUCK_OFFICIAL_ONNX` and `ORT_DYLIB_PATH` set to produce a comparable trace.
#[test]
fn preview_target_distance_only_finishes_for_positive_finite_targets() {
    assert!(!has_reached_target(None, 10.0));
    assert!(!has_reached_target(Some(0.0), 10.0));
    assert!(!has_reached_target(Some(f64::NAN), 10.0));
    assert!(!has_reached_target(Some(2.0), 1.99));
    assert!(has_reached_target(Some(2.0), 2.0));
}
