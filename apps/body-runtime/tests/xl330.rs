#![cfg(feature = "native")]

use std::time::Duration;

use loop_body::{BackendPositionTargets, ControlBackend, GainCapability};
use microduck_body_runtime::models::assets::{OfficialAssets, load_official_body};
use microduck_body_runtime::models::resolve::defaults;
use microduck_body_runtime::simulation::SnapshotHandle;
use microduck_body_runtime::simulation::mujoco::NativeSimulation;
use microduck_body_runtime::{ExecutionKind, ModelKind, Selection, resolve_simulation};

#[test]
fn official_scene_exposes_backlash_feedback_and_fixed_model_gain() {
    let source = OfficialAssets::Embedded;
    let body = load_official_body(&source).unwrap();
    let model = ModelKind::Xl330;
    let selection = Selection {
        execution_kind: ExecutionKind::Simulation,
        model_kind: model,
        servo_model: model.servo_model(),
    };
    let profile = resolve_simulation(selection, &body, &source, defaults(model)).unwrap();
    let simulation = profile.simulation.as_ref().unwrap();
    assert_eq!(simulation.scenario.scene_xml, Some("scene_backlash.xml"));
    assert_eq!(
        simulation.scenario.robot_xml,
        "robot_groundcontact_backlash.xml"
    );
    assert_eq!(simulation.response.delay_steps, [3, 6]);

    let snapshot = SnapshotHandle::default();
    let mut backend =
        NativeSimulation::open(body.template(), &profile, &source, snapshot.clone()).unwrap();
    assert_eq!(backend.gain_capability(), GainCapability::FixedModel);
    assert!(backend.set_gain(200).is_err());
    backend.set_torque(true).unwrap();
    backend
        .write_positions(
            &[8, 9],
            &BackendPositionTargets {
                positions: vec![0.1, 0.2],
            },
        )
        .unwrap();
    for _ in 0..5 {
        backend
            .advance_execution(4, Duration::from_millis(5))
            .unwrap();
    }
    let expected = snapshot.lock().unwrap();
    let expected = expected.state.as_ref().unwrap();
    let ids: Vec<_> = (1..=15).collect();
    let sample = backend.read_positions(&ids).unwrap();
    assert_eq!(sample.positions.len(), 15);
    let mouth = profile
        .joint_order
        .iter()
        .position(|name| name == "mouth")
        .unwrap();
    assert_eq!(expected.passive_positions[mouth], 0.0);
    assert!(
        expected
            .passive_positions
            .iter()
            .any(|angle| angle.abs() > 1e-6)
    );
    for (index, position) in sample.positions.iter().enumerate() {
        assert_eq!(
            *position,
            expected.motor_positions[index] + expected.passive_positions[index],
            "{}",
            profile.joint_order[index]
        );
    }
}
