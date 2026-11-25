#![cfg(feature = "native")]

use std::time::Duration;

use loop_body::{BackendPositionTargets, ControlBackend};
use microduck_body_runtime::models::assets::{OfficialAssets, load_official_body};
use microduck_body_runtime::models::resolve::defaults;
use microduck_body_runtime::simulation::SnapshotHandle;
use microduck_body_runtime::simulation::mujoco::NativeSimulation;
use microduck_body_runtime::{ExecutionKind, ModelKind, Selection, resolve_simulation};

const SUBSTEPS: u32 = 4;
const PHYSICS_STEP: Duration = Duration::from_millis(5);
const TARGETS: [[f64; 3]; 8] = [
    [0.0, 0.0, 0.0],
    [0.03, -0.02, 0.01],
    [0.03, -0.02, 0.01],
    [-0.025, 0.035, -0.01],
    [-0.025, 0.035, -0.01],
    [0.02, 0.0, 0.025],
    [0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0],
];

#[test]
fn native_response_remains_finite_across_servo_models() {
    let assets = OfficialAssets::Embedded;
    let body = load_official_body(&assets).unwrap();
    let order = &body.template().control.joint_order;
    let ids: Vec<_> = (1..=order.len() as u8).collect();
    let index = |name: &str| order.iter().position(|joint| joint == name).unwrap();
    let slots = [index("left_knee"), index("right_knee"), index("head_yaw")];

    for (label, model) in [
        ("xl330", ModelKind::Xl330),
        ("hd1910", ModelKind::Hd1910),
        ("hd1910-compatible", ModelKind::Hd1910Compatible),
    ] {
        let mut tuning = defaults(model);
        if model == ModelKind::Hd1910Compatible {
            tuning.kp *= 1.5;
            tuning.friction_loss *= 1.25;
        }
        let selection = Selection {
            execution_kind: ExecutionKind::Simulation,
            model_kind: model,
            servo_model: model.servo_model(),
        };
        let profile = resolve_simulation(selection, &body, &assets, tuning).unwrap();
        let snapshot = SnapshotHandle::default();
        let mut native =
            NativeSimulation::open(body.template(), &profile, &assets, snapshot.clone()).unwrap();
        let initial = snapshot.lock().unwrap().state.clone().unwrap();
        assert_eq!(initial.physics_step, 0);
        native.set_torque(true).unwrap();
        for (tick, offsets) in TARGETS.into_iter().enumerate() {
            let mut positions = profile.home_pose_rad.clone();
            for (slot, offset) in slots.into_iter().zip(offsets) {
                positions[slot] += offset;
            }
            native
                .write_positions(&ids, &BackendPositionTargets { positions })
                .unwrap();
            native.advance_execution(SUBSTEPS, PHYSICS_STEP).unwrap();
            let sample = native.read_positions(&ids).unwrap();
            let state = snapshot.lock().unwrap();
            let state = state.state.as_ref().unwrap();
            assert_eq!(state.physics_step, (tick as u64 + 1) * u64::from(SUBSTEPS));
            assert_eq!(sample.positions.len(), order.len());
            assert_eq!(sample.velocities.len(), order.len());
            assert!(
                slots
                    .into_iter()
                    .all(|slot| sample.positions[slot].is_finite()
                        && sample.velocities[slot].is_finite()
                        && state.motor_torques[slot].is_finite()
                        && state.friction_budgets[slot].is_finite()),
                "{label}, control tick {tick}"
            );
        }
    }
}
