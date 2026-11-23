use super::{assets, hd_parameters};
use duck_body::{
    BodyError, BodyTargets, BodyTemplate, DynamicsBackend, DynamicsInput, FileAssets, VirtualBody,
};
use std::time::Duration;

#[test]
fn body_does_not_need_a_real_servo_driver_to_run() {
    let body = BodyTemplate::official(assets(), &FileAssets).unwrap();
    let body_assets = body.assets.clone();
    assert!(body_assets.display_mjcf.exists());
    assert!(body_assets.pose_scene.is_some());
    body_assets.validate(&FileAssets).unwrap();
    let virtual_body = VirtualBody::new(body, hd_parameters()).unwrap();
    assert_eq!(virtual_body.read_state().positions.len(), 15);
}

#[test]
fn body_targets_are_range_checked() {
    let body = BodyTemplate::official(assets(), &FileAssets).unwrap();
    let mut virtual_body = VirtualBody::new(body, hd_parameters()).unwrap();
    let mut positions = vec![0.0; 15];
    positions[0] = 99.0;
    assert!(matches!(
        virtual_body.write_targets(BodyTargets { positions }),
        Err(BodyError::TargetOutOfRange { .. })
    ));
}

struct ResetStepBackend;

impl DynamicsBackend for ResetStepBackend {
    fn reset(&mut self, _state: &duck_body::BodyState) {}

    fn step(
        &mut self,
        input: &DynamicsInput,
        dt: Duration,
    ) -> Result<duck_body::BodyFeedback, BodyError> {
        assert_eq!(input.state.positions.len(), input.targets.positions.len());
        Ok(duck_body::BodyFeedback {
            state: duck_body::BodyState {
                positions: input.targets.positions.clone(),
                velocities: vec![0.0; input.targets.positions.len()],
                elapsed: input.state.elapsed + dt,
            },
            targets: input.targets.clone(),
        })
    }
}

#[test]
fn dynamics_backend_is_a_reset_step_failure_boundary() {
    let body = BodyTemplate::official(assets(), &FileAssets).unwrap();
    let mut virtual_body =
        VirtualBody::with_backend(body, hd_parameters(), Box::new(ResetStepBackend)).unwrap();
    virtual_body.reset_dynamics();
    assert!(virtual_body.step(Duration::from_millis(20)).is_ok());
    assert!(matches!(
        virtual_body.step(Duration::ZERO),
        Err(BodyError::InvalidTimeStep)
    ));
}
