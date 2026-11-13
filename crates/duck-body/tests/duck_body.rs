use std::time::Duration;

use duck_body::{
    ActuatorCapability, BodyError, BodyTargets, BodyTemplate, DynamicsBackend, DynamicsInput,
    ServoParameterSet, VirtualBody,
};
use servo::ServoModel;

#[test]
fn vendor_kinematics_fixture_has_the_official_sites_and_ranges() {
    let model = kinematics::Model::alpha();
    assert_eq!(model.num_joints(), 14);
    for joint in ["left_hip_yaw", "left_knee", "neck_pitch", "right_ankle"] {
        let index = model.joint_index(joint).expect("vendor joint");
        let (lo, hi) = model.joint_range(index).expect("vendor range");
        assert!(lo < hi);
    }
    for site in ["left_foot", "right_foot", "head_camera", "tof"] {
        assert!(model.site(site).is_some(), "missing vendor site {site}");
    }
}

#[test]
fn official_body_has_explicit_fifteen_slot_contract() {
    let body = BodyTemplate::official().unwrap();
    assert_eq!(body.control.joint_order.len(), 15);
    assert_eq!(body.joints.len(), 15);
    assert_eq!(body.actuators.len(), 15);
    assert_eq!(body.joints[9].name, "mouth");
    assert!(body.joints[9].scene_joint.is_none());
}

#[test]
fn hd1910_is_default_and_xl330_is_an_explicit_compatible_override() {
    let body = BodyTemplate::official().unwrap();
    let hd = ServoParameterSet::all_hd1910(&body);
    assert!(hd.validate_for(&body).is_ok());
    let xl = ServoParameterSet::bind_all(&body, ServoModel::Xl330M288);
    assert!(xl.validate_for(&body).is_ok());
}

#[test]
fn body_does_not_need_a_real_servo_driver_to_run() {
    let body = BodyTemplate::official().unwrap();
    let body_assets = body.assets.clone();
    assert!(body_assets.display_mjcf.exists());
    assert!(body_assets.physics_scene.is_some());
    assert!(body_assets.physics_robot.is_some());
    let virtual_body = VirtualBody::new(body, hd_parameters()).unwrap();
    assert_eq!(virtual_body.read_state().positions.len(), 15);
}

#[test]
fn template_parser_reports_missing_required_fields() {
    let err = BodyTemplate::from_toml("id = \"x\"\n").unwrap_err();
    assert!(matches!(err, BodyError::Toml(_)));
}

#[test]
fn template_validation_reports_missing_actuator_joint() {
    let mut body = BodyTemplate::official().unwrap();
    body.actuators[0].joint = "missing".to_owned();
    assert!(matches!(
        body.validate(),
        Err(BodyError::MissingJoint { .. })
    ));
}

#[test]
fn body_targets_are_range_checked() {
    let body = BodyTemplate::official().unwrap();
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
    let body = BodyTemplate::official().unwrap();
    let mut virtual_body =
        VirtualBody::with_backend(body, hd_parameters(), Box::new(ResetStepBackend)).unwrap();
    virtual_body.reset_dynamics();
    assert!(virtual_body.step(Duration::from_millis(20)).is_ok());
    assert!(matches!(
        virtual_body.step(Duration::ZERO),
        Err(BodyError::InvalidTimeStep)
    ));
}

fn hd_parameters() -> ServoParameterSet {
    let body = BodyTemplate::official().unwrap();
    ServoParameterSet::all_hd1910(&body)
}

#[test]
fn capability_names_are_product_neutral() {
    let body = BodyTemplate::official().unwrap();
    assert!(
        body.actuators[0]
            .required_capabilities
            .contains(&ActuatorCapability::PositionControl)
    );
}

#[test]
fn display_and_physics_assets_are_vendor_fixtures() {
    let body = BodyTemplate::official().unwrap();
    let path = body.assets.display_mjcf;
    assert!(
        path.ends_with("vendor/microduck_rl/src/mjlab_microduck/robot/microduck/robot_walk.xml")
    );
}

#[test]
fn official_mjcf_mass_and_inertia_are_loaded_in_grams() {
    let body = BodyTemplate::official().unwrap();
    let leg = body
        .physical
        .links
        .iter()
        .find(|link| link.name == "leg")
        .unwrap();
    assert!((leg.mass_g - 21.5844).abs() < 0.001);
    assert!(leg.inertia.is_some());
    assert!(body.physical.mass_g.unwrap() > 0.0);
}

#[test]
fn mounted_servo_mass_composes_per_slot() {
    let mut body = BodyTemplate::official().unwrap();
    for slot in &mut body.actuators {
        slot.mounted_mass_g = Some(1.0);
    }
    let servos = hd_parameters();
    let total = servos.total_mass_g(&body).unwrap();
    assert!((total - (body.physical.mass_g.unwrap() + 15.0)).abs() < 1e-9);
}

#[test]
fn official_asset_validation_uses_the_vendor_path() {
    let body = BodyTemplate::official().unwrap();
    assert!(body.validate_assets().is_ok());
}
