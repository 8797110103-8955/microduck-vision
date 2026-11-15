//! Virtual body templates and calculation boundaries.
//!
//! This crate deliberately has no serial, USB, bus or `ServoDriver` dependency. It consumes
//! servo model keys and immutable parameter projections, then exposes a virtual body state for
//! simulation and for the future loop-body adapter.

mod dynamics;
mod error;
mod model;
pub mod scene;
mod servo_binding;
mod servo_catalog;

pub use dynamics::{
    BodyFeedback, BodyState, BodyTargets, DeterministicDynamics, DynamicsBackend, DynamicsInput,
    VirtualBody,
};
pub use error::BodyError;
pub use model::{
    ActuatorCapability, ActuatorSlot, BodyAssets, BodyControlContract, BodyJoint, BodyLink,
    BodyPhysical, BodyStatus, BodyTemplate,
};
pub use scene::{
    OfficialScene, SceneBody, SceneError, SceneFrame, SceneGeom, SceneJoint, SceneModel, SceneSite,
};
pub use servo_binding::{ServoParameterSet, ServoParameters};
pub use servo_catalog::{
    ServoCatalogEntry, ServoModelKey, ServoProfile, servo_catalog, servo_profiles,
};

pub const OFFICIAL_BODY_ID: &str = "official";
pub const OFFICIAL_BODY_NAME: &str = "Microduck official";
pub const OFFICIAL_MJCF: &str =
    include_str!("../../../vendor/microduck_rl/src/mjlab_microduck/robot/microduck/robot_walk.xml");
#[cfg(test)]
mod tests {
    use std::time::Duration;

    use servo::ServoModel;

    use super::*;

    #[test]
    fn official_template_uses_vendor_contract() {
        let body = BodyTemplate::official().expect("official body");
        assert_eq!(body.joints.len(), duck_ipc_proto::JOINT_NAMES.len());
        assert_eq!(body.joints[9].name, "mouth");
        assert_eq!(
            body.actuators[0].suggested_servo_type.as_deref(),
            Some("hd1910")
        );
        assert_eq!(body.control.action_size, 14);
    }

    #[test]
    fn invalid_template_rejects_duplicate_joint() {
        let mut body = BodyTemplate::official().expect("official body");
        body.joints[1].name = body.joints[0].name.clone();
        assert!(matches!(
            body.validate(),
            Err(BodyError::Duplicate { kind: "joint", .. })
        ));
    }

    #[test]
    fn servo_binding_is_read_only_and_checks_capabilities() {
        let body = BodyTemplate::official().expect("official body");
        let set = ServoParameterSet::all_hd1910(&body);
        assert!(set.validate_for(&body).is_ok());
        let before = set.clone();
        let _ = VirtualBody::new(body, set).expect("virtual body");
        assert_eq!(
            before,
            ServoParameterSet::all_hd1910(&BodyTemplate::official().unwrap())
        );
    }

    #[test]
    fn servo_profiles_expose_the_complete_builtin_library_view() {
        let profiles = servo_profiles();
        let xl = profiles
            .iter()
            .find(|profile| profile.model == ServoModel::Xl330M288)
            .expect("XL330 profile");
        assert_eq!(xl.model_number, 1200);
        assert_eq!(xl.vin_range, [6.5, 8.2]);
        assert_eq!(xl.delay_steps, [3, 6]);
        assert_eq!(xl.bam_motor, "xl330");
        assert!((xl.bam_kt - 0.3459739511711113).abs() < 1e-12);
        assert!((xl.bam_command_delay - 0.010220594809412403).abs() < 1e-12);
        assert!(xl.supports_fast_sync);

        let hd = profiles
            .iter()
            .find(|profile| profile.model == ServoModel::Hd1910)
            .expect("HD1910 profile");
        assert_eq!(hd.model_number, 7946);
        assert_eq!(hd.p_at_kp_fw, Some(32));
        assert_eq!(hd.hardware_mass_g, Some(21.0));
        assert_eq!(hd.hardware_backlash_deg, Some(0.5));
        assert_eq!(hd.bam_motor, "feetech_hd1910_7_4V");
        assert!((hd.bam_kp - 32.0).abs() < 1e-12);
        assert!((hd.bam_kt - 0.5644993059031307).abs() < 1e-12);
        assert!((hd.bam_command_delay - 0.0009781055842204172).abs() < 1e-12);
        assert!(!hd.supports_fast_sync);
        assert!(hd.register_count > 0);
    }

    #[test]
    fn deterministic_body_roundtrip() {
        let body = BodyTemplate::official().expect("official body");
        let count = body.joints.len();
        let mut virtual_body = VirtualBody::new(
            body,
            ServoParameterSet::all_hd1910(&BodyTemplate::official().unwrap()),
        )
        .unwrap();
        let mut targets = vec![0.0; count];
        targets[0] = 0.2;
        virtual_body
            .write_targets(BodyTargets {
                positions: targets.clone(),
            })
            .unwrap();
        let feedback = virtual_body.step(Duration::from_millis(20)).unwrap();
        assert_eq!(feedback.state.positions, targets);
        assert_eq!(virtual_body.read_state(), feedback.state);
    }

    #[test]
    fn official_scene_loads_vendor_duck_meshes_and_frame() {
        let scene = OfficialScene::load().expect("official display scene");
        assert!(
            scene
                .model()
                .bodies
                .iter()
                .any(|body| !body.geoms.is_empty())
        );
        assert!(scene.mesh_dir().exists());
        let frame = scene
            .frame(&[0.0; duck_ipc_proto::JOINT_NAMES.len()])
            .unwrap();
        assert_eq!(frame.bodies.len(), scene.model().bodies.len());
        assert_eq!(frame.markers.len(), duck_ipc_proto::JOINT_NAMES.len());
    }

    #[test]
    fn scene_markers_use_motor_axes_and_keep_calibrated_reference_slots() {
        let scene = OfficialScene::load().expect("official display scene");
        let frame = scene
            .frame(&[0.0; duck_ipc_proto::JOINT_NAMES.len()])
            .unwrap();
        let body_pos = |index: usize| {
            [
                frame.bodies[index][0],
                frame.bodies[index][1],
                frame.bodies[index][2],
            ]
        };

        // Slots 2, 6 and 7 are the visual calibration anchors confirmed in
        // the desktop view and therefore remain at their link origins.
        for index in [1, 5, 6] {
            let body = scene.model().joints[index].body as usize;
            assert_eq!(frame.markers[index], body_pos(body));
        }

        // The mouth has no independent kinematic joint.  Its marker must use
        // the mouth motor mount rather than the head-roll body fallback.
        let head_roll_body = scene.model().joints[8].body as usize;
        assert_ne!(frame.markers[9], body_pos(head_roll_body));
    }
}
