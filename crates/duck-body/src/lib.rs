//! Virtual body templates and calculation boundaries.
//!
//! This crate deliberately has no serial, USB, bus or `ServoDriver` dependency. It consumes
//! servo model keys and immutable parameter projections, then exposes a virtual body state for
//! simulation and for the future loop-body adapter.

mod action_space;
mod binding;
mod contract;
mod dynamics;
mod error;
mod model;
mod profile;
pub mod scene;
mod servo_binding;
mod servo_catalog;
pub mod types;

pub use action_space::{BodyActionCodec, BodyActionSpace};
pub use binding::{BodyBinding, JointCalibration};
pub use contract::BodyContractSnapshot;
pub use dynamics::{
    BodyFeedback, BodyState, BodyTargets, DeterministicDynamics, DynamicsBackend, DynamicsInput,
    VirtualBody,
};
pub use error::BodyError;
pub use model::{
    ActuatorCapability, ActuatorSlot, BodyAssets, BodyControlContract, BodyJoint, BodyLink,
    BodyPhysical, BodyStatus, BodyTemplate,
};
pub use profile::{BodyIdentity, BodyProfile, BoundBody};
pub use scene::{
    OfficialScene, SceneBody, SceneError, SceneFrame, SceneGeom, SceneJoint, SceneModel, SceneSite,
};
pub use servo_binding::{ServoParameterSet, ServoParameters};
pub use servo_catalog::{ServoProfile, servo_catalog, servo_profiles};
pub use types::{ServoCatalogEntry, ServoModelKey};

pub const OFFICIAL_BODY_ID: &str = "official";
pub const OFFICIAL_BODY_NAME: &str = "Microduck official";
pub const OFFICIAL_MJCF: &str =
    include_str!("../../../vendor/microduck_rl/src/mjlab_microduck/robot/microduck/robot_walk.xml");
pub const OFFICIAL_SCENE: &str =
    include_str!("../../../vendor/microduck_rl/src/mjlab_microduck/robot/microduck/scene.xml");
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
        assert!(body.reference_poses.contains_key("STAND"));
        assert!(body.reference_poses.contains_key("SIT"));
    }

    #[test]
    fn official_profile_exposes_one_canonical_action_projection() {
        let profile = BodyProfile::official().expect("official profile");
        assert_eq!(profile.identity().id, OFFICIAL_BODY_ID);
        assert!(!profile.identity().source_hash.is_empty());
        assert_eq!(profile.action_space().canonical_len(), 15);
        assert_eq!(profile.action_space().action_len(), 14);
        assert!(!profile.action_space().policy_action_slots().contains(&9));

        let action: Vec<f32> = (0..14).map(|value| value as f32).collect();
        let canonical = profile.action_codec().decode(&action).unwrap();
        assert_eq!(canonical[0], 0.0);
        assert_eq!(canonical[9], 0.0);
        assert_eq!(canonical[10], 9.0);
        assert_eq!(profile.action_codec().encode(&canonical).unwrap(), action);
    }

    #[test]
    fn official_profile_reads_joint_axes_from_mjcf() {
        let profile = BodyProfile::official().expect("official profile");
        assert_eq!(profile.template().joints[0].axis, [0.0, 0.0, 1.0]);
    }

    #[test]
    fn official_home_pose_is_loaded_from_the_scene_stand_pose() {
        let body = BodyTemplate::official().expect("official body");
        assert_eq!(body.control.home_pose, body.reference_poses["STAND"]);
        assert_eq!(body.control.home_pose[9], 0.0, "mouth is not a scene joint");
        assert!((body.control.home_pose[2] + 0.457924).abs() < 1e-6);
    }

    #[test]
    fn home_matches_training_and_vendor() {
        let body = BodyTemplate::official().expect("official body");
        let training = include_str!(
            "../../../vendor/microduck_rl/src/mjlab_microduck/robot/microduck_constants.py"
        );
        let section = training
            .split_once("HOME_FRAME =")
            .and_then(|(_, rest)| rest.split_once("    },"))
            .map(|(section, _)| section)
            .expect("training HOME_FRAME section");
        for joint in body
            .joints
            .iter()
            .filter(|joint| joint.scene_joint.is_some())
        {
            let pattern_name = if joint.name.ends_with("hip_yaw") {
                "hip_yaw"
            } else {
                &joint.name
            };
            let marker = format!(".*{}.*\":", pattern_name);
            let line = section
                .lines()
                .find(|line| line.contains(&marker))
                .unwrap_or_else(|| panic!("training HOME_FRAME missing {}", joint.name));
            let actual = line
                .split_once(':')
                .and_then(|(_, value)| value.trim().trim_end_matches(',').parse::<f64>().ok())
                .expect("training home value");
            let index = body.joint_index(&joint.name).expect("body joint");
            assert!(
                (actual - body.control.home_pose[index]).abs() < 1e-4,
                "{}: training={actual}, body={}",
                joint.name,
                body.control.home_pose[index]
            );
        }
        assert_eq!(body.control.home_pose, body.reference_poses["STAND"]);
    }

    #[test]
    fn body_profile_exports_a_self_validating_compatibility_snapshot() {
        let profile = BodyProfile::official().expect("official profile");
        let snapshot = profile.compatibility_snapshot();
        assert_eq!(snapshot.body_id, OFFICIAL_BODY_ID);
        assert_eq!(snapshot.home_pose_rad, profile.template().control.home_pose);
        assert!(snapshot.to_json().unwrap().contains("source_hash"));
        snapshot.validate_against(&profile).unwrap();
    }

    #[test]
    fn body_binding_owns_machine_conversion_and_validation() {
        let profile = BodyProfile::official().expect("official profile");
        let binding = BodyBinding {
            body_id: profile.identity().id.clone(),
            body_version: profile.identity().version,
            joints: profile
                .template()
                .actuators
                .iter()
                .enumerate()
                .map(|(index, slot)| JointCalibration {
                    slot_id: slot.id.clone(),
                    physical_id: index as u8 + 1,
                    direction: if index == 0 { -1 } else { 1 },
                    zero_offset_rad: 0.1,
                    min_rad: None,
                    max_rad: None,
                })
                .collect(),
        };
        let bound = profile.bind(binding).expect("valid machine binding");
        let servo = bound.binding().to_servo_positions(&[0.2; 15]);
        let body = bound.binding().to_body_positions(&servo);
        assert!((body[0] - 0.2).abs() < 1e-9);
        assert!((body[1] - 0.2).abs() < 1e-9);
        assert_eq!(bound.binding().to_body_velocities(&[2.0; 15])[0], -2.0);
    }

    #[test]
    fn conversion_roundtrip_exact() {
        let profile = BodyProfile::official().expect("official profile");
        let binding = BodyBinding {
            body_id: profile.identity().id.clone(),
            body_version: profile.identity().version,
            joints: profile
                .template()
                .actuators
                .iter()
                .enumerate()
                .map(|(index, slot)| JointCalibration {
                    slot_id: slot.id.clone(),
                    physical_id: index as u8 + 1,
                    direction: if index % 2 == 0 { -1 } else { 1 },
                    zero_offset_rad: (index as f64 - 7.0) * 0.01,
                    min_rad: None,
                    max_rad: None,
                })
                .collect(),
        };
        let canonical: Vec<f64> = (0..15).map(|index| index as f64 * 0.02 - 0.1).collect();
        let physical = binding.to_servo_positions(&canonical);
        assert!(
            binding
                .to_body_positions(&physical)
                .iter()
                .zip(&canonical)
                .all(|(actual, expected)| (actual - expected).abs() < 1e-12)
        );
        let velocities: Vec<f64> = (0..15).map(|index| index as f64 * 0.3).collect();
        let canonical_velocities = binding.to_body_velocities(&velocities);
        assert!(
            canonical_velocities
                .iter()
                .enumerate()
                .all(|(index, value)| {
                    let expected = velocities[index] * f64::from(binding.joints[index].direction);
                    (*value - expected).abs() < 1e-12
                })
        );
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
