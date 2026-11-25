use std::path::Path;

use duck_body::{AssetResolver, BodyProfile};
use microduck_body_runtime::models::assets::{OfficialAssets, load_official_body};
use microduck_body_runtime::models::profile::BodyPeriods;
use microduck_body_runtime::models::resolve::defaults;
use microduck_body_runtime::training::{
    TaskBodyDescription, export_native_simulation, export_recorded_native_simulation,
    resolve_mjlab_body, resolve_training_body,
};
use microduck_body_runtime::{ExecutionKind, ModelKind, Selection};
use serde_json::{Value, json};
use sha2::{Digest, Sha256};

fn task(body: &BodyProfile, robot_xml: &str, backlash: bool) -> TaskBodyDescription {
    let template = body.template();
    let bytes = OfficialAssets::Embedded.read(Path::new(robot_xml)).unwrap();
    TaskBodyDescription {
        task_id: format!("test-{robot_xml}"),
        robot_xml: robot_xml.to_owned(),
        robot_sha256: format!("{:x}", Sha256::digest(bytes)),
        joint_order: template.control.joint_order.clone(),
        robot_joints: template
            .joints
            .iter()
            .map(|joint| joint.scene_joint.clone())
            .collect(),
        home_pose_rad: template.control.home_pose.clone(),
        root_home_pose: [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
        periods: BodyPeriods {
            control_s: 0.02,
            physics_s: Some(0.005),
            physics_substeps: Some(4),
        },
        requires_backlash: backlash,
        name_servo_collision_geoms: robot_xml.starts_with("robot_allcollisions"),
        target_delay_steps: [3, 6],
        policy_layout: json!({"action_len": 14}),
        assembly: json!({"entities": ["robot"]}),
    }
}

fn select(model_kind: ModelKind) -> Selection {
    Selection {
        execution_kind: ExecutionKind::Simulation,
        model_kind,
        servo_model: model_kind.servo_model(),
    }
}

#[test]
fn training_uses_each_task_robot_and_preserves_its_source_assets() {
    let assets = OfficialAssets::Embedded;
    let body = load_official_body(&assets).unwrap();
    let model = ModelKind::Xl330;
    for (variant, backlash) in [
        ("robot_walk.xml", false),
        ("robot_allcollisions.xml", false),
        ("robot_groundcontact.xml", false),
        ("robot_walk_backlash.xml", true),
    ] {
        let source = task(&body, variant, backlash);
        let profile = resolve_training_body(
            select(model),
            &body,
            &assets,
            defaults(model),
            source.clone(),
        )
        .unwrap();
        let execution = profile.profile.simulation.as_ref().unwrap();
        assert_eq!(execution.scenario.robot_xml, variant);
        assert!(execution.scenario.scene_xml.is_none());
        assert_eq!(execution.response.encoder_through_backlash, backlash);
        assert_eq!(execution.response.delay_steps, [3, 6]);
        assert_eq!(execution.root_home_pose, source.root_home_pose);
        let export: Value = serde_json::from_slice(&profile.export(&assets).unwrap()).unwrap();
        assert_eq!(export["format_version"], 2);
        assert_eq!(export["backend"]["kind"], "mjlab-bam");
        assert_eq!(export["task"]["robot_sha256"], source.robot_sha256);
        assert!(
            export["assets"]
                .as_array()
                .unwrap()
                .iter()
                .any(|asset| asset["path"] == variant)
        );
        assert!(
            export["assets"]
                .as_array()
                .unwrap()
                .iter()
                .all(|asset| !asset["path"].as_str().unwrap().starts_with("scene"))
        );
    }
}

#[test]
fn mjlab_parameters_preserve_profile_provenance_and_apply_response_once() {
    let assets = OfficialAssets::Embedded;
    let body = load_official_body(&assets).unwrap();
    for model in [
        ModelKind::Xl330,
        ModelKind::Hd1910,
        ModelKind::Hd1910Compatible,
    ] {
        let mut tuning = defaults(model);
        tuning.kp *= 1.5;
        tuning.friction_loss *= 1.25;
        let resolved = resolve_training_body(
            select(model),
            &body,
            &assets,
            tuning,
            task(&body, "robot_walk.xml", false),
        )
        .unwrap();
        let before = serde_json::to_value(&resolved.profile).unwrap();
        let mut profile = resolved.profile.clone();
        // Exercise non-unit transformations independently of model dispatch.
        let simulation = profile.simulation.as_mut().unwrap();
        simulation.response.back_emf_scale = 0.478;
        simulation.response.friction_scale = 0.755;
        let motor = simulation.motor;
        let response = simulation.response;
        let backend = resolve_mjlab_body(&profile).unwrap();
        let fit: Value = serde_json::from_str(&backend.bam_fit.content).unwrap();
        let kt = fit["kt"].as_f64().unwrap();
        let resistance = fit["R"].as_f64().unwrap();
        assert!((kt / resistance - motor.kt / motor.resistance).abs() < 1e-12);
        assert!(
            (kt * kt / resistance / (motor.kt * motor.kt / motor.resistance) - 0.478).abs() < 1e-12
        );
        assert!(
            (fit["friction_base"].as_f64().unwrap() - motor.friction_base * 0.755).abs() < 1e-15
        );
        assert!((fit["friction_viscous"].as_f64().unwrap() - motor.friction_viscous).abs() < 1e-15);
        assert_eq!(
            backend.actuator.kp_fw,
            motor.kp * response.position_gain_scale
        );
        assert_eq!(backend.actuator.error_gain, motor.error_gain);
        assert_eq!(backend.actuator.max_pwm, motor.max_pwm);
        assert!(backend.actuator.max_current.is_none());
        assert_eq!(
            backend.bam_fit.sha256,
            format!("{:x}", Sha256::digest(backend.bam_fit.content.as_bytes()))
        );
        assert_eq!(backend.execution_fingerprint, profile.execution_fingerprint);
        assert_eq!(serde_json::to_value(&resolved.profile).unwrap(), before);
        profile.simulation.as_mut().unwrap().motor.resistance = f64::NAN;
        assert!(resolve_mjlab_body(&profile).is_err());
        profile.simulation.as_mut().unwrap().motor = motor;
        profile.simulation.as_mut().unwrap().response.back_emf_scale = 0.0;
        assert!(resolve_mjlab_body(&profile).is_err());
    }
}

#[test]
fn native_and_recorded_exports_keep_the_recording_contract() {
    let assets = OfficialAssets::Embedded;
    let body = load_official_body(&assets).unwrap();
    let profile = microduck_body_runtime::resolve_simulation(
        select(ModelKind::Xl330),
        &body,
        &assets,
        defaults(ModelKind::Xl330),
    )
    .unwrap();
    let native: Value =
        serde_json::from_slice(&export_native_simulation(&profile, &assets).unwrap()).unwrap();
    let recorded: Value = serde_json::from_slice(
        &export_recorded_native_simulation(&native["profile"], &assets).unwrap(),
    )
    .unwrap();
    assert_eq!(native, recorded);
    assert_eq!(native["format_version"], 1);
    assert!(native.get("backend").is_none());
}

#[test]
fn hd_profiles_keep_the_task_robot_and_reject_backlash_requirements() {
    let assets = OfficialAssets::Embedded;
    let body = load_official_body(&assets).unwrap();
    let walk = task(&body, "robot_walk.xml", false);
    let backlash = task(&body, "robot_walk_backlash.xml", true);
    for model in [ModelKind::Hd1910, ModelKind::Hd1910Compatible] {
        let profile =
            resolve_training_body(select(model), &body, &assets, defaults(model), walk.clone())
                .unwrap();
        let simulation = profile.profile.simulation.unwrap();
        assert_eq!(simulation.scenario.robot_xml, "robot_walk.xml");
        assert_eq!(simulation.response.delay_steps, [0, 0]);
        let result = resolve_training_body(
            select(model),
            &body,
            &assets,
            defaults(model),
            backlash.clone(),
        );
        assert!(result.err().unwrap().contains("requires backlash hinges"));
    }
}
