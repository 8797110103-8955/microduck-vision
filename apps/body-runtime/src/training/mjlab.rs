//! Translate a resolved product profile into final parameters for official BAM.

use serde::Serialize;
use sha2::{Digest, Sha256};

use crate::models::profile::BodyExecutionProfile;
use crate::models::scene_recipe::SceneOperation;

#[derive(Debug, Serialize)]
pub struct MjlabBodyConfig {
    pub kind: &'static str,
    pub version: u32,
    pub execution_fingerprint: String,
    pub encoder_through_backlash: bool,
    pub name_servo_collision_geoms: bool,
    pub actuator: MjlabActuatorConfig,
    pub bam_fit: BamFitFile,
}

#[derive(Debug, Serialize)]
pub struct MjlabActuatorConfig {
    pub vin: f64,
    pub kp_fw: f64,
    pub error_gain: f64,
    pub max_pwm: f64,
    pub max_current: Option<f64>,
    pub delay_min_lag: usize,
    pub delay_max_lag: usize,
    pub vin_min: f64,
    pub voltage_drop_gain: f64,
}

#[derive(Debug, Serialize)]
pub struct BamFitFile {
    pub file: &'static str,
    pub content: String,
    pub sha256: String,
}

pub fn resolve_mjlab_body(profile: &BodyExecutionProfile) -> Result<MjlabBodyConfig, String> {
    let simulation = profile
        .simulation
        .as_ref()
        .ok_or("MJLab requires a simulation profile")?;
    let response = simulation.response;
    let mut motor = simulation.motor;
    if !response.back_emf_scale.is_finite()
        || response.back_emf_scale <= 0.0
        || !response.friction_scale.is_finite()
        || response.friction_scale < 0.0
    {
        return Err("Invalid BAM response scales".into());
    }
    // Preserve kt/R (command torque), while changing kt²/R (back-EMF damping).
    motor.kt *= response.back_emf_scale;
    motor.resistance *= response.back_emf_scale;
    for friction in [
        &mut motor.friction_base,
        &mut motor.friction_stribeck,
        &mut motor.load_friction_motor,
        &mut motor.load_friction_external,
        &mut motor.load_friction_motor_stribeck,
        &mut motor.load_friction_external_stribeck,
        &mut motor.load_friction_motor_quad,
        &mut motor.load_friction_external_quad,
    ] {
        *friction *= response.friction_scale;
    }
    let mut fit = serde_json::to_value(motor).map_err(|error| error.to_string())?;
    let fields = fit.as_object_mut().ok_or("Invalid motor parameters")?;
    if fields
        .iter()
        .any(|(name, value)| !matches!(name.as_str(), "motor_name" | "model") && !value.is_number())
        || motor.resistance <= 0.0
        || motor.kt <= 0.0
        || motor.dtheta_stribeck <= 0.0
        || !(0.0..=1.0).contains(&motor.max_pwm)
    {
        return Err("Invalid effective BAM motor parameters".into());
    }
    for name in [
        "motor_name",
        "vin",
        "kp",
        "error_gain",
        "max_pwm",
        "identified_delay_s",
    ] {
        fields.remove(name);
    }
    for (source, target) in [
        ("resistance", "R"),
        ("identified_position_offset_rad", "q_offset"),
    ] {
        let value = fields.remove(source).ok_or("Missing BAM fit parameter")?;
        fields.insert(target.into(), value);
    }
    // This selects BAM's voltage-control implementation, not the product servo.
    fields.insert("actuator".into(), "xl330".into());
    let content = serde_json::to_string(&fit).map_err(|error| error.to_string())?;
    let actuator = MjlabActuatorConfig {
        vin: motor.vin,
        kp_fw: motor.kp * response.position_gain_scale,
        error_gain: motor.error_gain,
        max_pwm: motor.max_pwm,
        max_current: None,
        delay_min_lag: response.delay_steps[0],
        delay_max_lag: response.delay_steps[1],
        vin_min: simulation.minimum_voltage_v,
        voltage_drop_gain: simulation.voltage_drop_gain,
    };
    if [
        actuator.vin,
        actuator.kp_fw,
        actuator.error_gain,
        actuator.max_pwm,
        actuator.vin_min,
        actuator.voltage_drop_gain,
    ]
    .iter()
    .any(|value| !value.is_finite() || *value < 0.0)
        || actuator.delay_min_lag > actuator.delay_max_lag
    {
        return Err("Invalid effective BAM actuator parameters".into());
    }
    Ok(MjlabBodyConfig {
        kind: "mjlab-bam",
        version: 1,
        execution_fingerprint: profile.execution_fingerprint.clone(),
        encoder_through_backlash: response.encoder_through_backlash,
        name_servo_collision_geoms: simulation
            .scenario
            .derived_scene_operations
            .contains(&SceneOperation::NameServoCollisionGeoms),
        actuator,
        bam_fit: BamFitFile {
            file: "bam-fit.json",
            sha256: format!("{:x}", Sha256::digest(content.as_bytes())),
            content,
        },
    })
}
