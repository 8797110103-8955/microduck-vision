use crate::models::profile::{ActuatorResponse, BamParameters, SimulationTuning};

pub(crate) const SLIDER_DEFAULTS: SimulationTuning = SimulationTuning {
    damping: 0.053,
    friction_loss: 0.0048,
    armature: 0.0018,
    kp: 0.55,
    force_range: 0.96,
};

pub(crate) const RESPONSE: ActuatorResponse = ActuatorResponse {
    delay_steps: [3, 6],
    encoder_through_backlash: true,
    position_gain_scale: 1.0,
    back_emf_scale: 1.0,
    friction_scale: 1.0,
};

/// Reference settings in the official XL330 training and body assets.
#[derive(Clone, Copy)]
pub struct TrainingReference {
    pub kp_fw: f64,
    pub vin_range: [f64; 2],
    pub vin_drop_gain_range: [f64; 2],
    pub vin_min: f64,
    pub delay_steps: [u32; 2],
    pub backlash_deg: f64,
    pub baseline_mass_g: f64,
}

pub const TRAINING_REFERENCE: TrainingReference = TrainingReference {
    kp_fw: MOTOR.kp,
    vin_range: [6.5, 8.2],
    vin_drop_gain_range: [0.0, 0.2],
    vin_min: 6.0,
    delay_steps: [
        RESPONSE.delay_steps[0] as u32,
        RESPONSE.delay_steps[1] as u32,
    ],
    backlash_deg: 2.0,
    baseline_mass_g: 18.0,
};

pub(crate) const FIT_SOURCE: &str =
    "pinned microduck_rl XL330/M6 motor fit and native infer_policy.py defaults";

pub(crate) const MOTOR: BamParameters = BamParameters {
    motor_name: "xl330",
    model: "m6",
    vin: 7.4,
    kp: 200.0,
    error_gain: 0.00287737750222636,
    max_pwm: 1.0,
    kt: 0.36601349688984386,
    resistance: 2.8113923539223227,
    armature: 0.0018077432831600838,
    identified_position_offset_rad: 0.0271132870444849,
    identified_delay_s: 0.010220594809412403,
    friction_base: 0.004771183165566,
    friction_stribeck: 0.004676345799486616,
    friction_viscous: 0.005359668274599504,
    load_friction_motor: 0.2667860954283698,
    load_friction_external: 8.515871897059342e-06,
    load_friction_motor_stribeck: 1.0722918395099123e-05,
    load_friction_external_stribeck: 0.08077928978935671,
    load_friction_motor_quad: 0.009972471242139415,
    load_friction_external_quad: 0.004902565732332559,
    dtheta_stribeck: 2.890372094130307,
    alpha: 8.683259907618984,
};
