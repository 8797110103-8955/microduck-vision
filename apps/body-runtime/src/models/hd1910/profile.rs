use crate::models::profile::{ActuatorResponse, BamParameters, SimulationTuning};

pub(crate) const SLIDER_DEFAULTS: SimulationTuning = crate::models::xl330::profile::SLIDER_DEFAULTS;

pub(crate) const RESPONSE: ActuatorResponse = ActuatorResponse {
    delay_steps: [0, 0],
    encoder_through_backlash: false,
    position_gain_scale: 1.0,
    back_emf_scale: 1.0,
    friction_scale: 1.0,
};

pub(crate) const FIT_SOURCE: &str = "HD1910 7.4 V M6 fit from the existing MicroDuck simulation";
pub(crate) const SCENE_ASSUMPTION: &str = "Use the pinned link inertials until HD1910 assembly mass distribution and inertia are measured";
pub(crate) const RESPONSE_ASSUMPTION: &str =
    "Native HD1910 delay and mechanical backlash are not calibrated in the initial solver model";

pub(crate) const MOTOR: BamParameters = BamParameters {
    motor_name: "feetech_hd1910_7_4V",
    model: "m6",
    vin: 7.4,
    kp: 32.0,
    error_gain: 0.163,
    max_pwm: 0.97,
    kt: 0.5644993059031307,
    resistance: 2.9664762312357853,
    armature: 0.0018273861091867637,
    identified_position_offset_rad: 0.008824155147854507,
    identified_delay_s: 0.0009781055842204172,
    friction_base: 0.011950659296477161,
    friction_stribeck: 0.0,
    friction_viscous: 0.004767196924056633,
    load_friction_motor: 0.3534653049182988,
    load_friction_external: 0.04335300722280222,
    load_friction_motor_stribeck: 1.6996676628765351e-6,
    load_friction_external_stribeck: 0.14250972390583813,
    load_friction_motor_quad: 0.006861907376104702,
    load_friction_external_quad: 0.009999951209775423,
    dtheta_stribeck: 0.716192604078371,
    alpha: 4.248291402139783,
};
