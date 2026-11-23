//! Load-dependent friction from the identified BAM actuator model.

use crate::models::profile::BamParameters;

pub(crate) fn friction_budget(p: &BamParameters, motor: f64, external: f64, dq: f64) -> f64 {
    let stribeck = (-(dq.abs() / p.dtheta_stribeck).powf(p.alpha)).exp();
    let base = (external * p.load_friction_external - motor * p.load_friction_motor).abs();
    let low_speed = (external * p.load_friction_external_stribeck
        - motor * p.load_friction_motor_stribeck)
        .abs();
    // BAM's equality case intentionally selects neither quadratic term.
    let quadratic = if external.signum() != motor.signum() {
        if motor.abs() > external.abs() {
            p.load_friction_external_quad * external.powi(2)
        } else if external.abs() > motor.abs() {
            p.load_friction_motor_quad * motor.powi(2)
        } else {
            0.0
        }
    } else {
        0.0
    };
    p.friction_base + base + stribeck * (p.friction_stribeck + low_speed + quadratic)
}
