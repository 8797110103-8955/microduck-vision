use crate::models::profile::{ActuatorResponse, BamParameters};

pub(crate) const REVISION: u32 = 2;

pub(crate) fn response(xl: BamParameters, hd: BamParameters) -> ActuatorResponse {
    ActuatorResponse {
        position_gain_scale: position_gain_scale(xl, hd),
        ..crate::models::hd1910::profile::RESPONSE
    }
}

/// Fit the compatibility response jointly to recordings of standing, walking,
/// crouching, rolling, picking up, kicking, and sit-to-stand transitions.
pub(crate) fn desktop_response(xl: BamParameters, hd: BamParameters) -> ActuatorResponse {
    ActuatorResponse {
        // Read targets from 3 to 6 steps ago at each 5 ms physics step to model
        // the XL330 command delay of 15 to 30 ms.
        delay_steps: [3, 6],
        // Include passive backlash in angle and velocity feedback so the policy
        // observes output motion similar to the XL330.
        encoder_through_backlash: true,
        // Compensate HD1910 using the nominal position stiffness ratio, then
        // refine tracking strength with a coefficient fitted across motions.
        position_gain_scale: position_gain_scale(xl, hd) * 0.989,
        // Scale back-EMF velocity damping to reduce HD1910 braking differences
        // during fast motions.
        back_emf_scale: 0.478,
        // Scale BAM static and load-dependent friction budgets to bring motion
        // onset and reversals closer to the reference trajectories.
        friction_scale: 0.755,
    }
}

pub(crate) fn desktop_motor(mut hd: BamParameters) -> BamParameters {
    // Lower the compatibility simulation PWM limit to cap drive force at large
    // errors and reduce overshoot during motions such as kicks and rolls.
    hd.max_pwm *= 0.720;
    hd
}

/// Use nominal position stiffness so recalculating compensation after UI tuning
/// does not cancel the user's gain adjustment.
fn position_gain_scale(xl: BamParameters, hd: BamParameters) -> f64 {
    xl.position_stiffness() / hd.position_stiffness()
}
