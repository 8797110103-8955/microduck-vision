use crate::models::profile::{ActuatorResponse, BamParameters};

pub(crate) const REVISION: u32 = 1;

pub(crate) fn response(xl: BamParameters, hd: BamParameters) -> ActuatorResponse {
    ActuatorResponse {
        position_gain_scale: super::compensation::position_gain_scale(xl, hd),
        ..crate::models::hd1910::profile::RESPONSE
    }
}
