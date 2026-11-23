use crate::models::profile::BamParameters;

/// Evaluate nominal stiffness before applying any tuning to the HD parameters.
pub(crate) fn position_gain_scale(xl: BamParameters, hd: BamParameters) -> f64 {
    xl.position_stiffness() / hd.position_stiffness()
}
