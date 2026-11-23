use crate::models::profile::PhysicalGains;

pub(crate) const GAINS: PhysicalGains = PhysicalGains::Xl330 { p: 200, i: 0, d: 0 };
pub(crate) const SOURCE: &str =
    "vendor/microduck/robotd/src/control.rs and duck-control/src/bus.rs";
