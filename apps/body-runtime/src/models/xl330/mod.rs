use crate::models::profile::PhysicalGains;

pub(crate) mod profile;

// Gains from vendor/microduck/robotd/src/control.rs and duck-control/src/bus.rs.
pub(crate) const PHYSICAL_GAINS: PhysicalGains = PhysicalGains::Xl330 { p: 200, i: 0, d: 0 };
