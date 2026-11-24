use crate::models::profile::PhysicalGains;

pub(crate) mod profile;

// Runtime mode 4 P gain read from HD1910 hardware by the desktop app.
pub(crate) const PHYSICAL_GAINS: PhysicalGains = PhysicalGains::Hd1910 { p: 32 };
