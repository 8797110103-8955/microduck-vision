use crate::models::profile::PhysicalGains;

pub(crate) const GAINS: PhysicalGains = PhysicalGains::Hd1910 { p: 32 };
pub(crate) const SOURCE: &str = "HD1910 mode 4 runtime P from desktop hardware readback";
