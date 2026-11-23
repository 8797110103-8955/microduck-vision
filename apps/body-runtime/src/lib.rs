//! Product configuration and executable body sources for MicroDuck.

pub mod models;
#[cfg(feature = "native")]
pub mod simulation;
pub mod training;

pub use models::profile::{
    BodyExecutionProfile, ExecutionKind, ModelKind, PhysicalGains, Selection,
};
pub use models::resolve::{resolve_physical, resolve_simulation};
