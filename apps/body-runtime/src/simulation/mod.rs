//! Native simulation and its body-loop integration.

mod bam;
pub mod mujoco;
mod registry;
mod snapshot;

pub use registry::register_simulation;
pub use snapshot::{SharedSnapshot, SimulationSnapshot, SnapshotHandle};
