//! Persisted desktop data and IPC types.

pub(crate) mod machine_store;
mod motion;
pub mod types;
pub(crate) mod world_placements;

pub(crate) use motion::MotionTracker;
