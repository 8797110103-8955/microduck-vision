//! Training task resolution and the export contract for Python adapters.

mod export;
mod mjlab;
mod task;

pub use export::export_native_simulation;
pub use export::export_recorded_native_simulation;
pub use mjlab::{MjlabBodyConfig, resolve_mjlab_body};
pub use task::{ResolvedTrainingBody, TaskBodyDescription, resolve_training_body};
