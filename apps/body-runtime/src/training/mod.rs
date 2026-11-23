//! Training task resolution and the export contract for Python adapters.

mod export;
mod task;

pub use export::export_native_simulation;
pub use task::{ResolvedTrainingBody, TaskBodyDescription, resolve_training_body};
