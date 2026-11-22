//! Application services that compose user intent with control, backend, and persisted state.
//!
//! Services may depend on `control`, `backend`, `state`, and pure helpers. They do not own Tauri
//! command registration or low-level serial I/O.

pub(crate) mod actions;
pub(crate) mod binding;
pub(crate) mod debug_machine;
pub(crate) mod devices;
pub(crate) mod encoder_calibration;
pub(crate) mod loop_service;
pub(crate) mod physical_io;
pub(crate) mod physical_machine;
pub(crate) mod policy;
pub(crate) mod preview;
pub(crate) mod registers;
pub(crate) mod training;
pub(crate) mod virtual_machine;
