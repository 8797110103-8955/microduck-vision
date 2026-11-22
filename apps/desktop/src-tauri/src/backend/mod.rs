//! Physical and simulation backend adapters.
//!
//! Backend modules implement the shared loop-body contract and own transport, device lifecycle,
//! actuator dynamics, and sensor conversion. They do not depend on command handlers or frontend
//! state shapes.

pub(crate) mod attached_body;
pub(crate) mod contract;
pub(crate) mod control;
pub(crate) mod encoder_calibration;
pub(crate) mod hd1910_body;
pub(crate) mod mujoco;
pub(crate) mod physical;
pub(crate) mod physical_backend;
pub(crate) mod policy_runtime;
pub(crate) mod read_only_body;
pub(crate) mod simulation;
