//! Physical and simulation backend adapters.
//!
//! Backend modules implement the shared loop-body contract and own transport, device lifecycle,
//! actuator dynamics, and sensor conversion. They do not depend on command handlers or frontend
//! state shapes.

pub(crate) mod contract;
pub(crate) mod control;
pub(crate) mod mujoco;
pub(crate) mod physical;
pub(crate) mod physical_backend;
pub(crate) mod policy_runtime;
pub(crate) mod simulation;
