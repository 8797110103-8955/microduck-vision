//! MicroDuck policy execution owned by the desktop product.
//!
//! `loop-body` supplies the generic body lifecycle and backend boundary. This module owns the
//! MicroDuck observation/action contract, ONNX session selection, and policy trace semantics.

mod core;
mod execution;
mod policy;
mod policy_input;
mod policy_output;
pub(crate) mod world;

pub(crate) use core::{ControlTickInput, PolicyConfig, PolicyControlCore};
pub(crate) use execution::PolicyExecution;
pub(crate) use policy::{MicroDuckPolicyRunner, PolicyDescriptor, PolicyTrace};
pub(crate) use policy_input::PolicyInputEncoder;
pub(crate) use policy_output::RawPolicyOutputDecoder;
