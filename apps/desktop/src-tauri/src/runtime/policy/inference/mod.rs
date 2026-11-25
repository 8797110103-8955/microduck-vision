//! MicroDuck policy execution owned by the desktop product.
//!
//! `loop-body` supplies the generic body lifecycle and backend boundary. This module owns the
//! MicroDuck observation/action contract, ONNX session selection, and policy trace semantics.

mod action_space;
mod contract;
mod core;
mod local_onnx;
mod policy;
mod policy_input;
mod policy_output;

pub(crate) fn validate_candidate(
    bytes: &[u8],
    spec: &local_onnx::LocalPolicySpec,
) -> Result<(), String> {
    let mut policy = local_onnx::LocalPolicy::load(bytes, spec).map_err(|e| e.to_string())?;
    policy
        .infer(&duck_control::obs::Observation::zeroed())
        .map_err(|e| e.to_string())?;
    Ok(())
}

pub(crate) use contract::PolicyDescriptor;
pub(crate) use core::{ControlTickInput, PolicyConfig, PolicyControlCore};
pub(crate) use local_onnx::{LocalPolicySpec, TensorSpec};
pub(crate) use policy::{
    MicroDuckPolicyRunner, OpenLoopInput, PolicyTrace, PreparedPolicy, open_loop,
};
pub(crate) use policy_input::PolicyInputEncoder;
pub(crate) use policy_output::RawPolicyOutputDecoder;

#[cfg(test)]
pub(crate) use policy::{RuntimeTiming, runtime_timing};
