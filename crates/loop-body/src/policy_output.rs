//! Fixed deployed policy output decoding and body action projection boundary.

use duck_control::obs::ACTION_LEN;

use crate::LoopError;

/// Decoder for the one deployed policy output layout (14 finite f32 values).
/// Model-specific tensor plumbing stays here; canonical joint projection belongs to duck-body.
#[derive(Debug, Clone, Copy, Default)]
pub struct RawPolicyOutputDecoder;

impl RawPolicyOutputDecoder {
    pub fn decode(&self, raw: &[f32]) -> Result<[f32; ACTION_LEN], LoopError> {
        let action: [f32; ACTION_LEN] = raw.try_into().map_err(|_| {
            LoopError::Policy(format!(
                "policy action length is {}, expected {ACTION_LEN}",
                raw.len()
            ))
        })?;
        if action.iter().any(|value| !value.is_finite()) {
            return Err(LoopError::Policy(
                "policy action contains a non-finite value".into(),
            ));
        }
        Ok(action)
    }
}
