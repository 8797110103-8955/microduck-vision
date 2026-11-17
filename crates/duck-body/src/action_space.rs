//! Canonical body action projection shared by deployed policies and previews.

use serde::{Deserialize, Serialize};

use crate::{BodyError, BodyTemplate};

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct BodyActionSpace {
    canonical_slots: Vec<String>,
    policy_action_slots: Vec<usize>,
    observation_slots: Vec<usize>,
}

impl BodyActionSpace {
    pub fn from_template(body: &BodyTemplate) -> Result<Self, BodyError> {
        let canonical_slots: Vec<_> = body.actuators.iter().map(|slot| slot.id.clone()).collect();
        if canonical_slots.is_empty() {
            return Err(BodyError::MissingField { field: "actuators" });
        }
        let mouth = body.joints.iter().position(|joint| joint.name == "mouth");
        let policy_action_slots: Vec<_> = if body.control.action_size == canonical_slots.len() {
            (0..canonical_slots.len()).collect()
        } else if body.control.action_size + 1 == canonical_slots.len() && mouth.is_some() {
            (0..canonical_slots.len())
                .filter(|index| Some(*index) != mouth)
                .collect()
        } else {
            return Err(BodyError::ActionSizeMismatch {
                actual: body.control.action_size,
                expected: canonical_slots.len(),
            });
        };
        let observation_slots = policy_action_slots.clone();
        let space = Self {
            canonical_slots,
            policy_action_slots,
            observation_slots,
        };
        if space.policy_action_slots.len() != body.control.action_size {
            return Err(BodyError::ActionSizeMismatch {
                actual: space.policy_action_slots.len(),
                expected: body.control.action_size,
            });
        }
        Ok(space)
    }

    pub fn canonical_slots(&self) -> &[String] {
        &self.canonical_slots
    }

    pub fn policy_action_slots(&self) -> &[usize] {
        &self.policy_action_slots
    }

    pub fn observation_slots(&self) -> &[usize] {
        &self.observation_slots
    }

    pub fn canonical_len(&self) -> usize {
        self.canonical_slots.len()
    }

    pub fn action_len(&self) -> usize {
        self.policy_action_slots.len()
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct BodyActionCodec {
    space: BodyActionSpace,
}

impl BodyActionCodec {
    pub fn new(space: BodyActionSpace) -> Self {
        Self { space }
    }

    pub fn space(&self) -> &BodyActionSpace {
        &self.space
    }

    /// Decode the fixed policy action layout into canonical actuator order.
    /// Non-policy slots are deliberately zero and must be filled by their owner later.
    pub fn decode(&self, action: &[f32]) -> Result<Vec<f64>, BodyError> {
        if action.len() != self.space.action_len() {
            return Err(BodyError::VectorLength {
                actual: action.len(),
                expected: self.space.action_len(),
            });
        }
        if action.iter().any(|value| !value.is_finite()) {
            return Err(BodyError::Dynamics(
                "policy action contains a non-finite value".into(),
            ));
        }
        let mut canonical = vec![0.0; self.space.canonical_len()];
        for (&slot, value) in self.space.policy_action_slots.iter().zip(action) {
            canonical[slot] = f64::from(*value);
        }
        Ok(canonical)
    }

    pub fn encode(&self, canonical: &[f64]) -> Result<Vec<f32>, BodyError> {
        if canonical.len() != self.space.canonical_len() {
            return Err(BodyError::VectorLength {
                actual: canonical.len(),
                expected: self.space.canonical_len(),
            });
        }
        Ok(self
            .space
            .policy_action_slots
            .iter()
            .map(|&slot| canonical[slot] as f32)
            .collect())
    }
}
