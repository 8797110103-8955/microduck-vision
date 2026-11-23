//! Canonical body action projection shared by deployed policies and previews.

use duck_body::{BodyError, BodyTemplate};
use duck_control::model::{JOINT_NAMES, MOUTH_INDEX};

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct BodyActionSpace {
    canonical_slots: Vec<String>,
    policy_action_slots: Vec<usize>,
    observation_slots: Vec<usize>,
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::runtime::adapters::body_assets;

    #[test]
    fn official_profile_exposes_one_canonical_action_projection() {
        let profile = body_assets::test_profile();
        assert_eq!(profile.identity().id, duck_body::OFFICIAL_BODY_ID);
        assert!(!profile.identity().source_hash.is_empty());
        let space = BodyActionSpace::official(profile.template()).unwrap();
        assert_eq!(space.canonical_len(), 15);
        assert_eq!(space.action_len(), 14);
        assert!(!space.policy_action_slots().contains(&9));

        let action: Vec<f32> = (0..14).map(|value| value as f32).collect();
        let codec = BodyActionCodec::new(space);
        let canonical = codec.decode(&action).unwrap();
        assert_eq!(canonical[0], 0.0);
        assert_eq!(canonical[9], 0.0);
        assert_eq!(canonical[10], 9.0);
        assert_eq!(codec.encode(&canonical).unwrap(), action);
    }

    #[test]
    fn one_body_can_compile_distinct_policy_layouts() {
        let profile = body_assets::test_profile();
        let official = BodyActionSpace::official(profile.template()).unwrap();
        let alternate = BodyActionSpace::from_joint_names(
            profile.template(),
            ["mouth", "left_hip_yaw"],
            ["right_ankle", "left_hip_yaw"],
        )
        .unwrap();
        assert_eq!(official.action_len(), 14);
        assert_eq!(alternate.canonical_len(), 15);
        assert_eq!(alternate.policy_action_slots(), [9, 0]);
        assert_eq!(alternate.observation_slots(), [14, 0]);
        let canonical = BodyActionCodec::new(alternate)
            .decode(&[0.25, -0.5])
            .unwrap();
        let mut expected = vec![0.0; 15];
        expected[9] = 0.25;
        expected[0] = -0.5;
        assert_eq!(canonical, expected);
    }
}

impl BodyActionSpace {
    pub fn official(body: &BodyTemplate) -> Result<Self, BodyError> {
        let names = JOINT_NAMES
            .iter()
            .enumerate()
            .filter_map(|(index, &name)| (index != MOUTH_INDEX).then_some(name));
        Self::from_joint_names(body, names.clone(), names)
    }

    pub fn from_joint_names<'a>(
        body: &BodyTemplate,
        actions: impl IntoIterator<Item = &'a str>,
        observations: impl IntoIterator<Item = &'a str>,
    ) -> Result<Self, BodyError> {
        let canonical_slots: Vec<_> = body.joints.iter().map(|joint| joint.name.clone()).collect();
        if canonical_slots.is_empty() {
            return Err(BodyError::MissingField { field: "joints" });
        }
        let compile = |names: Vec<&str>| {
            let mut slots = Vec::with_capacity(names.len());
            for name in names {
                let index = body
                    .joint_index(name)
                    .ok_or_else(|| BodyError::MissingSceneJoint(name.to_owned()))?;
                if slots.contains(&index) {
                    return Err(BodyError::Duplicate {
                        kind: "policy joint",
                        id: name.to_owned(),
                    });
                }
                slots.push(index);
            }
            Ok(slots)
        };
        let policy_action_slots = compile(actions.into_iter().collect())?;
        let observation_slots = compile(observations.into_iter().collect())?;
        Ok(Self {
            canonical_slots,
            policy_action_slots,
            observation_slots,
        })
    }

    pub fn canonical_slots(&self) -> &[String] {
        &self.canonical_slots
    }

    #[cfg(test)]
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

    #[cfg(test)]
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
