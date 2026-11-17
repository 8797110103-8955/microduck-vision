//! Read-only compatibility contract exported from the typed body profile.

use std::collections::BTreeMap;

use serde::{Deserialize, Serialize};

use crate::{BodyError, BodyProfile};

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct BodyContractSnapshot {
    pub generated_by: String,
    pub body_id: String,
    pub body_version: u32,
    pub source_hash: String,
    pub joint_names: Vec<String>,
    pub joint_axes: Vec<[f64; 3]>,
    pub joint_ranges_rad: Vec<[f64; 2]>,
    pub actuator_slots: Vec<String>,
    pub home_pose_rad: Vec<f64>,
    pub reference_poses_rad: BTreeMap<String, Vec<f64>>,
    pub obs_joint_count: usize,
    pub action_len: usize,
    pub canonical_action_len: usize,
    pub policy_action_slots: Vec<usize>,
}

impl BodyContractSnapshot {
    pub fn from_profile(profile: &BodyProfile) -> Self {
        let template = profile.template();
        let identity = profile.identity();
        Self {
            generated_by: format!("duck-body/{}", env!("CARGO_PKG_VERSION")),
            body_id: identity.id,
            body_version: identity.version,
            source_hash: identity.source_hash,
            joint_names: template
                .joints
                .iter()
                .map(|joint| joint.name.clone())
                .collect(),
            joint_axes: template.joints.iter().map(|joint| joint.axis).collect(),
            joint_ranges_rad: template
                .joints
                .iter()
                .map(|joint| joint.range_rad)
                .collect(),
            actuator_slots: template
                .actuators
                .iter()
                .map(|slot| slot.id.clone())
                .collect(),
            home_pose_rad: template.control.home_pose.clone(),
            reference_poses_rad: template.reference_poses.clone(),
            obs_joint_count: template.control.observation_joint_count,
            action_len: profile.action_space().action_len(),
            canonical_action_len: profile.action_space().canonical_len(),
            policy_action_slots: profile.action_space().policy_action_slots().to_vec(),
        }
    }

    pub fn validate_against(&self, profile: &BodyProfile) -> Result<(), BodyError> {
        let expected = Self::from_profile(profile);
        if self != &expected {
            return Err(BodyError::ContractMismatch);
        }
        Ok(())
    }

    pub fn to_json(&self) -> Result<String, BodyError> {
        serde_json::to_string_pretty(self)
            .map_err(|error| BodyError::ContractSerialization(error.to_string()))
    }
}
