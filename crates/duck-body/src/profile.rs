//! The typed public facade for a body and its capabilities.

use serde::{Deserialize, Serialize};

use crate::{
    BodyActionCodec, BodyActionSpace, BodyBinding, BodyContractSnapshot, BodyError, BodyTemplate,
};

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct BodyIdentity {
    pub id: String,
    pub version: u32,
    pub source_hash: String,
}

#[derive(Debug, Clone, PartialEq)]
pub struct BodyProfile {
    template: BodyTemplate,
    action_space: BodyActionSpace,
}

impl BodyProfile {
    pub fn official() -> Result<Self, BodyError> {
        Self::from_template(BodyTemplate::official()?)
    }

    pub fn from_template(template: BodyTemplate) -> Result<Self, BodyError> {
        template.validate()?;
        let action_space = BodyActionSpace::from_template(&template)?;
        Ok(Self {
            template,
            action_space,
        })
    }

    pub fn identity(&self) -> BodyIdentity {
        BodyIdentity {
            id: self.template.id.clone(),
            version: self.template.version,
            source_hash: self.source_hash(),
        }
    }

    /// Stable, local provenance hash for compatibility snapshots and diagnostics.
    pub fn source_hash(&self) -> String {
        use std::hash::{Hash, Hasher};

        let mut hasher = std::collections::hash_map::DefaultHasher::new();
        self.template.id.hash(&mut hasher);
        self.template.version.hash(&mut hasher);
        for joint in &self.template.joints {
            joint.name.hash(&mut hasher);
            joint.order.hash(&mut hasher);
            for value in joint.range_rad {
                value.to_bits().hash(&mut hasher);
            }
            for value in joint.axis {
                value.to_bits().hash(&mut hasher);
            }
            joint.scene_joint.hash(&mut hasher);
            joint.actuator.hash(&mut hasher);
        }
        for slot in &self.template.actuators {
            slot.id.hash(&mut hasher);
            slot.joint.hash(&mut hasher);
        }
        for value in &self.template.control.home_pose {
            value.to_bits().hash(&mut hasher);
        }
        for (name, pose) in &self.template.reference_poses {
            name.hash(&mut hasher);
            for value in pose {
                value.to_bits().hash(&mut hasher);
            }
        }
        format!("{:016x}", hasher.finish())
    }

    pub fn template(&self) -> &BodyTemplate {
        &self.template
    }

    pub fn action_space(&self) -> &BodyActionSpace {
        &self.action_space
    }

    pub fn action_codec(&self) -> BodyActionCodec {
        BodyActionCodec::new(self.action_space.clone())
    }

    pub fn bind(&self, binding: BodyBinding) -> Result<BoundBody<'_>, BodyError> {
        binding.validate(&self.template)?;
        Ok(BoundBody {
            profile: self,
            binding,
        })
    }

    pub fn compatibility_snapshot(&self) -> BodyContractSnapshot {
        BodyContractSnapshot::from_profile(self)
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct BoundBody<'a> {
    profile: &'a BodyProfile,
    binding: BodyBinding,
}

impl<'a> BoundBody<'a> {
    pub fn profile(&self) -> &'a BodyProfile {
        self.profile
    }

    pub fn binding(&self) -> &BodyBinding {
        &self.binding
    }
}
