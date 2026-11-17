//! Machine-specific binding and pure servo↔canonical conversions.

use serde::{Deserialize, Serialize};

use crate::{BodyError, BodyTemplate};

/// One physical servo mapped to one canonical actuator slot.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct JointCalibration {
    pub slot_id: String,
    pub physical_id: u8,
    pub direction: i8,
    pub zero_offset_rad: f64,
    pub min_rad: Option<f64>,
    pub max_rad: Option<f64>,
}

/// Per-machine binding. The body definition remains machine-independent.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct BodyBinding {
    #[serde(default)]
    pub body_id: String,
    #[serde(default)]
    pub body_version: u32,
    pub joints: Vec<JointCalibration>,
}

impl BodyBinding {
    /// The driver order is the body's actuator order.
    pub fn physical_ids(&self) -> Vec<u8> {
        self.joints.iter().map(|joint| joint.physical_id).collect()
    }

    /// Validate slot ownership, direction, offsets, IDs and installation limits.
    pub fn validate(&self, body: &BodyTemplate) -> Result<(), BodyError> {
        if (!self.body_id.is_empty() && self.body_id != body.id)
            || (self.body_version != 0 && self.body_version != body.version)
        {
            return Err(BodyError::BindingSlotsMismatch);
        }
        let expected: Vec<_> = body.actuators.iter().map(|slot| slot.id.as_str()).collect();
        let actual: Vec<_> = self
            .joints
            .iter()
            .map(|joint| joint.slot_id.as_str())
            .collect();
        if actual != expected {
            return Err(BodyError::BindingSlotsMismatch);
        }

        let mut ids = std::collections::BTreeSet::new();
        for joint in &self.joints {
            if !ids.insert(joint.physical_id) {
                return Err(BodyError::BindingSlotsMismatch);
            }
            if !matches!(joint.direction, -1 | 1) {
                return Err(BodyError::InvalidDirection {
                    slot: joint.slot_id.clone(),
                    direction: joint.direction,
                });
            }
            if !joint.zero_offset_rad.is_finite()
                || joint.min_rad.is_some_and(|value| !value.is_finite())
                || joint.max_rad.is_some_and(|value| !value.is_finite())
                || matches!((joint.min_rad, joint.max_rad), (Some(min), Some(max)) if min > max)
            {
                return Err(BodyError::InvalidInstallationRange {
                    slot: joint.slot_id.clone(),
                });
            }
        }
        Ok(())
    }

    /// Convert servo positions into canonical body positions.
    pub fn to_body_positions(&self, positions: &[f64]) -> Vec<f64> {
        positions
            .iter()
            .zip(&self.joints)
            .map(|(position, calibration)| {
                *position * f64::from(calibration.direction) - calibration.zero_offset_rad
            })
            .collect()
    }

    /// Convert servo velocities into canonical body velocities. Offsets do not apply.
    pub fn to_body_velocities(&self, velocities: &[f64]) -> Vec<f64> {
        velocities
            .iter()
            .zip(&self.joints)
            .map(|(velocity, calibration)| *velocity * f64::from(calibration.direction))
            .collect()
    }

    /// Convert canonical body targets into calibrated servo positions.
    pub fn to_servo_positions(&self, positions: &[f64]) -> Vec<f64> {
        positions
            .iter()
            .zip(&self.joints)
            .map(|(position, calibration)| {
                let bounded = match (calibration.min_rad, calibration.max_rad) {
                    (Some(min), Some(max)) => position.clamp(min, max),
                    (Some(min), None) => position.max(min),
                    (None, Some(max)) => position.min(max),
                    (None, None) => *position,
                };
                (bounded + calibration.zero_offset_rad) * f64::from(calibration.direction)
            })
            .collect()
    }
}
