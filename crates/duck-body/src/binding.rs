//! Machine-specific binding and pure servo↔canonical conversions.

use serde::{Deserialize, Serialize};

use crate::{BodyError, BodyTemplate};

/// One physical servo mapped to one canonical actuator slot.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct JointCalibration {
    pub slot_id: String,
    pub physical_id: u8,
    pub direction: i8,
    /// Raw servo angle at canonical joint zero, independent of installation direction.
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

/// Set installation zero offsets from raw servo positions while preserving every other
/// calibration field. The slice may contain a partial mapping; callers decide which slots were
/// successfully resolved before calling this value-only operation.
pub fn apply_pose_zero_positions(
    bindings: &mut [JointCalibration],
    servo_positions: &[f64],
) -> Result<(), BodyError> {
    if bindings.len() != servo_positions.len() {
        return Err(BodyError::VectorLength {
            actual: servo_positions.len(),
            expected: bindings.len(),
        });
    }
    let offsets = bindings
        .iter()
        .zip(servo_positions)
        .map(|(binding, position)| {
            let offset = *position;
            if !offset.is_finite() {
                return Err(BodyError::NonFiniteOffset {
                    slot: binding.slot_id.clone(),
                });
            }
            Ok(offset)
        })
        .collect::<Result<Vec<_>, BodyError>>()?;
    for (binding, offset) in bindings.iter_mut().zip(offsets) {
        binding.zero_offset_rad = offset;
    }
    Ok(())
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
                (*position - calibration.zero_offset_rad) * f64::from(calibration.direction)
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
                // Register encoding belongs to the physical driver. Simulation targets are
                // angles too, but are not constrained by a physical position register domain.
                bounded * f64::from(calibration.direction) + calibration.zero_offset_rad
            })
            .collect()
    }

    /// Convert canonical targets without applying installation limits. Body safety limits are
    /// applied by the loop before this boundary; physical register validation belongs to the
    /// selected servo driver.
    pub fn to_servo_positions_unchecked(&self, positions: &[f64]) -> Vec<f64> {
        positions
            .iter()
            .zip(&self.joints)
            .map(|(position, calibration)| {
                position * f64::from(calibration.direction) + calibration.zero_offset_rad
            })
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use super::{JointCalibration, apply_pose_zero_positions};
    use crate::BodyError;

    fn binding(slot_id: &str, direction: i8) -> JointCalibration {
        JointCalibration {
            slot_id: slot_id.into(),
            physical_id: 7,
            direction,
            zero_offset_rad: 0.8,
            min_rad: Some(-1.0),
            max_rad: Some(1.0),
        }
    }

    #[test]
    fn pose_zero_positions_store_raw_angles_and_preserve_installation_limits() {
        let mut bindings = vec![binding("left_hip_yaw", 1), binding("right_hip_yaw", -1)];
        apply_pose_zero_positions(&mut bindings, &[0.25, 0.4]).unwrap();
        assert_eq!(bindings[0].zero_offset_rad, 0.25);
        assert_eq!(bindings[1].zero_offset_rad, 0.4);
        assert_eq!(bindings[0].physical_id, 7);
        assert_eq!(bindings[0].min_rad, Some(-1.0));
        assert_eq!(bindings[0].max_rad, Some(1.0));
    }

    #[test]
    fn pose_zero_positions_support_partial_mappings() {
        let mut bindings = vec![binding("left_hip_yaw", -1)];
        apply_pose_zero_positions(&mut bindings, &[0.3]).unwrap();
        assert_eq!(bindings.len(), 1);
        assert_eq!(bindings[0].zero_offset_rad, 0.3);
    }

    #[test]
    fn reversed_joint_keeps_its_captured_zero_when_direction_changes() {
        let mut calibration = binding("left_hip_yaw", -1);
        calibration.zero_offset_rad = 2.7;
        let mut binding = crate::BodyBinding {
            body_id: String::new(),
            body_version: 0,
            joints: vec![calibration],
        };
        assert_eq!(binding.to_body_positions(&[2.7]), vec![0.0]);
        assert!((binding.to_body_positions(&[2.6])[0] - 0.1).abs() < 1e-12);
        assert!((binding.to_servo_positions(&[0.1])[0] - 2.6).abs() < 1e-12);
        assert_eq!(binding.to_body_velocities(&[0.5]), vec![-0.5]);

        binding.joints[0].direction = 1;
        assert_eq!(binding.to_body_positions(&[2.7]), vec![0.0]);
        assert_eq!(binding.to_servo_positions(&[0.0]), vec![2.7]);
        assert!((binding.to_body_positions(&[2.6])[0] + 0.1).abs() < 1e-12);
    }

    #[test]
    fn pose_zero_positions_reject_length_mismatch_without_mutating_bindings() {
        let mut bindings = vec![binding("left_hip_yaw", 1)];
        let before = bindings.clone();
        let error = apply_pose_zero_positions(&mut bindings, &[]).unwrap_err();
        assert_eq!(
            error,
            BodyError::VectorLength {
                actual: 0,
                expected: 1
            }
        );
        assert_eq!(bindings, before);
    }

    #[test]
    fn pose_zero_positions_reject_non_finite_sensor_values_without_partial_update() {
        let mut bindings = vec![binding("left_hip_yaw", 1), binding("right_hip_yaw", 1)];
        let before = bindings.clone();
        let error = apply_pose_zero_positions(&mut bindings, &[0.2, f64::NAN]).unwrap_err();
        assert_eq!(
            error,
            BodyError::NonFiniteOffset {
                slot: "right_hip_yaw".into()
            }
        );
        assert_eq!(bindings, before);
    }

    #[test]
    fn binding_preserves_calibrated_angles_for_backend_register_validation() {
        let mut calibration = binding("left_hip_yaw", 1);
        calibration.zero_offset_rad = std::f64::consts::PI - 0.01;
        let binding = crate::BodyBinding {
            body_id: String::new(),
            body_version: 0,
            joints: vec![calibration],
        };

        let targets = binding.to_servo_positions(&[0.5]);

        assert_eq!(targets, vec![std::f64::consts::PI + 0.49]);
    }
}
