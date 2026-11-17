//! The single canonical target validation boundary before adapter writes.

use duck_body::BodyTemplate;

use crate::LoopError;

pub(crate) fn validate_targets(body: &BodyTemplate, targets: &[f64]) -> Result<(), LoopError> {
    if targets.len() != body.joints.len() {
        return Err(LoopError::TargetLength {
            actual: targets.len(),
            expected: body.joints.len(),
        });
    }
    for (joint, target) in body.joints.iter().zip(targets) {
        if !target.is_finite() {
            return Err(LoopError::NonFiniteTarget {
                joint: joint.name.clone(),
            });
        }
    }
    Ok(())
}
