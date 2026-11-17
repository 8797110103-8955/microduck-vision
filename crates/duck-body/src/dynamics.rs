use std::time::Duration;

use super::{BodyError, BodyTemplate, ServoParameterSet};

#[derive(Debug, Clone, PartialEq)]
pub struct BodyState {
    pub positions: Vec<f64>,
    pub velocities: Vec<f64>,
    pub elapsed: Duration,
}

#[derive(Debug, Clone, PartialEq)]
pub struct BodyTargets {
    pub positions: Vec<f64>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct DynamicsInput {
    pub state: BodyState,
    pub targets: BodyTargets,
}

#[derive(Debug, Clone, PartialEq)]
pub struct BodyFeedback {
    pub state: BodyState,
    pub targets: BodyTargets,
}

pub trait DynamicsBackend: Send {
    fn reset(&mut self, state: &BodyState);
    fn step(&mut self, input: &DynamicsInput, dt: Duration) -> Result<BodyFeedback, BodyError>;
}

#[derive(Debug, Default)]
pub struct DeterministicDynamics;

impl DynamicsBackend for DeterministicDynamics {
    fn reset(&mut self, _state: &BodyState) {}

    fn step(&mut self, input: &DynamicsInput, dt: Duration) -> Result<BodyFeedback, BodyError> {
        if dt.is_zero() {
            return Err(BodyError::InvalidTimeStep);
        }
        let dt_seconds = dt.as_secs_f64();
        let velocities = input
            .targets
            .positions
            .iter()
            .zip(&input.state.positions)
            .map(|(target, position)| (target - position) / dt_seconds)
            .collect();
        Ok(BodyFeedback {
            state: BodyState {
                positions: input.targets.positions.clone(),
                velocities,
                elapsed: input.state.elapsed + dt,
            },
            targets: input.targets.clone(),
        })
    }
}

pub struct VirtualBody {
    template: BodyTemplate,
    servos: ServoParameterSet,
    state: BodyState,
    targets: BodyTargets,
    dynamics: Box<dyn DynamicsBackend>,
}

impl VirtualBody {
    pub fn new(template: BodyTemplate, servos: ServoParameterSet) -> Result<Self, BodyError> {
        Self::with_backend(template, servos, Box::new(DeterministicDynamics))
    }

    pub fn with_backend(
        template: BodyTemplate,
        servos: ServoParameterSet,
        dynamics: Box<dyn DynamicsBackend>,
    ) -> Result<Self, BodyError> {
        template.validate()?;
        servos.validate_for(&template)?;
        let len = template.joints.len();
        let state = BodyState {
            positions: vec![0.0; len],
            velocities: vec![0.0; len],
            elapsed: Duration::ZERO,
        };
        let targets = BodyTargets {
            positions: vec![0.0; len],
        };
        Ok(Self {
            template,
            servos,
            state,
            targets,
            dynamics,
        })
    }

    pub fn template(&self) -> &BodyTemplate {
        &self.template
    }

    pub fn servo_parameters(&self) -> &ServoParameterSet {
        &self.servos
    }

    pub fn read_state(&self) -> BodyState {
        self.state.clone()
    }

    pub fn read_targets(&self) -> BodyTargets {
        self.targets.clone()
    }

    pub fn apply_sensor_state(&mut self, state: BodyState) -> Result<(), BodyError> {
        let expected = self.template.joints.len();
        if state.positions.len() != expected || state.velocities.len() != expected {
            return Err(BodyError::VectorLength {
                actual: state.positions.len(),
                expected,
            });
        }
        if state
            .positions
            .iter()
            .chain(state.velocities.iter())
            .any(|value| !value.is_finite())
        {
            return Err(BodyError::Dynamics(
                "sensor state contains a non-finite value".into(),
            ));
        }
        self.state = state;
        Ok(())
    }

    pub fn write_targets(&mut self, targets: BodyTargets) -> Result<(), BodyError> {
        if targets.positions.len() != self.template.joints.len() {
            return Err(BodyError::VectorLength {
                actual: targets.positions.len(),
                expected: self.template.joints.len(),
            });
        }
        for (joint, position) in self.template.joints.iter().zip(&targets.positions) {
            if !position.is_finite()
                || *position < joint.range_rad[0]
                || *position > joint.range_rad[1]
            {
                return Err(BodyError::TargetOutOfRange {
                    joint: joint.name.clone(),
                });
            }
        }
        self.targets = targets;
        Ok(())
    }

    /// Write finite targets without applying the body's posture range. The caller is responsible
    /// for deciding whether the range should be enforced at the driver boundary.
    pub fn write_targets_unchecked(&mut self, targets: BodyTargets) -> Result<(), BodyError> {
        if targets.positions.len() != self.template.joints.len() {
            return Err(BodyError::VectorLength {
                actual: targets.positions.len(),
                expected: self.template.joints.len(),
            });
        }
        if targets.positions.iter().any(|position| !position.is_finite()) {
            return Err(BodyError::Dynamics(
                "target state contains a non-finite value".into(),
            ));
        }
        self.targets = targets;
        Ok(())
    }

    pub fn reset_dynamics(&mut self) {
        self.dynamics.reset(&self.state);
    }

    pub fn step(&mut self, dt: Duration) -> Result<BodyFeedback, BodyError> {
        if dt.is_zero() || !dt.as_secs_f64().is_finite() {
            return Err(BodyError::InvalidTimeStep);
        }
        let input = DynamicsInput {
            state: self.state.clone(),
            targets: self.targets.clone(),
        };
        let feedback = self.dynamics.step(&input, dt)?;
        if feedback.state.positions.len() != self.template.joints.len()
            || feedback.state.velocities.len() != self.template.joints.len()
        {
            return Err(BodyError::VectorLength {
                actual: feedback.state.positions.len(),
                expected: self.template.joints.len(),
            });
        }
        self.state = feedback.state.clone();
        Ok(feedback)
    }
}
