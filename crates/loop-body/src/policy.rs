//! ONNX policy execution layered on top of the base body loop.

use std::path::Path;
use std::time::Duration;

use duck_control::imu::ImuData;
use duck_control::model::{DEFAULT_POSITION, MOUTH_INDEX, NUM_JOINTS};
use duck_control::obs::{ACTION_LEN, Command, Observation};
use duck_control::policy::{DEFAULT_STANDING_THRESHOLD, Net, Policy, PolicyPaths};

use crate::{
    BodyCommand, BodyLoop, LoopAction, LoopBinding, LoopConfig, LoopError, LoopPhase, LoopStatus,
};

// These are the official walking controller's default post-processing parameters. The ONNX
// network produces offsets; sending those raw offsets directly to the servos makes each
// inference tick visible as a target jump.
const WALK_ACTION_SCALE: f64 = 0.9;
const HEAD_LOWPASS: f64 = 0.5;
const LEGS_LOWPASS: f64 = 0.7;

fn walking_targets(
    action: &[f32; ACTION_LEN],
    previous: Option<&[f64; NUM_JOINTS]>,
    ranges: &[[f64; 2]],
) -> [f64; NUM_JOINTS] {
    let offsets = Observation::scatter_action(action);
    let mut targets = std::array::from_fn(|joint| {
        (DEFAULT_POSITION[joint] + WALK_ACTION_SCALE * offsets[joint])
            .clamp(ranges[joint][0], ranges[joint][1])
    });
    if let Some(previous) = previous {
        for (joint, target) in targets.iter_mut().enumerate() {
            let alpha = if (5..9).contains(&joint) {
                HEAD_LOWPASS
            } else if joint == MOUTH_INDEX {
                continue;
            } else {
                LEGS_LOWPASS
            };
            *target = alpha * *target + (1.0 - alpha) * previous[joint];
        }
    }
    targets
}

#[derive(Debug, Clone, PartialEq)]
pub struct PolicyTrace {
    pub phase: LoopPhase,
    pub command: Command,
    pub observation: Vec<f32>,
    pub action: Vec<f32>,
    pub body_targets: Option<Vec<f64>>,
}

pub struct PolicyLoop {
    body: BodyLoop,
    policy: Option<Policy>,
    last_action: [f32; ACTION_LEN],
    command: Command,
    ranges: Vec<[f64; 2]>,
    last_trace: Option<PolicyTrace>,
    previous_targets: Option<[f64; NUM_JOINTS]>,
}

impl PolicyLoop {
    pub fn open(
        binding: LoopBinding,
        driver: Box<dyn servo::ServoLoopDriver>,
        config: LoopConfig,
    ) -> Result<Self, LoopError> {
        Self::open_with_command(binding, driver, config, Command::default())
    }

    pub fn open_with_command(
        binding: LoopBinding,
        driver: Box<dyn servo::ServoLoopDriver>,
        config: LoopConfig,
        command: Command,
    ) -> Result<Self, LoopError> {
        let ranges = binding
            .body
            .template()
            .joints
            .iter()
            .map(|joint| joint.range_rad)
            .collect();
        let body = BodyLoop::open(binding, driver, config)?;
        Ok(Self {
            body,
            policy: None,
            last_action: [0.0; ACTION_LEN],
            command,
            ranges,
            last_trace: None,
            previous_targets: None,
        })
    }

    pub fn select_policy(&mut self, path: impl AsRef<Path>) -> Result<(), LoopError> {
        // Keep the target currently being held as the filter's starting point. Resetting this
        // to `None` makes the first frame of the new policy jump straight from the old pose to
        // the new raw action, which is the visible twitch during hot switching.
        let held_targets = self.body.status().targets.try_into().ok();
        let policy = Policy::load(
            &PolicyPaths {
                walk: path.as_ref().to_owned(),
                ..PolicyPaths::default()
            },
            DEFAULT_STANDING_THRESHOLD,
        )
        .map_err(|error| LoopError::Policy(error.to_string()))?;
        self.policy = Some(policy);
        self.last_action = [0.0; ACTION_LEN];
        self.previous_targets = held_targets;
        self.last_trace = None;
        Ok(())
    }

    pub fn set_policy_command(&mut self, command: Command) {
        self.command = command;
    }

    pub fn send(&mut self, action: LoopAction) -> Result<(), LoopError> {
        self.body.send(action)
    }

    pub fn set_command(&mut self, command: BodyCommand) -> Result<(), LoopError> {
        self.body.set_command(command)
    }

    pub fn status(&self) -> LoopStatus {
        self.body.status()
    }

    pub fn policy_trace(&self) -> Option<&PolicyTrace> {
        self.last_trace.as_ref()
    }

    pub fn tick(&mut self, dt: Duration) -> Result<(), LoopError> {
        let current = self.body.status();
        if let Some(policy) = &mut self.policy {
            let positions: [f64; NUM_JOINTS] =
                current
                    .positions
                    .clone()
                    .try_into()
                    .map_err(|_| LoopError::SensorLength {
                        actual: current.positions.len(),
                        expected: NUM_JOINTS,
                    })?;
            let velocities: [f64; NUM_JOINTS] = self
                .body
                .sensor_velocities()
                .to_vec()
                .try_into()
                .map_err(|_| LoopError::SensorLength {
                actual: self.body.sensor_velocities().len(),
                expected: NUM_JOINTS,
            })?;
            let observation = Observation::build(
                &ImuData::default(),
                &positions,
                &velocities,
                &DEFAULT_POSITION,
                &self.last_action,
                &self.command,
            );
            let action = match policy.infer(&observation, Net::Walk) {
                Ok(action) => action,
                Err(error) => {
                    let detail = error.to_string();
                    self.body.report_policy_error(detail.clone());
                    return Err(LoopError::Policy(detail));
                }
            };
            self.last_action = action;
            let body_targets = if matches!(
                current.phase,
                LoopPhase::Ready | LoopPhase::Driving | LoopPhase::Holding
            ) {
                if current.phase == LoopPhase::Ready {
                    self.body.send(LoopAction::Drive(true))?;
                }
                // The ONNX output is 14 policy slots. `scatter_action` restores the
                // canonical 15-slot body order (including the untouched mouth slot);
                // BodyLoop then applies calibration to translate that order to the
                // simulation IDs or the bound physical IDs.
                let target_array =
                    walking_targets(&action, self.previous_targets.as_ref(), &self.ranges);
                self.previous_targets = Some(target_array);
                let targets = target_array.to_vec();
                let trace_targets = targets.clone();
                let session = self.body.status().session;
                self.body.set_command(BodyCommand { session, targets })?;
                Some(trace_targets)
            } else {
                None
            };
            self.last_trace = Some(PolicyTrace {
                phase: current.phase,
                command: self.command,
                observation: observation.as_slice().to_vec(),
                action: action.to_vec(),
                body_targets,
            });
        }
        self.body.tick(dt)
    }
}

#[cfg(test)]
mod tests {
    use super::{HEAD_LOWPASS, LEGS_LOWPASS, WALK_ACTION_SCALE, walking_targets};
    use duck_control::model::{DEFAULT_POSITION, MOUTH_INDEX, NUM_JOINTS};
    use duck_control::obs::ACTION_LEN;

    #[test]
    fn walking_targets_use_official_scale_and_low_pass() {
        let action = [1.0; ACTION_LEN];
        let ranges = vec![[-10.0, 10.0]; NUM_JOINTS];
        let first = walking_targets(&action, None, &ranges);
        assert_eq!(
            first[0],
            DEFAULT_POSITION[0] + WALK_ACTION_SCALE,
            "the first target is scaled but not filtered"
        );
        assert_eq!(first[MOUTH_INDEX], DEFAULT_POSITION[MOUTH_INDEX]);

        let previous = DEFAULT_POSITION;
        let second = walking_targets(&action, Some(&previous), &ranges);
        let expected_leg = DEFAULT_POSITION[0] + LEGS_LOWPASS * WALK_ACTION_SCALE;
        let expected_head = DEFAULT_POSITION[5] + HEAD_LOWPASS * WALK_ACTION_SCALE;
        assert_eq!(second[0], expected_leg);
        assert_eq!(second[5], expected_head);
    }
}
