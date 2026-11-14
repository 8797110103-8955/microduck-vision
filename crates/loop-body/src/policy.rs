//! ONNX policy execution layered on top of the base body loop.

use std::path::Path;
use std::time::Duration;

use duck_control::imu::ImuData;
use duck_control::model::{DEFAULT_POSITION, NUM_JOINTS};
use duck_control::obs::{ACTION_LEN, Command, Observation};
use duck_control::policy::{DEFAULT_STANDING_THRESHOLD, Net, Policy, PolicyPaths};

use crate::{
    BodyCommand, BodyLoop, LoopAction, LoopBinding, LoopConfig, LoopError, LoopPhase, LoopStatus,
};

pub struct PolicyLoop {
    body: BodyLoop,
    policy: Option<Policy>,
    last_action: [f32; ACTION_LEN],
    command: Command,
    ranges: Vec<[f64; 2]>,
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
        })
    }

    pub fn select_policy(&mut self, path: impl AsRef<Path>) -> Result<(), LoopError> {
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
            if matches!(current.phase, LoopPhase::Ready | LoopPhase::Holding) {
                if current.phase == LoopPhase::Ready {
                    self.body.send(LoopAction::Drive(true))?;
                }
                let targets = DEFAULT_POSITION
                    .iter()
                    .zip(Observation::scatter_action(&action))
                    .zip(&self.ranges)
                    .map(|((home, offset), range)| (home + offset).clamp(range[0], range[1]))
                    .collect();
                let session = self.body.status().session;
                self.body.set_command(BodyCommand { session, targets })?;
            }
        }
        self.body.tick(dt)
    }
}
