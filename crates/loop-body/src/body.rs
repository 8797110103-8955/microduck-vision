//! Base body control loop shared by physical and virtual servo drivers.

use std::time::Duration;

use duck_body::{BodyState, BodyTargets, ServoParameterSet, VirtualBody};
use servo::ServoLoopDriver;

use crate::{
    BodyCommand, JointRangeState, LoopAction, LoopBinding, LoopConfig, LoopError, LoopPhase,
    LoopStatus,
    adapter::ServoAdapter, lifecycle, safety,
};

pub struct BodyLoop {
    body: VirtualBody,
    adapter: ServoAdapter,
    config: LoopConfig,
    phase: LoopPhase,
    session: u64,
    tick: u64,
    homing_ticks: u32,
    sensor_failures: u32,
    command: Option<BodyCommand>,
    last_targets: Vec<f64>,
    last_error: Option<String>,
    last_velocities: Vec<f64>,
    joint_range_states: Vec<JointRangeState>,
}

impl BodyLoop {
    pub fn open(
        binding: LoopBinding,
        driver: Box<dyn ServoLoopDriver>,
        config: LoopConfig,
    ) -> Result<Self, LoopError> {
        if config.gain == 0 {
            return Err(LoopError::ZeroGain);
        }
        if config.period.is_zero() || config.sensor_failure_limit == 0 {
            return Err(LoopError::InvalidTick);
        }
        binding.servos.validate(binding.body.template())?;
        binding.calibration.validate(binding.body.template())?;
        let template = binding.body.template().clone();
        let count = template.joints.len();
        let body = VirtualBody::new(
            template.clone(),
            ServoParameterSet::bind_all(&template, binding.servos.model),
        )?;
        let adapter = ServoAdapter::new(driver, binding.calibration);
        Ok(Self {
            body,
            adapter,
            config,
            phase: LoopPhase::Limp,
            session: 0,
            tick: 0,
            homing_ticks: 0,
            sensor_failures: 0,
            command: None,
            last_targets: vec![0.0; count],
            last_error: None,
            last_velocities: vec![0.0; count],
            joint_range_states: template
                .joints
                .iter()
                .map(|joint| JointRangeState {
                    slot_id: joint.name.clone(),
                    target_rad: 0.0,
                    min_rad: joint.range_rad[0],
                    max_rad: joint.range_rad[1],
                    below_min: false,
                    above_max: false,
                })
                .collect(),
        })
    }

    pub fn set_command(&mut self, command: BodyCommand) -> Result<(), LoopError> {
        if command.session != self.session {
            return Err(LoopError::StaleSession {
                command: command.session,
                current: self.session,
            });
        }
        self.validate_targets(&command.targets)?;
        self.command = Some(command);
        Ok(())
    }

    pub fn send(&mut self, action: LoopAction) -> Result<(), LoopError> {
        match action {
            LoopAction::Power(true) => {
                if matches!(self.phase, LoopPhase::Estop) {
                    return Err(LoopError::InvalidAction(self.phase));
                }
                self.adapter.set_torque(true)?;
                self.adapter.set_gain(self.config.gain)?;
                self.phase = if self.config.homing_ticks == 0 {
                    LoopPhase::Ready
                } else {
                    LoopPhase::Homing
                };
                self.homing_ticks = 0;
                self.sensor_failures = 0;
            }
            LoopAction::Power(false) => {
                self.adapter.set_torque(false)?;
                self.phase = LoopPhase::Limp;
                self.command = None;
            }
            LoopAction::Drive(true)
                if matches!(self.phase, LoopPhase::Ready | LoopPhase::Holding) =>
            {
                self.phase = LoopPhase::Driving;
            }
            LoopAction::Drive(false) if matches!(self.phase, LoopPhase::Driving) => {
                self.phase = LoopPhase::Holding;
            }
            LoopAction::Stop => {
                self.phase = LoopPhase::Stopped;
                self.adapter.set_torque(false)?;
            }
            LoopAction::Estop => {
                self.phase = LoopPhase::Estop;
                self.adapter.set_torque(false)?;
                self.command = None;
            }
            LoopAction::Reconnect { session } => {
                if session <= self.session {
                    return Err(LoopError::StaleSession {
                        command: session,
                        current: self.session,
                    });
                }
                self.session = session;
                self.phase = LoopPhase::Reconnecting;
                self.sensor_failures = 0;
                self.command = None;
            }
            _ => return Err(LoopError::InvalidAction(self.phase)),
        }
        Ok(())
    }

    pub fn report_policy_error(&mut self, detail: impl Into<String>) {
        self.phase = LoopPhase::Holding;
        self.last_error = Some(detail.into());
    }

    pub fn tick(&mut self, dt: Duration) -> Result<(), LoopError> {
        self.update_sensors(dt)?;
        self.apply_targets()
    }

    pub(crate) fn update_sensors(&mut self, dt: Duration) -> Result<(), LoopError> {
        if dt != self.config.period {
            return Err(LoopError::InvalidTick);
        }
        self.tick += 1;
        if !lifecycle::reads_sensors(self.phase) {
            return Ok(());
        }
        if self.phase == LoopPhase::Reconnecting {
            self.phase = LoopPhase::Homing;
            self.adapter.set_torque(true)?;
            self.adapter.set_gain(self.config.gain)?;
            self.homing_ticks = 0;
        }
        let sample = match self.adapter.read() {
            Ok(sample) => sample,
            Err(error) => {
                self.sensor_failures += 1;
                self.last_error = Some(error.to_string());
                if self.sensor_failures >= self.config.sensor_failure_limit {
                    self.phase = LoopPhase::Holding;
                }
                return Err(error);
            }
        };
        self.sensor_failures = 0;
        self.last_error = None;
        self.last_velocities = sample.velocities.clone();
        self.body.apply_sensor_state(BodyState {
            positions: sample.positions,
            velocities: sample.velocities,
            elapsed: self.body.read_state().elapsed + dt,
        })?;

        Ok(())
    }

    pub(crate) fn apply_targets(&mut self) -> Result<(), LoopError> {
        let positions = self.body.read_state().positions;

        let targets = match self.phase {
            // Homing must establish a safe hold, not drive every joint to the numeric zero
            // frame.  The policy's neutral pose is non-zero for several joints, and a physical
            // robot may already be in a valid pose when torque is enabled.  Holding the measured
            // canonical positions avoids an artificial jump before policy inference starts.
            LoopPhase::Homing => positions.clone(),
            LoopPhase::Driving => self
                .command
                .as_ref()
                .map(|command| command.targets.clone())
                .unwrap_or_else(|| self.last_targets.clone()),
            LoopPhase::Holding => self.last_targets.clone(),
            _ => return Ok(()),
        };
        self.validate_targets(&targets)?;
        self.joint_range_states = self
            .body
            .template()
            .joints
            .iter()
            .zip(&targets)
            .map(|(joint, target)| JointRangeState {
                slot_id: joint.name.clone(),
                target_rad: *target,
                min_rad: joint.range_rad[0],
                max_rad: joint.range_rad[1],
                below_min: *target < joint.range_rad[0],
                above_max: *target > joint.range_rad[1],
            })
            .collect();
        let applied_targets = if self.config.enforce_body_range {
            targets
                .iter()
                .zip(&self.joint_range_states)
                .map(|(target, range)| target.clamp(range.min_rad, range.max_rad))
                .collect()
        } else {
            targets.clone()
        };
        let body_targets = BodyTargets {
            positions: applied_targets.clone(),
        };
        if self.config.enforce_body_range {
            self.body.write_targets(body_targets)?;
        } else {
            self.body.write_targets_unchecked(body_targets)?;
        }
        self.adapter.write(&applied_targets)?;
        self.last_targets = applied_targets;
        if self.phase == LoopPhase::Homing {
            self.homing_ticks += 1;
            if self.homing_ticks >= self.config.homing_ticks {
                self.phase = LoopPhase::Ready;
            }
        }
        Ok(())
    }

    pub fn status(&self) -> LoopStatus {
        let state = self.body.read_state();
        LoopStatus {
            phase: self.phase,
            session: self.session,
            tick: self.tick,
            torque: lifecycle::torque_enabled(self.phase),
            positions: state.positions,
            targets: self.last_targets.clone(),
            sensor_failures: self.sensor_failures,
            last_error: self.last_error.clone(),
            joint_range_states: self.joint_range_states.clone(),
        }
    }

    pub(crate) fn sensor_velocities(&self) -> &[f64] {
        &self.last_velocities
    }

    fn validate_targets(&self, targets: &[f64]) -> Result<(), LoopError> {
        safety::validate_targets(self.body.template(), targets)
    }
}
