//! Base body control loop shared by physical and virtual servo drivers.

use std::time::Duration;

use duck_body::{BodyState, BodyTargets, ServoParameterSet, VirtualBody};
use servo::{ServoLoopDriver, ServoPositionTargets};

use crate::{
    BodyCommand, Calibration, LoopAction, LoopBinding, LoopConfig, LoopError, LoopPhase, LoopStatus,
};

pub struct BodyLoop {
    body: VirtualBody,
    calibration: Calibration,
    driver: Box<dyn ServoLoopDriver>,
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
        if config.period.is_zero() || config.homing_ticks == 0 || config.sensor_failure_limit == 0 {
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
        Ok(Self {
            body,
            calibration: binding.calibration,
            driver,
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
                self.driver.set_torque(true)?;
                self.driver.set_gain(self.config.gain)?;
                self.phase = LoopPhase::Homing;
                self.homing_ticks = 0;
                self.sensor_failures = 0;
            }
            LoopAction::Power(false) => {
                self.driver.set_torque(false)?;
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
                self.driver.set_torque(false)?;
            }
            LoopAction::Estop => {
                self.phase = LoopPhase::Estop;
                self.driver.set_torque(false)?;
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
        if dt != self.config.period {
            return Err(LoopError::InvalidTick);
        }
        self.tick += 1;
        if matches!(
            self.phase,
            LoopPhase::Limp | LoopPhase::Stopped | LoopPhase::Estop
        ) {
            return Ok(());
        }
        if self.phase == LoopPhase::Reconnecting {
            self.phase = LoopPhase::Homing;
            self.driver.set_torque(true)?;
            self.driver.set_gain(self.config.gain)?;
            self.homing_ticks = 0;
        }
        let ids: Vec<u8> = self
            .calibration
            .joints
            .iter()
            .map(|joint| joint.physical_id)
            .collect();
        let sample = match self.driver.read_positions(&ids) {
            Ok(sample) => sample,
            Err(error) => {
                self.sensor_failures += 1;
                self.last_error = Some(error.to_string());
                if self.sensor_failures >= self.config.sensor_failure_limit {
                    self.phase = LoopPhase::Holding;
                }
                return Err(error.into());
            }
        };
        if sample.positions.len() != ids.len() || sample.velocities.len() != ids.len() {
            self.sensor_failures += 1;
            return Err(LoopError::SensorLength {
                actual: sample.positions.len(),
                expected: ids.len(),
            });
        }
        self.sensor_failures = 0;
        self.last_error = None;
        let positions = self.to_body_positions(&sample.positions);
        let velocities = self.to_body_positions(&sample.velocities);
        self.last_velocities = velocities.clone();
        self.body.apply_sensor_state(BodyState {
            positions: positions.clone(),
            velocities,
            elapsed: self.body.read_state().elapsed + dt,
        })?;

        let targets = match self.phase {
            LoopPhase::Homing => vec![0.0; ids.len()],
            LoopPhase::Driving => self
                .command
                .as_ref()
                .map(|command| command.targets.clone())
                .unwrap_or_else(|| self.last_targets.clone()),
            LoopPhase::Holding => self.last_targets.clone(),
            _ => return Ok(()),
        };
        self.validate_targets(&targets)?;
        self.body.write_targets(BodyTargets {
            positions: targets.clone(),
        })?;
        let servo_targets = self.to_servo_positions(&targets);
        self.driver.write_positions(
            &ids,
            &ServoPositionTargets {
                positions: servo_targets,
            },
        )?;
        self.last_targets = targets;
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
            torque: matches!(
                self.phase,
                LoopPhase::Homing | LoopPhase::Ready | LoopPhase::Driving | LoopPhase::Holding
            ),
            positions: state.positions,
            targets: self.last_targets.clone(),
            sensor_failures: self.sensor_failures,
            last_error: self.last_error.clone(),
        }
    }

    pub(crate) fn sensor_velocities(&self) -> &[f64] {
        &self.last_velocities
    }

    fn validate_targets(&self, targets: &[f64]) -> Result<(), LoopError> {
        if targets.len() != self.body.template().joints.len() {
            return Err(LoopError::TargetLength {
                actual: targets.len(),
                expected: self.body.template().joints.len(),
            });
        }
        for (joint, target) in self.body.template().joints.iter().zip(targets) {
            if !target.is_finite() {
                return Err(LoopError::NonFiniteTarget {
                    joint: joint.name.clone(),
                });
            }
        }
        Ok(())
    }

    fn to_servo_positions(&self, positions: &[f64]) -> Vec<f64> {
        positions
            .iter()
            .zip(&self.calibration.joints)
            .map(|(position, calibration)| {
                (*position + calibration.zero_offset_rad) * f64::from(calibration.direction)
            })
            .collect()
    }

    fn to_body_positions(&self, positions: &[f64]) -> Vec<f64> {
        positions
            .iter()
            .zip(&self.calibration.joints)
            .map(|(position, calibration)| {
                *position * f64::from(calibration.direction) - calibration.zero_offset_rad
            })
            .collect()
    }
}
