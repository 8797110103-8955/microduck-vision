//! Base body control loop shared by physical and virtual servo drivers.

use std::time::Duration;

use crate::{
    BodyCommand, JointRangeState, LoopAction, LoopBinding, LoopConfig, LoopError, LoopPhase,
    LoopStatus, adapter::ServoAdapter, lifecycle, safety,
};
use duck_body::{BodyState, BodyTargets, ServoParameterSet, VirtualBody};

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
    pending_velocities: Option<Vec<f64>>,
    last_imu: Option<servo::ServoImuSample>,
    valid: Vec<bool>,
    joint_range_states: Vec<JointRangeState>,
}

impl BodyLoop {
    pub fn open(
        binding: LoopBinding,
        driver: Box<dyn crate::ControlBackend>,
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
            pending_velocities: None,
            last_imu: None,
            valid: vec![false; count],
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
                self.command = None;
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

    pub fn report_controller_error(&mut self, detail: impl Into<String>) {
        self.phase = LoopPhase::Holding;
        self.last_error = Some(detail.into());
    }

    pub fn tick(&mut self, dt: Duration) -> Result<(), LoopError> {
        self.update_sensors(dt)?;
        self.apply_targets()?;
        self.advance_execution(self.config.physics_substeps, self.config.physics_dt)
    }

    /// Run one controller-backed tick through the same lifecycle and backend path used by every
    /// execution mode. The controller is called after sensors are sampled and before targets are
    /// applied, so a desktop policy scheduler can switch an already-loaded session for this tick
    /// without opening a second loop or restarting the backend.
    pub fn tick_with_controller<C: crate::LoopController>(
        &mut self,
        dt: Duration,
        controller: &mut C,
    ) -> Result<(), LoopError> {
        let sensors = self.read_sensors(dt)?;
        let status = self.status();
        if !matches!(status.phase, LoopPhase::Ready | LoopPhase::Driving) {
            self.apply_targets()?;
            return self.advance_execution(self.config.physics_substeps, self.config.physics_dt);
        }

        let command = match controller.on_loop(crate::ControllerInput { sensors, status }) {
            Ok(command) => command,
            Err(error) => {
                self.report_controller_error(error.to_string());
                return Err(error);
            }
        };
        if let Some(command) = command {
            if self.phase == LoopPhase::Ready {
                self.send(LoopAction::Drive(true))?;
            }
            self.set_command(command)?;
        }
        self.apply_targets()?;
        self.advance_execution(self.config.physics_substeps, self.config.physics_dt)
    }

    /// Sample the selected backend and return the canonical frame for an injected controller.
    /// Policy and product code can build a command from this frame while the lifecycle and
    /// backend ownership remain inside `BodyLoop`.
    pub fn read_sensors(&mut self, dt: Duration) -> Result<crate::SensorFrame, LoopError> {
        self.update_sensors(dt)?;
        Ok(self.sensor_frame())
    }

    pub fn sensor_frame(&self) -> crate::SensorFrame {
        let state = self.body.read_state();
        crate::SensorFrame {
            valid: self.valid.clone(),
            positions: state.positions,
            velocities: self.last_velocities.clone(),
            imu: self.last_imu,
        }
    }

    /// Advance backend execution independently from target submission.
    pub fn advance_execution(
        &mut self,
        physics_steps: u32,
        physics_dt: Duration,
    ) -> Result<(), LoopError> {
        self.adapter.advance_execution(physics_steps, physics_dt)
    }

    pub fn reset_execution(&mut self) -> Result<(), LoopError> {
        self.adapter.reset_execution()?;
        self.command = None;
        self.pending_velocities = None;
        self.last_velocities.fill(0.0);
        self.last_imu = None;
        self.valid.fill(false);
        self.sensor_failures = 0;
        self.last_error = None;
        let sample = self.adapter.read()?;
        self.apply_sensor_frame(sample, Duration::ZERO)?;
        self.last_targets = self.body.read_state().positions;
        Ok(())
    }

    pub fn config(&self) -> &LoopConfig {
        &self.config
    }

    pub fn servo_ids(&self) -> impl Iterator<Item = u8> + '_ {
        self.adapter.servo_ids()
    }

    pub fn set_velocity_observation_delay(&mut self, ticks: u32) -> Result<(), LoopError> {
        if ticks > 1 {
            return Err(LoopError::Controller(
                "only current or one-tick velocity sampling is supported".into(),
            ));
        }
        self.config.velocity_observation_delay_ticks = ticks;
        self.pending_velocities = None;
        self.last_velocities.fill(0.0);
        Ok(())
    }

    /// Read the current servo state without changing lifecycle, torque, or target outputs.
    ///
    /// This is used by the desktop debug view, which must be able to observe a limp robot while
    /// leaving all actuator writes to its explicit, separate debug commands.
    pub fn read_sensors_only(&mut self, dt: Duration) -> Result<(), LoopError> {
        if dt != self.config.period {
            return Err(LoopError::InvalidTick);
        }
        self.tick += 1;
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
        self.apply_sensor_frame(sample, dt)
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
        self.apply_sensor_frame(sample, dt)
    }

    fn apply_sensor_frame(
        &mut self,
        sample: crate::SensorFrame,
        dt: Duration,
    ) -> Result<(), LoopError> {
        if sample.valid.len() != sample.positions.len() {
            return Err(LoopError::InvalidBackendState {
                field: "feedback validity",
            });
        }
        self.valid = sample.valid;
        self.sensor_failures = 0;
        self.last_error = None;
        let current_velocities = sample.velocities;
        let observed_velocities = if self.config.velocity_observation_delay_ticks == 0 {
            current_velocities.clone()
        } else {
            self.pending_velocities
                .replace(current_velocities.clone())
                .unwrap_or_else(|| vec![0.0; current_velocities.len()])
        };
        self.last_velocities = observed_velocities;
        self.last_imu = sample.imu;
        self.body.apply_sensor_state(BodyState {
            positions: sample.positions,
            velocities: current_velocities,
            elapsed: self.body.read_state().elapsed + dt,
        })?;
        Ok(())
    }

    pub fn apply_targets(&mut self) -> Result<(), LoopError> {
        let positions = self.body.read_state().positions;

        let targets = match self.phase {
            // Homing must establish a safe hold, not drive every joint to the numeric zero
            // frame.  The controller's neutral pose is non-zero for several joints, and a physical
            // robot may already be in a valid pose when torque is enabled.  Holding the measured
            // canonical positions avoids an artificial jump before controller inference starts.
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

    fn validate_targets(&self, targets: &[f64]) -> Result<(), LoopError> {
        safety::validate_targets(self.body.template(), targets)
    }
}
