//! Shared control-loop types and validated body/servo bindings.

use std::time::Duration;

use duck_body::{BodyBinding, BodyError, BodyProfile, BodyTemplate, ServoParameterSet};
use serde::{Deserialize, Serialize};
use servo::ServoModel;
use thiserror::Error;

/// Monotonic logical sample time. The value is deliberately independent of a host `Instant` so
/// traces and MCU adapters share the same contract.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub struct SampleTime(pub u64);

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct ControlTick(pub u64);

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ExecutionInterval {
    pub control_tick: ControlTick,
    pub physics_steps: u32,
    pub physics_dt: Duration,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BackendLifecycleAction {
    Stop,
    Reset,
    Estop,
    Resume,
}

/// Timebase used by a backend-owned delay. Keeping this explicit prevents vendor physics steps
/// from being mistaken for policy control ticks.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DelayTimebase {
    PhysicsStep,
    ControlTick,
    WallClock,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ActuatorDelaySpec {
    pub timebase: DelayTimebase,
    pub min_steps: u32,
    pub max_steps: u32,
}

impl ActuatorDelaySpec {
    pub const fn physics_steps(min_steps: u32, max_steps: u32) -> Self {
        Self {
            timebase: DelayTimebase::PhysicsStep,
            min_steps,
            max_steps,
        }
    }

    pub fn validate(self) -> Result<(), LoopError> {
        if self.min_steps > self.max_steps {
            return Err(LoopError::Controller(format!(
                "actuator delay minimum {} exceeds maximum {}",
                self.min_steps, self.max_steps
            )));
        }
        Ok(())
    }
}

#[derive(Debug, Error, PartialEq)]
pub enum LoopError {
    #[error("body error: {0}")]
    Body(#[from] BodyError),
    #[error("servo driver error: {0}")]
    Servo(#[from] servo::ServoError),
    #[error("body has no actuator slots")]
    NoActuators,
    #[error("servo binding slots do not match body actuator slots")]
    BindingSlotsMismatch,
    #[error("calibration entries do not match body actuator slots")]
    CalibrationSlotsMismatch,
    #[error("calibration for slot {slot:?} has invalid direction {direction}")]
    InvalidDirection { slot: String, direction: i8 },
    #[error("calibration for slot {slot:?} has a non-finite zero offset")]
    NonFiniteOffset { slot: String },
    #[error("command belongs to session {command}, current session is {current}")]
    StaleSession { command: u64, current: u64 },
    #[error("target vector has length {actual}, expected {expected}")]
    TargetLength { actual: usize, expected: usize },
    #[error("target for joint {joint:?} is not finite")]
    NonFiniteTarget { joint: String },
    #[error("loop action is invalid in phase {0:?}")]
    InvalidAction(LoopPhase),
    #[error("tick duration must equal the configured period")]
    InvalidTick,
    #[error("loop gain must be greater than zero")]
    ZeroGain,
    #[error("sensor frame has length {actual}, expected {expected}")]
    SensorLength { actual: usize, expected: usize },
    #[error("backend state contains invalid or non-finite {field}")]
    InvalidBackendState { field: &'static str },
    #[error("controller requires an IMU sample but the backend reported none")]
    MissingImu,
    #[error("controller error: {0}")]
    Controller(String),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum LoopPhase {
    Limp,
    Homing,
    Ready,
    Driving,
    Holding,
    Stopped,
    Reconnecting,
    Estop,
}

#[derive(Debug, Clone, PartialEq)]
pub struct LoopConfig {
    pub period: Duration,
    pub physics_dt: Duration,
    pub physics_substeps: u32,
    /// Startup sensor-hold ticks. Zero skips the physical homing phase, which is useful for a
    /// virtual driver whose initial positions are already known.
    pub homing_ticks: u32,
    pub sensor_failure_limit: u32,
    pub gain: u16,
    /// Number of control ticks by which joint velocity observations are delayed. Physical
    /// servo telemetry uses the deployed one-tick sensor path; a virtual backend can set this to
    /// zero when it advances physics after writing the current controller action.
    pub velocity_observation_delay_ticks: u32,
    /// Clamp requested body targets before writing to a physical driver. Virtual previews keep
    /// this disabled so the scene can show the requested out-of-range posture while reporting it.
    pub enforce_body_range: bool,
}

impl Default for LoopConfig {
    fn default() -> Self {
        Self {
            period: Duration::from_millis(20),
            physics_dt: Duration::from_millis(5),
            physics_substeps: 4,
            homing_ticks: 5,
            sensor_failure_limit: 3,
            gain: 200,
            velocity_observation_delay_ticks: 1,
            enforce_body_range: true,
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct BodyRuntimeView {
    template: BodyTemplate,
    profile: BodyProfile,
}

impl BodyRuntimeView {
    pub fn new(template: BodyTemplate) -> Result<Self, LoopError> {
        let profile = BodyProfile::from_template(template.clone())?;
        if template.actuators.is_empty() {
            return Err(LoopError::NoActuators);
        }
        Ok(Self { template, profile })
    }

    pub fn template(&self) -> &BodyTemplate {
        &self.template
    }

    pub fn profile(&self) -> &BodyProfile {
        &self.profile
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct ServoBinding {
    pub model: ServoModel,
    pub slot_ids: Vec<String>,
}

impl ServoBinding {
    pub fn all_slots(template: &BodyTemplate, model: ServoModel) -> Self {
        Self {
            model,
            slot_ids: template
                .actuators
                .iter()
                .map(|slot| slot.id.clone())
                .collect(),
        }
    }

    pub(crate) fn validate(&self, template: &BodyTemplate) -> Result<(), LoopError> {
        let expected: Vec<_> = template
            .actuators
            .iter()
            .map(|slot| slot.id.clone())
            .collect();
        if self.slot_ids != expected {
            return Err(LoopError::BindingSlotsMismatch);
        }
        let params = ServoParameterSet::bind_all(template, self.model);
        params.validate_for(template)?;
        Ok(())
    }
}

pub use duck_body::JointCalibration;
pub type Calibration = BodyBinding;

#[cfg(test)]
mod calibration_tests {
    use super::{Calibration, JointCalibration};

    #[test]
    fn canonical_slots_translate_to_bound_machine_ids() {
        let calibration = Calibration {
            body_id: String::new(),
            body_version: 0,
            joints: vec![
                JointCalibration {
                    slot_id: "left_hip_yaw".into(),
                    physical_id: 42,
                    direction: -1,
                    zero_offset_rad: 0.2,
                    min_rad: None,
                    max_rad: None,
                },
                JointCalibration {
                    slot_id: "left_hip_roll".into(),
                    physical_id: 43,
                    direction: 1,
                    zero_offset_rad: -0.1,
                    min_rad: None,
                    max_rad: None,
                },
            ],
        };
        assert_eq!(calibration.physical_ids(), vec![42, 43]);
        let servo = calibration.to_servo_positions(&[0.5, 0.25]);
        assert!((servo[0] + 0.3).abs() < 1e-9);
        assert!((servo[1] - 0.15).abs() < 1e-9);
        let body = calibration.to_body_positions(&servo);
        assert!((body[0] - 0.5).abs() < 1e-9);
        assert!((body[1] - 0.25).abs() < 1e-9);
        assert_eq!(calibration.to_body_velocities(&[0.0, 0.0]), vec![0.0, 0.0]);
        assert_eq!(calibration.to_body_velocities(&[2.0, 3.0]), vec![-2.0, 3.0]);
    }

    #[test]
    fn installation_range_clamps_servo_targets() {
        let calibration = Calibration {
            body_id: String::new(),
            body_version: 0,
            joints: vec![JointCalibration {
                slot_id: "left_hip_yaw".into(),
                physical_id: 42,
                direction: 1,
                zero_offset_rad: 0.1,
                min_rad: Some(-0.5),
                max_rad: Some(0.5),
            }],
        };
        assert_eq!(calibration.to_servo_positions(&[1.0]), vec![0.6]);
        assert_eq!(calibration.to_servo_positions_unchecked(&[1.0]), vec![1.1]);
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct LoopBinding {
    pub body: BodyRuntimeView,
    pub servos: ServoBinding,
    pub calibration: Calibration,
}

#[derive(Debug, Clone, PartialEq)]
pub struct BodyCommand {
    pub session: u64,
    pub targets: Vec<f64>,
}

/// Input delivered to the product controller once per control tick.
///
/// The frame is already translated into canonical body order. A controller can choose a policy
/// session, run inference and return a body command without knowing which backend will execute it.
#[derive(Debug, Clone, PartialEq)]
pub struct ControllerInput {
    pub sensors: SensorFrame,
    pub status: LoopStatus,
}

/// Product controller hook for the single body execution loop.
///
/// Returning `None` keeps the loop in its current lifecycle phase and lets it continue its normal
/// sensor, target and physics handling without an inference result.
pub trait LoopController {
    fn on_loop(&mut self, input: ControllerInput) -> Result<Option<BodyCommand>, LoopError>;
}

/// Canonical sensor frame exposed by the adapter boundary. Values are already
/// in body order and body coordinates. Controllers never see physical IDs.
#[derive(Debug, Clone, PartialEq)]
pub struct SensorFrame {
    pub valid: Vec<bool>,
    pub positions: Vec<f64>,
    pub velocities: Vec<f64>,
    pub imu: Option<crate::BackendImuSample>,
}

/// Canonical state returned by a selected control backend.
///
/// Backend implementations may use a physical bus, a local simulation or a transport to
/// another device. The control core only consumes canonical body-joint values and never sees
/// physical servo IDs.
#[derive(Debug, Clone, PartialEq)]
pub struct BackendState {
    pub positions: Vec<f64>,
    pub velocities: Vec<f64>,
    pub imu: Option<crate::BackendImuSample>,
    pub sequence: u64,
    pub sampled_at: SampleTime,
    pub valid_positions: Vec<bool>,
    pub valid_velocities: Vec<bool>,
    pub source: Option<String>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct JointRangeState {
    pub slot_id: String,
    pub target_rad: f64,
    pub min_rad: f64,
    pub max_rad: f64,
    pub below_min: bool,
    pub above_max: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LoopAction {
    Power(bool),
    Drive(bool),
    Stop,
    Estop,
    Reconnect { session: u64 },
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct LoopStatus {
    pub phase: LoopPhase,
    pub session: u64,
    pub tick: u64,
    pub torque: bool,
    pub positions: Vec<f64>,
    pub targets: Vec<f64>,
    pub sensor_failures: u32,
    pub last_error: Option<String>,
    pub joint_range_states: Vec<JointRangeState>,
}
