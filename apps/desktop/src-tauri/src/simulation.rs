//! Virtual preview driver and desktop adapter for the shared policy loop.
//!
//! Policy inference and control-loop scheduling live in `loop-body`. This module only supplies a
//! memory-backed servo driver and converts the crate status into the Tauri status shape.

use std::path::{Path, PathBuf};
use std::time::Duration;

use duck_body::BodyTemplate;
use duck_control::obs::Command;
use loop_body::{
    BodyCommand, BodyRuntimeView, Calibration, JointCalibration, LoopAction, LoopBinding,
    LoopConfig, LoopPhase as BodyLoopPhase, PolicyConfig, PolicyDescriptor, PolicyLoop,
    ServoBinding,
};
use servo::{ServoError, ServoLoopDriver, ServoModel, ServoPositionSample, ServoPositionTargets};

use crate::logging::{
    LoggedServoDriver, append_control_event, append_policy_trace, begin_control_loop,
};
use crate::types::{LoopPhase, LoopStatus, WorldState};
use crate::types::{PolicyCommand, PreviewInput};

const PERIOD: Duration = Duration::from_millis(20);
const PREVIEW_SERVO_RESPONSE: f64 = 0.35;
// Sitstand training treats joint speeds above roughly 3 rad/s as an overspeed event.  The
// previous unconstrained first-order preview could turn a one-radian target change into
// 15--20 rad/s feedback, which is far outside the policy's closed-loop envelope.
const PREVIEW_MAX_JOINT_VELOCITY_RAD_S: f64 = 3.0;
const STAND_POSITION_TOLERANCE: f64 = 0.03;
const STAND_VELOCITY_TOLERANCE: f64 = 0.3;
const STAND_STABLE_TICKS: u8 = 3;

struct PreviewDriver {
    positions: Vec<f64>,
    targets: Vec<f64>,
    /// The virtual plant's just-computed velocity, kept one sample behind the returned sensor
    /// value to match Dynamixel present_velocity's one-control-tick observation delay.
    velocities: Vec<f64>,
    pending_velocities: Vec<f64>,
}

impl ServoLoopDriver for PreviewDriver {
    fn read_positions(&mut self, ids: &[u8]) -> Result<ServoPositionSample, ServoError> {
        if self.positions.len() != ids.len() {
            self.positions.resize(ids.len(), 0.0);
            self.targets.resize(ids.len(), 0.0);
            self.velocities.resize(ids.len(), 0.0);
            self.pending_velocities.resize(ids.len(), 0.0);
        }
        let dt = PERIOD.as_secs_f64();
        let reported_velocities = self.velocities.clone();
        for ((position, target), velocity) in self
            .positions
            .iter_mut()
            .zip(&self.targets)
            .zip(&mut self.pending_velocities)
        {
            let response_step = PREVIEW_SERVO_RESPONSE * (*target - *position);
            let max_step = PREVIEW_MAX_JOINT_VELOCITY_RAD_S * dt;
            let step = response_step.clamp(-max_step, max_step);
            *position += step;
            *velocity = step / dt;
        }
        self.velocities.clone_from(&self.pending_velocities);
        Ok(ServoPositionSample {
            positions: self.positions.clone(),
            velocities: reported_velocities,
        })
    }

    fn write_positions(
        &mut self,
        _ids: &[u8],
        targets: &ServoPositionTargets,
    ) -> Result<(), ServoError> {
        self.targets = targets.positions.clone();
        Ok(())
    }

    fn set_torque(&mut self, _enabled: bool) -> Result<(), ServoError> {
        Ok(())
    }

    fn set_gain(&mut self, _gain: u16) -> Result<(), ServoError> {
        Ok(())
    }

    fn reboot(&mut self, _id: u8) -> Result<(), ServoError> {
        Ok(())
    }
}

#[derive(Debug, Clone)]
struct PreviewPolicySelection {
    path: PathBuf,
    command: Option<PolicyCommand>,
    descriptor: PolicyDescriptor,
    config: PolicyConfig,
    episodic: bool,
    duration: Option<Duration>,
}

pub(crate) struct PreviewPolicyLoop {
    loop_state: PolicyLoop,
    input: PreviewInput,
    policy_command: Option<PolicyCommand>,
    world: WorldState,
    config_dir: Option<PathBuf>,
    policy_ticks: u64,
    home_pose: Vec<f64>,
    active_policy: Option<PreviewPolicySelection>,
    pending_policy: Option<PreviewPolicySelection>,
    resume_policy: Option<PreviewPolicySelection>,
    preparing_stand: bool,
    stand_stable_ticks: u8,
    previous_positions: Option<Vec<f64>>,
    skill_elapsed: Duration,
}

impl PreviewPolicyLoop {
    pub(crate) fn open(
        model: ServoModel,
        config_dir: Option<&Path>,
        machine_id: &str,
        machine_name: &str,
    ) -> Result<Self, String> {
        let template = BodyTemplate::official().map_err(|error| error.to_string())?;
        let home = template.control.home_pose.clone();
        let binding_summary = template
            .actuators
            .iter()
            .enumerate()
            .map(|(index, slot)| format!("{}->{} dir=1 offset=0", slot.id, index + 1))
            .collect::<Vec<_>>()
            .join(" | ");
        let binding = LoopBinding {
            body: BodyRuntimeView::new(template.clone()).map_err(|error| error.to_string())?,
            servos: ServoBinding::all_slots(&template, model),
            calibration: Calibration {
                body_id: template.id.clone(),
                body_version: template.version,
                // Preview uses the canonical simulation IDs in body actuator order.  The
                // policy layer translates ONNX's 14 outputs to this 15-slot order; a physical
                // loop replaces only this calibration with its saved body-to-servo binding.
                joints: template
                    .actuators
                    .iter()
                    .enumerate()
                    .map(|(index, slot)| JointCalibration {
                        slot_id: slot.id.clone(),
                        physical_id: u8::try_from(index + 1)
                            .expect("official body has fewer than 256 joints"),
                        direction: 1,
                        zero_offset_rad: 0.0,
                        min_rad: None,
                        max_rad: None,
                    })
                    .collect(),
            },
        };
        let driver: Box<dyn ServoLoopDriver> = if let Some(handle) =
            begin_control_loop(config_dir, machine_id, machine_name, &binding_summary)
        {
            Box::new(LoggedServoDriver::new(
                Box::new(PreviewDriver {
                    positions: home.clone(),
                    targets: home.clone(),
                    velocities: vec![0.0; home.len()],
                    pending_velocities: vec![0.0; home.len()],
                }),
                handle,
                machine_id,
            ))
        } else {
            Box::new(PreviewDriver {
                positions: home.clone(),
                targets: home.clone(),
                velocities: vec![0.0; home.len()],
                pending_velocities: vec![0.0; home.len()],
            })
        };
        let preview_config = LoopConfig {
            homing_ticks: 0,
            enforce_body_range: false,
            ..LoopConfig::default()
        };
        let mut loop_state =
            PolicyLoop::open(binding, driver, preview_config).map_err(|error| error.to_string())?;
        // A preview starts from the policy's neutral command.  The observation is assembled
        // inside `PolicyLoop::tick` from the virtual sensors, the previous action and this
        // command; policy-specific commands (phase/posture/teleop) are applied by the preview
        // input layer after the manifest has been resolved.
        loop_state.set_policy_command(Command::default());
        let mut preview = Self {
            loop_state,
            input: PreviewInput {
                twist: [0.0; 3],
                head: [0.0; 4],
                body: [0.0; 3],
                target_distance_m: None,
            },
            policy_command: None,
            world: WorldState {
                position: [0.0, 0.0],
                yaw: 0.0,
                distance_m: 0.0,
            },
            config_dir: config_dir.map(Path::to_path_buf),
            policy_ticks: 0,
            home_pose: home,
            active_policy: None,
            pending_policy: None,
            resume_policy: None,
            preparing_stand: false,
            stand_stable_ticks: 0,
            previous_positions: None,
            skill_elapsed: Duration::ZERO,
        };
        preview
            .loop_state
            .send(LoopAction::Power(true))
            .map_err(|error| error.to_string())?;
        Ok(preview)
    }

    pub(crate) fn select_policy(
        &mut self,
        path: &Path,
        command: Option<PolicyCommand>,
        descriptor: PolicyDescriptor,
        config: PolicyConfig,
        kind: Option<&str>,
        duration_s: Option<f64>,
    ) -> Result<(), String> {
        if self.preparing_stand
            || self
                .active_policy
                .as_ref()
                .is_some_and(|policy| policy.episodic)
        {
            return Err("a skill is already running".into());
        }
        let episodic = kind == Some("episodic");
        let duration = if episodic {
            let seconds = duration_s
                .filter(|seconds| seconds.is_finite() && *seconds > 0.0)
                .ok_or_else(|| "episodic policy is missing a valid duration_s".to_owned())?;
            Some(
                Duration::try_from_secs_f64(seconds)
                    .map_err(|_| "episodic policy duration is out of range".to_owned())?,
            )
        } else {
            None
        };
        let selection = PreviewPolicySelection {
            path: path.to_owned(),
            command,
            descriptor,
            config,
            episodic,
            duration,
        };
        duck_control::policy::validate(path).map_err(|error| error.to_string())?;
        self.resume_policy = self.active_policy.clone().filter(|policy| !policy.episodic);
        self.pending_policy = Some(selection);
        self.begin_standing_transition()?;
        Ok(())
    }

    pub(crate) fn set_input(&mut self, input: PreviewInput) {
        append_control_event(
            self.config_dir.as_deref(),
            format!(
                "preview_input twist={:?} head={:?} body={:?} target_distance_m={:?}",
                input.twist, input.head, input.body, input.target_distance_m
            ),
        );
        self.input = input;
        self.apply_command();
    }

    pub(crate) fn reset_world(&mut self) {
        append_control_event(self.config_dir.as_deref(), "preview_world_reset");
        self.world = WorldState {
            position: [0.0, 0.0],
            yaw: 0.0,
            distance_m: 0.0,
        };
    }

    pub(crate) fn tick(&mut self) -> Result<LoopStatus, String> {
        let current = self.loop_state.status();
        let twist = if self.preparing_stand {
            [0.0; 3]
        } else {
            self.world_twist()
        };
        if current.phase == BodyLoopPhase::Driving {
            let seconds = 0.02;
            self.world.position[0] += twist[0] * seconds;
            self.world.position[1] += twist[1] * seconds;
            self.world.yaw += twist[2] * seconds;
            self.world.distance_m += twist[0].hypot(twist[1]) * seconds;
        }
        self.ensure_driving();
        if self.preparing_stand {
            self.set_standing_command()?;
        }
        self.apply_command();
        self.loop_state
            .tick(PERIOD)
            .map_err(|error| error.to_string())?;
        let has_trace = self.loop_state.policy_trace().is_some();
        if let Some(trace) = self.loop_state.policy_trace() {
            append_policy_trace(
                self.config_dir.as_deref(),
                self.loop_state.status().tick,
                trace,
            );
            self.policy_ticks += 1;
        }
        if self.preparing_stand {
            self.update_stand_transition();
        } else if has_trace {
            self.skill_elapsed += PERIOD;
            if let Some(policy) = &self.active_policy
                && policy.episodic
                && self.skill_elapsed >= policy.duration.unwrap_or_default()
            {
                let resume = self.resume_policy.take();
                self.active_policy = None;
                self.pending_policy = resume;
                self.begin_standing_transition()?;
            }
        }
        Ok(self.status())
    }

    fn begin_standing_transition(&mut self) -> Result<(), String> {
        append_control_event(
            self.config_dir.as_deref(),
            format!(
                "preview_prepare_stand pending={} resume={}",
                self.pending_policy.is_some(),
                self.resume_policy.is_some()
            ),
        );
        self.loop_state.clear_policy();
        self.policy_command = None;
        self.policy_ticks = 0;
        self.skill_elapsed = Duration::ZERO;
        self.preparing_stand = true;
        self.stand_stable_ticks = 0;
        self.previous_positions = None;
        self.set_standing_command()
    }

    fn set_standing_command(&mut self) -> Result<(), String> {
        let session = self.loop_state.status().session;
        self.loop_state
            .set_command(BodyCommand {
                session,
                targets: self.home_pose.clone(),
            })
            .map_err(|error| error.to_string())
    }

    fn ensure_driving(&mut self) {
        if matches!(
            self.loop_state.status().phase,
            BodyLoopPhase::Ready | BodyLoopPhase::Holding
        ) {
            let _ = self.loop_state.send(LoopAction::Drive(true));
        }
    }

    fn update_stand_transition(&mut self) {
        let positions = self.loop_state.status().positions;
        let position_error = positions
            .iter()
            .zip(&self.home_pose)
            .map(|(position, home)| (position - home).abs())
            .fold(0.0, f64::max);
        let velocity = self
            .previous_positions
            .as_ref()
            .map(|previous| {
                positions
                    .iter()
                    .zip(previous)
                    .map(|(position, previous)| (position - previous).abs() / PERIOD.as_secs_f64())
                    .fold(0.0, f64::max)
            })
            .unwrap_or(f64::INFINITY);
        self.previous_positions = Some(positions);
        if position_error <= STAND_POSITION_TOLERANCE && velocity <= STAND_VELOCITY_TOLERANCE {
            self.stand_stable_ticks = self.stand_stable_ticks.saturating_add(1);
        } else {
            self.stand_stable_ticks = 0;
        }
        if self.stand_stable_ticks < STAND_STABLE_TICKS
            || self.loop_state.status().phase != BodyLoopPhase::Driving
        {
            return;
        }
        self.preparing_stand = false;
        self.stand_stable_ticks = 0;
        self.previous_positions = None;
        let Some(selection) = self.pending_policy.take() else {
            append_control_event(
                self.config_dir.as_deref(),
                "preview_stand_ready policy=none",
            );
            return;
        };
        if let Err(error) = self.activate_policy(selection) {
            append_control_event(
                self.config_dir.as_deref(),
                format!("preview_policy_activation_error error={error}"),
            );
        }
    }

    fn activate_policy(&mut self, selection: PreviewPolicySelection) -> Result<(), String> {
        self.loop_state
            .select_policy_with_descriptor(&selection.path, &selection.descriptor, selection.config)
            .map_err(|error| error.to_string())?;
        append_control_event(
            self.config_dir.as_deref(),
            format!(
                "preview_policy_selected path={:?} config={:?} episodic={} duration_s={:?}",
                selection.path,
                selection.config,
                selection.episodic,
                selection.duration.map(|duration| duration.as_secs_f64()),
            ),
        );
        self.policy_command = selection.command.clone();
        self.policy_ticks = 0;
        self.skill_elapsed = Duration::ZERO;
        self.active_policy = Some(selection);
        self.apply_command();
        Ok(())
    }

    fn apply_command(&mut self) {
        let twist = self.effective_twist();
        let (head, body) = if self
            .active_policy
            .as_ref()
            .is_some_and(|policy| policy.episodic)
        {
            ([0.0; 4], [0.0; 3])
        } else {
            (self.input.head, self.input.body)
        };
        self.loop_state.set_policy_command(Command {
            twist,
            head,
            body: duck_control::obs::BodyPose {
                z: body[0],
                roll: body[1],
                pitch: body[2],
            },
        });
    }

    fn effective_twist(&self) -> [f64; 3] {
        let mut twist = if self
            .active_policy
            .as_ref()
            .is_some_and(|policy| policy.episodic)
        {
            [0.0; 3]
        } else {
            self.input.twist
        };
        if let Some(command) = &self.policy_command {
            match command.encoding.as_deref() {
                Some("phase") => {
                    let period = command
                        .period_s
                        .filter(|period| *period > 0.0)
                        .unwrap_or(4.0);
                    let phase = self.policy_ticks as f64 * PERIOD.as_secs_f64() / period;
                    let phase = command
                        .end_phase
                        .filter(|end| end.is_finite() && *end >= 0.0)
                        .map_or(phase, |end| phase.min(end));
                    let angle = std::f64::consts::TAU * phase;
                    twist = [angle.cos(), angle.sin(), 0.0];
                }
                Some("posture_flag") => {
                    let sit = command.sit.unwrap_or(1.0);
                    let stand = command.stand.unwrap_or(0.0);
                    // The UI carries the requested posture in twist.vx.  Pick the nearest
                    // manifest value so the policy receives the exact trained flag instead of
                    // an arbitrary slider value.
                    twist[0] = if (self.input.twist[0] - sit).abs()
                        <= (self.input.twist[0] - stand).abs()
                    {
                        sit
                    } else {
                        stand
                    };
                }
                _ => {
                    if self.input.twist == [0.0; 3]
                        && let Some(idle) = command.idle
                    {
                        twist = idle;
                    }
                }
            }
        }
        if let Some(target) = self.input.target_distance_m
            && target.is_finite()
            && target > 0.0
            && self.world.distance_m >= target
        {
            twist = [0.0; 3];
        }
        twist
    }

    fn world_twist(&self) -> [f64; 3] {
        match self
            .policy_command
            .as_ref()
            .and_then(|command| command.encoding.as_deref())
        {
            // These policies use the twist block as their own phase/pose input.  It is not a
            // locomotion request and must not move the preview's world pose.
            Some("phase" | "posture_flag") => [0.0; 3],
            _ => self.effective_twist(),
        }
    }

    pub(crate) fn status(&self) -> LoopStatus {
        let status = self.loop_state.status();
        LoopStatus {
            phase: status.phase.into(),
            session: status.session,
            tick: status.tick,
            torque: status.torque,
            positions: status.positions,
            targets: status.targets,
            world: self.world,
            sensor_failures: status.sensor_failures,
            last_error: status.last_error,
            joint_range_states: status
                .joint_range_states
                .into_iter()
                .map(|state| crate::types::JointRangeState {
                    slot_id: state.slot_id,
                    target_rad: state.target_rad,
                    min_rad: state.min_rad,
                    max_rad: state.max_rad,
                    below_min: state.below_min,
                    above_max: state.above_max,
                })
                .collect(),
        }
    }
}

impl From<BodyLoopPhase> for LoopPhase {
    fn from(phase: BodyLoopPhase) -> Self {
        match phase {
            BodyLoopPhase::Limp => Self::Limp,
            BodyLoopPhase::Homing => Self::Homing,
            BodyLoopPhase::Ready => Self::Ready,
            BodyLoopPhase::Driving => Self::Driving,
            BodyLoopPhase::Holding => Self::Holding,
            BodyLoopPhase::Stopped => Self::Stopped,
            BodyLoopPhase::Reconnecting => Self::Reconnecting,
            BodyLoopPhase::Estop => Self::Estop,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::{PREVIEW_MAX_JOINT_VELOCITY_RAD_S, PreviewDriver, PreviewPolicyLoop};
    use duck_body::BodyTemplate;
    use servo::{ServoLoopDriver, ServoModel, ServoPositionTargets};
    use std::time::{SystemTime, UNIX_EPOCH};

    #[test]
    fn preview_loop_uses_shared_policy_loop_without_hardware() {
        let mut loop_state =
            PreviewPolicyLoop::open(ServoModel::Hd1910, None, "test-machine", "测试机器").unwrap();
        assert_eq!(loop_state.status().phase, crate::types::LoopPhase::Ready);
        loop_state.tick().unwrap();
        let status = loop_state.status();
        assert_eq!(status.phase, crate::types::LoopPhase::Driving);
        assert_eq!(status.positions.len(), 15);
        assert_eq!(status.targets.len(), 15);
        assert_eq!(
            status.positions,
            BodyTemplate::official().unwrap().control.home_pose
        );
        assert_eq!(status.sensor_failures, 0);
    }

    #[test]
    fn preview_loop_logs_virtual_servo_io() {
        let suffix = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_nanos();
        let root = std::env::temp_dir().join(format!("microduck-preview-log-{suffix}"));
        let config_dir = root.join("config");
        let mut loop_state =
            PreviewPolicyLoop::open(ServoModel::Hd1910, Some(&config_dir), "virtual-1", "模拟机")
                .unwrap();
        loop_state.tick().unwrap();
        let log =
            std::fs::read_to_string(crate::logging::current_log_file(Some(&config_dir))).unwrap();
        assert!(log.contains("loop_open machine_id=virtual-1"));
        assert!(log.contains("servo_read id=1"));
        let _ = std::fs::remove_dir_all(root);
    }

    #[test]
    fn preview_servo_limits_velocity_and_delays_feedback_one_tick() {
        let mut driver = PreviewDriver {
            positions: vec![0.0],
            targets: vec![0.0],
            velocities: vec![0.0],
            pending_velocities: vec![0.0],
        };
        driver
            .write_positions(
                &[1],
                &ServoPositionTargets {
                    positions: vec![1.0],
                },
            )
            .unwrap();

        let first = driver.read_positions(&[1]).unwrap();
        assert_eq!(first.velocities, vec![0.0]);
        assert!((first.positions[0] - 0.06).abs() < 1e-12);

        let second = driver.read_positions(&[1]).unwrap();
        assert_eq!(second.velocities, vec![PREVIEW_MAX_JOINT_VELOCITY_RAD_S]);
        assert!((second.positions[0] - 0.12).abs() < 1e-12);
    }
}
