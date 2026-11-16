//! Virtual preview driver and desktop adapter for the shared policy loop.
//!
//! Policy inference and control-loop scheduling live in `loop-body`. This module only supplies a
//! memory-backed servo driver and converts the crate status into the Tauri status shape.

use std::path::{Path, PathBuf};
use std::time::Duration;

use duck_body::BodyTemplate;
use duck_control::model::DEFAULT_POSITION;
use duck_control::obs::Command;
use loop_body::{
    BodyRuntimeView, Calibration, JointCalibration, LoopAction, LoopBinding, LoopConfig,
    LoopPhase as BodyLoopPhase, PolicyLoop, ServoBinding,
};
use servo::{ServoError, ServoLoopDriver, ServoModel, ServoPositionSample, ServoPositionTargets};

use crate::logging::{
    LoggedServoDriver, append_control_event, append_policy_trace, begin_control_loop,
};
use crate::types::{LoopPhase, LoopStatus, WorldState};
use crate::types::{PolicyCommand, PreviewInput};

const PERIOD: Duration = Duration::from_millis(20);

struct PreviewDriver {
    positions: Vec<f64>,
}

impl ServoLoopDriver for PreviewDriver {
    fn read_positions(&mut self, ids: &[u8]) -> Result<ServoPositionSample, ServoError> {
        if self.positions.len() != ids.len() {
            self.positions.resize(ids.len(), 0.0);
        }
        Ok(ServoPositionSample {
            positions: self.positions.clone(),
            velocities: vec![0.0; ids.len()],
        })
    }

    fn write_positions(
        &mut self,
        _ids: &[u8],
        targets: &ServoPositionTargets,
    ) -> Result<(), ServoError> {
        self.positions = targets.positions.clone();
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

pub(crate) struct PreviewPolicyLoop {
    loop_state: PolicyLoop,
    input: PreviewInput,
    policy_command: Option<PolicyCommand>,
    world: WorldState,
    config_dir: Option<PathBuf>,
}

impl PreviewPolicyLoop {
    pub(crate) fn open(
        model: ServoModel,
        config_dir: Option<&Path>,
        machine_id: &str,
        machine_name: &str,
    ) -> Result<Self, String> {
        let template = BodyTemplate::official().map_err(|error| error.to_string())?;
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
                    positions: DEFAULT_POSITION.to_vec(),
                }),
                handle,
                machine_id,
            ))
        } else {
            Box::new(PreviewDriver {
                positions: DEFAULT_POSITION.to_vec(),
            })
        };
        let mut loop_state = PolicyLoop::open(binding, driver, LoopConfig::default())
            .map_err(|error| error.to_string())?;
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
    ) -> Result<(), String> {
        self.loop_state
            .select_policy(path)
            .map_err(|error| error.to_string())?;
        append_control_event(
            self.config_dir.as_deref(),
            format!("preview_policy_selected path={path:?}"),
        );
        self.policy_command = command;
        self.apply_command();
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
        let twist = self.world_twist();
        if current.phase == BodyLoopPhase::Driving {
            let seconds = 0.02;
            self.world.position[0] += twist[0] * seconds;
            self.world.position[1] += twist[1] * seconds;
            self.world.yaw += twist[2] * seconds;
            self.world.distance_m += twist[0].hypot(twist[1]) * seconds;
        }
        self.apply_command();
        self.loop_state
            .tick(PERIOD)
            .map_err(|error| error.to_string())?;
        if let Some(trace) = self.loop_state.policy_trace() {
            append_policy_trace(
                self.config_dir.as_deref(),
                self.loop_state.status().tick,
                trace,
            );
        }
        Ok(self.status())
    }

    fn apply_command(&mut self) {
        let twist = self.effective_twist();
        self.loop_state.set_policy_command(Command {
            twist,
            head: self.input.head,
            body: duck_control::obs::BodyPose {
                z: self.input.body[0],
                roll: self.input.body[1],
                pitch: self.input.body[2],
            },
        });
    }

    fn effective_twist(&self) -> [f64; 3] {
        let mut twist = self.input.twist;
        if let Some(command) = &self.policy_command {
            match command.encoding.as_deref() {
                Some("phase") => {
                    let period = command
                        .period_s
                        .filter(|period| *period > 0.0)
                        .unwrap_or(4.0);
                    let phase = self.loop_state.status().tick as f64 * 0.02 / period;
                    let angle = std::f64::consts::TAU * phase;
                    twist = [angle.cos(), angle.sin(), 0.0];
                }
                Some("posture_flag") => {
                    twist[0] = command.stand.unwrap_or(0.0);
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
    use super::PreviewPolicyLoop;
    use servo::ServoModel;
    use std::time::{SystemTime, UNIX_EPOCH};

    #[test]
    fn preview_loop_uses_shared_policy_loop_without_hardware() {
        let mut loop_state =
            PreviewPolicyLoop::open(ServoModel::Hd1910, None, "test-machine", "测试机器").unwrap();
        assert_eq!(loop_state.status().phase, crate::types::LoopPhase::Homing);
        for _ in 0..5 {
            loop_state.tick().unwrap();
        }
        let status = loop_state.status();
        assert_eq!(status.phase, crate::types::LoopPhase::Ready);
        assert_eq!(status.positions.len(), 15);
        assert_eq!(status.targets.len(), 15);
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
        let log = std::fs::read_to_string(root.join("logs").join("control-loop.log")).unwrap();
        assert!(log.contains("loop_open machine_id=virtual-1"));
        assert!(log.contains("servo_read id=1"));
        let _ = std::fs::remove_dir_all(root);
    }
}
