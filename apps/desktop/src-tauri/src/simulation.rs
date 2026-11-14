//! Virtual preview driver and desktop adapter for the shared policy loop.
//!
//! Policy inference and control-loop scheduling live in `loop-body`. This module only supplies a
//! memory-backed servo driver and converts the crate status into the Tauri status shape.

use std::path::Path;
use std::time::Duration;

use duck_body::BodyTemplate;
use duck_control::model::DEFAULT_POSITION;
use duck_control::obs::Command;
use loop_body::{
    BodyRuntimeView, Calibration, JointCalibration, LoopAction, LoopBinding, LoopConfig,
    LoopPhase as BodyLoopPhase, PolicyLoop, ServoBinding,
};
use servo::{ServoError, ServoLoopDriver, ServoModel, ServoPositionSample, ServoPositionTargets};

use crate::types::{LoopPhase, LoopStatus};

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
}

impl PreviewPolicyLoop {
    pub(crate) fn open(model: ServoModel) -> Result<Self, String> {
        let template = BodyTemplate::official().map_err(|error| error.to_string())?;
        let binding = LoopBinding {
            body: BodyRuntimeView::new(template.clone()).map_err(|error| error.to_string())?,
            servos: ServoBinding::all_slots(&template, model),
            calibration: Calibration {
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
                    })
                    .collect(),
            },
        };
        let mut loop_state = PolicyLoop::open(
            binding,
            Box::new(PreviewDriver {
                positions: DEFAULT_POSITION.to_vec(),
            }),
            LoopConfig::default(),
        )
        .map_err(|error| error.to_string())?;
        loop_state.set_policy_command(Command {
            twist: [0.15, 0.0, 0.0],
            ..Command::default()
        });
        let mut preview = Self { loop_state };
        preview
            .loop_state
            .send(LoopAction::Power(true))
            .map_err(|error| error.to_string())?;
        Ok(preview)
    }

    pub(crate) fn select_policy(&mut self, path: &Path, _id: String) -> Result<(), String> {
        self.loop_state
            .select_policy(path)
            .map_err(|error| error.to_string())
    }

    pub(crate) fn tick(&mut self) -> Result<LoopStatus, String> {
        self.loop_state
            .tick(PERIOD)
            .map_err(|error| error.to_string())?;
        Ok(self.status())
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

    #[test]
    fn preview_loop_uses_shared_policy_loop_without_hardware() {
        let mut loop_state = PreviewPolicyLoop::open(ServoModel::Hd1910).unwrap();
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
}
