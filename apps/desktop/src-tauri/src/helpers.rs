//! Small pure conversions shared by the client command layer and state.

use duck_body::ServoModelKey;
use servo::ServoModel;

use crate::types::{
    LoopPhase as FrontendLoopPhase, LoopStatus as FrontendLoopStatus, SimulationParameters,
};

pub(crate) fn model(key: &ServoModelKey) -> ServoModel {
    match key {
        ServoModelKey::Hd1910 => ServoModel::Hd1910,
        ServoModelKey::Xl330 => ServoModel::Xl330M288,
    }
}

pub(crate) fn model_key(key: &str) -> Result<ServoModelKey, String> {
    match key {
        "hd1910" => Ok(ServoModelKey::Hd1910),
        "xl330" => Ok(ServoModelKey::Xl330),
        other => Err(format!("unknown servo model: {other}")),
    }
}

pub(crate) fn default_simulation(key: ServoModelKey) -> SimulationParameters {
    let model = model(&key);
    let profile = duck_body::servo_profiles()
        .into_iter()
        .find(|profile| profile.model == model)
        .expect("built-in servo profile");
    SimulationParameters {
        damping: profile.sim_damping,
        friction_loss: profile.sim_friction_loss,
        armature: profile.sim_armature,
        kp: profile.sim_kp,
        force_range: profile.sim_force_range,
    }
}

impl From<loop_body::LoopStatus> for FrontendLoopStatus {
    fn from(status: loop_body::LoopStatus) -> Self {
        Self {
            phase: match status.phase {
                loop_body::LoopPhase::Limp => FrontendLoopPhase::Limp,
                loop_body::LoopPhase::Homing => FrontendLoopPhase::Homing,
                loop_body::LoopPhase::Ready => FrontendLoopPhase::Ready,
                loop_body::LoopPhase::Driving => FrontendLoopPhase::Driving,
                loop_body::LoopPhase::Holding => FrontendLoopPhase::Holding,
                loop_body::LoopPhase::Stopped => FrontendLoopPhase::Stopped,
                loop_body::LoopPhase::Reconnecting => FrontendLoopPhase::Reconnecting,
                loop_body::LoopPhase::Estop => FrontendLoopPhase::Estop,
            },
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
