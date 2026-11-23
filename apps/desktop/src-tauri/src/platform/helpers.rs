//! Small pure conversions shared by the client command layer and state.

use crate::types::ServoModelKey;
use servo::ServoModel;

use crate::types::{
    JointRangeState as FrontendJointRangeState, LoopPhase as FrontendLoopPhase,
    LoopStatus as FrontendLoopStatus, SimulationParameters, WorldState as FrontendWorldState,
};

pub(crate) fn new_machine_id(kind: &str) -> String {
    format!("{kind}-{}", uuid::Uuid::new_v4())
}

pub(crate) fn default_joint_bindings() -> Vec<crate::types::VirtualJointBinding> {
    duck_control::model::JOINT_NAMES
        .iter()
        .zip(duck_control::model::JOINT_IDS)
        .map(|(slot, id)| crate::types::VirtualJointBinding {
            slot_id: (*slot).into(),
            physical_id: id,
        })
        .collect()
}

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
    let kind = match key {
        ServoModelKey::Xl330 => microduck_body_runtime::ModelKind::Xl330,
        ServoModelKey::Hd1910 => microduck_body_runtime::ModelKind::Hd1910Compatible,
    };
    let tuning = microduck_body_runtime::models::resolve::defaults(kind);
    SimulationParameters {
        damping: tuning.damping,
        friction_loss: tuning.friction_loss,
        armature: tuning.armature,
        kp: tuning.kp,
        force_range: tuning.force_range,
        world_position: [0.0; 3],
    }
}

impl From<loop_body::LoopStatus> for FrontendLoopStatus {
    fn from(status: loop_body::LoopStatus) -> Self {
        Self {
            has_execution: true,
            active_body: None,
            simulation: None,
            runtime_state: crate::types::RuntimeState::Running,
            read_sources: Vec::new(),
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
            world: FrontendWorldState {
                position: [0.0, 0.0],
                yaw: 0.0,
                distance_m: 0.0,
            },
            sensor_failures: status.sensor_failures,
            last_error: status.last_error,
            joint_range_states: status
                .joint_range_states
                .into_iter()
                .map(|state| FrontendJointRangeState {
                    slot_id: state.slot_id,
                    target_rad: state.target_rad,
                    min_rad: state.min_rad,
                    max_rad: state.max_rad,
                    below_min: state.below_min,
                    above_max: state.above_max,
                })
                .collect(),
            runtime_generation: 0,
            execution_generation: status.session,
            config_revision: 0,
        }
    }
}

impl From<&loop_body::ReadOnlySource> for crate::types::ReadSourceStatus {
    fn from(source: &loop_body::ReadOnlySource) -> Self {
        let sample = source.latest();
        Self {
            valid_positions: sample.map_or_else(Vec::new, |sample| sample.valid_positions.clone()),
            valid_velocities: sample
                .map_or_else(Vec::new, |sample| sample.valid_velocities.clone()),
            id: source.id().into(),
            generation: source.generation(),
            sequence: source.sequence(),
            sampled_at: sample.map_or(0, |sample| sample.sampled_at.0),
            age_ms: source.sample_age().map(|age| age.as_secs_f64() * 1000.0),
            health: match source.health() {
                loop_body::SourceHealth::Starting => crate::types::ReadSourceHealth::Starting,
                loop_body::SourceHealth::Fresh => crate::types::ReadSourceHealth::Fresh,
                loop_body::SourceHealth::Stale => crate::types::ReadSourceHealth::Stale,
                loop_body::SourceHealth::Offline => crate::types::ReadSourceHealth::Offline,
            },
            positions: sample.map_or_else(Vec::new, |sample| sample.positions.clone()),
            velocities: sample.map_or_else(Vec::new, |sample| sample.velocities.clone()),
            last_error: source.last_error().map(str::to_owned),
        }
    }
}

#[cfg(test)]
pub(crate) fn test_directory() -> std::path::PathBuf {
    std::path::PathBuf::from(env!("DUCK_DESKTOP_TEST_DIR"))
}

#[cfg(test)]
mod tests {
    use super::{default_simulation, model, model_key};
    use crate::types::ServoModelKey;
    use servo::ServoModel;

    #[test]
    fn model_helpers_keep_catalog_keys_and_profiles_aligned() {
        assert_eq!(model(&ServoModelKey::Hd1910), ServoModel::Hd1910);
        assert_eq!(model(&ServoModelKey::Xl330), ServoModel::Xl330M288);
        assert_eq!(model_key("hd1910").unwrap(), ServoModelKey::Hd1910);
        assert_eq!(model_key("xl330").unwrap(), ServoModelKey::Xl330);
        assert!(model_key("unknown").is_err());
    }

    #[test]
    fn default_simulation_is_finite_for_each_builtin_model() {
        for key in [ServoModelKey::Hd1910, ServoModelKey::Xl330] {
            let simulation = default_simulation(key);
            assert!(simulation.damping.is_finite());
            assert!(simulation.armature.is_finite());
            assert!(simulation.kp.is_finite());
            assert!(simulation.force_range.is_finite());
        }
    }
}
