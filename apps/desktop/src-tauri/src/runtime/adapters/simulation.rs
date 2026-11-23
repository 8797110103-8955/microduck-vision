//! Projection between desktop simulation settings and the product runtime.

use duck_body::{BodyProfile, BodyTemplate};
use microduck_body_runtime::models::assets::{AssetBundle, OfficialAssets};
use microduck_body_runtime::models::profile::SimulationTuning;
use microduck_body_runtime::simulation::SimulationSnapshot;
use microduck_body_runtime::{BodyExecutionProfile, Selection};

use crate::types::{SceneFrame, SimulationParameters, SimulationState};

pub(crate) fn resolve(
    selection: Selection,
    simulation: &SimulationParameters,
    template: &BodyTemplate,
) -> Result<BodyExecutionProfile, String> {
    let body = BodyProfile::from_template(template.clone()).map_err(|error| error.to_string())?;
    microduck_body_runtime::resolve_simulation(
        selection,
        &body,
        &OfficialAssets::Embedded,
        tuning(simulation),
    )
}

pub(crate) fn tuning(simulation: &SimulationParameters) -> SimulationTuning {
    SimulationTuning {
        damping: simulation.damping,
        friction_loss: simulation.friction_loss,
        armature: simulation.armature,
        kp: simulation.kp,
        force_range: simulation.force_range,
    }
}

pub(crate) fn official_sources() -> Result<Vec<(String, Vec<u8>)>, String> {
    let assets = AssetBundle::read(&OfficialAssets::Embedded, &["robot_walk.xml", "scene.xml"])
        .map_err(|error| error.to_string())?;
    Ok(assets
        .files()
        .map(|(path, bytes)| (path.to_owned(), bytes.to_vec()))
        .collect())
}

impl From<SimulationSnapshot> for SimulationState {
    fn from(state: SimulationSnapshot) -> Self {
        Self {
            physics_step: state.physics_step,
            time: state.time,
            trunk_position: state.trunk_position,
            trunk_quaternion: state.trunk_quaternion,
            motor_positions: state.motor_positions,
            passive_positions: state.passive_positions,
            base_velocity: state.base_velocity,
            contacts: state.contacts,
            motor_torques: state.motor_torques,
            friction_budgets: state.friction_budgets,
            effective_voltage: state.effective_voltage,
            frame: SceneFrame {
                bodies: state.frame.bodies,
                markers: state.frame.markers,
            },
        }
    }
}

#[cfg(test)]
mod tests {
    use loop_body::ControlBackend;
    use servo::ServoError;

    #[test]
    fn simulation_gain_rejection_keeps_the_control_session_running() {
        let mut registry = loop_body::BackendRegistry::default();
        let key = crate::types::ServoModelKey::Xl330;
        let (snapshot, _, _) = crate::test_support::register_simulation(
            &mut registry,
            "fixed-gain",
            crate::test_support::simulation_selection(microduck_body_runtime::ModelKind::Xl330),
            &crate::platform::helpers::default_simulation(key),
            &crate::runtime::adapters::body_assets::test_template(),
        )
        .unwrap();
        registry.select("fixed-gain").unwrap();
        let mut session = registry
            .open("fixed-gain", loop_body::BackendAccess::Control)
            .unwrap();
        assert_eq!(
            session.gain_capability(),
            loop_body::GainCapability::FixedModel
        );
        assert_eq!(session.set_gain(32), Err(ServoError::UnsupportedGain));
        session.set_torque(true).unwrap();
        session
            .advance_execution(4, std::time::Duration::from_millis(5))
            .unwrap();
        assert_eq!(
            snapshot
                .lock()
                .unwrap()
                .state
                .as_ref()
                .unwrap()
                .physics_step,
            4
        );
        session.set_torque(false).unwrap();
    }
}
