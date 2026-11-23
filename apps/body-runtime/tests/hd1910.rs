use microduck_body_runtime::models::assets::{OfficialAssets, load_official_body};
use microduck_body_runtime::models::parameter_sources::{ParameterClass, RealMachineMapping};
use microduck_body_runtime::models::resolve::defaults;
use microduck_body_runtime::models::scene_recipe::SceneOperation;
use microduck_body_runtime::{ExecutionKind, ModelKind, Selection, resolve_simulation};

#[test]
fn native_hd_uses_its_own_response_with_unverified_link_inertials() {
    let source = OfficialAssets::Embedded;
    let body = load_official_body(&source).unwrap();
    let model = ModelKind::Hd1910;
    let selection = Selection {
        execution_kind: ExecutionKind::Simulation,
        model_kind: model,
        servo_model: model.servo_model(),
    };
    let profile = resolve_simulation(selection, &body, &source, defaults(model)).unwrap();
    let simulation = profile.simulation.as_ref().unwrap();
    assert_eq!(simulation.scenario.scene_xml, Some("scene.xml"));
    assert_eq!(simulation.scenario.robot_xml, "robot_groundcontact.xml");
    assert_eq!(
        simulation.scenario.derived_scene_operations,
        [SceneOperation::ConfigureBamActuators]
    );
    assert_eq!(simulation.response.position_gain_scale, 1.0);
    assert_eq!(simulation.response.back_emf_scale, 1.0);
    assert_eq!(simulation.response.friction_scale, 1.0);
    assert_eq!(simulation.response.delay_steps, [0, 0]);
    assert!(!simulation.response.encoder_through_backlash);
    assert_eq!(simulation.compatibility_revision, None);
    for parameter in ["link_mass", "link_inertia"] {
        let entry = profile
            .parameter_sources
            .iter()
            .find(|source| source.parameter == parameter)
            .unwrap();
        assert_eq!(entry.class, ParameterClass::SimulationApproximation);
        assert_eq!(entry.real_machine_mapping, RealMachineMapping::Unverified);
        assert!(entry.assumption.is_some());
    }
}
