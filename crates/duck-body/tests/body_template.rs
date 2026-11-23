use super::assets;
use duck_body::{ActuatorCapability, BodyError, BodyTemplate, FileAssets};

#[test]
fn official_body_has_explicit_fifteen_slot_contract() {
    let body = BodyTemplate::official(assets(), &FileAssets).unwrap();
    assert_eq!(body.control.joint_order.len(), 15);
    assert_eq!(body.joints.len(), 15);
    assert_eq!(body.actuators.len(), 15);
    assert_eq!(body.joints[9].name, "mouth");
    assert!(body.joints[9].scene_joint.is_none());
}

#[test]
fn template_parser_reports_missing_required_fields() {
    let err = BodyTemplate::from_toml("id = \"x\"\n").unwrap_err();
    assert!(matches!(err, BodyError::Toml(_)));
}

#[test]
fn template_validation_reports_missing_actuator_joint() {
    let mut body = BodyTemplate::official(assets(), &FileAssets).unwrap();
    body.actuators[0].joint = "missing".to_owned();
    assert!(matches!(
        body.validate(),
        Err(BodyError::MissingJoint { .. })
    ));
}

#[test]
fn capability_names_are_product_neutral() {
    let body = BodyTemplate::official(assets(), &FileAssets).unwrap();
    assert!(
        body.actuators[0]
            .required_capabilities
            .contains(&ActuatorCapability::PositionControl)
    );
}
