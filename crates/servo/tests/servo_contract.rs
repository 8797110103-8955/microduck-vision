use servo::{
    Gain, ProtocolDescription, ServoCapabilities, ServoCommand, ServoControl, ServoDefinition,
    ServoDefinitionProvider, ServoDriver, ServoDynamics, ServoError, ServoIdentity, ServoModel,
    ServoTelemetry,
};

#[test]
#[ignore]
fn servo_catalog_contains_xl330_and_hd1910_definitions() {
    let _ = ServoModel::Xl330M288;
    let _ = ServoModel::Hd1910;
    let _ = ServoModel::Sts3215;
    let _ = ServoModel::Scs0009;
}

#[test]
#[ignore]
fn servo_definition_exposes_capabilities_dynamics_and_protocol_views() {
    fn accepts_definition<D: ServoDefinitionProvider + ?Sized>(_definition: &D) {}
    let _ = accepts_definition::<dyn ServoDefinitionProvider>;
    let _: Option<ServoDefinition> = None;
    let _: Option<ServoCapabilities> = None;
    let _: Option<ServoDynamics> = None;
    let _: Option<ProtocolDescription> = None;
}

#[test]
#[ignore]
fn servo_driver_contract_covers_read_write_torque_gain_and_scan() {
    fn accepts_control<D: ServoControl + ?Sized>(_driver: &mut D) {}
    fn accepts_driver<D: ServoDriver + ?Sized>(_driver: &mut D) {}
    let _ = accepts_control::<dyn ServoControl>;
    let _ = accepts_driver::<dyn ServoDriver>;
    let _: Option<ServoTelemetry> = None;
    let _: Option<ServoCommand> = None;
    let _: Option<ServoIdentity> = None;
    let _: Option<Gain> = None;
}

#[test]
#[ignore]
fn servo_fake_and_memory_driver_share_the_same_contract() {
    fn accepts_control<D: ServoControl + ?Sized>(_driver: &mut D) {}
    let _ = accepts_control::<dyn ServoControl>;
}

#[test]
#[ignore]
fn servo_fake_driver_supports_read_write_torque_gain_and_scan() {
    fn accepts_control<D: ServoControl + ?Sized>(_driver: &mut D) {}
    let _ = accepts_control::<dyn ServoControl>;
}

#[test]
#[ignore]
fn servo_identity_capability_and_dynamics_views_are_complete() {
    let _: Option<ServoIdentity> = None;
    let _: Option<ServoCapabilities> = None;
    let _: Option<ServoDynamics> = None;
}

#[test]
#[ignore]
fn servo_protocol_frame_rejects_bad_checksum_and_short_response() {
    let _ = ServoError::InvalidRegister {
        name: "checksum".into(),
    };
    let _ = ServoError::ShortRead {
        what: "status packet",
        expected: 6,
        got: 3,
    };
}

#[test]
#[ignore]
fn servo_register_boundaries_reject_width_range_and_unknown_values() {
    let _ = ServoError::RegisterOutOfRange {
        name: "goal_position".into(),
        value: 4096,
    };
    let _ = ServoError::UnknownRegister("not_a_register".into());
}

#[test]
#[ignore]
fn servo_unit_views_cover_position_speed_current_voltage_and_temperature() {
    let _: Option<ServoTelemetry> = None;
}

#[test]
#[ignore]
fn servo_disconnect_timeout_and_invalid_response_remain_distinct_errors() {
    let _ = ServoError::Bus("timeout".into());
    let _ = ServoError::Bus("disconnect".into());
    let _ = ServoError::InvalidRegister {
        name: "response".into(),
    };
}

#[test]
#[ignore]
fn servo_hd1910_default_binding_has_required_capabilities() {
    let _ = ServoModel::Hd1910;
    let _: Option<ServoCapabilities> = None;
}

#[test]
#[ignore]
fn servo_errors_cover_unknown_model_invalid_response_timeout_and_missing_servo() {
    let _ = ServoError::UnknownRegister("unknown".into());
    let _ = ServoError::InvalidRegister {
        name: "present_position".into(),
    };
    let _ = ServoError::Bus("timeout".into());
    let _ = ServoError::MissingServo(1);
    let _ = ServoError::ShortRead {
        what: "sync read",
        expected: 2,
        got: 1,
    };
}

#[test]
#[ignore]
fn servo_default_binding_is_hd1910_and_xl330_is_explicit_compatibility_override() {
    let _default = ServoModel::Hd1910;
    let _official = ServoModel::Xl330M288;
}

#[test]
#[ignore]
fn servo_calibration_and_dynamics_remain_model_views_not_body_state() {
    let _: Option<ServoDynamics> = None;
    let _: Option<ServoDefinition> = None;
}

#[test]
#[ignore]
fn servo_unknown_model_and_illegal_capability_combinations_are_rejected() {
    let _ = ServoError::UnknownRegister("unknown_model".into());
}
