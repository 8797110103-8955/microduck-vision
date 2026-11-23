use super::{assets, hd_parameters};
use duck_body::{BodyTemplate, FileAssets, ServoParameterSet};
use servo::ServoModel;

#[test]
fn hd1910_is_default_and_xl330_is_an_explicit_compatible_override() {
    let body = BodyTemplate::official(assets(), &FileAssets).unwrap();
    let hd = ServoParameterSet::all_hd1910(&body);
    assert!(hd.validate_for(&body).is_ok());
    let xl = ServoParameterSet::bind_all(&body, ServoModel::Xl330M288);
    assert!(xl.validate_for(&body).is_ok());
}

#[test]
fn mounted_servo_mass_composes_per_slot() {
    let mut body = BodyTemplate::official(assets(), &FileAssets).unwrap();
    for slot in &mut body.actuators {
        slot.mounted_mass_g = Some(1.0);
    }
    let servos = hd_parameters();
    let total = servos.total_mass_g(&body).unwrap();
    assert!((total - (body.physical.mass_g.unwrap() + 15.0)).abs() < 1e-9);
}

#[test]
fn official_link_mass_already_includes_the_actuators() {
    let body = BodyTemplate::official(assets(), &FileAssets).unwrap();
    let servos = hd_parameters();
    assert_eq!(servos.total_mass_g(&body), body.physical.mass_g);
}
