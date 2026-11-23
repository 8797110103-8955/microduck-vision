use microduck_body_runtime::models::assets::{OfficialAssets, load_official_body};
use microduck_body_runtime::{
    ExecutionKind, ModelKind, PhysicalGains, Selection, resolve_physical,
};

#[test]
fn physical_selection_distinguishes_device_defaults_and_rejects_compensation() {
    let body = load_official_body(&OfficialAssets::Embedded).unwrap();
    let resolve = |model_kind: ModelKind| {
        resolve_physical(
            Selection {
                execution_kind: ExecutionKind::Physical,
                model_kind,
                servo_model: model_kind.servo_model(),
            },
            &body,
        )
    };
    assert_eq!(
        resolve(ModelKind::Xl330).unwrap().physical_gains,
        Some(PhysicalGains::Xl330 { p: 200, i: 0, d: 0 })
    );
    assert_eq!(
        resolve(ModelKind::Hd1910).unwrap().physical_gains,
        Some(PhysicalGains::Hd1910 { p: 32 })
    );
    assert_eq!(
        resolve(ModelKind::Hd1910Compatible).unwrap_err(),
        "HD1910 compatibility is only available in simulation"
    );
}
