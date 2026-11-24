use microduck_body_runtime::models::assets::{OfficialAssets, load_official_body};
use microduck_body_runtime::models::resolve::defaults;
use microduck_body_runtime::{ExecutionKind, ModelKind, Selection, resolve_simulation};

#[test]
fn live_gain_tuning_scales_the_hd_motor_once_and_preserves_nominal_compensation() {
    let source = OfficialAssets::Embedded;
    let body = load_official_body(&source).unwrap();
    let mut model_kind = ModelKind::Hd1910Compatible;
    let resolve = |kind, tuning| {
        resolve_simulation(
            Selection {
                execution_kind: ExecutionKind::Simulation,
                model_kind: kind,
                servo_model: ModelKind::Hd1910.servo_model(),
            },
            &body,
            &source,
            tuning,
        )
        .unwrap()
    };
    let default = defaults(model_kind);
    let stock = resolve(model_kind, default);
    let tuned = resolve(
        model_kind,
        microduck_body_runtime::models::profile::SimulationTuning {
            kp: default.kp * 2.0,
            ..default
        },
    );
    model_kind = ModelKind::Hd1910;
    let native = resolve(model_kind, default);
    assert_ne!(stock.execution_fingerprint, tuned.execution_fingerprint);
    assert_ne!(stock.execution_fingerprint, native.execution_fingerprint);
    let stock = stock.simulation.unwrap();
    let tuned = tuned.simulation.unwrap();
    let native = native.simulation.unwrap();
    assert_eq!(native.response.position_gain_scale, 1.0);
    assert!(stock.response.position_gain_scale < 1.0);
    assert_eq!(stock.response, tuned.response);
    assert_eq!(tuned.motor.kp, stock.motor.kp * 2.0);
    assert_eq!(stock.nominal_motor, tuned.nominal_motor);
    assert_eq!(stock.source_assets, native.source_assets);
    assert_eq!(stock.response.delay_steps, [3, 6]);
    assert!(stock.response.encoder_through_backlash);
    assert!(stock.motor.max_pwm < native.motor.max_pwm);
    assert_eq!(stock.nominal_motor, native.nominal_motor);
}
