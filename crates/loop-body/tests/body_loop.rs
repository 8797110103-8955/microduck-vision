mod support;

use std::time::Duration;

use loop_body::{BodyCommand, BodyLoop, LoopAction, LoopConfig, LoopError, LoopPhase};
use servo::ServoModel;
use support::{FakeDriver, binding, loop_with};

#[test]
fn power_homing_ready_drive_hold_and_stop_are_explicit() {
    let mut body_loop = loop_with(FakeDriver::new(15));
    assert_eq!(body_loop.status().phase, LoopPhase::Limp);
    body_loop.send(LoopAction::Power(true)).unwrap();
    body_loop.tick(Duration::from_millis(20)).unwrap();
    assert_eq!(body_loop.status().phase, LoopPhase::Homing);
    body_loop.tick(Duration::from_millis(20)).unwrap();
    assert_eq!(body_loop.status().phase, LoopPhase::Ready);
    body_loop.send(LoopAction::Drive(true)).unwrap();
    let mut targets = vec![0.0; 15];
    targets[0] = 0.2;
    body_loop
        .set_command(BodyCommand {
            session: 0,
            targets,
        })
        .unwrap();
    body_loop.tick(Duration::from_millis(20)).unwrap();
    assert_eq!(body_loop.status().phase, LoopPhase::Driving);
    body_loop.send(LoopAction::Drive(false)).unwrap();
    assert_eq!(body_loop.status().phase, LoopPhase::Holding);
    body_loop.send(LoopAction::Stop).unwrap();
    assert_eq!(body_loop.status().phase, LoopPhase::Stopped);
}

#[test]
fn calibration_direction_and_zero_are_applied_after_body_targets() {
    let mut binding = binding(ServoModel::Hd1910, 15);
    binding.calibration.joints[0].direction = -1;
    binding.calibration.joints[0].zero_offset_rad = 0.1;
    let mut body_loop = BodyLoop::open(
        binding,
        Box::new(FakeDriver::new(15)),
        LoopConfig {
            homing_ticks: 1,
            ..LoopConfig::default()
        },
    )
    .unwrap();
    body_loop.send(LoopAction::Power(true)).unwrap();
    body_loop.tick(Duration::from_millis(20)).unwrap();
    body_loop.send(LoopAction::Drive(true)).unwrap();
    let mut targets = vec![0.0; 15];
    targets[0] = 0.2;
    body_loop
        .set_command(BodyCommand {
            session: 0,
            targets,
        })
        .unwrap();
    body_loop.tick(Duration::from_millis(20)).unwrap();
    assert_eq!(body_loop.status().targets[0], 0.2);
}

#[test]
fn deadman_stops_motion_without_turning_torque_off() {
    let mut body_loop = loop_with(FakeDriver::new(15));
    body_loop.send(LoopAction::Power(true)).unwrap();
    body_loop.tick(Duration::from_millis(20)).unwrap();
    body_loop.tick(Duration::from_millis(20)).unwrap();
    body_loop.send(LoopAction::Drive(true)).unwrap();
    body_loop.send(LoopAction::Drive(false)).unwrap();
    assert_eq!(body_loop.status().phase, LoopPhase::Holding);
    assert!(body_loop.status().torque);
}

#[test]
fn repeated_sensor_failures_degrade_to_hold_and_keep_diagnostic_error() {
    let mut driver = FakeDriver::new(15);
    driver.fail_reads = 2;
    let mut body_loop = loop_with(driver);
    body_loop.send(LoopAction::Power(true)).unwrap();
    assert!(body_loop.tick(Duration::from_millis(20)).is_err());
    assert!(body_loop.tick(Duration::from_millis(20)).is_err());
    assert_eq!(body_loop.status().phase, LoopPhase::Holding);
    assert!(body_loop.status().last_error.is_some());
}

#[test]
fn stale_session_and_nonfinite_target_are_rejected() {
    let mut body_loop = loop_with(FakeDriver::new(15));
    body_loop
        .send(LoopAction::Reconnect { session: 1 })
        .unwrap();
    assert!(matches!(
        body_loop.set_command(BodyCommand {
            session: 0,
            targets: vec![0.0; 15]
        }),
        Err(LoopError::StaleSession { .. })
    ));
    let mut targets = vec![0.0; 15];
    targets[0] = f64::NAN;
    assert!(matches!(
        body_loop.set_command(BodyCommand {
            session: 1,
            targets
        }),
        Err(LoopError::NonFiniteTarget { .. })
    ));
}

#[test]
fn body_range_is_checked_before_a_servo_write() {
    let mut body_loop = loop_with(FakeDriver::new(15));
    body_loop.send(LoopAction::Power(true)).unwrap();
    body_loop.tick(Duration::from_millis(20)).unwrap();
    body_loop.tick(Duration::from_millis(20)).unwrap();
    body_loop.send(LoopAction::Drive(true)).unwrap();
    let mut targets = vec![0.0; 15];
    targets[0] = 99.0;
    body_loop
        .set_command(BodyCommand {
            session: 0,
            targets,
        })
        .unwrap();
    assert!(matches!(
        body_loop.tick(Duration::from_millis(20)),
        Err(LoopError::Body(_))
    ));
}

#[test]
fn zero_gain_is_rejected_at_loop_open() {
    let result = BodyLoop::open(
        binding(ServoModel::Hd1910, 15),
        Box::new(FakeDriver::new(15)),
        LoopConfig {
            gain: 0,
            ..LoopConfig::default()
        },
    );
    assert!(matches!(result, Err(LoopError::ZeroGain)));
}

#[test]
fn policy_failure_holds_the_last_safe_target_with_a_diagnostic() {
    let mut body_loop = loop_with(FakeDriver::new(15));
    body_loop.send(LoopAction::Power(true)).unwrap();
    body_loop.tick(Duration::from_millis(20)).unwrap();
    body_loop.tick(Duration::from_millis(20)).unwrap();
    body_loop.send(LoopAction::Drive(true)).unwrap();
    body_loop.report_policy_error("inference failed");
    let status = body_loop.status();
    assert_eq!(status.phase, LoopPhase::Holding);
    assert_eq!(status.last_error.as_deref(), Some("inference failed"));
}

#[test]
fn reconnect_starts_a_new_homing_session() {
    let mut body_loop = loop_with(FakeDriver::new(15));
    body_loop
        .send(LoopAction::Reconnect { session: 1 })
        .unwrap();
    assert_eq!(body_loop.status().phase, LoopPhase::Reconnecting);
    body_loop.tick(Duration::from_millis(20)).unwrap();
    assert_eq!(body_loop.status().phase, LoopPhase::Homing);
    assert_eq!(body_loop.status().session, 1);
}

#[test]
fn hd1910_default_and_xl330_override_share_the_same_loop_binding() {
    let hd = BodyLoop::open(
        binding(ServoModel::Hd1910, 15),
        Box::new(FakeDriver::new(15)),
        LoopConfig::default(),
    );
    assert!(hd.is_ok());
    let xl = BodyLoop::open(
        binding(ServoModel::Xl330M288, 15),
        Box::new(FakeDriver::new(15)),
        LoopConfig::default(),
    );
    assert!(xl.is_ok());
}
