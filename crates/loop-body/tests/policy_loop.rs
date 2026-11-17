mod support;

use std::time::Duration;

use duck_control::{FakeIo, JointTargets, RobotIo};
use loop_body::{LoopAction, LoopConfig, LoopPhase, PolicyLoop};
use servo::ServoModel;
use support::{FakeDriver, binding};

#[test]
fn policy_loop_reuses_the_same_scheduler_before_a_policy_is_selected() {
    let mut policy_loop = PolicyLoop::open(
        binding(ServoModel::Hd1910, 15),
        Box::new(FakeDriver::new(15)),
        LoopConfig::default(),
    )
    .unwrap();
    policy_loop.send(LoopAction::Power(true)).unwrap();
    for _ in 0..5 {
        policy_loop.tick(Duration::from_millis(20)).unwrap();
    }
    assert_eq!(policy_loop.status().phase, LoopPhase::Ready);
    assert_eq!(policy_loop.status().positions.len(), 15);
}

#[test]
fn official_fake_io_remains_a_separate_vendor_control_contract() {
    let mut io = FakeIo::new();
    let targets = JointTargets::new([0.1; 15]);
    io.write(&targets).unwrap();
    assert_eq!(io.positions(), [0.1; 15]);
}

#[test]
fn loop_body_runs_with_test_double_and_reads_before_writing() {
    let driver = FakeDriver::new(15);
    let events = driver.event_log();
    let mut policy_loop = PolicyLoop::open(
        binding(ServoModel::Hd1910, 15),
        Box::new(driver),
        LoopConfig::default(),
    )
    .unwrap();
    policy_loop.send(LoopAction::Power(true)).unwrap();
    policy_loop.tick(Duration::from_millis(20)).unwrap();
    let events = events.lock().unwrap().clone();
    let read = events.iter().position(|event| *event == "read").unwrap();
    let write = events.iter().position(|event| *event == "write").unwrap();
    assert!(
        read < write,
        "adapter wrote before reading sensors: {events:?}"
    );
}
