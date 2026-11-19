mod support;

use loop_body::{ReadOnlySource, ReadOnlySourceConfig, SourceHealth};
use servo::{ServoError, ServoLoopDriver, ServoModel, ServoPositionSample, ServoPositionTargets};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, mpsc};
use std::time::{Duration, Instant};
use support::{FakeDriver, binding};

struct BlockingReader {
    started: mpsc::Sender<()>,
    release: mpsc::Receiver<()>,
}

impl ServoLoopDriver for BlockingReader {
    fn read_positions(&mut self, _: &[u8]) -> Result<ServoPositionSample, ServoError> {
        self.started.send(()).unwrap();
        self.release.recv().unwrap();
        Ok(ServoPositionSample {
            positions: vec![0.2; 15],
            velocities: vec![0.0; 15],
        })
    }
    fn write_positions(&mut self, _: &[u8], _: &ServoPositionTargets) -> Result<(), ServoError> {
        panic!("read-only task wrote targets")
    }
    fn set_torque(&mut self, _: bool) -> Result<(), ServoError> {
        panic!("read-only task changed torque")
    }
    fn set_gain(&mut self, _: u16) -> Result<(), ServoError> {
        panic!("read-only task changed gain")
    }
    fn reboot(&mut self, _: u8) -> Result<(), ServoError> {
        panic!("read-only task rebooted hardware")
    }
}

fn config(timeout: Duration) -> ReadOnlySourceConfig {
    ReadOnlySourceConfig {
        id: "physical-test".into(),
        sample_period: Duration::from_secs(1),
        timeout,
        max_age: Duration::from_millis(10),
        cache_capacity: 1,
    }
}

fn blocking_source() -> (ReadOnlySource, mpsc::Receiver<()>, mpsc::Sender<()>) {
    let (started_tx, started_rx) = mpsc::channel();
    let (release_tx, release_rx) = mpsc::channel();
    (
        ReadOnlySource::open(
            binding(ServoModel::Hd1910, 15),
            Box::new(BlockingReader {
                started: started_tx,
                release: release_rx,
            }),
        ),
        started_rx,
        release_tx,
    )
}

#[test]
fn blocking_read_does_not_block_snapshot_collection_and_cancel_retains_ownership() {
    let (mut source, started, release) = blocking_source();
    source.start(config(Duration::from_millis(20))).unwrap();
    started.recv_timeout(Duration::from_secs(1)).unwrap();
    let before = Instant::now();
    assert!(source.read().is_err());
    assert!(before.elapsed() < Duration::from_millis(100));
    assert!(
        source
            .stop()
            .unwrap_err()
            .contains("resource remains owned")
    );
    release.send(()).unwrap();
    let deadline = Instant::now() + Duration::from_secs(1);
    while source.stop().is_err() {
        assert!(Instant::now() < deadline);
    }
    assert!(
        source.latest().is_none(),
        "cancelled result cannot enter the cache"
    );
}

#[test]
fn late_read_is_discarded_and_marked_offline() {
    let (mut source, started, release) = blocking_source();
    source.start(config(Duration::from_millis(5))).unwrap();
    started.recv_timeout(Duration::from_secs(1)).unwrap();
    std::thread::sleep(Duration::from_millis(15));
    release.send(()).unwrap();
    let deadline = Instant::now() + Duration::from_secs(1);
    while source.health() != SourceHealth::Offline {
        assert!(Instant::now() < deadline);
        std::thread::yield_now();
    }
    assert!(
        source
            .read()
            .unwrap_err()
            .to_string()
            .contains("late sample discarded")
    );
    assert_eq!(source.sequence(), 0);
    assert!(source.latest().is_none());
    source.stop().unwrap();
}

#[test]
fn successful_source_keeps_source_identity_and_exposes_stale_age_without_writes() {
    let driver = FakeDriver::new(15);
    let events = driver.event_log();
    let mut source = ReadOnlySource::open(binding(ServoModel::Hd1910, 15), Box::new(driver));
    source.start(config(Duration::from_secs(1))).unwrap();
    let deadline = Instant::now() + Duration::from_secs(1);
    while source.read().is_err() {
        assert!(Instant::now() < deadline);
        std::thread::yield_now();
    }
    let sample = source.latest().unwrap();
    assert_eq!(sample.source.as_deref(), Some("physical-test"));
    assert_eq!(sample.sequence, 1);
    std::thread::sleep(Duration::from_millis(20));
    assert_eq!(source.health(), SourceHealth::Stale);
    source.stop().unwrap();
    assert_eq!(*events.lock().unwrap(), ["read"]);
}

#[test]
fn cancellation_reaches_a_backend_transaction_boundary_before_join() {
    struct CancellableReader {
        cancel: Option<Arc<AtomicBool>>,
        started: mpsc::Sender<()>,
    }
    impl ServoLoopDriver for CancellableReader {
        fn set_read_cancel(&mut self, cancel: Arc<AtomicBool>) {
            self.cancel = Some(cancel);
        }
        fn read_positions(&mut self, _: &[u8]) -> Result<ServoPositionSample, ServoError> {
            self.started.send(()).unwrap();
            while !self.cancel.as_ref().unwrap().load(Ordering::Acquire) {
                std::thread::yield_now();
            }
            Err(ServoError::Bus("cancelled".into()))
        }
        fn write_positions(
            &mut self,
            _: &[u8],
            _: &ServoPositionTargets,
        ) -> Result<(), ServoError> {
            panic!("write")
        }
        fn set_torque(&mut self, _: bool) -> Result<(), ServoError> {
            panic!("torque")
        }
        fn set_gain(&mut self, _: u16) -> Result<(), ServoError> {
            panic!("gain")
        }
        fn reboot(&mut self, _: u8) -> Result<(), ServoError> {
            panic!("reboot")
        }
    }
    let (started, receiver) = mpsc::channel();
    let mut source = ReadOnlySource::open(
        binding(ServoModel::Hd1910, 15),
        Box::new(CancellableReader {
            cancel: None,
            started,
        }),
    );
    source.start(config(Duration::from_secs(1))).unwrap();
    receiver.recv_timeout(Duration::from_secs(1)).unwrap();
    source.stop().unwrap();
    assert!(source.latest().is_none());
}
