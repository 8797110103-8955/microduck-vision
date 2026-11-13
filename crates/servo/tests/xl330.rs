use duck_control::io::{FakeIo, JointTargets, Result as VendorResult, RobotIo};
use duck_control::model::NUM_JOINTS;
use rustypot::bus::Bus;
use rustypot::servo::dynamixel::xl330::Xl330Controller;
use rustypot::servo::dynamixel::xl330::{DEFINITION, INFO};
use rustypot::servo::dynamixel::{xl330, xl430};
use rustypot::servo::feetech::sts3215;
use rustypot::servo::{Encoding, RegisterError, ServoKind, WordOrder, find_model, scan_timeout};
use servo::{
    MemoryXl330Bus, RawBus, RustypotXl330Bus, ServoCommand, ServoDriver, ServoError,
    ServoLoopDriver, ServoPositionTargets, ServoTelemetry, XL330, XL330_CAPABILITIES,
    XL330_DEFAULT_BAUD, XL330_MODEL_NAME, XL330_MODEL_NUMBER, XL330_RESOLUTION, Xl330Driver,
    decode_register, encode_register, position_rad_from_raw, position_raw_from_rad, register,
    registers, velocity_rad_s_from_raw,
};

use crate::support::ScriptedPort;
use std::time::Duration;

#[test]
#[ignore = "signature-first stage: vendor contract test is not executed yet"]
fn servo_public_keys_and_vendor_types_compile() {
    let _ = ServoTelemetry;
    let _ = ServoCommand;
    let _ = XL330;
    let _ = XL330_CAPABILITIES;
    let _ = XL330_MODEL_NAME;
    let _ = XL330_MODEL_NUMBER;
    let _ = XL330_DEFAULT_BAUD;
    let _ = XL330_RESOLUTION;
}

#[derive(Default)]
struct DriverBus {
    goal: Vec<u8>,
    torque: Vec<u8>,
    gain: Vec<u8>,
}

impl RawBus for DriverBus {
    fn read_raw(&mut self, _id: u8, _addr: u8, _len: u8) -> Result<Vec<u8>, ServoError> {
        Ok(Vec::new())
    }

    fn write_raw(&mut self, _id: u8, addr: u8, data: &[u8]) -> Result<(), ServoError> {
        match addr {
            64 => self.torque = data.to_vec(),
            84 => self.gain = data.to_vec(),
            _ => {}
        }
        Ok(())
    }

    fn sync_read_raw(
        &mut self,
        ids: &[u8],
        _addr: u8,
        _len: u8,
    ) -> Result<Vec<Vec<u8>>, ServoError> {
        Ok(ids
            .iter()
            .map(|_| [0, 0, 0, 0, 0, 8, 0, 0].to_vec())
            .collect())
    }

    fn sync_write_raw(
        &mut self,
        _ids: &[u8],
        _addr: u8,
        data: &[Vec<u8>],
    ) -> Result<(), ServoError> {
        self.goal = data.first().cloned().unwrap_or_default();
        Ok(())
    }

    fn ping(&mut self, _id: u8) -> Result<bool, ServoError> {
        Ok(true)
    }

    fn reboot(&mut self, _id: u8) -> Result<(), ServoError> {
        Ok(())
    }
}

#[test]
fn xl330_driver_uses_vendor_register_layout_for_loop_io() {
    let mut driver = Xl330Driver::new(DriverBus::default()).with_ids([1]);
    let sample = ServoLoopDriver::read_positions(&mut driver, &[1]).unwrap();
    assert_eq!(sample.positions.len(), 1);
    assert_eq!(sample.velocities, vec![0.0]);
    ServoLoopDriver::write_positions(
        &mut driver,
        &[1],
        &ServoPositionTargets {
            positions: vec![0.0],
        },
    )
    .unwrap();
    ServoLoopDriver::set_torque(&mut driver, true).unwrap();
    ServoLoopDriver::set_gain(&mut driver, 200).unwrap();
    let bus = driver.into_inner();
    assert_eq!(bus.goal, vec![0, 8, 0, 0]);
    assert_eq!(bus.torque, vec![1]);
    assert_eq!(bus.gain, vec![200, 0]);
}

#[test]
fn xl330_memory_bus_round_trips_vendor_registers_and_presence() {
    let mut bus = MemoryXl330Bus::with_servos(&[1]);
    assert!(bus.ping(1).unwrap());
    assert!(!bus.ping(2).unwrap());
    assert_eq!(bus.register(1, "present_position"), 2048);
    bus.set_register(1, "present_position", 2500).unwrap();
    assert_eq!(bus.register(1, "present_position"), 2500);
    assert_eq!(bus.read_raw(1, 132, 4).unwrap(), vec![196, 9, 0, 0]);
    assert_eq!(bus.read_raw(2, 132, 4), Err(ServoError::MissingServo(2)));
}

#[test]
#[ignore = "signature-first stage: vendor contract test is not executed yet"]
fn servo_driver_signatures_accept_vendor_adapter_shape() {
    fn accepts_driver<D: ServoDriver + ?Sized>(_driver: &mut D) {}
    fn accepts_raw_bus<B: RawBus>(_bus: &mut B) {}
    fn read_telemetry<D: ServoDriver + ?Sized>(
        driver: &mut D,
    ) -> Result<ServoTelemetry, ServoError> {
        driver.read_telemetry(0)
    }

    let _new: fn(()) -> Xl330Driver<()> = Xl330Driver::new;
    let _command = ServoCommand;
    let _ = accepts_driver::<dyn ServoDriver>;
    let _ = accepts_raw_bus::<RustypotXl330Bus>;
    let _ = read_telemetry::<dyn ServoDriver>;
}

#[test]
#[ignore = "signature-first stage: vendor contract test is not executed yet"]
fn vendor_xl330_definition_and_conversions_are_the_protocol_baseline() {
    let definition = rustypot::servo::dynamixel::xl330::DEFINITION;
    assert_eq!(definition.protocol, 2);
    assert_eq!(definition.info.resolution, Some(4096));
    assert!(definition.register("goal_position").is_some());
    assert!(definition.register("present_position").is_some());

    let _ = register("goal_position");
    let _ = registers();
    let _ = encode_register("goal_position", 2048);
    let _ = decode_register("goal_position", &[0, 8, 0, 0]);
    let _ = position_rad_from_raw(2048);
    let _ = position_raw_from_rad(0.0);
    let _ = velocity_rad_s_from_raw(1);
}

#[test]
#[ignore = "signature-first stage: vendor contract test is not executed yet"]
fn vendor_fake_io_preserves_the_control_loop_contract() -> VendorResult<()> {
    let mut io = FakeIo::new();
    let mut positions = [0.0; NUM_JOINTS];
    positions[0] = 0.25;
    io.write(&JointTargets::new(positions))?;
    assert_eq!(io.read()?.positions, positions);
    Ok(())
}

#[test]
#[ignore = "signature-first stage: vendor controller path is not executed yet"]
fn vendor_xl330_controller_protocol_two_path_compiles() {
    let controller = Xl330Controller::new()
        .with_protocol_v2()
        .with_fast_sync_read();
    let _controller = controller;
}

#[test]
fn official_xl330_definition_facts_are_reused() {
    assert_eq!(DEFINITION.protocol, 2);
    assert_eq!(INFO.resolution, Some(4096));
    assert_eq!(INFO.word_order, WordOrder::Little);
    const {
        assert!(INFO.supports_sync_read);
        assert!(INFO.supports_broadcast_ping);
    }
    assert_eq!(INFO.factory_baudrate, Some(57_600));
    assert_eq!(INFO.homing_offset_sign, Some(1));
    assert_eq!(DEFINITION.register("goal_position").unwrap().addr, 116);
    assert_eq!(DEFINITION.register("present_position").unwrap().addr, 132);
}

#[test]
fn official_xl330_register_encodings_are_reused() {
    assert_eq!(
        DEFINITION.register("goal_position").unwrap().encoding,
        Encoding::TwosComplement
    );
    assert_eq!(
        DEFINITION.register("present_current").unwrap().encoding,
        Encoding::TwosComplement
    );
    assert_eq!(DEFINITION.register("torque_enable").unwrap().size, 1);
    assert_eq!(
        DEFINITION.register("present_input_voltage").unwrap().size,
        2
    );
}

#[test]
fn official_xl330_bus_torque_error_semantics_are_reused() {
    let refused = vec![
        0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x04, 0x00, 0x55, 0x07, 0xB0, 0x8C,
    ];
    let ok = vec![
        0xFF, 0xFF, 0xFD, 0x00, 0x02, 0x04, 0x00, 0x55, 0x00, 0x29, 0x0C,
    ];
    let (port, written) = ScriptedPort::new(vec![refused, ok]);
    let mut bus = Bus::new(
        Box::new(port),
        std::collections::BTreeMap::from([(1, DEFINITION), (2, DEFINITION)]),
    );

    let failed = bus.set_torque(&[1, 2], false, 2);
    assert_eq!(failed.keys().copied().collect::<Vec<_>>(), [1]);
    assert!(
        failed[&1]
            .downcast_ref::<rustypot::StatusPacketError>()
            .is_some()
    );
    assert_eq!(
        written
            .lock()
            .unwrap()
            .iter()
            .map(|packet| packet[4])
            .collect::<Vec<_>>(),
        [1, 2]
    );
}

#[test]
fn official_xl330_bus_sync_write_semantics_are_reused() {
    let (port, written) = ScriptedPort::new(Vec::new());
    let mut bus = Bus::new(
        Box::new(port),
        std::collections::BTreeMap::from([(1, DEFINITION), (2, DEFINITION)]),
    );

    bus.sync_write_register(&[1, 2], "goal_position", &[100, 200])
        .unwrap();
    let written = written.lock().unwrap();
    assert_eq!(written.len(), 1);
    assert_eq!(&written[0][..4], &[0xFF, 0xFF, 0xFD, 0x00]);
    assert_eq!(written[0][4], 0xFE);
    assert_eq!(written[0][7], 0x83);
    assert_eq!(&written[0][8..12], &[0x74, 0x00, 0x04, 0x00]);
    assert_eq!(written[0][12], 1);
    assert_eq!(written[0][17], 2);
}

#[test]
fn official_xl330_model_registry_semantics_are_reused() {
    assert_eq!(find_model("xl330-m288"), Some((1200, xl330::DEFINITION)));
    assert!(matches!(
        ServoKind::try_from(1200),
        Ok(ServoKind::dynamixel_XL330M288)
    ));
    assert!(ServoKind::try_from(2307).is_err());
}

#[test]
fn official_xl330_scan_timeout_semantics_are_reused() {
    assert_eq!(scan_timeout(1_000_000), Duration::from_millis(5));
    assert_eq!(scan_timeout(57_600), Duration::from_micros(5_555));
}

#[test]
fn official_xl330_bus_rejects_invalid_group_values_before_writing() {
    let (port, written) = ScriptedPort::new(Vec::new());
    let mut bus = Bus::new(
        Box::new(port),
        std::collections::BTreeMap::from([
            (1, xl330::DEFINITION),
            (2, xl330::DEFINITION),
            (11, sts3215::DEFINITION),
        ]),
    );

    let error = bus
        .sync_write_register(&[1, 2, 11], "goal_position", &[100, 200, 70_000])
        .unwrap_err();
    assert!(matches!(
        error.downcast_ref::<RegisterError>(),
        Some(RegisterError::OutOfRange { .. })
    ));
    assert!(written.lock().unwrap().is_empty());
}

#[test]
fn official_xl330_bus_groups_protocol_two_layouts() {
    let (port, written) = ScriptedPort::new(Vec::new());
    let mut bus = Bus::new(
        Box::new(port),
        std::collections::BTreeMap::from([
            (1, sts3215::DEFINITION),
            (2, xl330::DEFINITION),
            (3, xl430::DEFINITION),
        ]),
    );

    bus.sync_write_register(&[2, 1, 3], "goal_position", &[100, 200, 300])
        .unwrap();
    let written = written.lock().unwrap();
    assert_eq!(written.len(), 2);
    assert_eq!(&written[0][..4], &[0xFF, 0xFF, 0xFD, 0x00]);
    assert_eq!(written[0][7], 0x83);
    assert_eq!([written[0][12], written[0][17]], [2, 3]);
    assert_eq!(&written[1][..3], &[0xFF, 0xFF, 0xFE]);
    assert_eq!(written[1][4], 0x83);
    assert_eq!(&written[1][7..10], &[0x01, 0xC8, 0x00]);
}
