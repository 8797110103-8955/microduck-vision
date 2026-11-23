use duck_control::io::{FakeIo, JointTargets, Result as VendorResult, RobotIo};
use duck_control::model::NUM_JOINTS;
use rustypot::bus::Bus;
use rustypot::servo::dynamixel::xl330::Xl330Controller;
use rustypot::servo::dynamixel::xl330::{DEFINITION, INFO};
use rustypot::servo::dynamixel::{xl330, xl430};
use rustypot::servo::feetech::sts3215;
use rustypot::servo::{Encoding, RegisterError, ServoKind, WordOrder, find_model, scan_timeout};
use servo::{
    MemoryXl330Bus, RawBus, RustypotXl330Bus, ServoCommand, ServoDeviceDriver, ServoDriver,
    ServoError, ServoPositionTargets, ServoTelemetry, XL330, XL330_CAPABILITIES,
    XL330_DEFAULT_BAUD, XL330_MODEL_NAME, XL330_MODEL_NUMBER, XL330_RESOLUTION, Xl330Driver,
    decode_register, encode_register, position_rad_from_raw, position_raw_from_rad, register,
    registers, velocity_rad_s_from_raw,
};

use crate::support::ScriptedPort;
use std::collections::VecDeque;
use std::time::Duration;

#[test]
fn position_pid_updates_only_the_selected_device() {
    let mut bus = MemoryXl330Bus::with_servos(&[1, 2]);
    for id in [1, 2] {
        bus.set_register(id, "position_p_gain", 32).unwrap();
        bus.set_register(id, "position_i_gain", 12).unwrap();
        bus.set_register(id, "position_d_gain", 40).unwrap();
    }
    servo::xl330::set_position_pid(&mut bus, 2, 200, 0, 0).unwrap();
    for (name, untouched, configured) in [
        ("position_p_gain", 32, 200),
        ("position_i_gain", 12, 0),
        ("position_d_gain", 40, 0),
    ] {
        assert_eq!(bus.register(1, name), untouched);
        assert_eq!(bus.register(2, name), configured);
    }
}

#[test]
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
    reads: Vec<(u8, u8, u8)>,
    writes: Vec<(u8, u8, Vec<u8>)>,
    sync_reads: Vec<(Vec<u8>, u8, u8)>,
    sync_writes: Vec<(Vec<u8>, u8, Vec<Vec<u8>>)>,
    reboots: Vec<u8>,
    pings: Vec<u8>,
    sync_read_answers: VecDeque<Vec<Vec<u8>>>,
}

impl RawBus for DriverBus {
    fn read_raw(&mut self, id: u8, addr: u8, len: u8) -> Result<Vec<u8>, ServoError> {
        self.reads.push((id, addr, len));
        Ok(Vec::new())
    }

    fn write_raw(&mut self, id: u8, addr: u8, data: &[u8]) -> Result<(), ServoError> {
        self.writes.push((id, addr, data.to_vec()));
        match addr {
            64 => self.torque = data.to_vec(),
            84 => self.gain = data.to_vec(),
            _ => {}
        }
        Ok(())
    }

    fn sync_read_raw(&mut self, ids: &[u8], addr: u8, len: u8) -> Result<Vec<Vec<u8>>, ServoError> {
        self.sync_reads.push((ids.to_vec(), addr, len));
        Ok(self.sync_read_answers.pop_front().unwrap_or_else(|| {
            ids.iter()
                .map(|_| [0, 0, 0, 0, 0, 8, 0, 0].to_vec())
                .collect()
        }))
    }

    fn sync_write_raw(&mut self, ids: &[u8], addr: u8, data: &[Vec<u8>]) -> Result<(), ServoError> {
        self.sync_writes.push((ids.to_vec(), addr, data.to_vec()));
        self.goal = data.first().cloned().unwrap_or_default();
        Ok(())
    }

    fn ping(&mut self, id: u8) -> Result<bool, ServoError> {
        self.pings.push(id);
        Ok(true)
    }

    fn reboot(&mut self, id: u8) -> Result<(), ServoError> {
        self.reboots.push(id);
        Ok(())
    }
}

#[test]
fn xl330_driver_uses_vendor_register_layout_for_loop_io() {
    let mut driver = Xl330Driver::new(DriverBus::default()).with_ids([1]);
    let sample = ServoDeviceDriver::read_positions(&mut driver, &[1]).unwrap();
    assert_eq!(sample.positions.len(), 1);
    assert_eq!(sample.velocities, vec![0.0]);
    ServoDeviceDriver::write_positions(
        &mut driver,
        &[1],
        &ServoPositionTargets {
            positions: vec![0.0],
        },
    )
    .unwrap();
    ServoDeviceDriver::set_torque(&mut driver, true).unwrap();
    ServoDeviceDriver::set_gain(&mut driver, 200).unwrap();
    let bus = driver.into_inner();
    assert_eq!(bus.goal, vec![0, 8, 0, 0]);
    assert_eq!(bus.torque, vec![1]);
    assert_eq!(bus.gain, vec![200, 0]);
}

#[test]
fn xl330_reads_all_requested_ids_with_the_contiguous_motion_window() {
    let bus = DriverBus {
        sync_read_answers: VecDeque::from([vec![
            vec![0, 0, 0, 0, 0, 8, 0, 0],
            vec![0, 0, 0, 0, 0, 12, 0, 0],
        ]]),
        ..DriverBus::default()
    };
    let mut driver = Xl330Driver::new(bus);
    let sample = ServoDeviceDriver::read_positions(&mut driver, &[2, 3]).unwrap();
    let bus = driver.into_inner();

    assert_eq!(bus.sync_reads, [(vec![2, 3], 128, 8)]);
    assert_eq!(sample.positions.len(), 2);
    assert_eq!(sample.positions[0], 0.0);
    assert!(sample.positions[1] > 0.0);
}

#[test]
fn xl330_rejects_sync_read_count_and_short_blocks() {
    let count_bus = DriverBus {
        sync_read_answers: VecDeque::from([vec![vec![0; 8]]]),
        ..DriverBus::default()
    };
    let mut count_driver = Xl330Driver::new(count_bus);
    let count_error = ServoDeviceDriver::read_positions(&mut count_driver, &[1, 2]).unwrap_err();
    assert!(matches!(count_error, ServoError::CountMismatch { .. }));
    let _count_bus = count_driver.into_inner();

    let short_bus = DriverBus {
        sync_read_answers: VecDeque::from([vec![vec![0; 7]]]),
        ..DriverBus::default()
    };
    let mut short_driver = Xl330Driver::new(short_bus);
    let short_error = ServoDeviceDriver::read_positions(&mut short_driver, &[1]).unwrap_err();
    assert_eq!(
        short_error,
        ServoError::ShortRead {
            what: "XL330 motion block",
            expected: 8,
            got: 7,
        }
    );
}

#[test]
fn xl330_writes_a_single_sync_packet_for_all_targets() {
    let bus = DriverBus::default();
    let mut driver = Xl330Driver::new(bus);
    ServoDeviceDriver::write_positions(
        &mut driver,
        &[4, 5],
        &ServoPositionTargets {
            positions: vec![0.0, std::f64::consts::FRAC_PI_2],
        },
    )
    .unwrap();
    let bus = driver.into_inner();

    assert_eq!(bus.sync_writes.len(), 1);
    assert_eq!(bus.sync_writes[0].0, [4, 5]);
    assert_eq!(bus.sync_writes[0].1, 116);
    assert_eq!(bus.sync_writes[0].2, [vec![0, 8, 0, 0], vec![0, 12, 0, 0]]);
}

#[test]
fn xl330_rejects_invalid_targets_without_emitting_a_sync_packet() {
    let bus = DriverBus::default();
    let mut driver = Xl330Driver::new(bus);
    let mismatch = ServoDeviceDriver::write_positions(
        &mut driver,
        &[1, 2],
        &ServoPositionTargets {
            positions: vec![0.0],
        },
    )
    .unwrap_err();
    assert!(matches!(mismatch, ServoError::CountMismatch { .. }));
    let out_of_range = ServoDeviceDriver::write_positions(
        &mut driver,
        &[1],
        &ServoPositionTargets {
            positions: vec![std::f64::consts::PI + 0.01],
        },
    )
    .unwrap_err();
    assert!(matches!(
        out_of_range,
        ServoError::RegisterOutOfRange { .. }
    ));
    let bus = driver.into_inner();
    assert!(bus.sync_writes.is_empty());
}

#[test]
fn xl330_torque_gain_ping_and_reboot_use_the_requested_id() {
    let bus = DriverBus::default();
    let mut driver = Xl330Driver::new(bus);
    ServoDriver::set_torque(&mut driver, 8, true).unwrap();
    ServoDriver::set_gain(&mut driver, 8, 300).unwrap();
    assert!(ServoDriver::ping(&mut driver, 8).unwrap());
    ServoDriver::reboot(&mut driver, 8).unwrap();
    let bus = driver.into_inner();

    assert_eq!(bus.writes, [(8, 64, vec![1]), (8, 84, vec![44, 1])]);
    assert_eq!(bus.pings, [8]);
    assert_eq!(bus.reboots, [8]);
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
fn xl330_driver_and_memory_bus_preserve_the_complete_loop_contract() {
    let mut driver = Xl330Driver::new(MemoryXl330Bus::with_servos(&[1, 2])).with_ids([1, 2]);
    let initial = ServoDeviceDriver::read_positions(&mut driver, &[1, 2]).unwrap();
    assert_eq!(initial.positions, [0.0, 0.0]);

    ServoDeviceDriver::set_torque(&mut driver, true).unwrap();
    ServoDeviceDriver::set_gain(&mut driver, 180).unwrap();
    ServoDeviceDriver::write_positions(
        &mut driver,
        &[1, 2],
        &ServoPositionTargets {
            positions: vec![0.25, -0.25],
        },
    )
    .unwrap();
    let bus = driver.into_inner();

    assert_eq!(bus.register(1, "torque_enable"), 1);
    assert_eq!(bus.register(2, "torque_enable"), 1);
    assert_eq!(bus.register(1, "position_p_gain"), 180);
    assert_eq!(bus.register(2, "position_p_gain"), 180);
    assert_eq!(
        bus.register(1, "goal_position"),
        position_raw_from_rad(0.25).unwrap() as i64
    );
    assert_eq!(
        bus.register(2, "goal_position"),
        position_raw_from_rad(-0.25).unwrap() as i64
    );
}

#[test]
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
fn xl330_protocol_round_trips_signed_and_little_endian_registers() {
    assert_eq!(
        encode_register("goal_position", 0x1234).unwrap(),
        [0x34, 0x12, 0, 0]
    );
    assert_eq!(
        decode_register("goal_position", &[0x34, 0x12, 0, 0]).unwrap(),
        0x1234
    );
    assert_eq!(encode_register("torque_enable", 1).unwrap(), [1]);
    assert_eq!(decode_register("torque_enable", &[1]).unwrap(), 1);
    assert!(decode_register("torque_enable", &[1, 0]).is_err());
    assert!(matches!(
        encode_register("missing", 1),
        Err(ServoError::UnknownRegister(name)) if name == "missing"
    ));
}

#[test]
fn xl330_position_conversion_rejects_nonfinite_and_out_of_range_values() {
    assert_eq!(position_raw_from_rad(0.0), Ok(2048));
    assert_eq!(position_raw_from_rad(-std::f64::consts::PI), Ok(0));
    assert_eq!(position_raw_from_rad(std::f64::consts::PI), Ok(4096));
    assert!(matches!(
        position_raw_from_rad(f64::NAN),
        Err(ServoError::NonFinitePosition(value)) if value.is_nan()
    ));
    assert!(matches!(
        position_raw_from_rad(std::f64::consts::PI + 0.001),
        Err(ServoError::RegisterOutOfRange { name, .. }) if name == "goal_position"
    ));
}

#[test]
fn vendor_fake_io_preserves_the_control_loop_contract() -> VendorResult<()> {
    let mut io = FakeIo::new();
    let mut positions = [0.0; NUM_JOINTS];
    positions[0] = 0.25;
    io.write(&JointTargets::new(positions))?;
    assert_eq!(io.read()?.positions, positions);
    Ok(())
}

#[test]
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
fn xl330_memory_bus_rejects_unknown_devices_and_mismatched_sync_data() {
    let mut bus = MemoryXl330Bus::with_servos(&[1, 2]);
    assert_eq!(bus.write_raw(9, 64, &[1]), Err(ServoError::MissingServo(9)));
    assert_eq!(bus.reboot(9), Err(ServoError::MissingServo(9)));
    assert_eq!(
        bus.sync_write_raw(&[1, 2], 116, &[vec![0, 8, 0, 0]]),
        Err(ServoError::CountMismatch {
            what: "XL330 fake sync write",
            expected: 2,
            got: 1,
        })
    );
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
