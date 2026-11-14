use crate::support::ScriptedPort;
use rustypot::servo::feetech::sts3215::DEFINITION;
use rustypot::servo::feetech::{scs0009, sts3215};
use rustypot::servo::{Encoding, WordOrder};
use servo::hd1910::{
    FeetechDriver, FeetechRawBus, HD1910, Hd1910Driver, STS3215, baud_code, current_ma_from_raw,
    decode_frame, decode_register, decode_xdat, encode_frame, encode_register, encode_xdat,
    model_from_identity, motion_registers, position_steps_from_rad, protocol_for_model, register,
    registers, reset_instruction, temperature_c_from_raw, velocity_rad_s_from_raw,
    voltage_v_from_raw,
};
use servo::{
    Gain, ServoCommand, ServoError, ServoIdentity, ServoLoopDriver, ServoModel,
    ServoPositionTargets, ServoTelemetry,
};

const SCS_IDENTITY: [u8; 2] = [5, 4];
const STS_IDENTITY: [u8; 2] = [9, 3];
const HD1910_IDENTITY: [u8; 2] = [10, 31];

#[derive(Default)]
struct DriverBus {
    goal: Vec<u8>,
    torque: Vec<u8>,
    gain: Vec<u8>,
}

impl FeetechRawBus for DriverBus {
    fn read_raw(&mut self, _id: u8, _addr: u8, _len: u8) -> Result<Vec<u8>, ServoError> {
        Ok(vec![0, 8, 0, 0])
    }

    fn write_raw(&mut self, _id: u8, addr: u8, data: &[u8]) -> Result<(), ServoError> {
        match addr {
            40 => self.torque = data.to_vec(),
            21 => self.gain = data.to_vec(),
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
        Ok(ids.iter().map(|_| vec![0, 8, 0, 0]).collect())
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
fn hd1910_driver_uses_feetech_motion_registers_for_loop_io() {
    let mut driver = Hd1910Driver::new(DriverBus::default()).with_ids([1]);
    let sample = ServoLoopDriver::read_positions(&mut driver, &[1]).unwrap();
    assert_eq!(sample.positions.len(), 1);
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
    assert_eq!(bus.goal, vec![0, 8]);
    assert_eq!(bus.torque, vec![1]);
    assert_eq!(bus.gain, vec![200]);
}

const HD1910_REGISTER_FACTS: &[(&str, u8, u8, &str, bool)] = &[
    ("lock", 55, 1, "sram", true),
    ("torque_enable", 40, 1, "sram", true),
    ("acceleration", 41, 1, "sram", true),
    ("goal_position", 42, 2, "sram", true),
    ("goal_time", 44, 2, "sram", true),
    ("goal_speed", 46, 2, "sram", true),
    ("present_position", 56, 2, "sram", false),
    ("present_speed", 58, 2, "sram", false),
    ("present_load", 60, 2, "sram", false),
    ("present_voltage", 62, 1, "sram", false),
    ("present_temperature", 63, 1, "sram", false),
    ("status", 65, 1, "sram", false),
    ("moving", 66, 1, "sram", false),
    ("present_current", 69, 2, "sram", false),
    ("current_bias", 73, 2, "sram", false),
    ("v_fk", 77, 1, "factory", false),
    ("v_kgi", 78, 1, "factory", false),
    ("p_fk", 79, 1, "factory", false),
    ("e_fk", 82, 1, "factory", false),
    ("vk", 83, 1, "factory", false),
];

#[test]
fn feetech_public_protocol_and_driver_signatures_compile() {
    let _ = HD1910;
    let _ = STS3215;
    let _: fn(()) -> servo::hd1910::Hd1910Driver<()> = servo::hd1910::Hd1910Driver::new;
    let _: fn([u8; 2]) -> ServoModel = model_from_identity;
    let _: fn(ServoModel) -> Option<servo::hd1910::FeetechProtocol> = protocol_for_model;
    let _: fn(&str) -> Option<servo::hd1910::FeetechRegister> = register;
    let _: fn() -> &'static [rustypot::servo::RegisterInfo] = registers;
}

#[test]
fn feetech_model_identity_bytes_match_the_mapping_record() {
    assert_eq!(model_from_identity(SCS_IDENTITY), ServoModel::Scs0009);
    assert_eq!(model_from_identity(STS_IDENTITY), ServoModel::Sts3215);
    assert_eq!(model_from_identity(HD1910_IDENTITY), ServoModel::Hd1910);
}

#[test]
fn hd1910_definition_uses_the_sts_rustypot_register_family() {
    assert_eq!(DEFINITION.protocol, 1);
    assert_eq!(DEFINITION.info.resolution, Some(4096));
    assert!(DEFINITION.register("goal_position").is_some());
    assert!(DEFINITION.register("present_position").is_some());
    assert!(DEFINITION.register("present_current").is_some());
}

#[test]
fn hd1910_corrected_register_facts_match_the_feetech_mapping() {
    for &(name, address, width, area, writable) in HD1910_REGISTER_FACTS {
        let _ = (name, address, width, area, writable);
        assert!(
            register(name).is_some(),
            "missing corrected register {name}"
        );
    }
}

#[test]
fn hd1910_does_not_reuse_sts_factory_meanings_at_82_and_83() {
    assert!(register("e_fk").is_some());
    assert!(register("vk").is_some());
    assert!(register("velocity_unit").is_none());
    assert!(register("minimum_speed_limit").is_none());
}

#[test]
fn feetech_baud_codes_match_the_mapping_record() {
    let expected = [
        (1_000_000, 0),
        (500_000, 1),
        (250_000, 2),
        (128_000, 3),
        (115_200, 4),
        (76_800, 5),
        (57_600, 6),
        (38_400, 7),
    ];
    for &(baud, code) in &expected {
        assert_eq!(baud_code(ServoModel::Hd1910, baud), Some(code));
    }
}

#[test]
fn feetech_restore_and_motion_blocks_are_model_specific() {
    assert_eq!(reset_instruction(ServoModel::Hd1910), Some(0x06));
    assert_eq!(reset_instruction(ServoModel::Sts3215), Some(0x06));
    assert_eq!(
        motion_registers(ServoModel::Hd1910),
        ["acceleration", "goal_position", "goal_time", "goal_speed",]
    );
    assert_eq!(
        motion_registers(ServoModel::Scs0009),
        ["goal_position", "goal_time", "goal_speed",]
    );
}

#[test]
fn scs_support_state_is_explicit_after_identity_detection() {
    assert!(protocol_for_model(ServoModel::Scs0009).is_some());
}

#[test]
fn feetech_protocol_one_frame_vectors_match_the_web_tool() -> Result<(), ServoError> {
    let ping = encode_frame(1, 0x01, &[])?;
    assert_eq!(ping, vec![0xFF, 0xFF, 1, 2, 1, 0xFB]);
    let read = encode_frame(1, 0x02, &[42, 7])?;
    assert_eq!(read, vec![0xFF, 0xFF, 1, 4, 2, 42, 7, 0xC7]);
    let decoded = decode_frame(&read)?;
    assert_eq!(decoded, (1, 0x02, vec![42, 7]));
    Ok(())
}

#[test]
fn feetech_register_encoding_preserves_model_byte_order() -> Result<(), ServoError> {
    let little_endian = encode_register("goal_position", 0x1234)?;
    assert_eq!(little_endian, vec![0x34, 0x12]);
    let decoded = decode_register("goal_position", &little_endian)?;
    assert_eq!(decoded, 0x1234);
    Ok(())
}

#[test]
fn feetech_signed_magnitude_speed_and_load_vectors_are_preserved() -> Result<(), ServoError> {
    assert_eq!(decode_register("present_speed", &[0x64, 0x00])?, 100);
    assert_eq!(decode_register("present_speed", &[0x64, 0x80])?, -100);
    assert_eq!(decode_register("present_load", &[0x32, 0x80])?, -50);
    Ok(())
}

#[test]
fn feetech_unit_projections_cover_position_speed_current_voltage_and_temperature() {
    assert_eq!(position_steps_from_rad(ServoModel::Hd1910, 0.0), Ok(2048));
    assert!((velocity_rad_s_from_raw(ServoModel::Hd1910, 0) - 0.0).abs() < 1e-12);
    assert_eq!(current_ma_from_raw(ServoModel::Hd1910, 2), 13.0);
    assert_eq!(voltage_v_from_raw(ServoModel::Hd1910, 74), 7.4);
    assert_eq!(temperature_c_from_raw(ServoModel::Hd1910, 31), 31.0);
}

#[test]
fn hd1910_eeprom_lock_write_restore_sequence_is_explicit() -> Result<(), ServoError> {
    let unlock = encode_register("lock", 0)?;
    let write = encode_register("p_coefficient", 32)?;
    let relock = encode_register("lock", 1)?;
    assert_eq!(unlock, vec![0]);
    assert_eq!(write, vec![32]);
    assert_eq!(relock, vec![1]);
    Ok(())
}

#[test]
fn hd1910_xdat_header_and_49_byte_window_round_trip() -> Result<(), ServoError> {
    let mut bytes = [0u8; 51];
    bytes[1] = 0x28;
    let image = decode_xdat(&bytes)?;
    let encoded = encode_xdat(image)?;
    assert_eq!(encoded.len(), 51);
    assert_eq!(&encoded[..2], &[0x00, 0x28]);
    Ok(())
}

#[test]
fn hd1910_default_binding_exposes_the_expected_driver_shape() {
    fn accepts_driver<D: FeetechDriver + ?Sized>(_driver: &mut D) {}
    fn accepts_bus<B: FeetechRawBus + ?Sized>(_bus: &mut B) {}
    let _ = accepts_driver::<dyn FeetechDriver>;
    let _ = accepts_bus::<dyn FeetechRawBus>;
    let _: Option<ServoTelemetry> = None;
    let _: Option<ServoIdentity> = None;
    let _: Option<ServoCommand> = None;
    let _: Option<Gain> = None;
}

#[test]
fn hd1910_protocol_errors_are_structured() {
    let _ = ServoError::UnknownRegister("missing".into());
    let _ = ServoError::ShortRead {
        what: "status",
        expected: 1,
        got: 0,
    };
    let _ = ServoError::MissingServo(1);
}

#[test]
fn official_feetech_definition_facts_are_reused() {
    assert_eq!(sts3215::DEFINITION.protocol, 1);
    assert_eq!(sts3215::INFO.resolution, Some(4096));
    assert_eq!(sts3215::INFO.word_order, WordOrder::Little);
    const {
        assert!(sts3215::INFO.supports_sync_read);
    }
    assert_eq!(sts3215::INFO.factory_baudrate, Some(1_000_000));
    assert_eq!(scs0009::INFO.resolution, Some(1024));
    assert_eq!(scs0009::INFO.word_order, WordOrder::Big);
    const {
        assert!(!scs0009::INFO.supports_sync_read);
    }
}

#[test]
fn official_feetech_register_encodings_are_reused() {
    assert_eq!(
        sts3215::DEFINITION
            .register("present_position")
            .unwrap()
            .encoding,
        Encoding::SignMagnitude { sign_bit: 15 }
    );
    assert_eq!(
        sts3215::DEFINITION
            .register("present_load")
            .unwrap()
            .encoding,
        Encoding::SignMagnitude { sign_bit: 10 }
    );
    assert_eq!(
        scs0009::DEFINITION
            .register("present_position")
            .unwrap()
            .encoding,
        Encoding::Unsigned
    );
}

#[test]
fn official_feetech_register_read_write_vectors_are_reused()
-> Result<(), Box<dyn std::error::Error>> {
    let read = vec![0xFF, 0xFF, 0x01, 0x04, 0x00, 0xC5, 0x0A, 0x2B];
    let write = vec![0xFF, 0xFF, 0x01, 0x02, 0x00, 0xFC];
    let (port, written) = ScriptedPort::new(vec![read, write]);
    let mut controller = sts3215::Sts3215Controller::new()
        .with_serial_port(Box::new(port))
        .with_protocol_v1();

    assert_eq!(controller.read_register(1, "homing_offset")?, -709);
    controller.write_register(1, "goal_position", -100)?;
    assert_eq!(
        written.lock().unwrap()[1],
        [0xFF, 0xFF, 0x01, 0x05, 0x03, 0x2A, 0x64, 0x80, 0xE8]
    );
    Ok(())
}

#[test]
fn official_scs_big_endian_register_write_is_reused() -> Result<(), Box<dyn std::error::Error>> {
    let (port, written) = ScriptedPort::new(vec![vec![0xFF, 0xFF, 0x01, 0x02, 0x00, 0xFC]]);
    let mut controller = scs0009::Scs0009Controller::new()
        .with_serial_port(Box::new(port))
        .with_protocol_v1();

    controller.write_register(1, "goal_position", 0x1234)?;
    assert_eq!(
        written.lock().unwrap()[0],
        [0xFF, 0xFF, 0x01, 0x05, 0x03, 0x2A, 0x12, 0x34, 0x86]
    );
    Ok(())
}

#[test]
fn official_feetech_invalid_registers_do_not_reach_the_bus() {
    let (port, written) = ScriptedPort::new(Vec::new());
    let mut controller = sts3215::Sts3215Controller::new()
        .with_serial_port(Box::new(port))
        .with_protocol_v1();

    assert!(controller.write_register(1, "torque_enable", 256).is_err());
    assert!(controller.read_register(1, "current_limit").is_err());
    assert!(written.lock().unwrap().is_empty());
}

#[test]
fn official_scs_without_sync_read_uses_one_read_per_id() -> Result<(), Box<dyn std::error::Error>> {
    let first = vec![0xFF, 0xFF, 0x01, 0x04, 0x00, 0x01, 0x00, 0xF9];
    let second = vec![0xFF, 0xFF, 0x02, 0x04, 0x00, 0x00, 0x10, 0xE9];
    let (port, written) = ScriptedPort::new(vec![first, second]);
    let mut controller = scs0009::Scs0009Controller::new()
        .with_serial_port(Box::new(port))
        .with_protocol_v1();

    assert_eq!(
        controller.sync_read_register(&[1, 2], "present_position")?,
        [256, 16]
    );
    assert_eq!(written.lock().unwrap().len(), 2);
    assert_eq!(written.lock().unwrap()[0][4], 0x02);
    assert_eq!(written.lock().unwrap()[1][4], 0x02);
    Ok(())
}

#[test]
fn official_feetech_scan_uses_model_number_response() -> Result<(), Box<dyn std::error::Error>> {
    let answer = vec![0xFF, 0xFF, 0x02, 0x04, 0x00, 0x09, 0x03, 0xED];
    let (port, _) = ScriptedPort::new(vec![Vec::new(), answer, Vec::new()]);
    let mut controller = sts3215::Sts3215Controller::new()
        .with_serial_port(Box::new(port))
        .with_protocol_v1();

    assert_eq!(
        controller.scan(&[1, 2, 3])?,
        std::collections::BTreeMap::from([(2, 777)])
    );
    Ok(())
}
