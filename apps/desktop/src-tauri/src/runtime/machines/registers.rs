//! Servo-register application service.
//!
//! Register table facts, register reads/writes, and persisted ID-cache updates live here. Tauri
//! command files only adapt the frontend request and return the service result.

use std::collections::HashSet;

use crate::types::ServoModelKey;

use crate::platform::helpers::model_key;
use crate::runtime::adapters::physical_identity as physical_backend;
use crate::runtime::adapters::servo_bus::RegisterDescriptor;
use crate::runtime::session::state::DesktopState;
use crate::types::{PhysicalMachine, PhysicalMachineRegisterWriteInput, PhysicalServoRegister};
fn register_table(model: servo::ServoModel) -> Vec<RegisterDescriptor> {
    let registers = match model {
        servo::ServoModel::Hd1910 => servo::hd1910::registers(),
        servo::ServoModel::Xl330M288 => servo::xl330::registers(),
        _ => &[],
    };
    let mut addresses = HashSet::new();
    let mut table = Vec::new();
    for register in registers {
        if model == servo::ServoModel::Hd1910 && register.addr == 3 {
            if !addresses.insert(3) {
                continue;
            }
            // FT exposes the servo model as two one-byte version fields.
            // rustypot's shared STS definition exposes a u16 alias at 0x03.
            table.extend([
                RegisterDescriptor {
                    name: "model",
                    addr: 3,
                    size: 1,
                    access: servo::RegisterAccess::Read,
                    is_default: false,
                },
                RegisterDescriptor {
                    name: "model_number",
                    addr: 4,
                    size: 1,
                    access: servo::RegisterAccess::Read,
                    is_default: false,
                },
            ]);
            addresses.insert(4);
            continue;
        }
        // rustypot's shared STS table has the old two-byte definition at 0x18
        // and the old factory meaning at 0x53. HD1910 uses the FT table here.
        if model == servo::ServoModel::Hd1910 && matches!(register.addr, 24 | 83) {
            continue;
        }
        if model == servo::ServoModel::Hd1910 && register.addr == 44 {
            table.push(RegisterDescriptor {
                name: "pwm_open_speed",
                addr: 44,
                size: 2,
                access: servo::RegisterAccess::ReadWrite,
                is_default: false,
            });
            addresses.insert(44);
            continue;
        }
        if !addresses.insert(register.addr) {
            continue;
        }
        table.push(RegisterDescriptor {
            name: register.name,
            addr: register.addr,
            size: register.size,
            access: register.access,
            is_default: model == servo::ServoModel::Hd1910 && register.addr >= 80,
        });
        if model == servo::ServoModel::Hd1910 && register.addr == 69 {
            table.extend([
                RegisterDescriptor {
                    name: "current_bias",
                    addr: 73,
                    size: 2,
                    access: servo::RegisterAccess::Read,
                    is_default: false,
                },
                RegisterDescriptor {
                    name: "v_fk",
                    addr: 77,
                    size: 1,
                    access: servo::RegisterAccess::Read,
                    is_default: true,
                },
                RegisterDescriptor {
                    name: "v_kgi",
                    addr: 78,
                    size: 1,
                    access: servo::RegisterAccess::Read,
                    is_default: true,
                },
                RegisterDescriptor {
                    name: "p_fk",
                    addr: 79,
                    size: 1,
                    access: servo::RegisterAccess::Read,
                    is_default: true,
                },
            ]);
            addresses.extend([73, 77, 78, 79]);
        }
        if model == servo::ServoModel::Hd1910 && register.addr == 82 {
            // 0x52/0x53 are eFk/Vk on HD1910, rather than the shared STS
            // velocity-unit/minimum-speed pair.
            table.pop();
            addresses.remove(&82);
            table.extend([
                RegisterDescriptor {
                    name: "e_fk",
                    addr: 82,
                    size: 1,
                    access: servo::RegisterAccess::Read,
                    is_default: true,
                },
                RegisterDescriptor {
                    name: "vk",
                    addr: 83,
                    size: 1,
                    access: servo::RegisterAccess::Read,
                    is_default: true,
                },
            ]);
            addresses.extend([82, 83]);
        }
        if model == servo::ServoModel::Hd1910 && register.addr == 23 {
            table.push(RegisterDescriptor {
                name: "minimum_startup_force",
                addr: 24,
                size: 1,
                access: servo::RegisterAccess::ReadWrite,
                is_default: false,
            });
            table.push(RegisterDescriptor {
                name: "integral_limit",
                addr: 25,
                size: 1,
                access: servo::RegisterAccess::ReadWrite,
                is_default: false,
            });
            addresses.extend([24, 25]);
        }
    }
    table
}

fn register_model(name: &str) -> Result<servo::ServoModel, String> {
    match model_key(&name.trim().to_ascii_lowercase())? {
        ServoModelKey::Hd1910 => Ok(servo::ServoModel::Hd1910),
        ServoModelKey::Xl330 => Ok(servo::ServoModel::Xl330M288),
    }
}

fn register_is_writable(register: RegisterDescriptor) -> bool {
    !register.is_default && !matches!(register.access, servo::RegisterAccess::Read)
}

pub(crate) fn physical_machine_registers(
    state: &DesktopState,
    id: String,
) -> Result<Vec<PhysicalServoRegister>, String> {
    let machine = state
        .physical_machines
        .lock()
        .map_err(|error| error.to_string())?
        .iter()
        .find(|machine| machine.id == id)
        .cloned()
        .ok_or_else(|| "unknown physical machine".to_string())?;
    crate::runtime::machines::physical_io::with_bus(
        state,
        &id,
        loop_body::BackendAccess::ReadOnly,
        |mut bus| {
            let mut result = Vec::new();
            for servo_identity in &machine.census {
                let model = register_model(&servo_identity.model)?;
                for register in register_table(model) {
                    let writable = register_is_writable(register);
                    let value = if matches!(register.access, servo::RegisterAccess::Write) {
                        None
                    } else {
                        bus.as_mut().and_then(|bus| {
                            bus.read_register(model, servo_identity.servo_id, register)
                                .ok()
                                .and_then(|value| i32::try_from(value).ok())
                        })
                    };
                    result.push(PhysicalServoRegister {
                        servo_id: servo_identity.servo_id,
                        address: register.addr,
                        name: register.name.to_owned(),
                        size: register.size,
                        default: register.is_default,
                        writable,
                        value,
                    });
                }
            }
            Ok(result)
        },
    )
}

pub(crate) fn physical_machine_read_register(
    state: &DesktopState,
    id: String,
    servo_id: u8,
    name: String,
) -> Result<PhysicalServoRegister, String> {
    let machine = state
        .physical_machines
        .lock()
        .map_err(|error| error.to_string())?
        .iter()
        .find(|machine| machine.id == id)
        .cloned()
        .ok_or_else(|| "unknown physical machine".to_string())?;
    let servo_identity = machine
        .census
        .iter()
        .find(|servo| servo.servo_id == servo_id)
        .ok_or_else(|| format!("servo {servo_id} is not in the machine census"))?;
    let model = register_model(&servo_identity.model)?;
    let register = register_table(model)
        .into_iter()
        .find(|register| register.name == name)
        .ok_or_else(|| format!("unknown register {name}"))?;
    let value = if matches!(register.access, servo::RegisterAccess::Write) {
        None
    } else {
        Some(crate::runtime::machines::physical_io::with_bus(
            state,
            &id,
            loop_body::BackendAccess::ReadOnly,
            |bus| {
                bus.ok_or_else(|| "physical machine is not connected".to_owned())?
                    .read_register(model, servo_id, register)
            },
        )?)
    };
    Ok(PhysicalServoRegister {
        servo_id,
        address: register.addr,
        name: register.name.to_owned(),
        size: register.size,
        default: register.is_default,
        writable: register_is_writable(register),
        value: value.and_then(|value| i32::try_from(value).ok()),
    })
}

pub(crate) fn physical_machine_write_register(
    state: &DesktopState,
    input: PhysicalMachineRegisterWriteInput,
) -> Result<PhysicalServoRegister, String> {
    crate::runtime::machines::physical_io::with_bus(
        state,
        &input.physical_machine_id.clone(),
        loop_body::BackendAccess::Control,
        |bus| {
            crate::runtime::machines::physical_io::validate_control_ids(state, &[input.servo_id])?;
            write_register_on_bus(state, input, bus)
        },
    )
}

fn write_register_on_bus(
    state: &DesktopState,
    input: PhysicalMachineRegisterWriteInput,
    bus: Option<&mut crate::runtime::adapters::servo_bus::ConnectedBus>,
) -> Result<PhysicalServoRegister, String> {
    let machine = state
        .physical_machines
        .lock()
        .map_err(|error| error.to_string())?
        .iter()
        .find(|machine| machine.id == input.physical_machine_id)
        .cloned()
        .ok_or_else(|| "unknown physical machine".to_string())?;
    let servo_identity = machine
        .census
        .iter()
        .find(|servo| servo.servo_id == input.servo_id)
        .ok_or_else(|| format!("servo {} is not in the machine census", input.servo_id))?;
    let model = register_model(&servo_identity.model)?;
    let register = register_table(model)
        .into_iter()
        .find(|register| register.name == input.name)
        .ok_or_else(|| format!("unknown register {}", input.name))?;
    let new_servo_id = if register.addr == 5 {
        Some(
            u8::try_from(input.value)
                .map_err(|_| "舵机 ID 必须是 0 到 255 之间的整数".to_owned())?,
        )
    } else {
        None
    };
    if let Some(new_servo_id) = new_servo_id
        && new_servo_id != input.servo_id
        && machine
            .census
            .iter()
            .any(|servo| servo.servo_id == new_servo_id)
    {
        return Err(format!("舵机 ID {new_servo_id} 已经存在"));
    }
    if !register_is_writable(register) {
        if register.is_default {
            return Err(format!(
                "register {} is a default value and read-only",
                register.name
            ));
        }
        return Err(format!("register {} is read-only", register.name));
    }
    bus.ok_or_else(|| "physical machine is not connected".to_owned())?
        .write_register(model, input.servo_id, register, i64::from(input.value))?;
    if let Some(new_servo_id) = new_servo_id {
        {
            let mut machines = state.physical_machines.lock().map_err(|e| e.to_string())?;
            let machine = machines
                .iter_mut()
                .find(|machine| machine.id == input.physical_machine_id)
                .ok_or("unknown physical machine")?;
            update_servo_id_cache(machine, input.servo_id, new_servo_id)?;
        }
        state.persist()?;
    }
    // The write status packet already confirms the command.  Do not issue an
    // immediate read-back here: EEPROM writes may still be committing after
    // the lock byte is restored, and that extra read can time out even though
    // the requested value was accepted by the servo.
    let value = Some(input.value);
    Ok(PhysicalServoRegister {
        servo_id: new_servo_id.unwrap_or(input.servo_id),
        address: register.addr,
        name: register.name.to_owned(),
        size: register.size,
        default: register.is_default,
        writable: true,
        value,
    })
}

/// Keep all persisted views of a physical servo ID coherent after a successful
/// write to register 5.  This is deliberately separate from the serial command
/// so it can be checked without hardware and cannot silently drift when the
/// census representation changes.
pub(crate) fn update_servo_id_cache(
    machine: &mut PhysicalMachine,
    old_servo_id: u8,
    new_servo_id: u8,
) -> Result<(), String> {
    if old_servo_id == new_servo_id {
        return Ok(());
    }
    if !machine
        .census
        .iter()
        .any(|servo| servo.servo_id == old_servo_id)
    {
        return Err(format!("servo {old_servo_id} is not in the machine census"));
    }
    if machine
        .census
        .iter()
        .any(|servo| servo.servo_id == new_servo_id)
    {
        return Err(format!("舵机 ID {new_servo_id} 已经存在"));
    }
    for servo in &mut machine.census {
        if servo.servo_id == old_servo_id {
            servo.servo_id = new_servo_id;
        }
    }
    machine.census = physical_backend::canonicalize_census(&machine.census)
        .map_err(|error| error.to_string())?;
    machine.machine_code =
        physical_backend::census_md5(&machine.census).map_err(|error| error.to_string())?;
    for id in &mut machine.online_servo_ids {
        if *id == old_servo_id {
            *id = new_servo_id;
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::{register_is_writable, register_table, update_servo_id_cache};
    use crate::types::{PhysicalMachine, ServoIdentity};

    fn test_machine() -> PhysicalMachine {
        PhysicalMachine {
            execution_kind: crate::types::ExecutionKindKey::Physical,
            model_kind: Some(crate::types::ModelKindKey::Hd1910),
            servo_model: Some(crate::types::ServoModelKey::Hd1910),
            id: "physical-register-test".into(),
            name: "寄存器测试".into(),
            usb_port: "COM-test".into(),
            machine_code: "old".into(),
            census: vec![
                ServoIdentity {
                    servo_id: 1,
                    model: "hd1910".into(),
                },
                ServoIdentity {
                    servo_id: 2,
                    model: "hd1910".into(),
                },
            ],
            reconnect_hint: None,
            virtual_machine_id: None,
            online_servo_ids: vec![1, 2],
            connected: false,
            last_seen: None,
            torque_enabled: false,
            joint_directions: Vec::new(),
            joint_bindings: vec![],
        }
    }

    #[test]
    fn register_table_keeps_default_rows_read_only_and_addresses_unique() {
        let table = register_table(servo::ServoModel::Hd1910);
        let mut addresses = std::collections::HashSet::new();
        assert!(table.iter().all(|register| addresses.insert(register.addr)));
        assert!(
            table
                .iter()
                .filter(|register| register.is_default)
                .all(|register| !register_is_writable(*register))
        );
    }

    #[test]
    fn updating_servo_id_cache_keeps_census_and_online_ids_in_sync() {
        let mut machine = test_machine();
        update_servo_id_cache(&mut machine, 1, 7).expect("servo ID cache should update");
        assert!(machine.census.iter().any(|servo| servo.servo_id == 7));
        assert!(!machine.census.iter().any(|servo| servo.servo_id == 1));
        assert_eq!(machine.online_servo_ids, vec![7, 2]);
        assert_ne!(machine.machine_code, "old");
    }

    #[test]
    fn updating_servo_id_cache_rejects_duplicate_ids_without_mutating() {
        let mut machine = test_machine();
        let before = machine.clone();
        let error = update_servo_id_cache(&mut machine, 1, 2)
            .expect_err("duplicate servo IDs must be rejected");
        assert!(error.contains("已经存在"));
        assert_eq!(machine.census, before.census);
        assert_eq!(machine.online_servo_ids, before.online_servo_ids);
    }
}
