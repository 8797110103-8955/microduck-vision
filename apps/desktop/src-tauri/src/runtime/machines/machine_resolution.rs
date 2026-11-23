//! Resolve the selected body and compose virtual IDs with physical installation settings.

use std::collections::{HashMap, HashSet};

use duck_body::{BodyProfile, OFFICIAL_BODY_ID};
#[cfg(test)]
use microduck_body_runtime::BodyExecutionProfile;
use microduck_body_runtime::{ExecutionKind, ModelKind, Selection};
#[cfg(test)]
use servo::ServoModel;

use crate::runtime::session::state::DesktopState;
use crate::types::{
    ExecutionKindKey, ModelKindKey, PhysicalMachine, ServoIdentity, ServoJointBinding,
    ServoModelKey, VirtualMachine,
};

pub(crate) fn body_profile(body_id: &str) -> Result<&'static BodyProfile, String> {
    match body_id {
        OFFICIAL_BODY_ID => crate::runtime::machines::body_catalog::official(),
        _ => Err(format!("unknown body: {body_id}")),
    }
}

#[cfg(test)]
pub(crate) fn physical_profile(
    body: &BodyProfile,
    servo_model: ServoModel,
) -> Result<BodyExecutionProfile, String> {
    let model_kind = match servo_model {
        ServoModel::Xl330M288 => ModelKind::Xl330,
        ServoModel::Hd1910 => ModelKind::Hd1910,
        _ => return Err(format!("unsupported physical servo model {servo_model:?}")),
    };
    microduck_body_runtime::resolve_physical(
        Selection {
            execution_kind: ExecutionKind::Physical,
            model_kind,
            servo_model,
        },
        body,
    )
}

pub(crate) fn model_selection(
    execution_kind: ExecutionKindKey,
    model_kind: ModelKindKey,
    servo_model: ServoModelKey,
) -> Result<Selection, String> {
    let selection = Selection {
        execution_kind: match execution_kind {
            ExecutionKindKey::Simulation => ExecutionKind::Simulation,
            ExecutionKindKey::Physical => ExecutionKind::Physical,
        },
        model_kind: match model_kind {
            ModelKindKey::Xl330 => ModelKind::Xl330,
            ModelKindKey::Hd1910 => ModelKind::Hd1910,
            ModelKindKey::Hd1910Compatible => ModelKind::Hd1910Compatible,
        },
        servo_model: crate::platform::helpers::model(&servo_model),
    };
    selection.validate()?;
    Ok(selection)
}

pub(crate) fn simulation_selection(machine: &VirtualMachine) -> Result<Selection, String> {
    if machine.execution_kind != ExecutionKindKey::Simulation {
        return Err("virtual machines require simulation execution".into());
    }
    model_selection(
        machine.execution_kind,
        machine.model_kind,
        machine.servo_model,
    )
}

pub(crate) fn virtual_machine(state: &DesktopState, id: &str) -> Result<VirtualMachine, String> {
    state
        .virtual_machines
        .lock()
        .map_err(|error| error.to_string())?
        .iter()
        .find(|machine| machine.id == id)
        .cloned()
        .ok_or_else(|| "unknown virtual machine".to_owned())
}

pub(crate) fn associated_virtual_machine(
    state: &DesktopState,
    physical: &PhysicalMachine,
    selected: Option<&str>,
) -> Result<VirtualMachine, String> {
    let id = selected
        .or(physical.virtual_machine_id.as_deref())
        .ok_or("实机没有关联模拟机")?;
    virtual_machine(state, id)
}

pub(crate) fn binding_conflict_error(bindings: &[ServoJointBinding]) -> Option<String> {
    let mut by_id = HashMap::<u8, Vec<&str>>::new();
    for binding in bindings {
        by_id
            .entry(binding.physical_id)
            .or_default()
            .push(binding.slot_id.as_str());
    }
    let mut conflicts = by_id
        .into_iter()
        .filter(|(_, slots)| slots.len() > 1)
        .map(|(id, mut slots)| {
            slots.sort_unstable();
            format!("{id} → {}", slots.join(","))
        })
        .collect::<Vec<_>>();
    conflicts.sort_unstable();
    (!conflicts.is_empty()).then(|| format!("绑定冲突：{}", conflicts.join("；")))
}

#[cfg(test)]
pub(crate) fn official_servo_bindings(census: &[ServoIdentity]) -> Vec<ServoJointBinding> {
    duck_control::model::JOINT_NAMES
        .iter()
        .copied()
        .zip(duck_control::model::JOINT_IDS)
        .filter(|(_, id)| census.iter().any(|servo| servo.servo_id == *id))
        .map(|(slot_id, physical_id)| ServoJointBinding {
            slot_id: slot_id.to_owned(),
            physical_id,
            direction: 1,
            zero_offset_rad: 0.0,
            min_rad: None,
            max_rad: None,
        })
        .collect()
}

pub(crate) fn effective_servo_bindings(
    census: &[ServoIdentity],
    virtual_bindings: &[crate::types::VirtualJointBinding],
) -> Vec<ServoJointBinding> {
    duck_control::model::JOINT_NAMES
        .iter()
        .filter_map(|slot_id| {
            let physical_id = virtual_bindings
                .iter()
                .find(|binding| binding.slot_id == *slot_id)
                .map(|binding| binding.physical_id)?;
            census
                .iter()
                .any(|servo| servo.servo_id == physical_id)
                .then_some(ServoJointBinding {
                    slot_id: (*slot_id).to_owned(),
                    physical_id,
                    direction: 1,
                    zero_offset_rad: 0.0,
                    min_rad: None,
                    max_rad: None,
                })
        })
        .collect()
}

/// Resolve each slot from the virtual machine's explicit binding.
/// A slot whose explicitly selected physical ID is not present in the
/// census stays unmapped so the read-only debug loop can continue with the other joints.
pub(crate) fn debug_servo_bindings(
    census: &[ServoIdentity],
    virtual_bindings: &[crate::types::VirtualJointBinding],
    slots: &[String],
) -> Vec<Option<ServoJointBinding>> {
    let bindings = effective_servo_bindings(census, virtual_bindings);
    slots
        .iter()
        .map(|slot_id| {
            bindings
                .iter()
                .find(|binding| binding.slot_id == *slot_id)
                .cloned()
        })
        .collect()
}

pub(crate) fn debug_driver_mapping(
    census: &[ServoIdentity],
    bindings: &[Option<ServoJointBinding>],
) -> Result<(Vec<u8>, Vec<Option<u8>>), String> {
    let census_ids = census
        .iter()
        .map(|servo| servo.servo_id)
        .collect::<HashSet<_>>();
    let mut used = HashSet::new();
    let mut driver_ids = Vec::with_capacity(bindings.len());
    let mut active_ids = Vec::with_capacity(bindings.len());
    for binding in bindings {
        if let Some(binding) = binding {
            if !used.insert(binding.physical_id) {
                return Err(format!("实机舵机 {} 被重复绑定", binding.physical_id));
            }
            active_ids.push(Some(binding.physical_id));
            driver_ids.push(binding.physical_id);
        } else {
            let placeholder = (0..=u8::MAX)
                .find(|id| !census_ids.contains(id) && !used.contains(id))
                .ok_or_else(|| "没有可用的未映射舵机占位 ID".to_owned())?;
            used.insert(placeholder);
            active_ids.push(None);
            driver_ids.push(placeholder);
        }
    }
    Ok((driver_ids, active_ids))
}

/// Resolve the physical mapping shared by debug and policy loops.  Missing virtual slots are
/// represented by `None` in `active_ids`, allowing the loop to keep running the matched joints
/// without attempting a bus read or write for the missing ones.
type DebugLoopMapping = (Vec<ServoJointBinding>, Vec<u8>, Vec<Option<u8>>);

pub(crate) fn resolve_physical_loop_mapping(
    virtual_bindings: &[crate::types::VirtualJointBinding],
    physical: &PhysicalMachine,
    slots: &[String],
) -> Result<DebugLoopMapping, String> {
    // The virtual machine owns the slot -> physical ID relation.  The physical
    // machine contributes installation direction and calibration by limb name.
    // A census confirms only that an ID exists, never which limb it operates.
    let bindings = debug_servo_bindings(
        &physical
            .census
            .iter()
            .filter(|servo| physical.online_servo_ids.contains(&servo.servo_id))
            .cloned()
            .collect::<Vec<_>>(),
        virtual_bindings,
        slots,
    )
    .into_iter()
    .flatten()
    .map(|mut binding| {
        if let Some(calibration) = physical
            .joint_bindings
            .iter()
            .find(|calibration| calibration.slot_id == binding.slot_id)
        {
            binding.zero_offset_rad = calibration.zero_offset_rad;
            binding.min_rad = calibration.min_rad;
            binding.max_rad = calibration.max_rad;
        }
        if let Some(installation) = physical
            .joint_directions
            .iter()
            .find(|direction| direction.slot_id == binding.slot_id)
        {
            binding.direction = installation.direction;
        }
        binding
    })
    .collect::<Vec<_>>();
    let resolved = slots
        .iter()
        .map(|slot_id| {
            bindings
                .iter()
                .find(|binding| binding.slot_id == *slot_id)
                .cloned()
        })
        .collect::<Vec<_>>();
    let (driver_ids, active_ids) = debug_driver_mapping(&physical.census, &resolved)?;
    Ok((bindings, driver_ids, active_ids))
}
