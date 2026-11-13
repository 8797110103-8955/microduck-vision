use std::collections::{BTreeMap, BTreeSet};

use servo::ServoModel;

use super::{ActuatorCapability, BodyError, BodyTemplate};

#[derive(Debug, Clone, PartialEq)]
pub struct ServoParameters {
    pub model: ServoModel,
    pub capabilities: BTreeSet<ActuatorCapability>,
    pub mass_g: Option<f64>,
}

#[derive(Debug, Clone, Default, PartialEq)]
pub struct ServoParameterSet {
    by_slot: BTreeMap<String, ServoParameters>,
}

impl ServoParameterSet {
    pub fn all_hd1910(template: &BodyTemplate) -> Self {
        Self::bind_all(template, ServoModel::Hd1910)
    }

    pub fn bind_all(template: &BodyTemplate, model: ServoModel) -> Self {
        let mut set = Self::default();
        for slot in &template.actuators {
            set.by_slot.insert(slot.id.clone(), parameters_for(model));
        }
        set
    }

    pub fn from_bindings(bindings: impl IntoIterator<Item = (String, ServoModel)>) -> Self {
        Self {
            by_slot: bindings
                .into_iter()
                .map(|(slot, model)| (slot, parameters_for(model)))
                .collect(),
        }
    }

    pub fn get(&self, slot: &str) -> Option<&ServoParameters> {
        self.by_slot.get(slot)
    }

    pub fn validate_for(&self, template: &BodyTemplate) -> Result<(), BodyError> {
        for slot in &template.actuators {
            let params = self
                .get(&slot.id)
                .ok_or_else(|| BodyError::MissingServoBinding {
                    slot: slot.id.clone(),
                })?;
            let key = servo_key(params.model);
            if !slot.allowed_servo_types.is_empty()
                && !slot
                    .allowed_servo_types
                    .iter()
                    .any(|allowed| allowed == key)
            {
                return Err(BodyError::ServoTypeNotAllowed {
                    slot: slot.id.clone(),
                    servo_type: key.to_owned(),
                });
            }
            for capability in &slot.required_capabilities {
                if !params.capabilities.contains(capability) {
                    return Err(BodyError::MissingCapability {
                        slot: slot.id.clone(),
                        servo_type: key.to_owned(),
                        capability: *capability,
                    });
                }
            }
        }
        Ok(())
    }

    pub fn total_mass_g(&self, template: &BodyTemplate) -> Option<f64> {
        let link_mass = template.physical.mass_g?;
        let actuator_mass = template
            .actuators
            .iter()
            .zip(template.actuators.iter())
            .map(|(slot, _)| {
                slot.mounted_mass_g
                    .or_else(|| self.get(&slot.id).and_then(|p| p.mass_g))
            })
            .collect::<Option<Vec<_>>>()?
            .into_iter()
            .sum::<f64>();
        Some(link_mass + actuator_mass)
    }
}

fn servo_key(model: ServoModel) -> &'static str {
    match model {
        ServoModel::Hd1910 => "hd1910",
        ServoModel::Xl330M288 => "xl330",
        ServoModel::Sts3215 => "sts3215",
        ServoModel::Scs0009 => "scs0009",
    }
}

fn parameters_for(model: ServoModel) -> ServoParameters {
    let capabilities = [
        ActuatorCapability::PositionControl,
        ActuatorCapability::PresentPosition,
        ActuatorCapability::PresentVelocity,
        ActuatorCapability::TorqueEnable,
        ActuatorCapability::SetGain,
        ActuatorCapability::PresentCurrent,
        ActuatorCapability::PresentVoltage,
        ActuatorCapability::PresentTemperature,
        ActuatorCapability::Reboot,
        ActuatorCapability::FastSyncRead,
    ]
    .into_iter()
    .collect();
    ServoParameters {
        model,
        capabilities,
        mass_g: None,
    }
}
