//! Runtime physical backend for the loop-body adapter.
//!
//! The backend owns servo IDs, sparse mapping and bus serialization. Canonical body calibration
//! (direction, zero and installation limits) remains in `duck-body::BodyBinding`, which is
//! applied by the shared loop-body adapter before this backend is asked to write positions.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, Weak};

use servo::{ServoLoopDriver, ServoModel, ServoPositionSample, ServoPositionTargets};

use super::control::ConnectedBus;

/// Physical execution backend that can safely represent a partial body-to-servo mapping.
pub(crate) struct PhysicalBackend {
    bus: Weak<Mutex<ConnectedBus>>,
    model: ServoModel,
    active_ids: Vec<Option<u8>>,
    read_cancel: Option<Arc<AtomicBool>>,
}

impl PhysicalBackend {
    pub(crate) fn new(
        bus: &Arc<Mutex<ConnectedBus>>,
        model: ServoModel,
        active_ids: Vec<Option<u8>>,
    ) -> Self {
        Self {
            bus: Arc::downgrade(bus),
            model,
            active_ids,
            read_cancel: None,
        }
    }

    fn bus(&self) -> Result<Arc<Mutex<ConnectedBus>>, servo::ServoError> {
        self.bus
            .upgrade()
            .ok_or_else(|| servo::ServoError::Bus("physical connection closed".into()))
    }
}

impl ServoLoopDriver for PhysicalBackend {
    fn set_read_cancel(&mut self, cancel: Arc<AtomicBool>) {
        self.read_cancel = Some(cancel);
    }
    fn feedback_validity(&self, ids: &[u8]) -> Vec<bool> {
        if ids.len() != self.active_ids.len() {
            return vec![false; ids.len()];
        }
        self.active_ids.iter().map(Option::is_some).collect()
    }
    fn read_positions(&mut self, ids: &[u8]) -> Result<ServoPositionSample, servo::ServoError> {
        if ids.len() != self.active_ids.len() {
            return Err(servo::ServoError::CountMismatch {
                what: "physical position mapping",
                expected: self.active_ids.len(),
                got: ids.len(),
            });
        }
        let connection = self.bus()?;
        let mut bus = connection
            .lock()
            .map_err(|_| servo::ServoError::Bus("physical bus lock poisoned".into()))?;
        let active_ids = self
            .active_ids
            .iter()
            .flatten()
            .copied()
            .collect::<Vec<_>>();
        let active_samples = if active_ids.is_empty() {
            ServoPositionSample {
                positions: vec![],
                velocities: vec![],
            }
        } else if let Some(cancel) = &self.read_cancel {
            // A source worker may stop after the current bounded bus transaction, rather
            // than waiting for every remaining servo in a multi-joint read.
            let mut samples = ServoPositionSample {
                positions: Vec::new(),
                velocities: Vec::new(),
            };
            for id in &active_ids {
                if cancel.load(Ordering::Acquire) {
                    return Err(servo::ServoError::Bus(
                        "read-only transaction cancelled".into(),
                    ));
                }
                let sample = bus
                    .read_position_samples(self.model, &[*id])
                    .map_err(servo::ServoError::Bus)?;
                samples.positions.extend(sample.positions);
                samples.velocities.extend(sample.velocities);
            }
            samples
        } else {
            bus.read_position_samples(self.model, &active_ids)
                .map_err(servo::ServoError::Bus)?
        };
        let positions = expand_positions(&self.active_ids, active_samples.positions);
        let velocities = expand_positions(&self.active_ids, active_samples.velocities);
        Ok(ServoPositionSample {
            velocities,
            positions,
        })
    }

    fn write_positions(
        &mut self,
        ids: &[u8],
        targets: &ServoPositionTargets,
    ) -> Result<(), servo::ServoError> {
        if ids.len() != targets.positions.len() {
            return Err(servo::ServoError::CountMismatch {
                what: "physical position targets",
                expected: ids.len(),
                got: targets.positions.len(),
            });
        }
        if ids.len() != self.active_ids.len() {
            return Err(servo::ServoError::CountMismatch {
                what: "physical position mapping",
                expected: self.active_ids.len(),
                got: ids.len(),
            });
        }
        for (id, position) in self.active_ids.iter().zip(&targets.positions) {
            if let Some(id) = id {
                let validated = match self.model {
                    ServoModel::Hd1910 => {
                        servo::hd1910::position_target_block(*position).map(|_| ())
                    }
                    ServoModel::Xl330M288 => servo::xl330::position_raw_from_rad(*position)
                        .and_then(|raw| {
                            servo::xl330::encode_register("goal_position", i64::from(raw))
                        })
                        .map(|_| ()),
                    _ => Err(servo::ServoError::NotImplemented(
                        "physical target encoding",
                    )),
                };
                validated.map_err(|error| {
                    servo::ServoError::Bus(format!("servo id={id} target_rad={position}: {error}"))
                })?;
            }
        }
        let connection = self.bus()?;
        let mut bus = connection
            .lock()
            .map_err(|_| servo::ServoError::Bus("physical bus lock poisoned".into()))?;
        for (id, position) in self.active_ids.iter().zip(&targets.positions) {
            if let Some(id) = id {
                bus.write_position(*id, *position)
                    .map_err(servo::ServoError::Bus)?;
            }
        }
        Ok(())
    }

    fn set_torque(&mut self, enabled: bool) -> Result<(), servo::ServoError> {
        let connection = self.bus()?;
        let mut bus = connection
            .lock()
            .map_err(|_| servo::ServoError::Bus("physical bus lock poisoned".into()))?;
        let ids = self
            .active_ids
            .iter()
            .flatten()
            .copied()
            .collect::<Vec<_>>();
        bus.set_torque(self.model, &ids, enabled)
            .map_err(servo::ServoError::Bus)
    }

    fn set_gain(&mut self, gain: u16) -> Result<(), servo::ServoError> {
        let ids = self
            .active_ids
            .iter()
            .flatten()
            .copied()
            .collect::<Vec<_>>();
        self.bus()?
            .lock()
            .map_err(|_| servo::ServoError::Bus("physical bus lock poisoned".into()))?
            .set_gain(self.model, &ids, gain)
            .map_err(servo::ServoError::Bus)
    }

    fn reboot(&mut self, id: u8) -> Result<(), servo::ServoError> {
        if !self.active_ids.contains(&Some(id)) {
            return Err(servo::ServoError::Bus(
                "reboot is outside physical execution binding".into(),
            ));
        }
        self.bus()?
            .lock()
            .map_err(|_| servo::ServoError::Bus("physical bus lock poisoned".into()))?
            .reboot(id)
            .map_err(servo::ServoError::Bus)
    }
}

fn expand_positions<I>(active_ids: &[Option<u8>], positions: I) -> Vec<f64>
where
    I: IntoIterator<Item = f64>,
{
    let mut positions = positions.into_iter();
    active_ids
        .iter()
        .map(|id| match id {
            Some(_) => positions.next().unwrap_or_default(),
            None => 0.0,
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::expand_positions;

    #[test]
    fn active_mapping_keeps_canonical_slot_count_for_partial_hardware() {
        let mapping = [Some(7), None, Some(11)];
        assert_eq!(mapping.len(), 3);
        assert_eq!(
            mapping.iter().flatten().copied().collect::<Vec<_>>(),
            [7, 11]
        );
    }

    #[test]
    fn unmatched_slots_are_explicitly_read_only_placeholders() {
        let mapping = [Some(7), None];
        assert_eq!(mapping[1], None);
        assert_eq!(mapping.iter().flatten().count(), 1);
        assert_eq!(expand_positions(&mapping, [1.25]), vec![1.25, 0.0]);
    }
}
