//! Runtime physical backend for the loop-body adapter.
//!
//! The backend owns servo IDs, sparse mapping and bus serialization. Canonical body calibration
//! (direction, zero and installation limits) remains in `duck-body::BodyBinding`, which is
//! applied by the shared loop-body adapter before this backend is asked to write positions.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, Weak};

use crate::platform::logging::append_control_event;
use loop_body::{BackendJointSample, BackendPositionTargets, ControlBackend};
use microduck_body_runtime::{BodyExecutionProfile, PhysicalGains};
use servo::ServoModel;

use super::servo_bus::ConnectedBus;

pub(crate) fn physical_loop_config(
    profile: &BodyExecutionProfile,
    gains: PhysicalGains,
) -> loop_body::LoopConfig {
    loop_body::LoopConfig {
        gain: gains.position_p(),
        period: std::time::Duration::from_secs_f64(profile.periods.control_s),
        ..Default::default()
    }
}

/// Result of selecting the execution backend for a loop.  The command layer supplies only the
/// resolved machine mapping; serial ownership and the read-only shared-driver decision stay here.
pub(crate) struct OpenBackendDriver {
    pub driver: Box<dyn ControlBackend>,
    pub shared_bus: Option<Arc<Mutex<ConnectedBus>>>,
    pub hd1910_position_domains: std::collections::HashMap<u8, servo::hd1910::PositionDomain>,
}

pub(crate) struct PhysicalOpenContext<'a> {
    pub registry: &'a Mutex<loop_body::BackendRegistry>,
    pub buses: &'a Mutex<std::collections::HashMap<String, ConnectedBus>>,
    pub config_dir: Option<&'a std::path::Path>,
}

pub(crate) fn open_backend_driver(
    context: PhysicalOpenContext<'_>,
    physical_id: &str,
    gains: PhysicalGains,
    access: loop_body::BackendAccess,
    active_ids: Vec<Option<u8>>,
    contract: &str,
) -> Result<OpenBackendDriver, String> {
    let model = gains.servo_model();
    let diagnostic_ids = active_ids.iter().flatten().copied().collect::<Vec<_>>();
    let mut registry = context.registry.lock().map_err(|error| error.to_string())?;
    let bus = context
        .buses
        .lock()
        .map_err(|error| error.to_string())?
        .remove(physical_id)
        .ok_or_else(|| "physical machine has no connected bus".to_owned())?;
    let shared = Arc::new(Mutex::new(bus));
    let weak_bus = Arc::downgrade(&shared);
    let opened = (|| {
        if registry.describe(physical_id).is_some() {
            if registry
                .describe(physical_id)
                .is_some_and(|descriptor| descriptor.kind != loop_body::BackendKind::Physical)
            {
                return Err("backend ID is already registered with another kind".into());
            }
            registry
                .unregister(physical_id)
                .map_err(|error| error.to_string())?;
        }
        registry
            .register(
                loop_body::BackendDescriptor {
                    id: physical_id.to_owned(),
                    kind: loop_body::BackendKind::Physical,
                    supports_control: true,
                    contract: contract.to_owned(),
                },
                move || {
                    let bus = weak_bus.upgrade().ok_or_else(|| {
                        servo::ServoError::Bus("physical connection closed".into())
                    })?;
                    Ok(Box::new(PhysicalBackend::new(
                        &bus,
                        gains,
                        active_ids.clone(),
                    )))
                },
            )
            .map_err(|error| error.to_string())?;
        if access == loop_body::BackendAccess::Control {
            registry
                .select(physical_id)
                .map_err(|error| error.to_string())?;
        }
        registry
            .open(physical_id, access)
            .map_err(|error| error.to_string())
    })();
    match opened {
        Ok(driver) => {
            if access == loop_body::BackendAccess::ReadOnly {
                registry
                    .gate()
                    .set_physical_writes_enabled(false)
                    .map_err(|error| error.to_string())?;
            } else {
                registry
                    .gate()
                    .set_physical_writes_enabled(true)
                    .map_err(|error| error.to_string())?;
            }
            append_control_event(
                context.config_dir,
                format!(
                    "backend_session backend_id={} backend_kind=physical access={access:?} generation={} contract_hash={contract}",
                    physical_id,
                    driver.generation()
                ),
            );
            let mut hd1910_position_domains = std::collections::HashMap::new();
            if model == servo::ServoModel::Hd1910 {
                for id in diagnostic_ids {
                    let snapshot = shared
                        .lock()
                        .map_err(|_| "physical bus lock poisoned".to_owned())
                        .and_then(|mut bus| bus.read_hd1910_configuration(id));
                    let (event, level, details, failed) = match snapshot {
                        Ok(configuration) => {
                            hd1910_position_domains.insert(id, configuration.domain);
                            let registers =
                                crate::runtime::adapters::servo_bus::hd1910_configuration_snapshot(
                                    &configuration,
                                );
                            (
                                "servo_configuration_snapshot",
                                "info",
                                serde_json::json!({"registers": registers}),
                                false,
                            )
                        }
                        Err(error) => (
                            "servo_configuration_snapshot_failed",
                            "warn",
                            serde_json::json!({"error": error}),
                            true,
                        ),
                    };
                    crate::platform::logging::diagnostic(
                        context.config_dir,
                        "servo",
                        level,
                        event,
                        serde_json::json!({"duck_id": physical_id, "servo_id": id,
                            "stage": "backend_open", "access": format!("{access:?}"), "details": details}),
                    );
                    // Diagnostic reads are best-effort. Do not repeat a failed/unsupported
                    // window across the whole bus or alter the control state to obtain it.
                    if failed {
                        break;
                    }
                }
            }
            Ok(OpenBackendDriver {
                driver: Box::new(driver),
                shared_bus: Some(shared),
                hd1910_position_domains,
            })
        }
        Err(error) => {
            if let Ok(shared) = Arc::try_unwrap(shared)
                && let Ok(bus) = shared.into_inner()
            {
                context
                    .buses
                    .lock()
                    .map_err(|error| error.to_string())?
                    .insert(physical_id.to_owned(), bus);
            }
            Err(error)
        }
    }
}

/// Physical execution backend that can safely represent a partial body-to-servo mapping.
pub(crate) struct PhysicalBackend {
    bus: Weak<Mutex<ConnectedBus>>,
    model: ServoModel,
    gains: PhysicalGains,
    active_ids: Vec<Option<u8>>,
    read_cancel: Option<Arc<AtomicBool>>,
}

impl PhysicalBackend {
    pub(crate) fn new(
        bus: &Arc<Mutex<ConnectedBus>>,
        gains: PhysicalGains,
        active_ids: Vec<Option<u8>>,
    ) -> Self {
        Self {
            bus: Arc::downgrade(bus),
            model: gains.servo_model(),
            gains,
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

impl ControlBackend for PhysicalBackend {
    fn set_read_cancel(&mut self, cancel: Arc<AtomicBool>) {
        self.read_cancel = Some(cancel);
    }
    fn feedback_validity(&self, ids: &[u8]) -> Vec<bool> {
        if ids.len() != self.active_ids.len() {
            return vec![false; ids.len()];
        }
        self.active_ids.iter().map(Option::is_some).collect()
    }
    fn read_positions(&mut self, ids: &[u8]) -> Result<BackendJointSample, servo::ServoError> {
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
            servo::ServoPositionSample {
                positions: vec![],
                velocities: vec![],
            }
        } else if let Some(cancel) = &self.read_cancel {
            // A source worker may stop after the current bounded bus transaction, rather
            // than waiting for every remaining servo in a multi-joint read.
            let mut samples = servo::ServoPositionSample {
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
        Ok(BackendJointSample {
            velocities,
            positions,
        })
    }

    fn write_positions(
        &mut self,
        ids: &[u8],
        targets: &BackendPositionTargets,
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
        let gains = match self.gains {
            PhysicalGains::Xl330 { i, d, .. } => PhysicalGains::Xl330 { p: gain, i, d },
            PhysicalGains::Hd1910 { .. } => PhysicalGains::Hd1910 {
                p: u8::try_from(gain).map_err(|_| servo::ServoError::RegisterOutOfRange {
                    name: "runtime_kp".into(),
                    value: i64::from(gain),
                })?,
            },
        };
        let ids = self
            .active_ids
            .iter()
            .flatten()
            .copied()
            .collect::<Vec<_>>();
        self.bus()?
            .lock()
            .map_err(|_| servo::ServoError::Bus("physical bus lock poisoned".into()))?
            .set_gain(&ids, gains)
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
