//! Instance registration and access control, independent of transports, models and UI state.

use std::collections::{BTreeMap, BTreeSet};
use std::sync::{Arc, RwLock, RwLockReadGuard};

use crate::{BackendImuSample, BackendJointSample, BackendPositionTargets};
use servo::ServoError;

use crate::{ControlBackend, GainCapability};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BackendKind {
    Simulation,
    Physical,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BackendAccess {
    ReadOnly,
    Control,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct BackendDescriptor {
    pub id: String,
    pub kind: BackendKind,
    pub supports_control: bool,
    pub contract: String,
}

struct AccessState {
    selected: Option<BackendDescriptor>,
    sessions: BTreeSet<u64>,
    control: Option<u64>,
    generation: u64,
    physical_writes_enabled: bool,
}

impl Default for AccessState {
    fn default() -> Self {
        Self {
            selected: None,
            sessions: BTreeSet::new(),
            control: None,
            generation: 0,
            physical_writes_enabled: true,
        }
    }
}

/// Shared by every hardware write entry point. A permit remains locked throughout the actual
/// operation, so selecting simulation waits for in-flight writes and blocks subsequent ones.
#[derive(Clone, Default)]
pub struct ExecutionGate(Arc<RwLock<AccessState>>);

pub struct PhysicalWritePermit<'a> {
    _guard: RwLockReadGuard<'a, AccessState>,
}

fn error(message: impl Into<String>) -> ServoError {
    ServoError::Bus(message.into())
}

impl ExecutionGate {
    pub fn set_physical_writes_enabled(&self, enabled: bool) -> Result<(), ServoError> {
        self.0
            .write()
            .map_err(|_| error("execution gate poisoned"))?
            .physical_writes_enabled = enabled;
        Ok(())
    }

    pub fn physical_write(&self) -> Result<PhysicalWritePermit<'_>, ServoError> {
        let guard = self
            .0
            .read()
            .map_err(|_| error("execution gate poisoned"))?;
        if !guard.physical_writes_enabled
            || guard
                .selected
                .as_ref()
                .is_some_and(|d| d.kind == BackendKind::Simulation)
        {
            return Err(error(
                "physical writes are disabled while a simulation machine is selected",
            ));
        }
        Ok(PhysicalWritePermit { _guard: guard })
    }

    pub fn selected(&self) -> Result<Option<BackendDescriptor>, ServoError> {
        Ok(self
            .0
            .read()
            .map_err(|_| error("execution gate poisoned"))?
            .selected
            .clone())
    }
}

type Factory = Box<dyn Fn() -> Result<Box<dyn ControlBackend>, ServoError> + Send + Sync>;

struct Registration {
    descriptor: BackendDescriptor,
    factory: Factory,
    sessions: Vec<u64>,
}

#[derive(Default)]
pub struct BackendRegistry {
    entries: BTreeMap<String, Registration>,
    gate: ExecutionGate,
}

impl BackendRegistry {
    pub fn gate(&self) -> ExecutionGate {
        self.gate.clone()
    }

    pub fn register(
        &mut self,
        descriptor: BackendDescriptor,
        factory: impl Fn() -> Result<Box<dyn ControlBackend>, ServoError> + Send + Sync + 'static,
    ) -> Result<(), ServoError> {
        if descriptor.id.trim().is_empty() || self.entries.contains_key(&descriptor.id) {
            return Err(error("empty or duplicate backend registration ID"));
        }
        if descriptor.contract.trim().is_empty() {
            return Err(error("backend registration requires a contract source"));
        }
        self.entries.insert(
            descriptor.id.clone(),
            Registration {
                descriptor,
                factory: Box::new(factory),
                sessions: Vec::new(),
            },
        );
        Ok(())
    }

    pub fn describe(&self, id: &str) -> Option<&BackendDescriptor> {
        self.entries.get(id).map(|entry| &entry.descriptor)
    }

    pub fn list(&self) -> impl Iterator<Item = &BackendDescriptor> {
        self.entries.values().map(|entry| &entry.descriptor)
    }

    /// Selection survives closing a loop: being idle must not silently restore hardware writes.
    pub fn select(&self, id: &str) -> Result<(), ServoError> {
        let entry = self
            .entries
            .get(id)
            .ok_or_else(|| error("unknown backend registration"))?;
        let mut state = self
            .gate
            .0
            .write()
            .map_err(|_| error("execution gate poisoned"))?;
        if state.control.is_some() {
            return Err(error(
                "close the active control session before selecting a machine",
            ));
        }
        state.selected = Some(entry.descriptor.clone());
        Ok(())
    }

    pub fn open(&mut self, id: &str, access: BackendAccess) -> Result<BackendSession, ServoError> {
        let entry = self
            .entries
            .get_mut(id)
            .ok_or_else(|| error("unknown backend registration"))?;
        if access == BackendAccess::Control && !entry.descriptor.supports_control {
            return Err(error("backend does not support control"));
        }
        let generation = {
            let mut state = self
                .gate
                .0
                .write()
                .map_err(|_| error("execution gate poisoned"))?;
            entry
                .sessions
                .retain(|generation| state.sessions.contains(generation));
            if access == BackendAccess::Control {
                if state.control.is_some() {
                    return Err(error("a control session is already active"));
                }
                if state.selected.as_ref() != Some(&entry.descriptor) {
                    return Err(error(
                        "explicitly select the backend before opening control",
                    ));
                }
            }
            state.generation = state
                .generation
                .checked_add(1)
                .ok_or_else(|| error("session generation exhausted"))?;
            let generation = state.generation;
            state.sessions.insert(generation);
            if access == BackendAccess::Control {
                state.control = Some(generation);
            }
            generation
        };
        let driver = match (entry.factory)() {
            Ok(driver) => driver,
            Err(failure) => {
                invalidate(&self.gate, generation);
                return Err(failure);
            }
        };
        entry.sessions.push(generation);
        Ok(BackendSession {
            descriptor: entry.descriptor.clone(),
            access,
            generation,
            gate: self.gate.clone(),
            driver,
            failed: false,
        })
    }

    pub fn unregister(&mut self, id: &str) -> Result<(), ServoError> {
        let mut state = self
            .gate
            .0
            .write()
            .map_err(|_| error("execution gate poisoned"))?;
        let entry = self
            .entries
            .remove(id)
            .ok_or_else(|| error("unknown backend registration"))?;
        for generation in entry.sessions {
            state.sessions.remove(&generation);
            if state.control == Some(generation) {
                state.control = None;
            }
        }
        // Keep the selected kind latched until a different registration is explicitly selected.
        // Unregistering a failed simulator must never enable hardware writes as a side effect.
        Ok(())
    }
}

fn invalidate(gate: &ExecutionGate, generation: u64) {
    if let Ok(mut state) = gate.0.write() {
        state.sessions.remove(&generation);
        if state.control == Some(generation) {
            state.control = None;
        }
    }
}

impl Drop for BackendRegistry {
    fn drop(&mut self) {
        if let Ok(mut state) = self.gate.0.write() {
            for registration in self.entries.values() {
                for generation in &registration.sessions {
                    state.sessions.remove(generation);
                    if state.control == Some(*generation) {
                        state.control = None;
                    }
                }
            }
        }
    }
}

pub struct BackendSession {
    descriptor: BackendDescriptor,
    access: BackendAccess,
    generation: u64,
    gate: ExecutionGate,
    driver: Box<dyn ControlBackend>,
    failed: bool,
}

impl BackendSession {
    pub fn descriptor(&self) -> &BackendDescriptor {
        &self.descriptor
    }
    pub fn generation(&self) -> u64 {
        self.generation
    }

    fn dispatch<T>(
        &mut self,
        write: bool,
        operation: impl FnOnce(&mut dyn ControlBackend) -> Result<T, ServoError>,
    ) -> Result<T, ServoError> {
        let gate = self.gate.clone();
        let state = gate
            .0
            .read()
            .map_err(|_| error("execution gate poisoned"))?;
        if !state.sessions.contains(&self.generation) {
            return Err(error("backend session has been revoked"));
        }
        if self.failed {
            return Err(error(
                "backend session failed; only torque-off and close are permitted",
            ));
        }
        if write
            && (self.access != BackendAccess::Control || state.control != Some(self.generation))
        {
            return Err(error("backend session is read-only"));
        }
        if write && self.descriptor.kind == BackendKind::Physical && !state.physical_writes_enabled
        {
            return Err(error("physical writes are disabled"));
        }
        let result = operation(self.driver.as_mut());
        drop(state);
        if result
            .as_ref()
            .is_err_and(|error| !matches!(error, ServoError::UnsupportedGain))
            && self.access == BackendAccess::Control
        {
            // Reserve ownership until successful stop/drop. Revoking the lease immediately
            // would prevent torque-off and allow a replacement controller before cleanup.
            self.failed = true;
        }
        result
    }
}

impl Drop for BackendSession {
    fn drop(&mut self) {
        invalidate(&self.gate, self.generation);
    }
}

impl ControlBackend for BackendSession {
    fn set_read_cancel(&mut self, cancel: Arc<std::sync::atomic::AtomicBool>) {
        self.driver.set_read_cancel(cancel);
    }
    fn feedback_validity(&self, ids: &[u8]) -> Vec<bool> {
        self.driver.feedback_validity(ids)
    }
    fn read_positions(&mut self, ids: &[u8]) -> Result<BackendJointSample, ServoError> {
        self.dispatch(false, |driver| driver.read_positions(ids))
    }
    fn read_imu(&mut self) -> Result<Option<BackendImuSample>, ServoError> {
        self.dispatch(false, |driver| driver.read_imu())
    }
    fn write_positions(
        &mut self,
        ids: &[u8],
        targets: &BackendPositionTargets,
    ) -> Result<(), ServoError> {
        self.dispatch(true, |driver| driver.write_positions(ids, targets))
    }
    fn set_torque(&mut self, enabled: bool) -> Result<(), ServoError> {
        if enabled {
            return self.dispatch(true, |driver| driver.set_torque(true));
        }
        let gate = self.gate.clone();
        let state = gate
            .0
            .read()
            .map_err(|_| error("execution gate poisoned"))?;
        if !state.sessions.contains(&self.generation)
            || self.access != BackendAccess::Control
            || state.control != Some(self.generation)
        {
            return Err(error("backend session is read-only or revoked"));
        }
        // A failed owner retains this single cleanup operation; it cannot acquire new targets.
        let result = self.driver.set_torque(false);
        if result.is_err() {
            self.failed = true;
        }
        result
    }
    fn gain_capability(&self) -> GainCapability {
        self.driver.gain_capability()
    }
    fn set_gain(&mut self, gain: u16) -> Result<(), ServoError> {
        self.dispatch(true, |driver| match driver.gain_capability() {
            GainCapability::Configurable => driver.set_gain(gain),
            GainCapability::FixedModel | GainCapability::Unsupported => {
                Err(ServoError::UnsupportedGain)
            }
        })
    }
    fn reboot(&mut self, id: u8) -> Result<(), ServoError> {
        self.dispatch(true, |driver| driver.reboot(id))
    }
    fn advance_execution(
        &mut self,
        physics_steps: u32,
        physics_dt: std::time::Duration,
    ) -> Result<(), ServoError> {
        self.dispatch(true, |driver| {
            driver.advance_execution(physics_steps, physics_dt)
        })
    }

    fn reset_execution(&mut self) -> Result<(), ServoError> {
        self.dispatch(true, |driver| driver.reset_execution())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::Mutex;

    struct Recorder(Arc<Mutex<Vec<&'static str>>>);
    impl ControlBackend for Recorder {
        fn read_positions(&mut self, _: &[u8]) -> Result<BackendJointSample, ServoError> {
            self.0.lock().unwrap().push("read");
            Ok(BackendJointSample {
                positions: vec![0.25],
                velocities: vec![0.5],
            })
        }
        fn write_positions(
            &mut self,
            _: &[u8],
            _: &BackendPositionTargets,
        ) -> Result<(), ServoError> {
            self.0.lock().unwrap().push("target");
            Ok(())
        }
        fn set_torque(&mut self, _: bool) -> Result<(), ServoError> {
            self.0.lock().unwrap().push("torque");
            Ok(())
        }
        fn set_gain(&mut self, _: u16) -> Result<(), ServoError> {
            self.0.lock().unwrap().push("gain");
            Ok(())
        }
        fn reboot(&mut self, _: u8) -> Result<(), ServoError> {
            self.0.lock().unwrap().push("reboot");
            Ok(())
        }
    }

    fn register(
        registry: &mut BackendRegistry,
        id: &str,
        kind: BackendKind,
        calls: &Arc<Mutex<Vec<&'static str>>>,
    ) {
        let calls = calls.clone();
        registry
            .register(
                BackendDescriptor {
                    id: id.into(),
                    kind,
                    supports_control: true,
                    contract: "test".into(),
                },
                move || {
                    calls.lock().unwrap().push("open");
                    Ok(Box::new(Recorder(calls.clone())))
                },
            )
            .unwrap();
    }

    #[test]
    fn simulation_control_and_physical_reads_coexist_without_hardware_writes() {
        let mut registry = BackendRegistry::default();
        let sim = Arc::new(Mutex::new(vec![]));
        let physical = Arc::new(Mutex::new(vec![]));
        register(&mut registry, "sim", BackendKind::Simulation, &sim);
        register(&mut registry, "physical", BackendKind::Physical, &physical);
        assert!(sim.lock().unwrap().is_empty());
        assert_eq!(registry.list().count(), 2);
        registry.select("sim").unwrap();
        let gate = registry.gate();
        assert!(gate.physical_write().is_err());
        let mut control = registry.open("sim", BackendAccess::Control).unwrap();
        let mut read = registry.open("physical", BackendAccess::ReadOnly).unwrap();
        assert_eq!(read.read_positions(&[1]).unwrap().positions, vec![0.25]);
        control
            .write_positions(
                &[1],
                &BackendPositionTargets {
                    positions: vec![1.0],
                },
            )
            .unwrap();
        assert!(
            read.write_positions(
                &[1],
                &BackendPositionTargets {
                    positions: vec![1.0]
                }
            )
            .is_err()
        );
        assert!(read.set_torque(true).is_err());
        assert!(read.set_gain(200).is_err());
        assert!(read.reboot(1).is_err());
        assert!(registry.select("physical").is_err());
        assert_eq!(*physical.lock().unwrap(), ["open", "read"]);
        assert_eq!(*sim.lock().unwrap(), ["open", "target"]);
        drop(control);
        assert!(
            gate.physical_write().is_err(),
            "closing simulation must not reenable hardware"
        );
        registry.select("physical").unwrap();
        assert!(gate.physical_write().is_ok());
    }

    #[test]
    fn unregister_revokes_old_sessions_and_reregister_does_not_reuse_generation() {
        let mut registry = BackendRegistry::default();
        let calls = Arc::new(Mutex::new(vec![]));
        register(&mut registry, "sim", BackendKind::Simulation, &calls);
        registry.select("sim").unwrap();
        let mut old = registry.open("sim", BackendAccess::Control).unwrap();
        let old_generation = old.generation();
        registry.unregister("sim").unwrap();
        assert!(old.read_positions(&[1]).is_err());
        assert!(old.set_torque(false).is_err());
        assert!(registry.open("sim", BackendAccess::ReadOnly).is_err());
        register(&mut registry, "sim", BackendKind::Simulation, &calls);
        registry.select("sim").unwrap();
        let mut new = registry.open("sim", BackendAccess::Control).unwrap();
        assert_ne!(old_generation, new.generation());
        drop(old);
        new.set_gain(200).unwrap();
        assert_eq!(*calls.lock().unwrap(), ["open", "open", "gain"]);
        drop(registry);
        assert!(new.set_torque(true).is_err());
    }

    #[test]
    fn failed_factory_keeps_simulation_selected_without_leaking_control_lease() {
        let mut registry = BackendRegistry::default();
        let descriptor = BackendDescriptor {
            id: "sim".into(),
            kind: BackendKind::Simulation,
            supports_control: true,
            contract: "test".into(),
        };
        registry
            .register(descriptor.clone(), || Err(error("engine unavailable")))
            .unwrap();
        assert!(registry.register(descriptor, || unreachable!()).is_err());
        assert!(registry.open("unknown", BackendAccess::ReadOnly).is_err());
        assert!(registry.open("sim", BackendAccess::Control).is_err());
        registry.select("sim").unwrap();
        assert!(registry.open("sim", BackendAccess::Control).is_err());
        assert!(registry.gate().physical_write().is_err());
        registry.select("sim").unwrap();
    }

    #[test]
    fn selecting_simulation_waits_for_inflight_physical_writes() {
        use std::sync::mpsc;
        use std::time::Duration;
        let mut registry = BackendRegistry::default();
        let calls = Arc::new(Mutex::new(vec![]));
        register(&mut registry, "sim", BackendKind::Simulation, &calls);
        let gate = registry.gate();
        let permit = gate.physical_write().unwrap();
        let (started_tx, started_rx) = mpsc::channel();
        let (done_tx, done_rx) = mpsc::channel();
        let selecting = std::thread::spawn(move || {
            started_tx.send(()).unwrap();
            registry.select("sim").unwrap();
            done_tx.send(()).unwrap();
        });
        started_rx.recv().unwrap();
        assert!(done_rx.recv_timeout(Duration::from_millis(30)).is_err());
        drop(permit);
        done_rx.recv_timeout(Duration::from_secs(5)).unwrap();
        selecting.join().unwrap();
        assert!(gate.physical_write().is_err());
    }

    #[test]
    fn readonly_access_can_disable_global_physical_writes_without_opening_control() {
        let gate = ExecutionGate::default();
        assert!(gate.physical_write().is_ok());
        gate.set_physical_writes_enabled(false).unwrap();
        assert!(gate.physical_write().is_err());
        gate.set_physical_writes_enabled(true).unwrap();
        assert!(gate.physical_write().is_ok());
    }

    #[test]
    fn provider_failure_reserves_control_for_acknowledged_cleanup() {
        struct Broken;
        impl ControlBackend for Broken {
            fn read_positions(&mut self, _: &[u8]) -> Result<BackendJointSample, ServoError> {
                Err(error("provider failed"))
            }
            fn write_positions(
                &mut self,
                _: &[u8],
                _: &BackendPositionTargets,
            ) -> Result<(), ServoError> {
                panic!("revoked provider must not run")
            }
            fn set_torque(&mut self, enabled: bool) -> Result<(), ServoError> {
                assert!(!enabled, "failed provider only accepts cleanup");
                Ok(())
            }
            fn set_gain(&mut self, _: u16) -> Result<(), ServoError> {
                panic!("revoked provider must not run")
            }
            fn reboot(&mut self, _: u8) -> Result<(), ServoError> {
                panic!("revoked provider must not run")
            }
        }
        let mut registry = BackendRegistry::default();
        registry
            .register(
                BackendDescriptor {
                    id: "sim".into(),
                    kind: BackendKind::Simulation,
                    supports_control: true,
                    contract: "broken".into(),
                },
                || Ok(Box::new(Broken)),
            )
            .unwrap();
        registry.select("sim").unwrap();
        let mut session = registry.open("sim", BackendAccess::Control).unwrap();
        assert!(
            session
                .read_positions(&[1])
                .unwrap_err()
                .to_string()
                .contains("provider failed")
        );
        assert!(
            session
                .set_torque(true)
                .unwrap_err()
                .to_string()
                .contains("failed")
        );
        assert!(registry.gate().physical_write().is_err());
        assert!(registry.select("sim").is_err());
        assert!(registry.open("sim", BackendAccess::Control).is_err());
        session.set_torque(false).unwrap();
        assert!(registry.open("sim", BackendAccess::Control).is_err());
        drop(session);
        registry.select("sim").unwrap();
        let replacement = registry.open("sim", BackendAccess::Control).unwrap();
        assert!(registry.select("sim").is_err());
        drop(replacement);
    }
}
