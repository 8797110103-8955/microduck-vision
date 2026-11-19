//! Resolve the app's physical connection without competing with an active sensor loop.
//!
//! Direct operations share the loop's bus and serialize with lifecycle transitions. Control
//! access additionally holds the execution gate through the complete hardware transaction.

use loop_body::BackendAccess;

use crate::backend::control::ConnectedBus;
use crate::state::DesktopState;

/// Validate joint scope while `with_bus` still holds the transition and write permit.
pub(crate) fn validate_control_ids(state: &DesktopState, ids: &[u8]) -> Result<(), String> {
    let runtime = state.runtime_loop.lock().map_err(|e| e.to_string())?;
    let execution = runtime.as_ref().ok_or("runtime is closed")?.physical()?;
    let allowed = execution.servo_ids().collect::<Vec<_>>();
    if ids.iter().any(|id| !allowed.contains(id)) {
        return Err("physical write is outside the selected execution binding".into());
    }
    Ok(())
}

pub(crate) fn with_bus<T>(
    state: &DesktopState,
    id: &str,
    access: BackendAccess,
    operation: impl FnOnce(Option<&mut ConnectedBus>) -> Result<T, String>,
) -> Result<T, String> {
    let _transition = state
        .execution_transition
        .lock()
        .map_err(|error| error.to_string())?;
    with_bus_locked(state, id, access, operation)
}

/// The caller owns `execution_transition` for the entire configuration/read transaction.
pub(crate) fn with_bus_locked<T>(
    state: &DesktopState,
    id: &str,
    access: BackendAccess,
    operation: impl FnOnce(Option<&mut ConnectedBus>) -> Result<T, String>,
) -> Result<T, String> {
    let _permit = if access == BackendAccess::Control {
        state.ensure_runtime_healthy()?;
        {
            let runtime = state
                .runtime_loop
                .lock()
                .map_err(|error| error.to_string())?;
            if let Some(runtime) = runtime.as_ref() {
                if runtime.physical().is_err() {
                    return Err("physical writes are disabled in this runtime configuration".into());
                }
                if runtime.has_policy() {
                    return Err("policy owns the physical execution source; use runtime actions or switch to manual control".into());
                }
                if state
                    .loop_physical_machine_id
                    .lock()
                    .map_err(|error| error.to_string())?
                    .as_deref()
                    != Some(id)
                {
                    return Err(
                        "physical write destination is not the selected execution backend".into(),
                    );
                }
            } else {
                return Err(
                    "physical writes require an explicit physical manual Control runtime".into(),
                );
            }
        }
        Some(
            state
                .execution_gate
                .physical_write()
                .map_err(|error| error.to_string())?,
        )
    } else {
        None
    };
    let loop_owns_bus = state
        .loop_physical_machine_id
        .lock()
        .map_err(|error| error.to_string())?
        .as_deref()
        == Some(id);
    let shared = if loop_owns_bus {
        state
            .loop_physical_bus
            .lock()
            .map_err(|error| error.to_string())?
            .clone()
    } else {
        None
    };
    if let Some(shared) = shared {
        let mut bus = shared
            .lock()
            .map_err(|_| "physical bus lock poisoned".to_owned())?;
        operation(Some(&mut bus))
    } else {
        let mut buses = state
            .connected_buses
            .lock()
            .map_err(|error| error.to_string())?;
        operation(buses.get_mut(id))
    }
}
