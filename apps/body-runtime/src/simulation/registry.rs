//! Register the product simulator as a lazy body-loop execution provider.

use std::sync::Arc;

use duck_body::BodyTemplate;
use loop_body::{BackendDescriptor, BackendKind, BackendRegistry};

use super::mujoco::NativeSimulation;
use super::snapshot::SnapshotHandle;
use crate::models::assets::OfficialAssets;
use crate::models::profile::BodyExecutionProfile;

/// Register a factory that can be opened for each execution session independently.
pub fn register_simulation(
    registry: &mut BackendRegistry,
    machine_id: &str,
    template: BodyTemplate,
    profile: BodyExecutionProfile,
    contract: String,
) -> Result<SnapshotHandle, String> {
    let snapshot = SnapshotHandle::default();
    let factory_snapshot = Arc::clone(&snapshot);
    if let Some(descriptor) = registry.describe(machine_id) {
        if descriptor.kind != BackendKind::Simulation {
            return Err("backend ID is already registered with another kind".into());
        }
        registry
            .unregister(machine_id)
            .map_err(|error| error.to_string())?;
    }
    registry
        .register(
            BackendDescriptor {
                id: machine_id.to_owned(),
                kind: BackendKind::Simulation,
                supports_control: true,
                contract,
            },
            move || {
                let simulation = NativeSimulation::open(
                    &template,
                    &profile,
                    &OfficialAssets::Embedded,
                    Arc::clone(&factory_snapshot),
                )?;
                Ok(Box::new(simulation))
            },
        )
        .map_err(|error| error.to_string())?;
    Ok(snapshot)
}
