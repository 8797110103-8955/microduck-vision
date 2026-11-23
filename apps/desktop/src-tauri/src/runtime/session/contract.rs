//! Desktop-resolved identity for a body, policy, and backend resource set.

use microduck_body_runtime::BodyExecutionProfile;

#[derive(Clone, serde::Serialize)]
pub(crate) struct PhysicalExecutionSources {
    pub physical: BodyExecutionProfile,
    pub fallback_simulation: Option<BodyExecutionProfile>,
    pub slots: Vec<SlotExecutionSource>,
}

#[derive(Clone, serde::Serialize)]
pub(crate) struct SlotExecutionSource {
    pub slot_id: String,
    pub physical_id: Option<u8>,
    pub source: SlotSource,
}

#[derive(Clone, Copy, serde::Serialize)]
#[serde(rename_all = "snake_case")]
pub(crate) enum SlotSource {
    Physical,
    Simulation,
    Unavailable,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct ResolvedContract {
    pub body_id: String,
    pub body_version: u32,
    pub policy_id: String,
    pub backend_kind: String,
    pub backend_profile: String,
    pub hash: String,
}

impl ResolvedContract {
    pub(crate) fn new(
        body_id: impl Into<String>,
        body_version: u32,
        policy_id: impl Into<String>,
        backend_kind: impl Into<String>,
        backend_profile: impl Into<String>,
    ) -> Self {
        let body_id = body_id.into();
        let policy_id = policy_id.into();
        let backend_kind = backend_kind.into();
        let backend_profile = backend_profile.into();
        let material =
            format!("{body_id}\0{body_version}\0{policy_id}\0{backend_kind}\0{backend_profile}");
        let mut hash = 0xcbf29ce484222325_u64;
        for byte in material.as_bytes() {
            hash ^= u64::from(*byte);
            hash = hash.wrapping_mul(0x100000001b3);
        }
        Self {
            body_id,
            body_version,
            policy_id,
            backend_kind,
            backend_profile,
            hash: format!("{hash:016x}"),
        }
    }

    pub(crate) fn with_sources(
        body_id: impl Into<String>,
        body_version: u32,
        policy_id: impl Into<String>,
        backend_kind: impl Into<String>,
        backend_profile: impl Into<String>,
        sources: impl IntoIterator<Item = (impl AsRef<str>, impl AsRef<[u8]>)>,
    ) -> Self {
        let mut contract = Self::new(
            body_id,
            body_version,
            policy_id,
            backend_kind,
            backend_profile,
        );
        let mut hash = 0xcbf29ce484222325_u64;
        for (label, bytes) in sources {
            for byte in label
                .as_ref()
                .as_bytes()
                .iter()
                .chain([0u8].iter())
                .chain(bytes.as_ref())
            {
                hash ^= u64::from(*byte);
                hash = hash.wrapping_mul(0x100000001b3);
            }
        }
        for byte in contract
            .hash
            .as_bytes()
            .iter()
            .chain([0u8].iter())
            .chain(format!("{hash:016x}").as_bytes())
        {
            hash ^= u64::from(*byte);
            hash = hash.wrapping_mul(0x100000001b3);
        }
        contract.hash = format!("{hash:016x}");
        contract
    }
}

pub(crate) fn register_simulation(
    registry: &mut loop_body::BackendRegistry,
    machine_id: &str,
    selection: microduck_body_runtime::Selection,
    parameters: &crate::types::SimulationParameters,
    template: &duck_body::BodyTemplate,
) -> Result<
    (
        microduck_body_runtime::simulation::SnapshotHandle,
        ResolvedContract,
        microduck_body_runtime::BodyExecutionProfile,
    ),
    String,
> {
    let profile = crate::runtime::adapters::simulation::resolve(selection, parameters, template)?;
    let contract = simulation_contract(&profile)?;
    let snapshot =
        register_prepared_simulation(registry, machine_id, template, &profile, &contract)?;
    Ok((snapshot, contract, profile))
}

pub(crate) fn simulation_contract(
    profile: &BodyExecutionProfile,
) -> Result<ResolvedContract, String> {
    let native = profile
        .simulation
        .as_ref()
        .ok_or("missing simulation profile")?;
    Ok(ResolvedContract::with_sources(
        &profile.body.id,
        profile.body.version,
        "none",
        "simulation",
        format!("{:?}:mujoco-3.10.0:{}", profile.model_kind, native.id),
        [(
            "execution-profile",
            profile.execution_fingerprint.as_bytes(),
        )],
    ))
}

pub(crate) fn register_prepared_simulation(
    registry: &mut loop_body::BackendRegistry,
    machine_id: &str,
    template: &duck_body::BodyTemplate,
    profile: &BodyExecutionProfile,
    contract: &ResolvedContract,
) -> Result<microduck_body_runtime::simulation::SnapshotHandle, String> {
    microduck_body_runtime::simulation::register_simulation(
        registry,
        machine_id,
        template.clone(),
        profile.clone(),
        contract.hash.clone(),
    )
}

#[cfg(test)]
mod tests {
    use super::ResolvedContract;

    #[test]
    fn stable_identity_changes_with_backend() {
        let first = ResolvedContract::new("official", 1, "walk", "simulation", "mujoco");
        let same = ResolvedContract::new("official", 1, "walk", "simulation", "mujoco");
        let changed = ResolvedContract::new("official", 1, "walk", "physical", "mujoco");
        assert_eq!(first, same);
        assert_ne!(first.hash, changed.hash);
        assert_eq!(first.hash.len(), 16);
    }

    #[test]
    fn resolved_resources_are_part_of_the_identity() {
        let first = ResolvedContract::with_sources(
            "official",
            1,
            "walk",
            "simulation",
            "mujoco",
            [
                ("scene.xml", b"<mujoco/>".as_slice()),
                ("policy.onnx", b"a".as_slice()),
            ],
        );
        let changed = ResolvedContract::with_sources(
            "official",
            1,
            "walk",
            "simulation",
            "mujoco",
            [
                ("scene.xml", b"<mujoco/>".as_slice()),
                ("policy.onnx", b"b".as_slice()),
            ],
        );
        assert_ne!(first.hash, changed.hash);
    }
}
