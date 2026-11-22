//! MuJoCo backend sources and registration for the desktop.

use std::sync::{Arc, Mutex};

use super::mujoco::{NativeSimulation, SharedSnapshot, SimulationActuatorProfile, SnapshotHandle};
use crate::backend::contract::ResolvedContract;
use crate::helpers::default_simulation;
use crate::types::SimulationParameters;
use duck_body::{BodyTemplate, servo_profiles};
use servo::ServoModel;

/// Register a lazy simulation factory. Selection can revoke hardware write permission before
/// any loop or physics state is created; the product implementation stays outside loop-body.
pub(crate) fn register_machine(
    registry: &mut loop_body::BackendRegistry,
    machine_id: &str,
    model: ServoModel,
    simulation: &SimulationParameters,
) -> Result<SnapshotHandle, String> {
    let template = BodyTemplate::official().map_err(|error| error.to_string())?;
    let mut profile = servo_profiles()
        .into_iter()
        .find(|profile| profile.model == model)
        .ok_or_else(|| format!("missing servo profile for {model:?}"))?;
    let actuator_profile =
        SimulationActuatorProfile::for_model(model).map_err(|error| error.to_string())?;
    let model_key = match model {
        ServoModel::Hd1910 => duck_body::ServoModelKey::Hd1910,
        ServoModel::Xl330M288 => duck_body::ServoModelKey::Xl330,
        _ => return Err(format!("unsupported servo model {model:?}")),
    };
    let defaults = default_simulation(model_key);
    // Product sliders are explicit profile multipliers. Neutral values preserve the pinned
    // BAM model exactly; changed values are logged as a custom profile, never reference parity.
    let ratio = |value: f64, base: f64| -> Result<f64, String> {
        if !value.is_finite() || value < 0.0 || base <= 0.0 {
            return Err("native simulation parameters must be finite and nonnegative".into());
        }
        Ok(value / base)
    };
    profile.bam_friction_viscous *= ratio(simulation.damping, defaults.damping)?;
    profile.bam_friction_base *= ratio(simulation.friction_loss, defaults.friction_loss)?;
    profile.bam_armature *= ratio(simulation.armature, defaults.armature)?;
    profile.bam_kp *= ratio(simulation.kp, defaults.kp)?;
    profile.bam_max_pwm =
        (profile.bam_max_pwm * ratio(simulation.force_range, defaults.force_range)?).min(1.0);
    let snapshot = Arc::new(Mutex::new(SharedSnapshot::default()));
    let factory_snapshot = snapshot.clone();
    let mut sources = official_sources()?;
    sources.push((
        "resolved-profile".into(),
        format!("actuator={actuator_profile:?};servo={profile:?}").into_bytes(),
    ));
    sources.push(("initial-state".into(), format!("home={:?};trunk=[0,0,0.125,1,0,0,0];randomization=disabled;encoder_through_backlash={};delay={:?};delay_schedule=physics_step_history;delay_sampler=xorshift64;delay_seed=1;startup=sampling_until_policy;drive_position_gain_scale={};dt=0.005;decimation=4", template.control.home_pose, actuator_profile.encoder_through_backlash, actuator_profile.response.delay_steps, actuator_profile.response.position_gain_scale).into_bytes()));
    let contract = ResolvedContract::with_sources(
        &template.id,
        template.version,
        "none",
        "simulation",
        format!("{model:?}:mujoco-3.10.0:{}", actuator_profile.id),
        sources.clone(),
    );
    if registry.describe(machine_id).is_some() {
        if registry
            .describe(machine_id)
            .is_some_and(|descriptor| descriptor.kind != loop_body::BackendKind::Simulation)
        {
            return Err("backend ID is already registered with another kind".into());
        }
        registry
            .unregister(machine_id)
            .map_err(|error| error.to_string())?;
    }
    registry
        .register(
            loop_body::BackendDescriptor {
                id: machine_id.to_owned(),
                kind: loop_body::BackendKind::Simulation,
                supports_control: true,
                contract: contract.hash,
            },
            move || {
                let driver = NativeSimulation::open(
                    &template,
                    profile.clone(),
                    actuator_profile,
                    factory_snapshot.clone(),
                )?;
                Ok(Box::new(driver))
            },
        )
        .map_err(|error| error.to_string())?;
    Ok(snapshot)
}

pub(crate) fn official_sources() -> Result<Vec<(String, Vec<u8>)>, String> {
    let root = super::mujoco::resource_directory().map_err(|e| e.to_string())?;
    let mut paths = Vec::new();
    fn visit(
        root: &std::path::Path,
        path: &std::path::Path,
        paths: &mut Vec<std::path::PathBuf>,
    ) -> Result<(), String> {
        for entry in std::fs::read_dir(path).map_err(|e| e.to_string())? {
            let path = entry.map_err(|e| e.to_string())?.path();
            if path.is_dir() {
                visit(root, &path, paths)?;
            } else if path
                .extension()
                .is_some_and(|ext| ext == "xml" || ext == "stl")
            {
                paths.push(path);
            }
        }
        let _ = root;
        Ok(())
    }
    visit(&root, &root, &mut paths)?;
    paths.sort();
    paths
        .into_iter()
        .map(|path| {
            let label = path
                .strip_prefix(&root)
                .unwrap_or(&path)
                .to_string_lossy()
                .replace('\\', "/");
            let bytes = std::fs::read(&path).map_err(|e| e.to_string())?;
            Ok((label, bytes))
        })
        .collect()
}
