//! Native official simulation and desktop adapter for the shared body loop.
//!
//! Policy inference lives in the desktop policy runner; the product scene, native model and BAM
//! are backend-owned.

use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use super::mujoco::{
    NativeSimulation, SharedSnapshot, SimulationActuatorProfile, SnapshotHandle, runtime_version,
};
use crate::backend::contract::ResolvedContract;
use crate::backend::policy_runtime::{MicroDuckPolicyRunner, PolicyConfig, PolicyDescriptor};
use duck_body::{BodyTemplate, servo_profiles};
use duck_control::obs::Command;
use loop_body::{
    BodyCommand, BodyRuntimeView, Calibration, JointCalibration, LoopAction, LoopBinding,
    LoopConfig, LoopPhase as BodyLoopPhase, ServoBinding,
};
use servo::ServoModel;

use crate::logging::{
    LoggedServoDriver, append_control_event, append_policy_trace, begin_control_loop,
};
use crate::types::{LoopPhase, LoopStatus, SimulationParameters, WorldState};
use crate::types::{PolicyCommand, PreviewInput};

const PERIOD: Duration = Duration::from_millis(20);
const PHYSICS_DT: f64 = 0.005;
const PHYSICS_SUBSTEPS: usize = 4;
const STAND_POSITION_TOLERANCE: f64 = 0.03;
const STAND_VELOCITY_TOLERANCE: f64 = 0.3;
const STAND_STABLE_TICKS: u8 = 3;

#[derive(Debug, Clone)]
struct PreviewPolicySelection {
    path: PathBuf,
    command: Option<PolicyCommand>,
    descriptor: PolicyDescriptor,
    config: PolicyConfig,
    episodic: bool,
    duration: Option<Duration>,
}

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
    let actuator_profile = SimulationActuatorProfile::for_model(model)
        .map_err(|error| error.to_string())?;
    let model_key = match model {
        ServoModel::Hd1910 => duck_body::ServoModelKey::Hd1910,
        ServoModel::Xl330M288 => duck_body::ServoModelKey::Xl330,
        _ => return Err(format!("unsupported servo model {model:?}")),
    };
    let defaults = crate::helpers::default_simulation(model_key);
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
    sources.push(("initial-state".into(), format!("home={:?};trunk=[0,0,0.125,1,0,0,0];randomization=disabled;encoder_through_backlash={};delay={:?};dt=0.005;decimation=4", template.control.home_pose, actuator_profile.encoder_through_backlash, actuator_profile.delay_steps).into_bytes()));
    let contract = ResolvedContract::with_sources(
        &template.id,
        template.version,
        "none",
        "simulation",
        format!(
            "{model:?}:mujoco-3.10.0:{}",
            actuator_profile.id
        ),
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

pub(crate) struct PreviewMicroDuckPolicyRunner {
    loop_state: MicroDuckPolicyRunner,
    input: PreviewInput,
    policy_command: Option<PolicyCommand>,
    world: WorldState,
    snapshot: SnapshotHandle,
    config_dir: Option<PathBuf>,
    policy_ticks: u64,
    home_pose: Vec<f64>,
    active_policy: Option<PreviewPolicySelection>,
    pending_policy: Option<PreviewPolicySelection>,
    resume_policy: Option<PreviewPolicySelection>,
    preparing_stand: bool,
    stand_stable_ticks: u8,
    previous_positions: Option<Vec<f64>>,
    skill_elapsed: Duration,
    body_id: String,
    body_version: u32,
    backend_profile: String,
    backend_contract: String,
}

impl PreviewMicroDuckPolicyRunner {
    pub(crate) fn open(
        model: ServoModel,
        simulation: SimulationParameters,
        config_dir: Option<&Path>,
        machine_id: &str,
        machine_name: &str,
        registry: &mut loop_body::BackendRegistry,
    ) -> Result<Self, String> {
        let template = BodyTemplate::official().map_err(|error| error.to_string())?;
        let home = template.control.home_pose.clone();
        let binding_summary = template
            .actuators
            .iter()
            .enumerate()
            .map(|(index, slot)| format!("{}->{} dir=1 offset=0", slot.id, index + 1))
            .collect::<Vec<_>>()
            .join(" | ");
        let binding = LoopBinding {
            body: BodyRuntimeView::new(template.clone()).map_err(|error| error.to_string())?,
            servos: ServoBinding::all_slots(&template, model),
            calibration: Calibration {
                body_id: template.id.clone(),
                body_version: template.version,
                // Preview uses the canonical simulation IDs in body actuator order.  The
                // policy layer translates ONNX's 14 outputs to this 15-slot order; a physical
                // loop replaces only this calibration with its saved body-to-servo binding.
                joints: template
                    .actuators
                    .iter()
                    .enumerate()
                    .map(|(index, slot)| JointCalibration {
                        slot_id: slot.id.clone(),
                        physical_id: u8::try_from(index + 1)
                            .expect("official body has fewer than 256 joints"),
                        direction: 1,
                        zero_offset_rad: 0.0,
                        min_rad: None,
                        max_rad: None,
                    })
                    .collect(),
            },
        };
        let actuator_profile = SimulationActuatorProfile::for_model(model)
            .map_err(|error| error.to_string())?;
        append_control_event(
            config_dir,
            format!(
                "preview_actuator_config backend_kind=simulation implementation=OfficialCompatSimulationBackend model={model:?} actuator_profile={actuator_profile} encoder_through_backlash={encoder_through_backlash} mujoco_binding_version=3.10.0 mujoco_runtime_version={version} scene_path={scene} bam_source_revision=62bd8ce12154340be97e06f7f41a0ca8f116d967 control_dt={control_dt} physics_dt={physics_dt} physics_substeps={substeps} delay_timebase=physics_step delay_schedule=absolute_deadline command_delay_physics_steps={delay_steps:?} observation_delay_ticks=0 enforce_body_range=false imu_source=backend",
                scene = if model == ServoModel::Xl330M288 {
                    "scene_backlash.xml"
                } else {
                    "scene.xml"
                },
                version = runtime_version(),
                control_dt = PERIOD.as_secs_f64(),
                physics_dt = PHYSICS_DT,
                substeps = PHYSICS_SUBSTEPS,
                actuator_profile = actuator_profile.id,
                encoder_through_backlash = actuator_profile.encoder_through_backlash,
                delay_steps = actuator_profile.delay_steps
            ),
        );
        let resolved_contract = ResolvedContract::with_sources(
            &template.id,
            template.version,
            "none",
            "simulation",
            format!("{model:?}:mujoco-3.10.0:{}", actuator_profile.id),
            official_sources()?,
        );
        append_control_event(
            config_dir,
            format!(
                "resolved_contract body_id={} body_version={} policy_id={} backend_kind={} backend_profile={} contract_hash={}",
                resolved_contract.body_id,
                resolved_contract.body_version,
                resolved_contract.policy_id,
                resolved_contract.backend_kind,
                resolved_contract.backend_profile,
                resolved_contract.hash,
            ),
        );
        let snapshot = register_machine(registry, machine_id, model, &simulation)?;
        registry
            .select(machine_id)
            .map_err(|error| error.to_string())?;
        let registered_driver = registry
            .open(machine_id, loop_body::BackendAccess::Control)
            .map_err(|error| error.to_string())?;
        let backend_contract = registered_driver.descriptor().contract.clone();
        append_control_event(
            config_dir,
            format!(
                "backend_session backend_id={machine_id} backend_kind=simulation access=Control generation={} contract_hash={} physical_write_allowed=false",
                registered_driver.generation(),
                registered_driver.descriptor().contract
            ),
        );
        let driver: Box<dyn loop_body::ControlBackend> = if let Some(handle) =
            begin_control_loop(config_dir, machine_id, machine_name, &binding_summary)
        {
            Box::new(LoggedServoDriver::new(
                Box::new(registered_driver),
                handle,
                machine_id,
            ))
        } else {
            Box::new(registered_driver)
        };
        let preview_config = LoopConfig {
            homing_ticks: 0,
            enforce_body_range: false,
            velocity_observation_delay_ticks: 0,
            ..LoopConfig::default()
        };
        let mut loop_state = MicroDuckPolicyRunner::open(binding, driver, preview_config)
            .map_err(|error| error.to_string())?;
        // A preview starts from the policy's neutral command.  The observation is assembled
        // inside `MicroDuckPolicyRunner::tick` from the virtual sensors, the previous action and this
        // command; policy-specific commands (phase/posture/teleop) are applied by the preview
        // input layer after the manifest has been resolved.
        loop_state.set_policy_command(Command::default());
        let mut preview = Self {
            backend_contract,
            snapshot,
            loop_state,
            input: PreviewInput {
                twist: [0.0; 3],
                head: [0.0; 4],
                body: [0.0; 3],
                target_distance_m: None,
            },
            policy_command: None,
            world: WorldState {
                position: [0.0, 0.0],
                yaw: 0.0,
                distance_m: 0.0,
            },
            config_dir: config_dir.map(Path::to_path_buf),
            policy_ticks: 0,
            home_pose: home,
            active_policy: None,
            pending_policy: None,
            resume_policy: None,
            preparing_stand: false,
            stand_stable_ticks: 0,
            previous_positions: None,
            skill_elapsed: Duration::ZERO,
            body_id: template.id.clone(),
            body_version: template.version,
            backend_profile: format!("{model:?}:{}", actuator_profile.id),
        };
        preview
            .loop_state
            .send(LoopAction::Power(true))
            .map_err(|error| error.to_string())?;
        Ok(preview)
    }

    pub(crate) fn select_policy(
        &mut self,
        path: &Path,
        command: Option<PolicyCommand>,
        descriptor: PolicyDescriptor,
        config: PolicyConfig,
        kind: Option<&str>,
        duration_s: Option<f64>,
    ) -> Result<(), String> {
        if self.preparing_stand
            || self
                .active_policy
                .as_ref()
                .is_some_and(|policy| policy.episodic)
        {
            return Err("a skill is already running".into());
        }
        let episodic = kind == Some("episodic");
        let duration = if episodic {
            let seconds = duration_s
                .filter(|seconds| seconds.is_finite() && *seconds > 0.0)
                .ok_or_else(|| "episodic policy is missing a valid duration_s".to_owned())?;
            Some(
                Duration::try_from_secs_f64(seconds)
                    .map_err(|_| "episodic policy duration is out of range".to_owned())?,
            )
        } else {
            None
        };
        let selection = PreviewPolicySelection {
            path: path.to_owned(),
            command,
            descriptor,
            config,
            episodic,
            duration,
        };
        duck_control::policy::validate(path).map_err(|error| error.to_string())?;
        self.resume_policy = if episodic {
            self.active_policy.clone().filter(|policy| !policy.episodic)
        } else {
            None
        };
        self.pending_policy = None;
        self.activate_policy(selection)
    }

    pub(crate) fn set_input(&mut self, input: PreviewInput) {
        append_control_event(
            self.config_dir.as_deref(),
            format!(
                "preview_input twist={:?} head={:?} body={:?} target_distance_m={:?}",
                input.twist, input.head, input.body, input.target_distance_m
            ),
        );
        self.input = input;
        self.apply_command();
    }

    pub(crate) fn reset_world(&mut self) -> Result<(), String> {
        append_control_event(self.config_dir.as_deref(), "preview_world_reset");
        self.world = WorldState {
            position: [0.0, 0.0],
            yaw: 0.0,
            distance_m: 0.0,
        };
        self.loop_state
            .reset_execution()
            .map_err(|error| error.to_string())?;
        self.loop_state.reset_policy();
        self.policy_ticks = 0;
        self.skill_elapsed = Duration::ZERO;
        self.previous_positions = None;
        Ok(())
    }

    pub(crate) fn tick(&mut self) -> Result<LoopStatus, String> {
        self.ensure_driving();
        if self.preparing_stand {
            self.set_standing_command()?;
        }
        self.apply_command();
        self.loop_state
            .tick(PERIOD)
            .map_err(|error| error.to_string())?;
        if let Some(state) = self
            .snapshot
            .lock()
            .map_err(|error| error.to_string())?
            .state
            .as_ref()
        {
            let next = [state.trunk_position[0], state.trunk_position[1]];
            self.world.distance_m +=
                (next[0] - self.world.position[0]).hypot(next[1] - self.world.position[1]);
            self.world.position = next;
            let [w, x, y, z] = state.trunk_quaternion;
            self.world.yaw = (2.0 * (w * z + x * y)).atan2(1.0 - 2.0 * (y * y + z * z));
        }
        let has_trace = self.loop_state.policy_trace().is_some();
        if let Some(trace) = self.loop_state.policy_trace() {
            append_policy_trace(
                self.config_dir.as_deref(),
                self.loop_state.status().tick,
                trace,
            );
            self.policy_ticks += 1;
        }
        if self.preparing_stand {
            self.update_stand_transition()?;
        } else if has_trace {
            self.skill_elapsed += PERIOD;
            if let Some(policy) = &self.active_policy
                && policy.episodic
                && self.skill_elapsed >= policy.duration.unwrap_or_default()
            {
                let resume = self.resume_policy.take();
                self.active_policy = None;
                self.pending_policy = resume;
                self.begin_standing_transition()?;
            }
        }
        Ok(self.status())
    }

    fn target_distance_reached(&self) -> bool {
        has_reached_target(self.input.target_distance_m, self.world.distance_m)
    }

    fn begin_standing_transition(&mut self) -> Result<(), String> {
        append_control_event(
            self.config_dir.as_deref(),
            format!(
                "preview_prepare_stand pending={} resume={}",
                self.pending_policy.is_some(),
                self.resume_policy.is_some()
            ),
        );
        self.loop_state.clear_policy();
        self.policy_command = None;
        self.policy_ticks = 0;
        self.skill_elapsed = Duration::ZERO;
        self.preparing_stand = true;
        self.stand_stable_ticks = 0;
        self.previous_positions = None;
        self.set_standing_command()
    }

    fn set_standing_command(&mut self) -> Result<(), String> {
        let session = self.loop_state.status().session;
        self.loop_state
            .set_command(BodyCommand {
                session,
                targets: self.home_pose.clone(),
            })
            .map_err(|error| error.to_string())
    }

    fn ensure_driving(&mut self) {
        if matches!(
            self.loop_state.status().phase,
            BodyLoopPhase::Ready | BodyLoopPhase::Holding
        ) {
            let _ = self.loop_state.send(LoopAction::Drive(true));
        }
    }

    fn update_stand_transition(&mut self) -> Result<(), String> {
        let positions = self.loop_state.status().positions;
        let position_error = positions
            .iter()
            .zip(&self.home_pose)
            .map(|(position, home)| (position - home).abs())
            .fold(0.0, f64::max);
        let velocity = self
            .previous_positions
            .as_ref()
            .map(|previous| {
                positions
                    .iter()
                    .zip(previous)
                    .map(|(position, previous)| (position - previous).abs() / PERIOD.as_secs_f64())
                    .fold(0.0, f64::max)
            })
            .unwrap_or(f64::INFINITY);
        self.previous_positions = Some(positions);
        if position_error <= STAND_POSITION_TOLERANCE && velocity <= STAND_VELOCITY_TOLERANCE {
            self.stand_stable_ticks = self.stand_stable_ticks.saturating_add(1);
        } else {
            self.stand_stable_ticks = 0;
        }
        if self.stand_stable_ticks < STAND_STABLE_TICKS
            || self.loop_state.status().phase != BodyLoopPhase::Driving
        {
            return Ok(());
        }
        self.preparing_stand = false;
        self.stand_stable_ticks = 0;
        self.previous_positions = None;
        let Some(selection) = self.pending_policy.take() else {
            append_control_event(
                self.config_dir.as_deref(),
                "preview_stand_ready policy=none",
            );
            return Ok(());
        };
        self.activate_policy(selection)
    }

    fn activate_policy(&mut self, selection: PreviewPolicySelection) -> Result<(), String> {
        self.loop_state
            .select_policy_with_descriptor(&selection.path, &selection.descriptor, selection.config)
            .map_err(|error| error.to_string())?;
        let sources = vec![
            (
                "backend-contract",
                self.backend_contract.as_bytes().to_vec(),
            ),
            (
                "policy/onnx",
                std::fs::read(&selection.path).map_err(|error| error.to_string())?,
            ),
            (
                "policy/resolved",
                format!("{:?};{:?}", selection.descriptor, selection.config).into_bytes(),
            ),
        ];
        let resolved_contract = ResolvedContract::with_sources(
            &self.body_id,
            self.body_version,
            selection.descriptor.id.clone(),
            "simulation",
            self.backend_profile.clone(),
            sources,
        );
        append_control_event(
            self.config_dir.as_deref(),
            format!(
                "preview_policy_selected path={:?} config={:?} episodic={} duration_s={:?} contract_hash={}",
                selection.path,
                selection.config,
                selection.episodic,
                selection.duration.map(|duration| duration.as_secs_f64()),
                resolved_contract.hash,
            ),
        );
        self.policy_command = selection.command.clone();
        self.policy_ticks = 0;
        self.skill_elapsed = Duration::ZERO;
        self.active_policy = Some(selection);
        self.apply_command();
        Ok(())
    }

    fn apply_command(&mut self) {
        let twist = self.effective_twist();
        let (head, body) = if self
            .active_policy
            .as_ref()
            .is_some_and(|policy| policy.episodic)
        {
            ([0.0; 4], [0.0; 3])
        } else {
            (self.input.head, self.input.body)
        };
        self.loop_state.set_policy_command(Command {
            twist,
            head,
            body: duck_control::obs::BodyPose {
                z: body[0],
                roll: body[1],
                pitch: body[2],
            },
        });
    }

    fn effective_twist(&self) -> [f64; 3] {
        let mut twist = if self
            .active_policy
            .as_ref()
            .is_some_and(|policy| policy.episodic)
        {
            [0.0; 3]
        } else {
            self.input.twist
        };
        if let Some(command) = &self.policy_command {
            match command.encoding.as_deref() {
                Some("phase") => {
                    let period = command
                        .period_s
                        .filter(|period| *period > 0.0)
                        .unwrap_or(4.0);
                    let phase = self.policy_ticks as f64 * PERIOD.as_secs_f64() / period;
                    let phase = command
                        .end_phase
                        .filter(|end| end.is_finite() && *end >= 0.0)
                        .map_or(phase, |end| phase.min(end));
                    let angle = std::f64::consts::TAU * phase;
                    twist = [angle.cos(), angle.sin(), 0.0];
                }
                Some("posture_flag") => {
                    let sit = command.sit.unwrap_or(1.0);
                    let stand = command.stand.unwrap_or(0.0);
                    // The UI carries the requested posture in twist.vx.  Pick the nearest
                    // manifest value so the policy receives the exact trained flag instead of
                    // an arbitrary slider value.
                    twist[0] = if (self.input.twist[0] - sit).abs()
                        <= (self.input.twist[0] - stand).abs()
                    {
                        sit
                    } else {
                        stand
                    };
                }
                _ => {
                    if self.input.twist == [0.0; 3]
                        && let Some(idle) = command.idle
                    {
                        twist = idle;
                    }
                }
            }
        }
        if self.target_distance_reached() {
            twist = [0.0; 3];
        }
        twist
    }

    pub(crate) fn action(&mut self, action: LoopAction) -> Result<LoopStatus, String> {
        self.loop_state
            .send(action)
            .map_err(|error| error.to_string())?;
        Ok(self.status())
    }

    pub(crate) fn status(&self) -> LoopStatus {
        let status = self.loop_state.status();
        LoopStatus {
            has_execution: true,
            runtime_state: crate::types::RuntimeState::Running,
            simulation: self
                .snapshot
                .lock()
                .ok()
                .and_then(|snapshot| snapshot.state.clone()),
            read_sources: Vec::new(),
            phase: status.phase.into(),
            session: status.session,
            tick: status.tick,
            torque: status.torque,
            positions: status.positions,
            targets: status.targets,
            world: self.world,
            sensor_failures: status.sensor_failures,
            last_error: status.last_error,
            joint_range_states: status
                .joint_range_states
                .into_iter()
                .map(|state| crate::types::JointRangeState {
                    slot_id: state.slot_id,
                    target_rad: state.target_rad,
                    min_rad: state.min_rad,
                    max_rad: state.max_rad,
                    below_min: state.below_min,
                    above_max: state.above_max,
                })
                .collect(),
            runtime_generation: 0,
            execution_generation: status.session,
            config_revision: 0,
        }
    }
}

fn has_reached_target(target: Option<f64>, distance: f64) -> bool {
    target.is_some_and(|target| target.is_finite() && target > 0.0 && distance >= target)
}

impl From<BodyLoopPhase> for LoopPhase {
    fn from(phase: BodyLoopPhase) -> Self {
        match phase {
            BodyLoopPhase::Limp => Self::Limp,
            BodyLoopPhase::Homing => Self::Homing,
            BodyLoopPhase::Ready => Self::Ready,
            BodyLoopPhase::Driving => Self::Driving,
            BodyLoopPhase::Holding => Self::Holding,
            BodyLoopPhase::Stopped => Self::Stopped,
            BodyLoopPhase::Reconnecting => Self::Reconnecting,
            BodyLoopPhase::Estop => Self::Estop,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::{PreviewMicroDuckPolicyRunner, PreviewPolicySelection, has_reached_target};
    use crate::backend::policy_runtime::{PolicyConfig, PolicyDescriptor};
    use duck_body::{BodyTemplate, ServoModelKey};
    use servo::ServoModel;
    use std::time::{SystemTime, UNIX_EPOCH};

    #[test]
    fn preview_loop_uses_shared_policy_loop_without_hardware() {
        let mut registry = loop_body::BackendRegistry::default();
        let mut loop_state = PreviewMicroDuckPolicyRunner::open(
            ServoModel::Xl330M288,
            crate::helpers::default_simulation(ServoModelKey::Xl330),
            None,
            "test-machine",
            "测试机器",
            &mut registry,
        )
        .unwrap();
        assert_eq!(loop_state.status().phase, crate::types::LoopPhase::Ready);
        loop_state.tick().unwrap();
        assert_eq!(
            loop_state.status().simulation.unwrap().physics_step,
            4,
            "logging must forward native execution"
        );
        let status = loop_state.status();
        assert_eq!(status.phase, crate::types::LoopPhase::Driving);
        assert_eq!(status.positions.len(), 15);
        assert_eq!(status.targets.len(), 15);
        assert_eq!(
            status.positions,
            BodyTemplate::official().unwrap().control.home_pose
        );
        assert_eq!(status.sensor_failures, 0);
        for _ in 0..20 {
            loop_state.tick().unwrap();
        }
        loop_state.reset_world().unwrap();
        let reset = loop_state.status();
        assert_eq!(reset.simulation.as_ref().unwrap().physics_step, 0);
        assert_eq!(reset.simulation.as_ref().unwrap().time, 0.0);
        assert_eq!(
            reset.positions,
            BodyTemplate::official().unwrap().control.home_pose
        );
        loop_state.tick().unwrap();
        assert_eq!(loop_state.status().simulation.unwrap().physics_step, 4);
    }

    #[test]
    fn preview_loop_logs_virtual_servo_io() {
        let suffix = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_nanos();
        let root = std::env::temp_dir().join(format!("microduck-preview-log-{suffix}"));
        let config_dir = root.join("config");
        let mut registry = loop_body::BackendRegistry::default();
        let mut loop_state = PreviewMicroDuckPolicyRunner::open(
            ServoModel::Xl330M288,
            crate::helpers::default_simulation(ServoModelKey::Xl330),
            Some(&config_dir),
            "virtual-1",
            "模拟机",
            &mut registry,
        )
        .unwrap();
        loop_state.tick().unwrap();
        let log =
            std::fs::read_to_string(crate::logging::current_log_file(Some(&config_dir))).unwrap();
        assert!(log.contains("loop_open machine_id=virtual-1"));
        assert!(log.contains("resolved_contract body_id=official"));
        assert!(log.contains("backend_kind=simulation"));
        assert!(log.contains("contract_hash="));
        assert!(log.contains("physics_dt=0.005"));
        assert!(log.contains("physics_substeps=4"));
        assert!(log.contains("delay_timebase=physics_step"));
        assert!(log.contains("servo_read id=1"));
        let _ = std::fs::remove_dir_all(root);
    }

    /// Export a product-side trace against the same pinned artifact used by the CPU MuJoCo
    /// reference runner. This is intentionally ignored because the desktop test environment
    /// does not ship ONNX Runtime or the artifact; run it manually with
    /// `MICRODUCK_OFFICIAL_ONNX` and `ORT_DYLIB_PATH` set to produce a comparable trace.
    #[test]
    #[ignore = "requires the official ONNX artifact and a dynamic ONNX Runtime"]
    fn official_velstand_native_parity_for_300_ticks() {
        let policy_path = std::env::var("MICRODUCK_OFFICIAL_ONNX")
            .expect("MICRODUCK_OFFICIAL_ONNX must point to the pinned velstand.onnx");
        use sha2::{Digest, Sha256};
        let policy_sha256 = format!("{:x}", Sha256::digest(std::fs::read(&policy_path).unwrap()));
        let output_path = std::env::var("MICRODUCK_OFFICIAL_PRODUCT_TRACE")
            .unwrap_or_else(|_| ".agentic/official-artifact/product-preview-trace.json".into());
        let mut registry = loop_body::BackendRegistry::default();
        let mut preview = PreviewMicroDuckPolicyRunner::open(
            servo::ServoModel::Xl330M288,
            crate::helpers::default_simulation(ServoModelKey::Xl330),
            None,
            "official-velstand-trace",
            "official velstand trace",
            &mut registry,
        )
        .unwrap();
        preview.set_input(super::PreviewInput {
            twist: [0.2, 0.0, 0.0],
            head: [0.0; 4],
            body: [0.0; 3],
            target_distance_m: None,
        });
        preview
            .activate_policy(PreviewPolicySelection {
                path: policy_path.into(),
                command: None,
                descriptor: PolicyDescriptor {
                    id: "velstand".into(),
                    body_id: "official".into(),
                    body_version: 1,
                    obs_len: 61,
                    action_len: 14,
                    control_hz: 50,
                    command_encoding: None,
                    recurrent: Some(false),
                    action_scale: Some(1.0),
                    head_lowpass: None,
                    legs_lowpass: None,
                    joint_velocity_delay_ticks: 0,
                },
                config: PolicyConfig::default(),
                episodic: false,
                duration: None,
            })
            .unwrap();

        let mut ticks = Vec::new();
        for tick in 0..300 {
            preview.set_input(super::PreviewInput {
                twist: [if (75..225).contains(&tick) { 0.2 } else { 0.0 }, 0.0, 0.0],
                head: [0.0; 4],
                body: [0.0; 3],
                target_distance_m: None,
            });
            preview.tick().unwrap();
            let trace = preview
                .loop_state
                .policy_trace()
                .cloned()
                .expect("active policy must emit a trace");
            let sample = preview.snapshot.lock().unwrap().state.clone().unwrap();
            ticks.push(serde_json::json!({
                "tick": tick,
                "observation": trace.observation,
                "action": trace.action,
                "raw_targets": trace.raw_targets,
                "canonical_target": trace.body_targets,
                "backend_positions": trace.backend_state.positions,
                "backend_velocities": trace.backend_state.velocities,
                "imu_gyro": trace.imu.gyro,
                "imu_gravity": trace.imu.gravity,
                "execution_positions": trace.execution_positions,
                "simulation": sample,
            }));
        }
        let reference_path = std::env::var("MICRODUCK_OFFICIAL_REFERENCE_TRACE")
            .expect("set MICRODUCK_OFFICIAL_REFERENCE_TRACE to the pinned CPU trace");
        let reference: serde_json::Value =
            serde_json::from_slice(&std::fs::read(reference_path).unwrap()).unwrap();
        assert_eq!(
            reference["policy_sha256"].as_str().unwrap(),
            policy_sha256,
            "ONNX must match the reference artifact"
        );
        let resource_root = super::super::mujoco::resource_directory().unwrap();
        for (path, hash) in reference["asset_sha256"].as_object().unwrap() {
            assert_eq!(
                format!(
                    "{:x}",
                    Sha256::digest(std::fs::read(resource_root.join(path)).unwrap())
                ),
                hash.as_str().unwrap(),
                "reference asset changed: {path}"
            );
        }
        let expected = reference["case"]["frames"].as_array().unwrap();
        assert_eq!(ticks.len(), expected.len());
        let mut maxima = std::collections::BTreeMap::<String, f64>::new();
        let mut first_failure = None;
        for (tick, (actual, expected)) in ticks.iter().zip(expected).enumerate() {
            assert_eq!(
                actual["simulation"]["contacts"], expected["contacts"],
                "contact pairs at tick {tick}"
            );
            for (field, tolerance) in reference["tolerances"].as_object().unwrap() {
                let value = match field.as_str() {
                    "target" => &actual["canonical_target"],
                    "positions" => &actual["backend_positions"],
                    "velocities" => &actual["backend_velocities"],
                    "gyro" => &actual["imu_gyro"],
                    "gravity" => &actual["imu_gravity"],
                    "observation" | "action" => &actual[field],
                    "motor_positions" => &actual["simulation"]["motorPositions"],
                    "passive_positions" => &actual["simulation"]["passivePositions"],
                    "trunk_position" => &actual["simulation"]["trunkPosition"],
                    "trunk_quaternion" => &actual["simulation"]["trunkQuaternion"],
                    "base_velocity" => &actual["simulation"]["baseVelocity"],
                    "time" => &actual["simulation"]["time"],
                    _ => panic!("unknown reference field {field}"),
                };
                // Feedback and IMU in PolicyTrace belong to the pre-execution sample of this
                // tick. The physics reference stores post-step fields; compare to prior frame.
                let feedback = matches!(
                    field.as_str(),
                    "positions" | "velocities" | "gyro" | "gravity"
                );
                if feedback && tick == 0 {
                    continue;
                }
                let expected = if feedback {
                    &reference["case"]["frames"][tick - 1][field]
                } else {
                    &expected[field]
                };
                let error = if field == "time" {
                    (value.as_f64().unwrap() - expected.as_f64().unwrap()).abs()
                } else {
                    let value = value.as_array().unwrap();
                    let expected = expected.as_array().unwrap();
                    assert_eq!(value.len(), expected.len(), "field={field} tick={tick}");
                    let direct = value
                        .iter()
                        .zip(expected)
                        .map(|(a, e)| (a.as_f64().unwrap() - e.as_f64().unwrap()).abs())
                        .fold(0.0, f64::max);
                    if field == "trunk_quaternion" {
                        direct.min(
                            value
                                .iter()
                                .zip(expected)
                                .map(|(a, e)| (a.as_f64().unwrap() + e.as_f64().unwrap()).abs())
                                .fold(0.0, f64::max),
                        )
                    } else {
                        direct
                    }
                };
                let max = maxima.entry(field.clone()).or_default();
                *max = max.max(error);
                if error > tolerance.as_f64().unwrap() && first_failure.is_none() {
                    first_failure = Some(format!("tick={tick} field={field} error={error}"));
                }
            }
        }
        let report = serde_json::json!({"ticks":ticks.len(),"max_abs_error":maxima,"first_failure":first_failure});
        std::fs::write(
            std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
                .join("../../../.agentic/native-policy-comparison.json"),
            serde_json::to_vec_pretty(&report).unwrap(),
        )
        .unwrap();
        if let Some(parent) = std::path::Path::new(&output_path).parent() {
            std::fs::create_dir_all(parent).unwrap();
        }
        std::fs::write(
            output_path,
            serde_json::to_vec_pretty(&serde_json::json!({
                "backend": "product_official_compat_simulation",
                "model": "xl330",
                "command": [0.2, 0.0, 0.0],
                "physics_dt": 0.005,
                "control_substeps": 4,
                "ticks": ticks,
            }))
            .unwrap(),
        )
        .unwrap();
        assert!(first_failure.is_none(), "{report}");
    }

    #[test]
    fn preview_target_distance_only_finishes_for_positive_finite_targets() {
        assert!(!has_reached_target(None, 10.0));
        assert!(!has_reached_target(Some(0.0), 10.0));
        assert!(!has_reached_target(Some(f64::NAN), 10.0));
        assert!(!has_reached_target(Some(2.0), 1.99));
        assert!(has_reached_target(Some(2.0), 2.0));
    }
}
