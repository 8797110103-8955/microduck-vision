//! Desktop policy scheduling over the active simulation or physical body.

use std::path::{Path, PathBuf};
use std::time::Duration;

use super::{MicroDuckPolicyRunner, PolicyConfig, PolicyDescriptor};
use crate::backend::contract::ResolvedContract;
use crate::backend::mujoco::{
    SimulationActuatorProfile, SnapshotHandle, runtime_version,
};
use crate::backend::simulation::{official_sources, register_machine};
use crate::logging::{
    LoggedServoDriver, append_control_event, append_policy_trace, begin_control_loop,
};
use crate::types::{
    LoopPhase, LoopStatus, PolicyCommand, PreviewInput, SimulationParameters, WorldState,
};
use duck_body::BodyTemplate;
use duck_control::obs::Command;
use loop_body::{
    BodyRuntimeView, Calibration, JointCalibration, LoopAction, LoopBinding, LoopConfig,
    LoopPhase as BodyLoopPhase, ServoBinding,
};
use servo::ServoModel;

const PERIOD: Duration = Duration::from_millis(20);
const PHYSICS_DT: f64 = 0.005;
const PHYSICS_SUBSTEPS: usize = 4;
#[derive(Debug, Clone)]
struct PreviewPolicySelection {
    path: PathBuf,
    command: Option<PolicyCommand>,
    descriptor: PolicyDescriptor,
    config: PolicyConfig,
    episodic: bool,
    duration: Option<Duration>,
}

#[derive(Clone)]
struct PreparedActivation {
    selection: PreviewPolicySelection,
    runtime: super::policy::PreparedPolicy,
    sha256: String,
}

pub(crate) struct PolicyExecution {
    loop_state: MicroDuckPolicyRunner,
    virtual_machine_id: String,
    input: PreviewInput,
    policy_command: Option<PolicyCommand>,
    world: WorldState,
    coordinates: super::world::WorldCoordinates,
    placements: Option<std::sync::Arc<super::world::WorldPlacements>>,
    command_distance_origin_m: f64,
    simulation_body: Option<SimulationBody>,
    config_dir: Option<PathBuf>,
    policy_ticks: u64,
    active_policy: Option<PreviewPolicySelection>,
    policies: std::collections::HashMap<String, PreparedActivation>,
    body_id: String,
    body_version: u32,
    backend_profile: String,
    backend_contract: String,
    physical_id: Option<String>,
    detached: bool,
    policy_sha256: Option<String>,
    control_paused: bool,
    stop_confirmed: bool,
}

struct SimulationBody {
    snapshot: SnapshotHandle,
    actuator_profile: SimulationActuatorProfile,
}

impl PolicyExecution {
    pub(crate) fn recording_metadata(&self) -> serde_json::Value {
        let policy = self.active_policy.as_ref();
        serde_json::json!({
            "body_id":self.body_id,"body_version":self.body_version,
            "backend_profile":self.backend_profile,"backend_contract":self.backend_contract,
            "policy_id":policy.map(|p| &p.descriptor.id),
            "policy_contract":policy.map(|p| &p.descriptor),
            "policy_sha256":self.policy_sha256,
            "policy_config":policy.map(|p| format!("{:?}", p.config)),
            "world_basis":"right_handed_z_up","world_units":"m_rad",
            "simulation_origin":"session_local","world_placement":"per_virtual_machine",
            "observation_len":policy.map(|p| p.descriptor.obs_len),
            "action_len":policy.map(|p| p.descriptor.action_len),
            "control_period_s":PERIOD.as_secs_f64(),
            "joint_units":"rad","joint_order":duck_control::model::JOINT_NAMES,
        })
    }
    pub(crate) fn open(
        model: ServoModel,
        simulation: SimulationParameters,
        config_dir: Option<&Path>,
        machine_id: &str,
        machine_name: &str,
        registry: &mut loop_body::BackendRegistry,
    ) -> Result<Self, String> {
        let template = BodyTemplate::official().map_err(|error| error.to_string())?;
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
        let actuator_profile =
            SimulationActuatorProfile::for_model(model).map_err(|error| error.to_string())?;
        append_control_event(
            config_dir,
            format!(
                "preview_actuator_config backend_kind=simulation implementation=OfficialCompatSimulationBackend model={model:?} actuator_profile={actuator_profile} encoder_through_backlash={encoder_through_backlash} drive_position_gain_scale={drive_position_gain_scale} drive_back_emf_scale={drive_back_emf_scale} drive_friction_scale={drive_friction_scale} mujoco_binding_version=3.10.0 mujoco_runtime_version={version} scene_path={scene} bam_source_revision=62bd8ce12154340be97e06f7f41a0ca8f116d967 control_dt={control_dt} physics_dt={physics_dt} physics_substeps={substeps} delay_timebase=physics_step delay_schedule=physics_step_history command_delay_physics_steps={delay_steps:?} observation_delay_ticks=0 enforce_body_range=false imu_source=backend",
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
                drive_position_gain_scale = actuator_profile.response.position_gain_scale,
                drive_back_emf_scale = actuator_profile.response.back_emf_scale,
                drive_friction_scale = actuator_profile.response.friction_scale,
                delay_steps = actuator_profile.response.delay_steps
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
            virtual_machine_id: machine_id.to_owned(),
            backend_contract,
            simulation_body: Some(SimulationBody {
                snapshot,
                actuator_profile,
            }),
            physical_id: None,
            detached: false,
            policy_sha256: None,
            control_paused: false,
            stop_confirmed: false,
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
            coordinates: super::world::WorldCoordinates::default(),
            placements: None,
            config_dir: config_dir.map(Path::to_path_buf),
            policy_ticks: 0,
            command_distance_origin_m: 0.0,
            active_policy: None,
            policies: std::collections::HashMap::new(),
            body_id: template.id.clone(),
            body_version: template.version,
            backend_profile: format!("{model:?}:{}", actuator_profile.id),
        };
        preview
            .loop_state
            .send(LoopAction::Power(true))
            .map_err(|error| error.to_string())?;
        preview.observe_world(false)?;
        preview.preload_cached_policies()?;
        Ok(preview)
    }

    pub(crate) fn restore_placement(
        &mut self,
        placements: std::sync::Arc<super::world::WorldPlacements>,
    ) {
        if let Some(world) = placements.get(&self.virtual_machine_id) {
            self.world = world;
            self.command_distance_origin_m = world.distance_m;
        }
        self.placements = Some(placements);
    }

    fn remember_placement(&self, flush: bool) {
        if let Some(placements) = &self.placements {
            placements.remember(&self.virtual_machine_id, self.world);
            if flush && let Err(error) = placements.flush() {
                append_control_event(
                    self.config_dir.as_deref(),
                    format!("world_placement_save_failed error={error}"),
                );
            }
        }
    }

    fn observe_world(&mut self, driven: bool) -> Result<(), String> {
        if let Some(body) = &self.simulation_body
            && let Some(state) = body
                .snapshot
                .lock()
                .map_err(|error| error.to_string())?
                .state
                .as_ref()
        {
            self.coordinates.observe(&mut self.world, state, driven);
        }
        self.remember_placement(false);
        Ok(())
    }

    fn preload_cached_policies(&mut self) -> Result<(), String> {
        use crate::{application::policy};
        let Some(config) = self.config_dir.as_deref() else {
            return Ok(());
        };
        if !policy::cache_root(Some(config))
            .join("manifest.json")
            .is_file()
        {
            return Ok(());
        }
        let entries = policy::list(Some(config))?;
        for entry in entries.into_iter().filter(|entry| entry.cached) {
            let selection = PreviewPolicySelection {
                path: policy::cache_root(Some(config)).join(&entry.file),
                command: entry.command.clone(),
                descriptor: policy::descriptor(Some(config), &entry)?,
                config: policy::runtime_config(Some(config), &entry)?,
                episodic: entry.kind.as_deref() == Some("episodic"),
                duration: entry
                    .duration_s
                    .filter(|duration| duration.is_finite() && *duration > 0.0)
                    .and_then(|duration| Duration::try_from_secs_f64(duration).ok()),
            };
            let id = selection.descriptor.id.clone();
            self.policies
                .insert(id.clone(), self.prepare_activation(selection)?);
            append_control_event(Some(config), format!("policy_prewarmed policy_id={id}"));
        }
        append_control_event(
            Some(config),
            format!("policy_prewarm_complete count={}", self.policies.len()),
        );
        Ok(())
    }

    pub(crate) fn physical_id(&self) -> Option<&str> {
        self.physical_id.as_deref()
    }

    pub(crate) fn virtual_machine_id(&self) -> &str {
        &self.virtual_machine_id
    }

    pub(crate) fn is_detached(&self) -> bool {
        self.detached
    }

    pub(crate) fn release_body(
        &mut self,
        registry: &mut loop_body::BackendRegistry,
        id: &str,
    ) -> Result<(), String> {
        if !self.detached {
            self.stop()?;
            registry.unregister(id).map_err(|error| error.to_string())?;
            self.mark_detached();
        }
        Ok(())
    }

    pub(crate) fn mark_detached(&mut self) {
        self.detached = true;
    }

    pub(crate) fn stop(&mut self) -> Result<(), String> {
        if !self.detached && !self.stop_confirmed {
            self.loop_state
                .send(LoopAction::Stop)
                .map_err(|error| error.to_string())?;
            self.stop_confirmed = true;
        }
        self.remember_placement(true);
        Ok(())
    }

    pub(crate) fn switch_to_simulation(&mut self, next: Self) -> Result<(), String> {
        self.install_body(next.loop_state)?;
        self.world = next.world;
        self.command_distance_origin_m = self.world.distance_m;
        self.coordinates = next.coordinates;
        self.simulation_body = next.simulation_body;
        self.physical_id = None;
        self.virtual_machine_id = next.virtual_machine_id;
        self.backend_contract = next.backend_contract;
        self.backend_profile = next.backend_profile;
        Ok(())
    }

    pub(crate) fn switch_to_physical(
        &mut self,
        id: String,
        runner: MicroDuckPolicyRunner,
        profile: String,
        contract: String,
    ) -> Result<(), String> {
        self.install_body(runner)?;
        self.simulation_body = None;
        self.physical_id = Some(id);
        self.backend_profile = profile;
        self.backend_contract = contract;
        Ok(())
    }

    fn install_body(&mut self, runner: MicroDuckPolicyRunner) -> Result<(), String> {
        let delay = self
            .active_policy
            .as_ref()
            .map_or(0, |policy| policy.descriptor.joint_velocity_delay_ticks);
        self.loop_state
            .replace_body(runner, delay)
            .map_err(|error| error.to_string())?;
        self.detached = false;
        self.stop_confirmed = false;
        self.command_distance_origin_m = self.world.distance_m;
        self.policy_ticks = 0;
        self.apply_command();
        Ok(())
    }

    pub(crate) fn runner(&self) -> &MicroDuckPolicyRunner {
        &self.loop_state
    }

    pub(crate) fn runner_mut(&mut self) -> &mut MicroDuckPolicyRunner {
        &mut self.loop_state
    }

    pub(crate) fn select_policy(&mut self, policy_id: &str) -> Result<(), String> {
        if self.physical_id.is_some() && !self.loop_state.status().torque {
            return Err("请先开启真机力矩".into());
        }
        let prepared = self
            .policies
            .get(policy_id)
            .cloned()
            .ok_or_else(|| format!("策略 {policy_id} 尚未预热，请重新启动控制环"))?;
        if self.physical_id.is_some() && prepared.selection.config.imu_required {
            return Err("当前真机后端无法提供该策略所需的 IMU 反馈".into());
        }
        if self
            .active_policy
            .as_ref()
            .is_some_and(|active| active.descriptor.id == policy_id)
        {
            self.command_distance_origin_m = self.world.distance_m;
            self.control_paused = false;
            self.ensure_driving();
            self.apply_command();
            return Ok(());
        }
        let previous = self
            .active_policy
            .as_ref()
            .map(|policy| policy.descriptor.id.clone());
        self.activate_prepared(prepared)?;
        self.control_paused = false;
        self.ensure_driving();
        append_control_event(
            self.config_dir.as_deref(),
            format!(
                "policy_handoff previous={previous:?} next={policy_id} pose_source=current_body gap_ticks=0"
            ),
        );
        Ok(())
    }

    pub(crate) fn set_input(&mut self, input: PreviewInput) {
        if input.target_distance_m != self.input.target_distance_m
            || (self.input.twist == [0.0; 3] && input.twist != [0.0; 3])
        {
            self.command_distance_origin_m = self.world.distance_m;
        }
        append_control_event(
            self.config_dir.as_deref(),
            format!(
                "preview_input twist={:?} head={:?} body={:?} target_distance_m={:?} distance_origin_m={}",
                input.twist,
                input.head,
                input.body,
                input.target_distance_m,
                self.command_distance_origin_m
            ),
        );
        self.input = input;
        self.apply_command();
    }

    pub(crate) fn reset_world(&mut self) -> Result<(), String> {
        if self.simulation_body.is_none() {
            return Err("当前真机没有可归零的世界坐标".into());
        }
        append_control_event(self.config_dir.as_deref(), "preview_world_reset");
        self.world = WorldState {
            position: [0.0, 0.0],
            yaw: 0.0,
            distance_m: 0.0,
        };
        self.command_distance_origin_m = 0.0;
        self.loop_state
            .reset_execution()
            .map_err(|error| error.to_string())?;
        self.loop_state.reset_policy();
        self.coordinates = super::world::WorldCoordinates::default();
        self.observe_world(false)?;
        self.remember_placement(true);
        self.policy_ticks = 0;
        Ok(())
    }

    pub(crate) fn tick(&mut self) -> Result<LoopStatus, String> {
        if self.control_paused {
            self.loop_state
                .sample_body()
                .map_err(|error| error.to_string())?;
            self.observe_world(false)?;
            return Ok(self.status());
        }
        if self.simulation_body.is_some() && self.loop_state.has_policy() {
            self.ensure_driving();
        }
        self.apply_command();
        self.loop_state
            .tick(PERIOD)
            .map_err(|error| error.to_string())?;
        let has_policy_trace = self.loop_state.policy_trace().is_some();
        self.observe_world(has_policy_trace)?;
        let status = self.status();
        if has_policy_trace
            && let Some(simulation) = status.simulation.as_ref()
        {
            append_control_event(
                self.config_dir.as_deref(),
                format!(
                    "simulation_state physics_step={} trunk_position={:?} trunk_quaternion={:?} base_velocity={:?} contacts={} world_position={:?} world_yaw={} world_distance_m={}",
                    simulation.physics_step,
                    simulation.trunk_position,
                    simulation.trunk_quaternion,
                    simulation.base_velocity,
                    simulation.contacts.len(),
                    status.world.position,
                    status.world.yaw,
                    status.world.distance_m,
                ),
            );
        }
        if let Some(trace) = self.loop_state.policy_trace() {
            append_policy_trace(
                self.config_dir.as_deref(),
                self.loop_state.status().tick,
                trace,
            );
            self.policy_ticks += 1;
        }
        Ok(status)
    }

    fn target_distance_reached(&self) -> bool {
        self.simulation_body.is_some()
            && has_reached_target(
                self.input.target_distance_m,
                (self.world.distance_m - self.command_distance_origin_m).max(0.0),
            )
    }

    fn ensure_driving(&mut self) {
        if matches!(
            self.loop_state.status().phase,
            BodyLoopPhase::Ready | BodyLoopPhase::Holding
        ) {
            let _ = self.loop_state.send(LoopAction::Drive(true));
        }
    }

    #[cfg(test)]
    fn activate_policy(&mut self, selection: PreviewPolicySelection) -> Result<(), String> {
        let prepared = self.prepare_activation(selection)?;
        self.activate_prepared(prepared)
    }

    fn prepare_activation(
        &self,
        selection: PreviewPolicySelection,
    ) -> Result<PreparedActivation, String> {
        use sha2::{Digest, Sha256};
        let policy_bytes = std::fs::read(&selection.path).map_err(|e| e.to_string())?;
        let policy_sha256 = format!("{:x}", Sha256::digest(&policy_bytes));
        if self.physical_id.is_some() && selection.config.imu_required {
            return Err("当前真机后端无法提供该策略所需的 IMU 反馈".into());
        }
        let runtime = self
            .loop_state
            .prepare_policy(&selection.path, &selection.descriptor, selection.config)
            .map_err(|error| error.to_string())?;
        Ok(PreparedActivation {
            selection,
            runtime,
            sha256: policy_sha256,
        })
    }

    fn activate_prepared(&mut self, prepared: PreparedActivation) -> Result<(), String> {
        let PreparedActivation {
            selection,
            runtime,
            sha256: policy_sha256,
        } = prepared;
        let response = self
            .simulation_body
                .as_ref()
                .map(|body| body.actuator_profile.response);
        let resolved_contract = ResolvedContract::with_sources(
            &self.body_id,
            self.body_version,
            selection.descriptor.id.clone(),
            if self.simulation_body.is_some() {
                "simulation"
            } else {
                "physical"
            },
            self.backend_profile.clone(),
            vec![
                (
                    "backend-contract",
                    self.backend_contract.as_bytes().to_vec(),
                ),
                ("policy/sha256", policy_sha256.as_bytes().to_vec()),
                (
                    "policy/resolved",
                    format!(
                        "{:?};{:?};actuator_response={response:?}",
                        selection.descriptor, selection.config
                    )
                    .into_bytes(),
                ),
            ],
        );
        self.loop_state
            .install_policy(runtime)
            .map_err(|error| error.to_string())?;
        append_control_event(
            self.config_dir.as_deref(),
            format!(
                "preview_policy_selected path={:?} config={:?} episodic={} duration_s={:?} contract_hash={} actuator_response={response:?}",
                selection.path,
                selection.config,
                selection.episodic,
                selection.duration.map(|duration| duration.as_secs_f64()),
                resolved_contract.hash,
            ),
        );
        self.policy_command = selection.command.clone();
        self.command_distance_origin_m = self.world.distance_m;
        self.policy_ticks = 0;
        self.active_policy = Some(selection);
        self.policy_sha256 = Some(policy_sha256);
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
        if matches!(action, LoopAction::Power(true)) {
            // Enabling torque can succeed before a later gain/write error. Cleanup must
            // retry until an explicit torque-off transaction has actually succeeded.
            self.stop_confirmed = false;
        }
        if matches!(action, LoopAction::Power(true)) && self.physical_id.is_some() {
            self.loop_state
                .arm_at_current_pose()
                .map_err(|error| error.to_string())?;
            self.control_paused = true;
            append_control_event(
                self.config_dir.as_deref(),
                format!(
                    "physical_torque_enabled physical_machine_id={:?} control_mode=hold pose_source=current_body",
                    self.physical_id
                ),
            );
            return Ok(self.status());
        }
        if matches!(action, LoopAction::Drive(true)) {
            if !self.loop_state.has_policy() {
                return Err("请先选择已缓存的 ONNX 策略".into());
            }
            if self.physical_id.is_some() && !self.loop_state.status().torque {
                return Err("请先开启真机力矩".into());
            }
            if self.physical_id.is_some()
                && self
                    .active_policy
                    .as_ref()
                    .is_some_and(|policy| policy.config.imu_required)
            {
                return Err("当前真机后端无法提供该策略所需的 IMU 反馈".into());
            }
        }
        if matches!(action, LoopAction::Drive(false)) {
            self.control_paused = true;
            if self.loop_state.status().phase != BodyLoopPhase::Driving {
                return Ok(self.status());
            }
        }
        self.loop_state
            .send(action)
            .map_err(|error| error.to_string())?;
        if matches!(
            action,
            LoopAction::Stop | LoopAction::Estop | LoopAction::Power(false)
        ) {
            self.stop_confirmed = true;
        }
        self.control_paused = !matches!(action, LoopAction::Drive(true) | LoopAction::Power(true));
        Ok(self.status())
    }

    pub(crate) fn status(&self) -> LoopStatus {
        let status = self.loop_state.status();
        LoopStatus {
            has_execution: true,
            active_body: (!self.detached).then(|| crate::types::ActiveBody {
                virtual_machine_id: Some(self.virtual_machine_id.clone()),
                physical_machine_id: self.physical_id.clone(),
            }),
            runtime_state: crate::types::RuntimeState::Running,
            simulation: self
                .simulation_body
                .as_ref()
                .filter(|_| !self.detached)
                .and_then(|body| {
                    body.snapshot
                        .lock()
                        .ok()
                        .and_then(|snapshot| snapshot.state.clone())
                }),
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
    use super::{PolicyExecution, PreviewPolicySelection, has_reached_target};
    use crate::backend::policy_runtime::{PolicyConfig, PolicyDescriptor};
    use duck_body::{BodyTemplate, ServoModelKey};
    use servo::ServoModel;
    use std::time::{SystemTime, UNIX_EPOCH};

    #[test]
    fn idle_sampling_and_reopening_preserve_the_body_and_scene_placement() {
        let placements =
            std::sync::Arc::new(super::super::world::WorldPlacements::load(None).unwrap());
        placements.remember(
            "test-duck",
            super::WorldState {
                position: [1.2, -0.4],
                yaw: 0.8,
                distance_m: 2.5,
            },
        );
        let mut registry = loop_body::BackendRegistry::default();
        let open = |registry: &mut loop_body::BackendRegistry| {
            let mut execution = PolicyExecution::open(
                ServoModel::Xl330M288,
                crate::helpers::default_simulation(ServoModelKey::Xl330),
                None,
                "test-duck",
                "test",
                registry,
            )
            .unwrap();
            execution.restore_placement(placements.clone());
            execution
        };
        let mut execution = open(&mut registry);
        let initial = execution.status();
        for _ in 0..50 {
            execution.tick().unwrap();
        }
        let settled = execution.status();
        assert_eq!(
            initial.simulation.unwrap().trunk_position,
            settled.simulation.unwrap().trunk_position
        );
        assert_eq!(settled.world.position, initial.world.position);
        assert_eq!(settled.world.yaw, initial.world.yaw);
        assert_eq!(settled.world.distance_m, initial.world.distance_m);
        execution.release_body(&mut registry, "test-duck").unwrap();
        let mut reopened = open(&mut registry);
        assert_eq!(reopened.status().world.position, initial.world.position);
        reopened.tick().unwrap();
        assert_eq!(reopened.status().world.position, initial.world.position);
        assert_eq!(reopened.status().world.distance_m, initial.world.distance_m);
        reopened.reset_world().unwrap();
        assert_eq!(reopened.status().world.position, [0.0; 2]);
        reopened.release_body(&mut registry, "test-duck").unwrap();
        assert_eq!(open(&mut registry).status().world.position, [0.0; 2]);
    }

    #[test]
    fn a_new_distance_request_uses_its_own_origin_without_resetting_world_coordinates() {
        let mut registry = loop_body::BackendRegistry::default();
        let mut preview = PolicyExecution::open(
            ServoModel::Xl330M288,
            crate::helpers::default_simulation(ServoModelKey::Xl330),
            None,
            "distance-test",
            "test",
            &mut registry,
        )
        .unwrap();
        preview.world.distance_m = 2.0;
        let input = super::PreviewInput {
            twist: [0.15, 0.0, 0.0],
            head: [0.0; 4],
            body: [0.0; 3],
            target_distance_m: Some(1.0),
        };
        preview.set_input(input);
        assert_eq!(preview.effective_twist(), [0.15, 0.0, 0.0]);
        assert_eq!(preview.world.distance_m, 2.0);
        preview.world.distance_m = 3.1;
        assert_eq!(preview.effective_twist(), [0.0; 3]);
        preview.set_input(input);
        assert_eq!(
            preview.effective_twist(),
            [0.0; 3],
            "repeated input cannot restart a completed distance request"
        );
        preview.set_input(super::PreviewInput {
            twist: [0.0; 3],
            ..input
        });
        preview.set_input(input);
        assert_eq!(preview.effective_twist(), [0.15, 0.0, 0.0]);
        assert_eq!(preview.world.distance_m, 3.1);
    }

    #[test]
    #[ignore = "requires cached official policies and ONNX Runtime"]
    fn velstand_walks_from_cold_start_and_handoff_without_changing_motor_parameters() {
        let cache = std::path::PathBuf::from(std::env::var_os("MICRODUCK_POLICY_CACHE").unwrap());
        let root = crate::helpers::test_directory()
            .join(crate::helpers::new_machine_id("walking-cold-start"));
        let official = root.join("policies/official");
        std::fs::create_dir_all(&official).unwrap();
        for entry in std::fs::read_dir(cache.join("official")).unwrap() {
            let entry = entry.unwrap();
            if entry
                .path()
                .extension()
                .is_some_and(|extension| extension == "onnx")
                || entry.file_name() == "manifest.json"
            {
                std::fs::copy(entry.path(), official.join(entry.file_name())).unwrap();
            }
        }
        for (model, key) in [
            (ServoModel::Xl330M288, ServoModelKey::Xl330),
            (ServoModel::Hd1910, ServoModelKey::Hd1910),
        ] {
            for id in ["velstand"] {
                for previous in [None, Some("ball-kick-left")] {
                    let mut registry = loop_body::BackendRegistry::default();
                    let mut preview = PolicyExecution::open(
                        model,
                        if model == ServoModel::Xl330M288 {
                            crate::types::SimulationParameters {
                                damping: 0.049268,
                                friction_loss: 0.004937,
                                armature: 0.001889,
                                kp: 0.536635,
                                force_range: 0.966805,
                                ..crate::helpers::default_simulation(key)
                            }
                        } else {
                            crate::helpers::default_simulation(key)
                        },
                        Some(&root.join("config")),
                        "cold-start-test",
                        "test",
                        &mut registry,
                    )
                    .unwrap();
                    // Desktop starts its clock before the frontend selects a policy.
                    for _ in 0..50 {
                        preview.tick().unwrap();
                    }
                    assert_eq!(preview.status().simulation.unwrap().physics_step, 0);
                    if let Some(previous) = previous {
                        preview.select_policy(previous).unwrap();
                        for _ in 0..10 {
                            preview.tick().unwrap();
                        }
                    }
                    preview.select_policy(id).unwrap();
                    preview.set_input(super::PreviewInput {
                        twist: [0.15, 0.0, 0.0],
                        head: [0.0; 4],
                        body: [0.0; 3],
                        target_distance_m: None,
                    });
                    let start = preview.status().simulation.unwrap().trunk_position;
                    let mut low = start[2];
                    let mut peak_speed: f64 = 0.0;
                    for _ in 0..250 {
                        let status = preview.tick().unwrap();
                        let simulation = status.simulation.unwrap();
                        low = low.min(simulation.trunk_position[2]);
                        peak_speed = peak_speed.max(simulation.base_velocity[0].abs());
                        assert_eq!(
                            preview.runner().policy_trace().unwrap().command.twist,
                            [0.15, 0.0, 0.0]
                        );
                    }
                    let end = preview.status().simulation.unwrap().trunk_position;
                    println!(
                        "COLD_START {model:?}/{id} previous={previous:?} delta={:?} low={low} peak_speed={peak_speed}",
                        [end[0] - start[0], end[1] - start[1], end[2] - start[2]]
                    );
                    assert!(
                        end[0] - start[0] > 0.18,
                        "{model:?}: cold start and handoff must both walk"
                    );
                    assert!(low > 0.10, "{model:?}: walking must keep the body upright");
                }
            }
        }
        crate::logging::flush();
        std::fs::remove_dir_all(root).unwrap();
    }

    #[test]
    #[ignore = "requires cached official policies and ONNX Runtime"]
    fn cached_walking_policies_execute_forward_commands() {
        let cache = std::path::PathBuf::from(std::env::var_os("MICRODUCK_POLICY_CACHE").unwrap());
        let root = crate::helpers::test_directory()
            .join(crate::helpers::new_machine_id("walking-regression"));
        let official = root.join("policies/official");
        std::fs::create_dir_all(&official).unwrap();
        for entry in std::fs::read_dir(cache.join("official")).unwrap() {
            let entry = entry.unwrap();
            if entry
                .path()
                .extension()
                .is_some_and(|extension| extension == "onnx")
                || entry.file_name() == "manifest.json"
            {
                std::fs::copy(entry.path(), official.join(entry.file_name())).unwrap();
            }
        }
        for (model, key) in [
            (ServoModel::Xl330M288, ServoModelKey::Xl330),
            (ServoModel::Hd1910, ServoModelKey::Hd1910),
        ] {
            for id in ["alpha-walking", "velstand"] {
                let mut registry = loop_body::BackendRegistry::default();
                let mut preview = PolicyExecution::open(
                    model,
                    crate::helpers::default_simulation(key),
                    Some(&root.join("config")),
                    "walking-test",
                    "test",
                    &mut registry,
                )
                .unwrap();
                preview.select_policy("ball-kick-left").unwrap();
                for _ in 0..10 {
                    preview.tick().unwrap();
                }
                preview.world.distance_m = 2.0;
                preview.select_policy(id).unwrap();
                for speed in [0.15, 0.3] {
                    if speed != 0.15 {
                        preview.reset_world().unwrap();
                    }

                    preview.set_input(super::PreviewInput {
                        twist: [speed, 0.0, 0.0],
                        head: [0.0; 4],
                        body: [0.0; 3],
                        target_distance_m: Some(1.0),
                    });
                    let mut low = f64::INFINITY;
                    let mut n = 0;
                    for _ in 0..250 {
                        let reached = preview.target_distance_reached();
                        let status = preview.tick().unwrap();
                        low = low.min(status.simulation.unwrap().trunk_position[2]);
                        if let Some(trace) = preview.runner().policy_trace() {
                            assert_eq!(
                                trace.command.twist,
                                if reached { [0.0; 3] } else { [speed, 0.0, 0.0] },
                                "only this walk's actual distance can stop its command"
                            );
                            n += 1;
                        }
                    }
                    let status = preview.status();
                    let position = status.simulation.unwrap().trunk_position;
                    assert!((status.world.position[0] - position[0]).abs() < 1e-10);
                    assert!((status.world.position[1] - position[1]).abs() < 1e-10);
                    let trace = preview.runner().policy_trace().unwrap();
                    println!(
                        "{model:?}/{id} vx={speed}: position={position:?}, low={low}, traces={n}, twist={:?}, gravity={:?}",
                        trace.command.twist, trace.imu.gravity
                    );
                    assert_eq!(n, 250);
                    if model == ServoModel::Xl330M288 || id == "velstand" || speed >= 0.3 {
                        assert!(
                            position[0] > 0.15,
                            "walking policy must execute forward motion"
                        );
                    }
                }
            }
        }
        crate::logging::flush();
        std::fs::remove_dir_all(root).unwrap();
    }

    #[test]
    #[ignore = "requires cached official policies and ONNX Runtime"]
    fn warmed_policies_run_until_explicit_handoff_from_the_current_body_pose() {
        use crate::{application::policy};
        let cache = std::path::PathBuf::from(std::env::var_os("MICRODUCK_POLICY_CACHE").unwrap());
        let root =
            crate::helpers::test_directory().join(crate::helpers::new_machine_id("policy-handoff"));
        let official = root.join("policies/official");
        std::fs::create_dir_all(&official).unwrap();
        for entry in std::fs::read_dir(cache.join("official")).unwrap() {
            let entry = entry.unwrap();
            if entry
                .path()
                .extension()
                .is_some_and(|extension| extension == "onnx")
                || entry.file_name() == "manifest.json"
            {
                std::fs::copy(entry.path(), official.join(entry.file_name())).unwrap();
            }
        }
        let config = root.join("config");
        let count = policy::list(Some(&config))
            .unwrap()
            .iter()
            .filter(|entry| entry.cached)
            .count();
        let mut registry = loop_body::BackendRegistry::default();
        let mut preview = PolicyExecution::open(
            ServoModel::Xl330M288,
            crate::helpers::default_simulation(ServoModelKey::Xl330),
            Some(&config),
            "handoff-test",
            "test",
            &mut registry,
        )
        .unwrap();
        assert_eq!(preview.policies.len(), count);
        assert!(count >= 10);
        assert_eq!(preview.status().tick, 0, "warm-up cannot advance the body");
        // An explicit handoff must work entirely from the warm session bank.
        for entry in std::fs::read_dir(&official).unwrap() {
            let entry = entry.unwrap();
            if entry
                .path()
                .extension()
                .is_some_and(|extension| extension == "onnx")
            {
                std::fs::remove_file(entry.path()).unwrap();
            }
        }
        preview.select_policy("ball-kick-left").unwrap();
        for _ in 0..80 {
            preview.tick().unwrap();
            assert!(preview.loop_state.policy_trace().is_some());
            assert_eq!(
                preview.active_policy.as_ref().unwrap().descriptor.id,
                "ball-kick-left"
            );
        }
        // The manifest's 0.5s window has passed; there is no automatic stop or stand policy.
        assert_eq!(preview.policy_ticks, 80);
        let before = preview.status();
        preview.select_policy("alpha-stand").unwrap();
        let after = preview.status();
        assert_eq!(before.world.position, after.world.position);
        assert_eq!(before.world.yaw, after.world.yaw);
        assert_eq!(before.world.distance_m, after.world.distance_m);
        assert_eq!(before.positions, after.positions);
        assert_eq!(before.targets, after.targets);
        assert_eq!(before.tick, after.tick);
        assert_eq!(before.session, after.session);
        assert_eq!(
            before.simulation.as_ref().unwrap().physics_step,
            after.simulation.as_ref().unwrap().physics_step
        );
        assert!(after.torque);
        preview.tick().unwrap();
        let trace = preview.loop_state.policy_trace().unwrap();
        let home = BodyTemplate::official().unwrap().control.home_pose;
        let offsets =
            std::array::from_fn(|index| trace.backend_state.positions[index] - home[index]);

        let expected = duck_control::obs::Observation::gather_action(&offsets);
        for (actual, expected) in trace.observation[34..48].iter().zip(expected) {
            assert!(
                (actual - expected).abs() < 1e-5,
                "handoff history must describe the current pose: actual={actual}, expected={expected}"
            );
        }
        assert_eq!(preview.status().tick, before.tick + 1);
        preview.select_policy("ball-kick-right").unwrap();
        preview.tick().unwrap();
        assert_eq!(
            preview.active_policy.as_ref().unwrap().descriptor.id,
            "ball-kick-right"
        );
        preview.action(loop_body::LoopAction::Drive(false)).unwrap();
        let paused = preview.status();
        for _ in 0..3 {
            preview.tick().unwrap();
            assert!(
                preview.runner().policy_trace().is_none(),
                "sampling must not repeat a previous ONNX trace"
            );
        }
        assert_eq!(preview.status().tick, paused.tick + 3);
        assert_eq!(
            preview.status().simulation.unwrap().physics_step,
            paused.simulation.unwrap().physics_step
        );
        preview.action(loop_body::LoopAction::Drive(true)).unwrap();
        preview.tick().unwrap();
        assert!(preview.runner().policy_trace().is_some());
        println!(
            "Prewarmed {count} policies; ran kick for 80 ticks; explicit handoffs preserved pose, time and inference."
        );
        drop(preview);
        crate::logging::flush();
        std::fs::remove_dir_all(root).unwrap();
    }

    #[test]
    fn preview_loop_uses_shared_policy_loop_without_hardware() {
        let mut registry = loop_body::BackendRegistry::default();
        let mut loop_state = PolicyExecution::open(
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
            0,
            "sampling without a policy must preserve the simulation's initial state"
        );
        let status = loop_state.status();
        assert_eq!(status.phase, crate::types::LoopPhase::Ready);
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
        assert_eq!(loop_state.status().simulation.as_ref().unwrap().time, 0.0);
        assert_eq!(loop_state.status().tick, 21);
        loop_state.reset_world().unwrap();
        let reset = loop_state.status();
        assert_eq!(reset.simulation.as_ref().unwrap().physics_step, 0);
        assert_eq!(reset.simulation.as_ref().unwrap().time, 0.0);
        assert_eq!(
            reset.positions,
            BodyTemplate::official().unwrap().control.home_pose
        );
        loop_state.tick().unwrap();
        assert_eq!(loop_state.status().simulation.unwrap().physics_step, 0);
    }

    #[test]
    fn preview_loop_logs_virtual_servo_io() {
        let suffix = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_nanos();
        let root = crate::helpers::test_directory().join(format!("microduck-preview-log-{suffix}"));
        let config_dir = root.join("config");
        let mut registry = loop_body::BackendRegistry::default();
        let mut loop_state = PolicyExecution::open(
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
        assert!(!log.contains("servo_write_request"));
        let _ = std::fs::remove_dir_all(root);
    }

    #[test]
    #[ignore = "requires the official ONNX artifact and a dynamic ONNX Runtime"]
    fn hd1910_velstand_walks_after_standing() {
        let path = std::env::var_os("MICRODUCK_OFFICIAL_ONNX").unwrap();
        for idle_ticks in [2, 3, 10, 50, 100, 200] {
            let mut registry = loop_body::BackendRegistry::default();
            let mut preview = PolicyExecution::open(
                ServoModel::Hd1910,
                crate::helpers::default_simulation(ServoModelKey::Hd1910),
                None,
                "hd1910-velstand",
                "HD1910 velstand",
                &mut registry,
            )
            .unwrap();
            preview
                .activate_policy(PreviewPolicySelection {
                    path: path.clone().into(),
                    command: None,
                    descriptor: PolicyDescriptor {
                        id: "velstand".into(),
                        body_id: "official".into(),
                        body_version: 1,
                        obs_len: 61,
                        action_len: 14,
                        control_hz: 50,
                        command_encoding: None,
                        recurrent: None,
                        action_scale: None,
                        head_lowpass: None,
                        legs_lowpass: None,
                        joint_velocity_delay_ticks: 1,
                    },
                    config: PolicyConfig::default(),
                    episodic: false,
                    duration: None,
                })
                .unwrap();

            for _ in 0..idle_ticks {
                preview.tick().unwrap();
            }
            let start = preview.status().simulation.unwrap().trunk_position;
            assert!(
                start[0].hypot(start[1]) < 0.04,
                "drifted at zero command: {start:?}"
            );
            preview.set_input(super::PreviewInput {
                twist: [0.15, 0.0, 0.0],
                head: [0.0; 4],
                body: [0.0; 3],
                target_distance_m: Some(1.0),
            });
            let mut lowest = f64::INFINITY;
            for _ in 0..250 {
                let state = preview.tick().unwrap().simulation.unwrap();
                lowest = lowest.min(state.trunk_position[2]);
            }
            let distance = preview.status().simulation.unwrap().trunk_position[0] - start[0];
            println!(
                "HD1910: idle_ticks={idle_ticks} forward_m={distance:.3} lowest_trunk_m={lowest:.3}"
            );
            assert!(
                distance > 0.18,
                "failed to walk after {idle_ticks} idle ticks: {distance} m"
            );
            assert!(lowest > 0.10, "trunk fell below 0.10 m");

        }
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
        let output_path =
            std::env::var_os("MICRODUCK_OFFICIAL_PRODUCT_TRACE").map(std::path::PathBuf::from);
        let mut registry = loop_body::BackendRegistry::default();
        let mut preview = PolicyExecution::open(
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
        let mut recorded_frames = Vec::new();
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
            let sample = preview
                .simulation_body
                .as_ref()
                .unwrap()
                .snapshot
                .lock()
                .unwrap()
                .state
                .clone()
                .unwrap();
            recorded_frames.push(crate::types::RecordingFrame {
                recording_id: "official-velstand-reference".into(),
                duck_id: "official-velstand-trace".into(),
                segment_id: 1,
                episode_id: 1,
                elapsed_s: (tick + 1) as f64 * super::PERIOD.as_secs_f64(),
                control_started_s: tick as f64 * super::PERIOD.as_secs_f64(),
                status: preview.status(),
                policy: Some(crate::types::RecordingPolicy::from(&trace)),
            });
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
        let resource_root = crate::backend::mujoco::resource_directory().unwrap();
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
        crate::recordings::report_reference(&report);
        if let Some(path) = output_path {
            crate::recordings::write_trace(&path, &recorded_frames, preview.recording_metadata())
                .unwrap();
        }
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
