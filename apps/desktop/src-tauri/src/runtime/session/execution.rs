//! Desktop policy scheduling over the active simulation or physical body.

use std::path::{Path, PathBuf};
use std::time::Duration;

use crate::platform::logging::{append_control_event, append_policy_trace, begin_control_loop};
use crate::runtime::adapters::logged_body::LoggedBody;
use crate::runtime::policy::inference::{
    MicroDuckPolicyRunner, PolicyConfig, PolicyDescriptor, PreparedPolicy,
};
use crate::runtime::scene::WorldCoordinates;
use crate::runtime::session::contract::register_simulation;
use crate::runtime::session::contract::{PhysicalExecutionSources, ResolvedContract};
use crate::types::{
    LoopPhase, LoopStatus, PolicyCommand, PreviewInput, SimulationParameters, WorldState,
};
use duck_body::BodyTemplate;
use duck_control::obs::Command;
use loop_body::{
    BodyRuntimeView, Calibration, JointCalibration, LoopAction, LoopBinding, LoopConfig,
    LoopPhase as BodyLoopPhase, ServoBinding,
};
use microduck_body_runtime::{
    BodyExecutionProfile, Selection, models::profile::ActuatorResponse, simulation::SnapshotHandle,
    simulation::mujoco::runtime_version,
};

const PERIOD: Duration = Duration::from_millis(20);

pub(crate) fn simulation_loop_config() -> LoopConfig {
    LoopConfig {
        homing_ticks: 0,
        enforce_body_range: false,
        velocity_observation_delay_ticks: 0,
        ..LoopConfig::default()
    }
}
#[derive(Debug, Clone)]
pub(crate) struct PreviewPolicySelection {
    pub(crate) path: PathBuf,
    pub(crate) command: Option<PolicyCommand>,
    pub(crate) descriptor: PolicyDescriptor,
    pub(crate) config: PolicyConfig,
    pub(crate) episodic: bool,
    pub(crate) duration: Option<Duration>,
    pub(crate) training: Option<crate::training::policy_artifacts::TrainingPolicy>,
}

pub(crate) struct PolicyCache<'a> {
    pub(crate) config_dir: Option<&'a Path>,
    pub(crate) resolve: fn(&Path) -> Result<Option<Vec<PreviewPolicySelection>>, String>,
}

#[derive(Clone)]
struct PreparedActivation {
    selection: PreviewPolicySelection,
    runtime: PreparedPolicy,
    sha256: String,
}

pub(crate) struct PolicyExecution {
    loop_state: MicroDuckPolicyRunner,
    virtual_machine_id: String,
    input: PreviewInput,
    policy_command: Option<PolicyCommand>,
    world: WorldState,
    coordinates: WorldCoordinates,
    placements: Option<std::sync::Arc<crate::platform::state::world_placements::WorldPlacements>>,
    command_distance_origin_m: f64,
    simulation_body: Option<SimulationBody>,
    physical_execution: Option<PhysicalExecutionSources>,
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
    profile: BodyExecutionProfile,
    snapshot: SnapshotHandle,
    actuator_response: ActuatorResponse,
}

impl PolicyExecution {
    pub(crate) fn control_period(&self) -> Duration {
        self.loop_state.control_period()
    }

    pub(crate) fn simulation_fingerprint(&self) -> Option<&str> {
        self.simulation_body
            .as_ref()
            .map(|body| body.profile.execution_fingerprint.as_str())
    }

    pub(crate) fn recording_metadata(&self) -> serde_json::Value {
        let policy = self.active_policy.as_ref();
        let mut metadata = serde_json::json!({
            "body_id":self.body_id,"body_version":self.body_version,
            "backend_profile":self.backend_profile,"backend_contract":self.backend_contract,
            "simulation_execution":self.simulation_body.as_ref().map(|body| &body.profile),
            "physical_execution":self.physical_execution,
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
        });
        if let Some(training) = policy.and_then(|policy| policy.training.as_ref()) {
            metadata["training_policy_source"] = training.source.clone();
            metadata["training_policy_home_rad"] = serde_json::json!(training.spec.home);
        }
        metadata
    }
    pub(crate) fn open(
        selection: Selection,
        simulation: SimulationParameters,
        cache: PolicyCache<'_>,
        machine_id: &str,
        machine_name: &str,
        registry: &mut loop_body::BackendRegistry,
        template: &BodyTemplate,
    ) -> Result<Self, String> {
        let config_dir = cache.config_dir;
        let model = selection.servo_model;
        let binding_summary = template
            .actuators
            .iter()
            .enumerate()
            .map(|(index, slot)| format!("{}->{} dir=1 offset=0", slot.id, index + 1))
            .collect::<Vec<_>>()
            .join(" | ");
        let binding = LoopBinding {
            body: BodyRuntimeView::new(template.clone()).map_err(|error| error.to_string())?,
            servos: ServoBinding::all_slots(template, model),
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
        let (snapshot, resolved_contract, execution) =
            register_simulation(registry, machine_id, selection, &simulation, template)?;
        let actuator = execution
            .simulation
            .as_ref()
            .ok_or("missing simulation profile")?;
        append_control_event(
            config_dir,
            format!(
                "preview_actuator_config backend_kind=simulation implementation=OfficialCompatSimulationBackend model={model:?} actuator_profile={actuator_profile} encoder_through_backlash={encoder_through_backlash} drive_position_gain_scale={drive_position_gain_scale} drive_back_emf_scale={drive_back_emf_scale} drive_friction_scale={drive_friction_scale} mujoco_binding_version=3.10.0 mujoco_runtime_version={version} scene_path={scene} bam_source_revision=62bd8ce12154340be97e06f7f41a0ca8f116d967 control_dt={control_dt} physics_dt={physics_dt} physics_substeps={substeps} delay_timebase=physics_step delay_schedule=physics_step_history command_delay_physics_steps={delay_steps:?} observation_delay_ticks=0 enforce_body_range=false imu_source=backend",
                scene = actuator
                    .scenario
                    .scene_xml
                    .ok_or("missing native simulation scene")?,
                version = runtime_version(),
                control_dt = execution.periods.control_s,
                physics_dt = execution.periods.physics_s.unwrap_or_default(),
                substeps = execution.periods.physics_substeps.unwrap_or_default(),
                actuator_profile = actuator.id,
                encoder_through_backlash = actuator.response.encoder_through_backlash,
                drive_position_gain_scale = actuator.response.position_gain_scale,
                drive_back_emf_scale = actuator.response.back_emf_scale,
                drive_friction_scale = actuator.response.friction_scale,
                delay_steps = actuator.response.delay_steps
            ),
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
            Box::new(LoggedBody::new(
                Box::new(registered_driver),
                handle,
                machine_id,
            ))
        } else {
            Box::new(registered_driver)
        };
        let mut loop_state = MicroDuckPolicyRunner::open(binding, driver, simulation_loop_config())
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
                actuator_response: actuator.response,
                profile: execution,
            }),
            physical_id: None,
            physical_execution: None,
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
            coordinates: WorldCoordinates::default(),
            placements: None,
            config_dir: config_dir.map(Path::to_path_buf),
            policy_ticks: 0,
            command_distance_origin_m: 0.0,
            active_policy: None,
            policies: std::collections::HashMap::new(),
            body_id: template.id.clone(),
            body_version: template.version,
            backend_profile: resolved_contract.backend_profile,
        };
        preview
            .loop_state
            .send(LoopAction::Power(true))
            .map_err(|error| error.to_string())?;
        preview.observe_world(false)?;
        preview.preload_cached_policies(cache)?;
        Ok(preview)
    }

    pub(crate) fn restore_placement(
        &mut self,
        placements: std::sync::Arc<crate::platform::state::world_placements::WorldPlacements>,
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
            self.coordinates.observe(
                &mut self.world,
                state.trunk_position,
                state.trunk_quaternion,
                driven,
            );
        }
        self.remember_placement(false);
        Ok(())
    }

    fn preload_cached_policies(&mut self, cache: PolicyCache<'_>) -> Result<(), String> {
        let Some(config) = cache.config_dir else {
            return Ok(());
        };
        let Some(selections) = (cache.resolve)(config)? else {
            return Ok(());
        };
        for selection in selections {
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
        self.physical_execution = None;
        self.virtual_machine_id = next.virtual_machine_id;
        self.backend_contract = next.backend_contract;
        self.backend_profile = next.backend_profile;
        Ok(())
    }

    pub(crate) fn switch_to_physical(
        &mut self,
        id: String,
        virtual_machine_id: String,
        runner: MicroDuckPolicyRunner,
        contract: ResolvedContract,
        sources: PhysicalExecutionSources,
    ) -> Result<(), String> {
        self.install_body(runner)?;
        self.simulation_body = None;
        self.physical_id = Some(id);
        self.physical_execution = Some(sources);
        self.virtual_machine_id = virtual_machine_id;
        self.backend_profile = contract.backend_profile;
        self.backend_contract = contract.hash;
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

    pub(crate) fn select_training_policy(
        &mut self,
        selection: PreviewPolicySelection,
    ) -> Result<(), String> {
        if self.simulation_body.is_none() {
            return Err("本地训练策略需要模拟机预览".into());
        }
        let prepared = self.prepare_activation(selection)?;
        self.activate_prepared(prepared)?;
        self.control_paused = false;
        self.ensure_driving();
        Ok(())
    }

    pub(crate) fn has_training_policy(&self) -> bool {
        self.active_policy
            .as_ref()
            .is_some_and(|policy| policy.training.is_some())
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
        self.coordinates = WorldCoordinates::default();
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
            .tick(self.control_period())
            .map_err(|error| error.to_string())?;
        let has_policy_trace = self.loop_state.policy_trace().is_some();
        self.observe_world(has_policy_trace)?;
        let status = self.status();
        if has_policy_trace && let Some(simulation) = status.simulation.as_ref() {
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
        let runtime = if let Some(training) = &selection.training {
            if training.requires_backlash()
                && !self.simulation_body.as_ref().is_some_and(|body| {
                    body.profile.model_kind
                        == microduck_body_runtime::models::profile::ModelKind::Xl330
                        && body.actuator_response.encoder_through_backlash
                        && body.profile.simulation.as_ref().is_some_and(|simulation| {
                            simulation.scenario.robot_xml == "robot_groundcontact_backlash.xml"
                        })
                })
            {
                return Err("Backlash 学生需要匹配的 XL330 groundcontact/backlash 模拟机".into());
            }
            if policy_sha256 != training.sha256 {
                return Err("训练 ONNX 文件摘要与产物清单不一致".into());
            }
            self.loop_state.prepare_local_policy(
                &policy_bytes,
                &selection.descriptor,
                selection.config,
                &training.spec,
            )
        } else {
            self.loop_state
                .prepare_policy(&selection.path, &selection.descriptor, selection.config)
        }
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
            .map(|body| body.actuator_response);
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
        if let Some(training) = &selection.training {
            append_control_event(
                self.config_dir.as_deref(),
                format!(
                    "preview_training_policy source={} preview_execution={} policy_contract={:?}",
                    training.source,
                    self.simulation_body
                        .as_ref()
                        .map(|body| body.profile.execution_fingerprint.as_str())
                        .unwrap_or_default(),
                    selection.descriptor,
                ),
            );
        }
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
            recording: false,
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
                        .and_then(|snapshot| snapshot.state.clone().map(Into::into))
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
#[path = "execution_tests.rs"]
mod tests;
