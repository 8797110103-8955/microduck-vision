//! ONNX policy execution layered on top of the base body loop.

use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use duck_body::BodyProfile;
use duck_control::imu::ImuData;
use duck_control::model::NUM_JOINTS;
use duck_control::obs::{ACTION_LEN, Command, Observation};
use duck_control::policy::{DEFAULT_STANDING_THRESHOLD, Net, Policy, PolicyPaths};

use loop_body::{
    BackendState, BodyCommand, BodyLoop, ControlBackend, ControllerInput, LoopAction, LoopBinding,
    LoopConfig, LoopController, LoopError, LoopPhase, LoopStatus, SampleTime,
};

#[cfg(test)]
use super::action_space::BodyActionCodec;
use super::contract::PolicyDescriptor;
use super::core::PolicyHistory;
use super::local_onnx::{LocalPolicy, LocalPolicySpec};
use super::{ControlTickInput, PolicyConfig, PolicyControlCore};
use crate::platform::logging::append_control_event;

#[cfg(test)]
use super::core::walking_target_trace;
#[cfg(test)]
pub(crate) use super::{PolicyInputEncoder, RawPolicyOutputDecoder};
#[cfg(test)]
const HEAD_LOWPASS: f64 = 0.5;
#[cfg(test)]
const LEGS_LOWPASS: f64 = 0.7;
#[cfg(test)]
const WALK_ACTION_SCALE: f64 = 1.0;
#[cfg(test)]
fn walking_targets(
    action: &[f32; ACTION_LEN],
    codec: &BodyActionCodec,
    home: &[f64; NUM_JOINTS],
    previous: Option<&[f64; NUM_JOINTS]>,
    config: PolicyConfig,
) -> [f64; NUM_JOINTS] {
    walking_target_trace(action, codec, home, previous, config).1
}

#[derive(Debug, Clone, PartialEq)]
pub struct PolicyTrace {
    pub phase: LoopPhase,
    pub command: Command,
    pub imu: ImuData,
    pub imu_source: &'static str,
    pub observation: Vec<f32>,
    pub action: Vec<f32>,
    pub body_targets: Option<Vec<f64>>,
    pub raw_targets: Option<Vec<f64>>,
    pub backend_state: BackendState,
    /// Fresh positions sampled after execution, when available. Never copy the pre-write state
    /// into this field: a backend write does not refresh BodyLoop's cached sensor sample.
    pub execution_positions: Option<Vec<f64>>,
    pub saturation_reasons: Vec<&'static str>,
    pub stages: Vec<&'static str>,
}

/// The last successful controller frame. The next tick reads a fresh backend state while this
/// frame supplies the previous action and the stop/resume snapshot.
#[derive(Debug, Clone, PartialEq)]
pub struct PolicyStateSnapshot {
    pub backend_state: BackendState,
    pub applied_targets: Vec<f64>,
    pub action: Vec<f32>,
    pub command: Command,
    pub history: PolicyHistory,
}

enum LoadedPolicy {
    Official(Box<Policy>),
    Local(Box<LocalPolicy>),
}

impl LoadedPolicy {
    fn will_stand(&self, speed: f64) -> bool {
        matches!(self, Self::Official(policy) if policy.will_stand(speed))
    }

    fn infer(
        &mut self,
        observation: &Observation,
        net: Net,
    ) -> Result<[f32; ACTION_LEN], LoopError> {
        match self {
            Self::Official(policy) => policy
                .infer(observation, net)
                .map_err(|error| LoopError::Controller(error.to_string())),
            Self::Local(policy) => policy.infer(observation),
        }
    }

    fn reset(&mut self) {
        if let Self::Official(policy) = self {
            policy.reset();
        }
    }
}

struct PolicyTickController<'a> {
    policy: Option<&'a mut LoadedPolicy>,
    command: Command,
    core: &'a mut PolicyControlCore,
    last_trace: &'a mut Option<PolicyTrace>,
    last_frame: &'a mut Option<PolicyStateSnapshot>,
    active_net: &'a mut Option<Net>,
    seed_from_sensors: &'a mut bool,
}

impl LoopController for PolicyTickController<'_> {
    fn on_loop(&mut self, input: ControllerInput) -> Result<Option<BodyCommand>, LoopError> {
        let Some(policy) = self.policy.as_deref_mut() else {
            *self.active_net = None;
            return Ok(None);
        };
        let ControllerInput { sensors, status } = input;
        let backend_state = BackendState {
            positions: sensors.positions,
            velocities: sensors.velocities,
            imu: sensors.imu,
            sequence: status.tick,
            sampled_at: SampleTime(status.tick),
            valid_positions: sensors.valid.clone(),
            valid_velocities: sensors.valid,
            source: Some("execution".into()),
        };
        if *self.seed_from_sensors {
            self.core.seed_from_pose(&backend_state.positions)?;
            *self.seed_from_sensors = false;
        }
        let (observation, _, _) = self
            .core
            .encode_observation(&backend_state, &self.command)?;
        let requested_net = self.core.config().net;
        let selected_net =
            if requested_net == Net::Walk && policy.will_stand(self.command.twist_magnitude()) {
                Net::Stand
            } else {
                requested_net
            };
        *self.active_net = Some(selected_net);
        let mut stages = vec!["read", "encode"];
        if selected_net != requested_net {
            stages.push("select_standing");
        }
        let raw_action = policy.infer(&observation, selected_net)?;
        let output = self.core.control_tick(&ControlTickInput {
            backend_state: backend_state.clone(),
            command: self.command,
            raw_action: raw_action.to_vec(),
        })?;
        stages.extend(["infer", "decode", "target", "validate", "bind"]);
        *self.last_trace = Some(PolicyTrace {
            phase: status.phase,
            command: self.command,
            imu: output.imu,
            imu_source: output.imu_source,
            observation: output.observation,
            action: output.action.clone(),
            raw_targets: Some(output.raw_targets),
            body_targets: Some(output.canonical_target.clone()),
            backend_state: backend_state.clone(),
            execution_positions: None,
            saturation_reasons: output.saturation_reasons,
            stages,
        });
        *self.last_frame = Some(PolicyStateSnapshot {
            backend_state,
            applied_targets: output.canonical_target.clone(),
            action: output.action,
            command: self.command,
            history: self.core.history(),
        });
        Ok(Some(BodyCommand {
            session: status.session,
            targets: output.canonical_target,
        }))
    }
}

#[derive(Clone)]
pub(crate) struct PreparedPolicy {
    policy: Arc<Mutex<LoadedPolicy>>,
    config: PolicyConfig,
    velocity_delay: u32,
    home: Option<[f64; NUM_JOINTS]>,
}

pub struct MicroDuckPolicyRunner {
    body: BodyLoop,
    policy: Option<Arc<Mutex<LoadedPolicy>>>,
    policy_home: Option<[f64; NUM_JOINTS]>,
    command: Command,
    last_trace: Option<PolicyTrace>,
    core: PolicyControlCore,
    body_profile: BodyProfile,
    control_period: Duration,
    hardware_velocity_feedback: bool,
    active_net: Option<Net>,
    seed_from_sensors: bool,
    last_frame: Option<PolicyStateSnapshot>,
    stop_snapshot: Option<PolicyStateSnapshot>,
}

impl MicroDuckPolicyRunner {
    pub(crate) fn control_period(&self) -> Duration {
        self.control_period
    }

    pub fn open(
        binding: LoopBinding,
        driver: Box<dyn ControlBackend>,
        config: LoopConfig,
    ) -> Result<Self, LoopError> {
        Self::open_with_command(binding, driver, config, Command::default())
    }

    pub fn open_with_command(
        binding: LoopBinding,
        driver: Box<dyn ControlBackend>,
        config: LoopConfig,
        command: Command,
    ) -> Result<Self, LoopError> {
        let body_profile = binding.body.profile().clone();
        let control_period = config.period;
        let ranges = binding
            .body
            .template()
            .joints
            .iter()
            .map(|joint| joint.range_rad)
            .collect();
        let core = PolicyControlCore::new(&body_profile, ranges, PolicyConfig::default())?;
        let body = BodyLoop::open(binding, driver, config)?;
        Ok(Self {
            body,
            policy: None,
            policy_home: None,
            command,
            last_trace: None,
            core,
            body_profile,
            control_period,
            hardware_velocity_feedback: false,
            active_net: None,
            seed_from_sensors: true,
            last_frame: None,
            stop_snapshot: None,
        })
    }

    pub fn select_policy_with_descriptor(
        &mut self,
        path: impl AsRef<Path>,
        descriptor: &PolicyDescriptor,
        config: PolicyConfig,
    ) -> Result<(), LoopError> {
        let prepared = self.prepare_policy(path.as_ref(), descriptor, config)?;
        self.install_policy(prepared)
    }

    pub(crate) fn use_hardware_velocity_feedback(&mut self) -> Result<(), LoopError> {
        // Training delays instantaneous simulation velocity to model firmware measurement
        // latency. A real present_velocity sample already contains that latency.
        self.hardware_velocity_feedback = true;
        self.apply_velocity_observation_delay(0)
    }

    fn apply_velocity_observation_delay(&mut self, simulation_delay: u32) -> Result<(), LoopError> {
        self.body
            .set_velocity_observation_delay(if self.hardware_velocity_feedback {
                0
            } else {
                simulation_delay
            })
    }

    fn validate_policy(
        &self,
        descriptor: &PolicyDescriptor,
        config: PolicyConfig,
    ) -> Result<(), LoopError> {
        super::core::validate_config(config)?;
        descriptor.validate(&self.body_profile, self.control_period)?;
        if let Some(action_scale) = descriptor.action_scale
            && (config.action_scale - action_scale).abs() > f64::EPSILON
        {
            return Err(LoopError::Controller(format!(
                "policy {} action_scale={} does not match runtime {}",
                descriptor.id, action_scale, config.action_scale
            )));
        }
        if descriptor.head_lowpass != config.head_lowpass
            || descriptor.legs_lowpass != config.legs_lowpass
        {
            return Err(LoopError::Controller(format!(
                "policy {} target filter does not match runtime policy config",
                descriptor.id
            )));
        }
        if descriptor.joint_velocity_delay_ticks > 1 {
            return Err(LoopError::Controller(format!(
                "policy {} requires unsupported joint velocity delay {} ticks",
                descriptor.id, descriptor.joint_velocity_delay_ticks
            )));
        }
        Ok(())
    }

    pub(crate) fn prepare_policy(
        &self,
        path: &Path,
        descriptor: &PolicyDescriptor,
        config: PolicyConfig,
    ) -> Result<PreparedPolicy, LoopError> {
        self.validate_policy(descriptor, config)?;
        let mut paths = PolicyPaths {
            walk: path.to_owned(),
            ..PolicyPaths::default()
        };
        match config.net {
            Net::Stand => paths.stand = Some(path.to_owned()),
            Net::SitStand => paths.sitstand = Some(path.to_owned()),
            Net::GroundPick => paths.ground_pick = Some(path.to_owned()),
            Net::Skill(0) => paths.skills.push(path.to_owned()),
            Net::Skill(_) => {
                return Err(LoopError::Controller("invalid policy configuration".into()));
            }
            Net::Walk => {}
        }
        duck_control::policy::validate(path)
            .map_err(|error| LoopError::Controller(error.to_string()))?;
        let policy = Policy::load(&paths, DEFAULT_STANDING_THRESHOLD)
            .map_err(|error| LoopError::Controller(error.to_string()))?;
        Ok(PreparedPolicy {
            policy: Arc::new(Mutex::new(LoadedPolicy::Official(Box::new(policy)))),
            config,
            velocity_delay: descriptor.joint_velocity_delay_ticks,
            home: None,
        })
    }

    pub(crate) fn prepare_local_policy(
        &self,
        bytes: &[u8],
        descriptor: &PolicyDescriptor,
        config: PolicyConfig,
        spec: &LocalPolicySpec,
    ) -> Result<PreparedPolicy, LoopError> {
        self.validate_policy(descriptor, config)?;
        let policy = LocalPolicy::load(bytes, spec)?;
        Ok(PreparedPolicy {
            policy: Arc::new(Mutex::new(LoadedPolicy::Local(Box::new(policy)))),
            config,
            velocity_delay: descriptor.joint_velocity_delay_ticks,
            home: Some(spec.home),
        })
    }

    pub(crate) fn install_policy(&mut self, prepared: PreparedPolicy) -> Result<(), LoopError> {
        if self.status().phase == LoopPhase::Homing {
            return Err(LoopError::Controller(
                "policy switch is not allowed during homing".into(),
            ));
        }
        self.core.set_config(prepared.config)?;
        self.apply_velocity_observation_delay(prepared.velocity_delay)?;
        prepared
            .policy
            .lock()
            .map_err(|_| LoopError::Controller("policy session lock poisoned".into()))?
            .reset();
        self.policy = Some(prepared.policy);
        self.policy_home = prepared.home;
        self.core.set_policy_home(self.policy_home);
        self.seed_from_sensors = true;
        self.last_trace = None;
        self.active_net = None;
        self.last_frame = None;
        self.stop_snapshot = None;
        Ok(())
    }

    pub fn reset_policy(&mut self) {
        if let Some(policy) = &self.policy
            && let Ok(mut policy) = policy.lock()
        {
            policy.reset();
        }
        self.seed_from_sensors = true;
        self.core.reset();
        self.last_trace = None;
        self.active_net = None;
        self.last_frame = None;
        self.stop_snapshot = None;
    }

    /// Install a fresh body while retaining the loaded ONNX session and current command.
    pub(crate) fn replace_body(
        &mut self,
        mut next: Self,
        velocity_delay: u32,
    ) -> Result<(), LoopError> {
        next.apply_velocity_observation_delay(velocity_delay)?;
        next.core.set_config(self.core.config())?;
        next.core.set_policy_home(self.policy_home);
        self.body = next.body;
        self.body_profile = next.body_profile;
        self.control_period = next.control_period;
        self.hardware_velocity_feedback = next.hardware_velocity_feedback;
        self.core = next.core;
        self.reset_policy();
        Ok(())
    }

    pub(crate) fn sample_body(&mut self) -> Result<(), LoopError> {
        self.last_trace = None;
        self.body.read_sensors_only(self.control_period)
    }

    pub(crate) fn arm_at_current_pose(&mut self) -> Result<(), LoopError> {
        self.body.arm_at_current_pose()
    }

    /// Reset backend state before the next policy observation is produced.
    pub fn reset_execution(&mut self) -> Result<(), LoopError> {
        self.body.reset_execution()
    }

    pub fn set_policy_command(&mut self, command: Command) {
        self.command = command;
    }

    pub fn send(&mut self, action: LoopAction) -> Result<(), LoopError> {
        // A lifecycle stop pauses the policy stream. Keep the last successful observation,
        // action and target history so a later Power(true) resumes from the same body state.
        if matches!(
            action,
            LoopAction::Power(false)
                | LoopAction::Drive(false)
                | LoopAction::Stop
                | LoopAction::Estop
        ) {
            self.stop_snapshot = self.last_frame.clone();
        }
        let result = self.body.send(action);
        if result.is_ok()
            && matches!(action, LoopAction::Power(true))
            && let Some(snapshot) = self.stop_snapshot.as_ref()
        {
            self.command = snapshot.command;
            self.core.restore_history(&snapshot.history)?;
        }
        result
    }

    pub fn set_command(&mut self, command: BodyCommand) -> Result<(), LoopError> {
        self.body.set_command(command)
    }

    pub fn status(&self) -> LoopStatus {
        self.body.status()
    }

    pub fn policy_trace(&self) -> Option<&PolicyTrace> {
        self.last_trace.as_ref()
    }

    pub fn has_policy(&self) -> bool {
        self.policy.is_some()
    }

    pub fn servo_ids(&self) -> impl Iterator<Item = u8> + '_ {
        self.body.servo_ids()
    }

    pub fn tick(&mut self, dt: Duration) -> Result<(), LoopError> {
        self.last_trace = None;
        // A body without a controller is a sampling source. In particular, do not let
        // simulation fall/settle while the UI is still installing its first policy.
        if self.policy.is_none() {
            self.body.read_sensors(dt)?;
            return Ok(());
        }
        // Holding and stopped phases do not advance the controller, but they also do not erase
        // its history. `reset_policy` is the explicit new-episode boundary.
        let mut policy = self
            .policy
            .as_ref()
            .map(|policy| policy.lock())
            .transpose()
            .map_err(|_| LoopError::Controller("policy session lock poisoned".into()))?;
        let mut controller = PolicyTickController {
            policy: policy.as_deref_mut(),
            command: self.command,
            core: &mut self.core,
            last_trace: &mut self.last_trace,
            last_frame: &mut self.last_frame,
            active_net: &mut self.active_net,
            seed_from_sensors: &mut self.seed_from_sensors,
        };
        let result = self.body.tick_with_controller(dt, &mut controller);
        if result.is_ok()
            && let Some(trace) = self.last_trace.as_mut()
        {
            let status = self.body.status();
            if let Some(requested) = trace.body_targets.as_ref()
                && requested
                    .iter()
                    .zip(&status.targets)
                    .any(|(requested, applied)| (requested - applied).abs() > f64::EPSILON)
            {
                trace.saturation_reasons.push("body_range_limit");
            }
            trace.body_targets = Some(status.targets.clone());
            trace.stages.push("write");
            if let Some(frame) = self.last_frame.as_mut() {
                frame.applied_targets = status.targets;
            }
        }
        result
    }
}

#[cfg(test)]
mod tests {
    use super::super::action_space::{BodyActionCodec, BodyActionSpace};
    use super::{
        HEAD_LOWPASS, LEGS_LOWPASS, PolicyConfig, PolicyDescriptor, WALK_ACTION_SCALE,
        walking_targets,
    };
    use duck_body::BodyProfile;
    use duck_control::imu::ImuData;
    use duck_control::model::{DEFAULT_POSITION, MOUTH_INDEX, NUM_JOINTS};
    use duck_control::obs::{ACTION_LEN, Command};
    use std::time::Duration;

    #[test]
    fn hardware_sampling_and_body_handoff_do_not_add_the_training_velocity_delay() {
        use loop_body::{BackendJointSample, BackendPositionTargets, ControlBackend};
        use loop_body::{
            BodyRuntimeView, Calibration, JointCalibration, LoopBinding, LoopConfig, ServoBinding,
        };
        use servo::{ServoError, ServoModel};

        struct VelocityProbe(u32);
        impl ControlBackend for VelocityProbe {
            fn read_positions(&mut self, ids: &[u8]) -> Result<BackendJointSample, ServoError> {
                self.0 += 1;
                Ok(BackendJointSample {
                    positions: vec![0.0; ids.len()],
                    velocities: vec![f64::from(self.0); ids.len()],
                })
            }
            fn write_positions(
                &mut self,
                _: &[u8],
                _: &BackendPositionTargets,
            ) -> Result<(), ServoError> {
                panic!("sampling must not write targets")
            }
            fn set_torque(&mut self, _: bool) -> Result<(), ServoError> {
                panic!("sampling must not change torque")
            }
            fn set_gain(&mut self, _: u16) -> Result<(), ServoError> {
                panic!("sampling must not change gain")
            }
            fn reboot(&mut self, _: u8) -> Result<(), ServoError> {
                panic!("sampling must not reboot")
            }
        }

        let runner = || {
            let template = crate::runtime::adapters::body_assets::test_template();
            let slots: Vec<_> = template
                .actuators
                .iter()
                .map(|slot| slot.id.clone())
                .collect();
            super::MicroDuckPolicyRunner::open(
                LoopBinding {
                    body: BodyRuntimeView::new(template.clone()).unwrap(),
                    servos: ServoBinding {
                        model: ServoModel::Hd1910,
                        slot_ids: slots.clone(),
                    },
                    calibration: Calibration {
                        body_id: template.id,
                        body_version: template.version,
                        joints: slots
                            .into_iter()
                            .enumerate()
                            .map(|(index, slot_id)| JointCalibration {
                                slot_id,
                                physical_id: (index + 1) as u8,
                                direction: 1,
                                zero_offset_rad: 0.0,
                                min_rad: None,
                                max_rad: None,
                            })
                            .collect(),
                    },
                },
                Box::new(VelocityProbe(0)),
                LoopConfig::default(),
            )
            .unwrap()
        };
        let mut active = runner();
        active.apply_velocity_observation_delay(1).unwrap();
        active.sample_body().unwrap();
        assert_eq!(active.body.sensor_frame().velocities, vec![0.0; 15]);
        active.sample_body().unwrap();
        assert_eq!(active.body.sensor_frame().velocities, vec![1.0; 15]);

        let mut physical = runner();
        physical.use_hardware_velocity_feedback().unwrap();
        active.replace_body(physical, 1).unwrap();
        active.sample_body().unwrap();
        assert_eq!(active.body.sensor_frame().velocities, vec![1.0; 15]);
        // Policy installation uses this same helper, including for prewarmed sessions.
        active.apply_velocity_observation_delay(1).unwrap();
        active.sample_body().unwrap();
        assert_eq!(active.body.sensor_frame().velocities, vec![2.0; 15]);

        active.replace_body(runner(), 1).unwrap();
        active.sample_body().unwrap();
        assert_eq!(active.body.sensor_frame().velocities, vec![0.0; 15]);
        active.sample_body().unwrap();
        assert_eq!(active.body.sensor_frame().velocities, vec![1.0; 15]);
    }

    #[test]
    fn raw_output_decoder_rejects_wrong_width_and_non_finite_values() {
        let decoder = super::RawPolicyOutputDecoder;
        assert!(decoder.decode(&[0.0; ACTION_LEN - 1]).is_err());
        assert!(decoder.decode(&[f32::NAN; ACTION_LEN]).is_err());
        assert!(decoder.decode(&[0.0; ACTION_LEN]).is_ok());
    }

    #[test]
    fn walking_targets_use_official_scale_without_implicit_filter() {
        let action = [1.0; ACTION_LEN];
        let space =
            BodyActionSpace::official(&crate::runtime::adapters::body_assets::test_template())
                .unwrap();
        let codec = BodyActionCodec::new(space);
        let first = walking_targets(
            &action,
            &codec,
            &DEFAULT_POSITION,
            None,
            PolicyConfig::default(),
        );
        assert_eq!(first[0], DEFAULT_POSITION[0] + WALK_ACTION_SCALE);
        assert_eq!(first[MOUTH_INDEX], DEFAULT_POSITION[MOUTH_INDEX]);

        let previous = DEFAULT_POSITION;
        let second = walking_targets(
            &action,
            &codec,
            &DEFAULT_POSITION,
            Some(&previous),
            PolicyConfig::default(),
        );
        assert_eq!(second[0], first[0]);
        assert_eq!(second[5], first[5]);
    }

    #[test]
    fn declared_filter_is_applied_only_when_policy_contract_requests_it() {
        let action = [1.0; ACTION_LEN];
        let space =
            BodyActionSpace::official(&crate::runtime::adapters::body_assets::test_template())
                .unwrap();
        let codec = BodyActionCodec::new(space);
        let previous = DEFAULT_POSITION;
        let targets = walking_targets(
            &action,
            &codec,
            &DEFAULT_POSITION,
            Some(&previous),
            PolicyConfig {
                head_lowpass: Some(HEAD_LOWPASS),
                legs_lowpass: Some(LEGS_LOWPASS),
                ..PolicyConfig::default()
            },
        );
        assert_eq!(
            targets[0],
            DEFAULT_POSITION[0] + LEGS_LOWPASS * WALK_ACTION_SCALE
        );
        assert_eq!(
            targets[5],
            DEFAULT_POSITION[5] + HEAD_LOWPASS * WALK_ACTION_SCALE
        );
    }

    #[test]
    fn policy_projection_matches_training() {
        let body = crate::runtime::adapters::body_assets::test_template();
        let space = BodyActionSpace::official(&body).unwrap();
        let expected: Vec<_> = body
            .actuators
            .iter()
            .filter(|slot| slot.id != "mouth")
            .map(|slot| slot.id.as_str())
            .collect();
        let actual: Vec<_> = space
            .policy_action_slots()
            .iter()
            .map(|&slot| body.actuators[slot].id.as_str())
            .collect();
        assert_eq!(actual, expected);
        assert!(!space.policy_action_slots().contains(&MOUTH_INDEX));

        let codec = BodyActionCodec::new(space);
        let action: Vec<f32> = (0..ACTION_LEN).map(|value| value as f32 + 1.0).collect();
        let canonical = codec.decode(&action).unwrap();
        assert_eq!(canonical[MOUTH_INDEX], 0.0);
        assert_eq!(codec.encode(&canonical).unwrap(), action);
    }

    #[test]
    fn policy_descriptor_rejects_incompatible_declarations_before_load() {
        let profile = crate::runtime::adapters::body_assets::test_template();
        let profile = BodyProfile::from_template(profile).unwrap();
        let valid = PolicyDescriptor {
            id: "walk".into(),
            body_id: "official".into(),
            body_version: 1,
            obs_len: 61,
            action_len: ACTION_LEN,
            control_hz: 50,
            command_encoding: Some("constant".into()),
            recurrent: Some(false),
            action_scale: None,
            head_lowpass: None,
            legs_lowpass: None,
            joint_velocity_delay_ticks: 1,
        };
        assert!(valid.validate(&profile, Duration::from_millis(20)).is_ok());

        for invalid in [
            PolicyDescriptor {
                obs_len: 60,
                ..valid.clone()
            },
            PolicyDescriptor {
                action_len: ACTION_LEN - 1,
                ..valid.clone()
            },
            PolicyDescriptor {
                body_id: "other".into(),
                ..valid.clone()
            },
            PolicyDescriptor {
                body_version: 2,
                ..valid.clone()
            },
            PolicyDescriptor {
                control_hz: 100,
                ..valid.clone()
            },
            PolicyDescriptor {
                command_encoding: Some("invented".into()),
                ..valid.clone()
            },
        ] {
            assert!(
                invalid
                    .validate(&profile, Duration::from_millis(20))
                    .is_err()
            );
        }
    }

    #[test]
    fn observation_last_action_is_raw() {
        let encoder = super::PolicyInputEncoder;
        let raw: [f32; ACTION_LEN] = std::array::from_fn(|index| index as f32 * 0.25 - 1.0);
        let observation = encoder.encode(
            &ImuData::default(),
            &DEFAULT_POSITION,
            &[0.0; NUM_JOINTS],
            &DEFAULT_POSITION,
            &raw,
            &Command::default(),
        );
        assert_eq!(&observation.as_slice()[34..48], &raw);
    }
}

pub(crate) struct OpenLoopInput {
    pub binding: LoopBinding,
    pub driver: Box<dyn ControlBackend>,
    pub loop_config: LoopConfig,
    pub policy: Option<(PathBuf, PolicyConfig, PolicyDescriptor)>,
    pub config_dir: Option<PathBuf>,
    pub backend_kind: &'static str,
    pub backend_profile: String,
    pub contract_hash: String,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct RuntimeTiming {
    pub(crate) physics_dt: f64,
    pub(crate) physics_substeps: usize,
    pub(crate) delay_timebase: &'static str,
    pub(crate) delay_steps: &'static str,
    pub(crate) decimation: usize,
}

pub(crate) fn runtime_timing(backend_kind: &str, control_dt: f64) -> RuntimeTiming {
    if backend_kind == "simulation" {
        // The preview driver advances four 5ms physics steps per 20ms policy tick and owns the
        // vendor's [3, 6] physics-step actuator delay queue.
        RuntimeTiming {
            physics_dt: 0.005,
            physics_substeps: 4,
            delay_timebase: "physics_step",
            delay_steps: "3..6",
            decimation: 4,
        }
    } else {
        // A physical servo firmware owns its internal timing and delay. The host only observes
        // the result, so there is no host-side physics substep to report.
        RuntimeTiming {
            physics_dt: control_dt,
            physics_substeps: 1,
            delay_timebase: "transport",
            delay_steps: "0",
            decimation: 1,
        }
    }
}

pub(crate) fn open_loop(input: OpenLoopInput) -> Result<MicroDuckPolicyRunner, String> {
    let loop_config = input.loop_config;
    let mut loop_state =
        MicroDuckPolicyRunner::open(input.binding, input.driver, loop_config.clone())
            .map_err(|error| error.to_string())?;
    let host_velocity_delay_ticks = if input.backend_kind == "physical" {
        loop_state
            .use_hardware_velocity_feedback()
            .map_err(|error| error.to_string())?;
        0
    } else {
        LoopConfig::default().velocity_observation_delay_ticks
    };
    if let Some((path, config, descriptor)) = input.policy.as_ref() {
        append_control_event(
            input.config_dir.as_deref(),
            format!("policy_selected path={path:?} config={config:?}"),
        );
        loop_state
            .select_policy_with_descriptor(path, descriptor, *config)
            .map_err(|error| {
                append_control_event(
                    input.config_dir.as_deref(),
                    format!("policy_load_error path={path:?} error={error}"),
                );
                error.to_string()
            })?;
    } else {
        append_control_event(input.config_dir.as_deref(), "policy_selected none");
    }
    let runtime_config = input
        .policy
        .as_ref()
        .map(|(_, config, _)| *config)
        .unwrap_or_default();
    let control_dt = loop_config.period.as_secs_f64();
    let timing = runtime_timing(input.backend_kind, control_dt);
    append_control_event(
        input.config_dir.as_deref(),
        format!(
            "policy_runtime backend_kind={} backend_profile={} contract_hash={} config={runtime_config:?} observation_dim=61 action_dim=14 control_dt={control_dt} physics_dt={} physics_substeps={} decimation={} delay_timebase={} delay_steps={} observation_delay_ticks={host_velocity_delay_ticks} imu_source=backend_or_default gain={}",
            input.backend_kind,
            input.backend_profile,
            input.contract_hash,
            timing.physics_dt,
            timing.physics_substeps,
            timing.decimation,
            timing.delay_timebase,
            timing.delay_steps,
            loop_config.gain
        ),
    );
    Ok(loop_state)
}
