//! ONNX policy execution layered on top of the base body loop.

use std::path::Path;
use std::sync::{Arc, Mutex};
use std::time::Duration;

use duck_body::BodyProfile;
use duck_control::imu::ImuData;
use duck_control::obs::{ACTION_LEN, Command};
use duck_control::policy::{DEFAULT_STANDING_THRESHOLD, Net, Policy, PolicyPaths};

use loop_body::{
    BackendState, BodyCommand, BodyLoop, ControlBackend, ControllerInput, LoopAction, LoopBinding,
    LoopConfig, LoopController, LoopError, LoopPhase, LoopStatus, SampleTime,
};

use super::core::PolicyHistory;
use super::{ControlTickInput, PolicyConfig, PolicyControlCore};

#[cfg(test)]
use super::core::walking_target_trace;
#[cfg(test)]
pub(crate) use super::{PolicyInputEncoder, RawPolicyOutputDecoder};
#[cfg(test)]
use duck_control::model::NUM_JOINTS;
#[cfg(test)]
const HEAD_LOWPASS: f64 = 0.5;
#[cfg(test)]
const LEGS_LOWPASS: f64 = 0.7;
#[cfg(test)]
const WALK_ACTION_SCALE: f64 = 1.0;
#[cfg(test)]
fn walking_targets(
    action: &[f32; ACTION_LEN],
    codec: &duck_body::BodyActionCodec,
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

struct PolicyTickController<'a> {
    policy: Option<&'a mut Policy>,
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
        let raw_action = policy
            .infer(&observation, selected_net)
            .map_err(|error| LoopError::Controller(error.to_string()))?;
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

/// Static policy artifact contract checked before an ONNX session enters the tick loop.
#[derive(Debug, Clone, PartialEq, serde::Serialize)]
pub struct PolicyDescriptor {
    pub id: String,
    pub body_id: String,
    pub body_version: u32,
    pub obs_len: usize,
    pub action_len: usize,
    pub control_hz: u32,
    pub command_encoding: Option<String>,
    pub recurrent: Option<bool>,
    pub action_scale: Option<f64>,
    pub head_lowpass: Option<f64>,
    pub legs_lowpass: Option<f64>,
    pub joint_velocity_delay_ticks: u32,
}

impl PolicyDescriptor {
    pub fn validate(&self, profile: &BodyProfile, period: Duration) -> Result<(), LoopError> {
        let identity = profile.identity();
        if self.body_id != identity.id || self.body_version != identity.version {
            return Err(LoopError::Controller(format!(
                "policy {} targets body {} v{}, runtime has {} v{}",
                self.id, self.body_id, self.body_version, identity.id, identity.version
            )));
        }
        if self.obs_len != 61 || self.action_len != ACTION_LEN {
            return Err(LoopError::Controller(format!(
                "policy {} layout is obs={} action={}, expected obs=61 action={ACTION_LEN}",
                self.id, self.obs_len, self.action_len
            )));
        }
        let runtime_hz = 1.0 / period.as_secs_f64();
        if !runtime_hz.is_finite() || (runtime_hz - f64::from(self.control_hz)).abs() > 0.01 {
            return Err(LoopError::Controller(format!(
                "policy {} control_hz={} does not match runtime {:.3} Hz",
                self.id, self.control_hz, runtime_hz
            )));
        }
        if let Some(encoding) = &self.command_encoding
            && !matches!(encoding.as_str(), "constant" | "phase" | "posture_flag")
        {
            return Err(LoopError::Controller(format!(
                "policy {} has unsupported command encoding {encoding:?}",
                self.id
            )));
        }
        for (name, value) in [
            ("action_scale", self.action_scale),
            ("head_lowpass", self.head_lowpass),
            ("legs_lowpass", self.legs_lowpass),
        ] {
            if let Some(value) = value
                && (!value.is_finite()
                    || (name == "action_scale" && value <= 0.0)
                    || (name != "action_scale" && !(0.0..=1.0).contains(&value)))
            {
                return Err(LoopError::Controller(format!(
                    "policy {} has invalid {name}={value}",
                    self.id
                )));
            }
        }
        Ok(())
    }
}

#[derive(Clone)]
pub(crate) struct PreparedPolicy {
    policy: Arc<Mutex<Policy>>,
    config: PolicyConfig,
    velocity_delay: u32,
}

pub struct MicroDuckPolicyRunner {
    body: BodyLoop,
    policy: Option<Arc<Mutex<Policy>>>,
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

    pub(crate) fn prepare_policy(
        &self,
        path: &Path,
        descriptor: &PolicyDescriptor,
        config: PolicyConfig,
    ) -> Result<PreparedPolicy, LoopError> {
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
            policy: Arc::new(Mutex::new(policy)),
            config,
            velocity_delay: descriptor.joint_velocity_delay_ticks,
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
    use super::{
        HEAD_LOWPASS, LEGS_LOWPASS, PolicyConfig, PolicyDescriptor, WALK_ACTION_SCALE,
        walking_targets,
    };
    use duck_body::{BodyActionSpace, BodyProfile, BodyTemplate};
    use duck_control::imu::ImuData;
    use duck_control::model::{DEFAULT_POSITION, MOUTH_INDEX, NUM_JOINTS};
    use duck_control::obs::{ACTION_LEN, Command};
    use std::time::Duration;

    #[test]
    fn hardware_sampling_and_body_handoff_do_not_add_the_training_velocity_delay() {
        use loop_body::{
            BodyRuntimeView, Calibration, JointCalibration, LoopBinding, LoopConfig, ServoBinding,
        };
        use servo::{
            ServoError, ServoLoopDriver, ServoModel, ServoPositionSample, ServoPositionTargets,
        };

        struct VelocityProbe(u32);
        impl ServoLoopDriver for VelocityProbe {
            fn read_positions(&mut self, ids: &[u8]) -> Result<ServoPositionSample, ServoError> {
                self.0 += 1;
                Ok(ServoPositionSample {
                    positions: vec![0.0; ids.len()],
                    velocities: vec![f64::from(self.0); ids.len()],
                })
            }
            fn write_positions(
                &mut self,
                _: &[u8],
                _: &ServoPositionTargets,
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
            let template = BodyTemplate::official().unwrap();
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
        let space = BodyActionSpace::from_template(&BodyTemplate::official().unwrap()).unwrap();
        let codec = duck_body::BodyActionCodec::new(space);
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
        let space = BodyActionSpace::from_template(&BodyTemplate::official().unwrap()).unwrap();
        let codec = duck_body::BodyActionCodec::new(space);
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
        let body = BodyTemplate::official().unwrap();
        let space = BodyActionSpace::from_template(&body).unwrap();
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

        let codec = duck_body::BodyActionCodec::new(space);
        let action: Vec<f32> = (0..ACTION_LEN).map(|value| value as f32 + 1.0).collect();
        let canonical = codec.decode(&action).unwrap();
        assert_eq!(canonical[MOUTH_INDEX], 0.0);
        assert_eq!(codec.encode(&canonical).unwrap(), action);
    }

    #[test]
    fn policy_descriptor_rejects_incompatible_declarations_before_load() {
        let profile = BodyTemplate::official().unwrap();
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
