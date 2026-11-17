//! ONNX policy execution layered on top of the base body loop.

use std::path::Path;
use std::time::Duration;

use duck_body::{BodyActionCodec, BodyProfile};
use duck_control::imu::ImuData;
use duck_control::model::{MOUTH_INDEX, NUM_JOINTS};
use duck_control::obs::{ACTION_LEN, Command};
use duck_control::policy::{DEFAULT_STANDING_THRESHOLD, Net, Policy, PolicyPaths};

use crate::{
    BodyCommand, BodyLoop, LoopAction, LoopBinding, LoopConfig, LoopError, LoopPhase, LoopStatus,
    PolicyInputEncoder, RawPolicyOutputDecoder,
};

// These are the official walking controller's default post-processing parameters. The ONNX
// network produces offsets; sending those raw offsets directly to the servos makes each
// inference tick visible as a target jump.
const WALK_ACTION_SCALE: f64 = 0.9;
const HEAD_LOWPASS: f64 = 0.5;
const LEGS_LOWPASS: f64 = 0.7;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PolicyConfig {
    pub net: Net,
    pub action_scale: f64,
    pub head_lowpass: Option<f64>,
    pub legs_lowpass: Option<f64>,
}

impl Default for PolicyConfig {
    fn default() -> Self {
        Self {
            net: Net::Walk,
            action_scale: WALK_ACTION_SCALE,
            head_lowpass: Some(HEAD_LOWPASS),
            legs_lowpass: Some(LEGS_LOWPASS),
        }
    }
}

fn walking_target_trace(
    action: &[f32; ACTION_LEN],
    codec: &BodyActionCodec,
    home: &[f64; NUM_JOINTS],
    previous: Option<&[f64; NUM_JOINTS]>,
    ranges: &[[f64; 2]],
    config: PolicyConfig,
) -> ([f64; NUM_JOINTS], [f64; NUM_JOINTS]) {
    let offsets = codec
        .decode(action)
        .expect("official action layout was validated when the loop opened");
    let raw = std::array::from_fn(|joint| home[joint] + config.action_scale * offsets[joint]);
    let mut targets =
        std::array::from_fn(|joint| raw[joint].clamp(ranges[joint][0], ranges[joint][1]));
    if let Some(previous) = previous {
        for (joint, target) in targets.iter_mut().enumerate() {
            let alpha = if (5..9).contains(&joint) {
                config.head_lowpass
            } else if joint == MOUTH_INDEX {
                continue;
            } else {
                config.legs_lowpass
            };
            if let Some(alpha) = alpha {
                *target = alpha * *target + (1.0 - alpha) * previous[joint];
            }
        }
    }
    (raw, targets)
}

#[cfg(test)]
fn walking_targets(
    action: &[f32; ACTION_LEN],
    codec: &BodyActionCodec,
    home: &[f64; NUM_JOINTS],
    previous: Option<&[f64; NUM_JOINTS]>,
    ranges: &[[f64; 2]],
    config: PolicyConfig,
) -> [f64; NUM_JOINTS] {
    walking_target_trace(action, codec, home, previous, ranges, config).1
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
    pub stages: Vec<&'static str>,
}

/// Static policy artifact contract checked before an ONNX session enters the tick loop.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PolicyDescriptor {
    pub id: String,
    pub body_id: String,
    pub body_version: u32,
    pub obs_len: usize,
    pub action_len: usize,
    pub control_hz: u32,
    pub command_encoding: Option<String>,
    pub recurrent: Option<bool>,
}

impl PolicyDescriptor {
    pub fn validate(&self, profile: &BodyProfile, period: Duration) -> Result<(), LoopError> {
        let identity = profile.identity();
        if self.body_id != identity.id || self.body_version != identity.version {
            return Err(LoopError::Policy(format!(
                "policy {} targets body {} v{}, runtime has {} v{}",
                self.id, self.body_id, self.body_version, identity.id, identity.version
            )));
        }
        if self.obs_len != 61 || self.action_len != ACTION_LEN {
            return Err(LoopError::Policy(format!(
                "policy {} layout is obs={} action={}, expected obs=61 action={ACTION_LEN}",
                self.id, self.obs_len, self.action_len
            )));
        }
        let runtime_hz = 1.0 / period.as_secs_f64();
        if !runtime_hz.is_finite() || (runtime_hz - f64::from(self.control_hz)).abs() > 0.01 {
            return Err(LoopError::Policy(format!(
                "policy {} control_hz={} does not match runtime {:.3} Hz",
                self.id, self.control_hz, runtime_hz
            )));
        }
        if let Some(encoding) = &self.command_encoding
            && !matches!(encoding.as_str(), "constant" | "phase" | "posture_flag")
        {
            return Err(LoopError::Policy(format!(
                "policy {} has unsupported command encoding {encoding:?}",
                self.id
            )));
        }
        Ok(())
    }
}

pub struct PolicyLoop {
    body: BodyLoop,
    policy: Option<Policy>,
    last_action: [f32; ACTION_LEN],
    command: Command,
    ranges: Vec<[f64; 2]>,
    last_trace: Option<PolicyTrace>,
    previous_targets: Option<[f64; NUM_JOINTS]>,
    policy_config: PolicyConfig,
    output_decoder: RawPolicyOutputDecoder,
    input_encoder: PolicyInputEncoder,
    action_codec: BodyActionCodec,
    body_profile: BodyProfile,
    home: [f64; NUM_JOINTS],
    control_period: Duration,
}

impl PolicyLoop {
    pub fn open(
        binding: LoopBinding,
        driver: Box<dyn servo::ServoLoopDriver>,
        config: LoopConfig,
    ) -> Result<Self, LoopError> {
        Self::open_with_command(binding, driver, config, Command::default())
    }

    pub fn open_with_command(
        binding: LoopBinding,
        driver: Box<dyn servo::ServoLoopDriver>,
        config: LoopConfig,
        command: Command,
    ) -> Result<Self, LoopError> {
        let action_codec = binding.body.profile().action_codec();
        let body_profile = binding.body.profile().clone();
        let home: [f64; NUM_JOINTS] = body_profile
            .template()
            .control
            .home_pose
            .clone()
            .try_into()
            .map_err(|_| LoopError::SensorLength {
                actual: body_profile.template().control.home_pose.len(),
                expected: NUM_JOINTS,
            })?;
        let control_period = config.period;
        let ranges = binding
            .body
            .template()
            .joints
            .iter()
            .map(|joint| joint.range_rad)
            .collect();
        let body = BodyLoop::open(binding, driver, config)?;
        Ok(Self {
            body,
            policy: None,
            last_action: [0.0; ACTION_LEN],
            command,
            ranges,
            last_trace: None,
            previous_targets: None,
            policy_config: PolicyConfig::default(),
            output_decoder: RawPolicyOutputDecoder,
            input_encoder: PolicyInputEncoder,
            action_codec,
            body_profile,
            home,
            control_period,
        })
    }

    pub fn select_policy(&mut self, path: impl AsRef<Path>) -> Result<(), LoopError> {
        self.select_policy_with_config(path, PolicyConfig::default())
    }

    pub fn select_policy_with_config(
        &mut self,
        path: impl AsRef<Path>,
        config: PolicyConfig,
    ) -> Result<(), LoopError> {
        if self.status().phase == LoopPhase::Homing {
            return Err(LoopError::Policy(
                "policy switch is not allowed during homing".into(),
            ));
        }
        if !config.action_scale.is_finite()
            || config.action_scale <= 0.0
            || [config.head_lowpass, config.legs_lowpass]
                .into_iter()
                .flatten()
                .any(|alpha| !alpha.is_finite() || !(0.0..=1.0).contains(&alpha))
            || matches!(config.net, Net::Skill(index) if index != 0)
        {
            return Err(LoopError::Policy("invalid policy configuration".into()));
        }
        // Seed the filter from the next real sensor pose. `BodyLoop::last_targets` is still
        // zero before the first sensor tick, so taking it here would pull the first policy
        // target toward the numeric zero frame instead of the pose the robot is holding.
        let path = path.as_ref().to_owned();
        let mut paths = PolicyPaths {
            walk: path.clone(),
            ..PolicyPaths::default()
        };
        match config.net {
            Net::Stand => paths.stand = Some(path.clone()),
            Net::SitStand => paths.sitstand = Some(path.clone()),
            Net::GroundPick => paths.ground_pick = Some(path.clone()),
            Net::Skill(_) => paths.skills.push(path.clone()),
            Net::Walk => {}
        }
        let policy = Policy::load(&paths, DEFAULT_STANDING_THRESHOLD)
            .map_err(|error| LoopError::Policy(error.to_string()))?;
        self.policy = Some(policy);
        self.last_action = [0.0; ACTION_LEN];
        self.previous_targets = None;
        self.policy_config = config;
        self.last_trace = None;
        Ok(())
    }

    pub fn select_policy_with_descriptor(
        &mut self,
        path: impl AsRef<Path>,
        descriptor: &PolicyDescriptor,
        config: PolicyConfig,
    ) -> Result<(), LoopError> {
        descriptor.validate(&self.body_profile, self.body_period())?;
        duck_control::policy::validate(path.as_ref())
            .map_err(|error| LoopError::Policy(error.to_string()))?;
        self.select_policy_with_config(path, config)
    }

    pub fn reset_policy(&mut self) {
        if let Some(policy) = &mut self.policy {
            policy.reset();
        }
        self.last_action = [0.0; ACTION_LEN];
        self.previous_targets = None;
        self.last_trace = None;
    }

    /// Stop policy inference while a runtime controller prepares the body for another policy.
    ///
    /// The body loop remains powered and can continue holding an explicit [`BodyCommand`].
    /// This keeps policy lifecycle decisions (for example, skill start and end) outside this
    /// crate while giving the owner a safe gap between two episodes.
    pub fn clear_policy(&mut self) {
        self.reset_policy();
        self.policy = None;
    }

    fn body_period(&self) -> Duration {
        self.control_period
    }

    pub fn set_policy_command(&mut self, command: Command) {
        self.command = command;
    }

    pub fn send(&mut self, action: LoopAction) -> Result<(), LoopError> {
        self.body.send(action)
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

    pub fn tick(&mut self, dt: Duration) -> Result<(), LoopError> {
        let mut stages = Vec::new();
        self.body.update_sensors(dt)?;
        stages.push("read");
        let current = self.body.status();
        self.last_trace = None;
        // A paused or unpowered loop must not advance recurrent state or the target filter.
        // On resume the filter starts from the newly measured pose.
        if !matches!(current.phase, LoopPhase::Ready | LoopPhase::Driving) {
            self.previous_targets = None;
            return self.body.apply_targets();
        }
        if let Some(policy) = &mut self.policy {
            let positions: [f64; NUM_JOINTS] =
                current
                    .positions
                    .clone()
                    .try_into()
                    .map_err(|_| LoopError::SensorLength {
                        actual: current.positions.len(),
                        expected: NUM_JOINTS,
                    })?;
            let velocities: [f64; NUM_JOINTS] = self
                .body
                .sensor_velocities()
                .to_vec()
                .try_into()
                .map_err(|_| LoopError::SensorLength {
                actual: self.body.sensor_velocities().len(),
                expected: NUM_JOINTS,
            })?;
            let imu = ImuData::default();
            let observation = self.input_encoder.encode(
                &imu,
                &positions,
                &velocities,
                &self.home,
                &self.last_action,
                &self.command,
            );
            stages.push("encode");
            let raw_action = match policy.infer(&observation, self.policy_config.net) {
                Ok(action) => action,
                Err(error) => {
                    let detail = error.to_string();
                    self.body.report_policy_error(detail.clone());
                    return Err(LoopError::Policy(detail));
                }
            };
            stages.push("infer");
            let action = self.output_decoder.decode(&raw_action)?;
            stages.push("decode");
            self.last_action = action;
            let body_targets = if matches!(
                current.phase,
                LoopPhase::Ready | LoopPhase::Driving | LoopPhase::Holding
            ) {
                if current.phase == LoopPhase::Ready {
                    self.body.send(LoopAction::Drive(true))?;
                }
                // The ONNX output is 14 policy slots. `BodyActionCodec` restores the
                // canonical 15-slot body order (including the untouched mouth slot);
                // BodyLoop then applies the bound body conversion to translate that order
                // to simulation IDs or physical IDs.
                // The first target after a policy load or switch is filtered from the current
                // sensor pose. Later ticks inherit the previous filtered endpoint.
                let previous = self.previous_targets.as_ref().unwrap_or(&positions);
                let (raw_target_array, target_array) = walking_target_trace(
                    &action,
                    &self.action_codec,
                    &self.home,
                    Some(previous),
                    &self.ranges,
                    self.policy_config,
                );
                stages.push("target");
                self.previous_targets = Some(target_array);
                let targets = target_array.to_vec();
                let trace_targets = targets.clone();
                let trace_raw_targets = raw_target_array.to_vec();
                let session = self.body.status().session;
                self.body.set_command(BodyCommand { session, targets })?;
                stages.push("validate");
                stages.push("bind");
                Some((trace_raw_targets, trace_targets))
            } else {
                None
            };
            self.last_trace = Some(PolicyTrace {
                phase: current.phase,
                command: self.command,
                imu,
                imu_source: "default",
                observation: observation.as_slice().to_vec(),
                action: action.to_vec(),
                raw_targets: body_targets.as_ref().map(|(raw, _)| raw.clone()),
                body_targets: body_targets.map(|(_, applied)| applied),
                stages,
            });
        }
        let result = self.body.apply_targets();
        if result.is_ok()
            && let Some(trace) = self.last_trace.as_mut()
        {
            trace.stages.push("write");
        }
        result
    }
}

#[cfg(test)]
mod tests {
    use super::{
        HEAD_LOWPASS, LEGS_LOWPASS, PolicyConfig, PolicyDescriptor, WALK_ACTION_SCALE,
        walking_target_trace, walking_targets,
    };
    use duck_body::{BodyActionSpace, BodyProfile, BodyTemplate};
    use duck_control::imu::ImuData;
    use duck_control::model::{DEFAULT_POSITION, MOUTH_INDEX, NUM_JOINTS};
    use duck_control::obs::{ACTION_LEN, Command};
    use serde::Deserialize;
    use std::time::Duration;

    #[derive(Debug, Deserialize)]
    struct ReferenceTraceFixture {
        body_id: String,
        body_version: u32,
        obs_len: usize,
        action_len: usize,
        gain: u16,
        action_scale: f64,
        head_lowpass: f64,
        legs_lowpass: f64,
        raw_action: Vec<f32>,
        home_pose: Vec<f64>,
        expected_observation_prefix: Vec<f32>,
        expected_last_action: Vec<f32>,
        expected_raw_target: Vec<f64>,
        expected_canonical_target: Vec<f64>,
        physical_ids: Vec<u8>,
        directions: Vec<i8>,
        zero_offsets: Vec<f64>,
    }

    #[test]
    fn raw_output_decoder_rejects_wrong_width_and_non_finite_values() {
        let decoder = super::RawPolicyOutputDecoder;
        assert!(decoder.decode(&[0.0; ACTION_LEN - 1]).is_err());
        assert!(decoder.decode(&[f32::NAN; ACTION_LEN]).is_err());
        assert!(decoder.decode(&[0.0; ACTION_LEN]).is_ok());
    }

    #[test]
    fn walking_targets_use_official_scale_and_low_pass() {
        let action = [1.0; ACTION_LEN];
        let ranges = vec![[-10.0, 10.0]; NUM_JOINTS];
        let space = BodyActionSpace::from_template(&BodyTemplate::official().unwrap()).unwrap();
        let codec = duck_body::BodyActionCodec::new(space);
        let first = walking_targets(
            &action,
            &codec,
            &DEFAULT_POSITION,
            None,
            &ranges,
            PolicyConfig::default(),
        );
        assert_eq!(
            first[0],
            DEFAULT_POSITION[0] + WALK_ACTION_SCALE,
            "the first target is scaled but not filtered"
        );
        assert_eq!(first[MOUTH_INDEX], DEFAULT_POSITION[MOUTH_INDEX]);

        let previous = DEFAULT_POSITION;
        let second = walking_targets(
            &action,
            &codec,
            &DEFAULT_POSITION,
            Some(&previous),
            &ranges,
            PolicyConfig::default(),
        );
        let expected_leg = DEFAULT_POSITION[0] + LEGS_LOWPASS * WALK_ACTION_SCALE;
        let expected_head = DEFAULT_POSITION[5] + HEAD_LOWPASS * WALK_ACTION_SCALE;
        assert_eq!(second[0], expected_leg);
        assert_eq!(second[5], expected_head);
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

    #[test]
    fn reference_trace_at_fixed_sample_points() {
        let fixture: ReferenceTraceFixture = serde_json::from_str(include_str!(
            "../tests/fixtures/official_reference_trace.json"
        ))
        .unwrap();
        let profile = BodyProfile::official().unwrap();
        let identity = profile.identity();
        assert_eq!(fixture.body_id, identity.id);
        assert_eq!(fixture.body_version, identity.version);
        assert_eq!(fixture.obs_len, 61);
        assert_eq!(fixture.action_len, ACTION_LEN);
        assert_eq!(fixture.gain, 200);
        assert_eq!(fixture.physical_ids, (1..=15).collect::<Vec<_>>());
        assert_eq!(fixture.directions, vec![1; 15]);
        assert_eq!(fixture.zero_offsets, vec![0.0; 15]);

        let home: [f64; NUM_JOINTS] = fixture.home_pose.clone().try_into().unwrap();
        let raw: [f32; ACTION_LEN] = fixture.raw_action.clone().try_into().unwrap();
        let encoder = super::PolicyInputEncoder;
        let observation = encoder.encode(
            &ImuData::default(),
            &home,
            &[0.0; NUM_JOINTS],
            &home,
            &raw,
            &Command::default(),
        );
        assert_eq!(observation.as_slice().len(), fixture.obs_len);
        assert_eq!(
            &observation.as_slice()[..fixture.expected_observation_prefix.len()],
            fixture.expected_observation_prefix.as_slice()
        );
        assert_eq!(
            &observation.as_slice()[34..48],
            fixture.expected_last_action
        );
        assert!(
            observation.as_slice()[6..34]
                .iter()
                .all(|value| *value == 0.0)
        );

        let codec = profile.action_codec();
        let ranges = vec![[-10.0, 10.0]; NUM_JOINTS];
        let target = walking_targets(
            &raw,
            &codec,
            &home,
            Some(&home),
            &ranges,
            PolicyConfig {
                action_scale: fixture.action_scale,
                head_lowpass: Some(fixture.head_lowpass),
                legs_lowpass: Some(fixture.legs_lowpass),
                ..PolicyConfig::default()
            },
        );
        let (raw_target, _) = walking_target_trace(
            &raw,
            &codec,
            &home,
            Some(&home),
            &ranges,
            PolicyConfig {
                action_scale: fixture.action_scale,
                head_lowpass: Some(fixture.head_lowpass),
                legs_lowpass: Some(fixture.legs_lowpass),
                ..PolicyConfig::default()
            },
        );
        for (actual, expected) in raw_target.iter().zip(fixture.expected_raw_target) {
            assert!((actual - expected).abs() < 1e-8);
        }
        for (actual, expected) in target.iter().zip(fixture.expected_canonical_target) {
            assert!(
                (actual - expected).abs() < 1e-8,
                "actual={actual} expected={expected}"
            );
        }

        let clipped_ranges = vec![[-0.01, 0.01]; NUM_JOINTS];
        let (raw_clipped, applied_clipped) = walking_target_trace(
            &raw,
            &codec,
            &home,
            None,
            &clipped_ranges,
            PolicyConfig {
                action_scale: fixture.action_scale,
                head_lowpass: None,
                legs_lowpass: None,
                ..PolicyConfig::default()
            },
        );
        assert!((raw_clipped[0] - 0.09).abs() < 1e-8);
        assert!((applied_clipped[0] - 0.01).abs() < 1e-8);
        assert!((raw_clipped[0] - applied_clipped[0] - 0.08).abs() < 1e-8);
    }
}
