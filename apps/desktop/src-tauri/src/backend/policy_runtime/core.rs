//! Deployment-independent observation, action decoding, and canonical target generation.
//!
//! `PolicyControlCore` deliberately knows nothing about serial transports, servo IDs, body-loop
//! lifecycle, or Tauri state. Every host or MCU adapter can feed it the same `BackendState` and
//! receive the same canonical target frame.

use duck_body::{BodyActionCodec, BodyProfile};
use duck_control::imu::ImuData;
use duck_control::model::{MOUTH_INDEX, NUM_JOINTS};
use duck_control::obs::{ACTION_LEN, Command, Observation};
use duck_control::policy::Net;

use loop_body::{BackendState, LoopError};

use super::{PolicyInputEncoder, RawPolicyOutputDecoder};

const WALK_ACTION_SCALE: f64 = 1.0;

#[derive(Debug, Clone, PartialEq)]
pub(crate) struct ControlTickInput {
    pub backend_state: BackendState,
    pub command: Command,
    pub raw_action: Vec<f32>,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PolicyConfig {
    pub net: Net,
    pub action_scale: f64,
    pub head_lowpass: Option<f64>,
    pub legs_lowpass: Option<f64>,
    /// Resolved contracts may require a physical/simulated IMU. When false, a missing sample is
    /// represented as the explicit nominal fallback and remains marked in the trace.
    pub imu_required: bool,
}

impl Default for PolicyConfig {
    fn default() -> Self {
        Self {
            net: Net::Walk,
            action_scale: WALK_ACTION_SCALE,
            head_lowpass: None,
            legs_lowpass: None,
            imu_required: false,
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct ControlTickOutput {
    pub imu: ImuData,
    pub imu_source: &'static str,
    pub observation: Vec<f32>,
    pub action: Vec<f32>,
    pub raw_targets: Vec<f64>,
    pub canonical_target: Vec<f64>,
    pub saturation_reasons: Vec<&'static str>,
}

/// History that belongs to the policy observation stream. It is kept across command changes and
/// can be restored when a stopped execution resumes with the same policy session.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct PolicyHistory {
    pub last_action: Vec<f32>,
    pub previous_targets: Option<Vec<f64>>,
}

pub struct PolicyControlCore {
    input_encoder: PolicyInputEncoder,
    output_decoder: RawPolicyOutputDecoder,
    action_codec: BodyActionCodec,
    home: [f64; NUM_JOINTS],
    ranges: Vec<[f64; 2]>,
    config: PolicyConfig,
    last_action: [f32; ACTION_LEN],
    previous_targets: Option<[f64; NUM_JOINTS]>,
}

impl PolicyControlCore {
    pub fn new(
        body_profile: &BodyProfile,
        ranges: Vec<[f64; 2]>,
        config: PolicyConfig,
    ) -> Result<Self, LoopError> {
        let home = body_profile
            .template()
            .control
            .home_pose
            .clone()
            .try_into()
            .map_err(|_| LoopError::SensorLength {
                actual: body_profile.template().control.home_pose.len(),
                expected: NUM_JOINTS,
            })?;
        if ranges.len() != NUM_JOINTS {
            return Err(LoopError::SensorLength {
                actual: ranges.len(),
                expected: NUM_JOINTS,
            });
        }
        Ok(Self {
            input_encoder: PolicyInputEncoder,
            output_decoder: RawPolicyOutputDecoder,
            action_codec: body_profile.action_codec(),
            home,
            ranges,
            config,
            last_action: [0.0; ACTION_LEN],
            previous_targets: None,
        })
    }

    pub fn config(&self) -> PolicyConfig {
        self.config
    }

    pub fn set_config(&mut self, config: PolicyConfig) -> Result<(), LoopError> {
        validate_config(config)?;
        self.config = config;
        self.reset();
        Ok(())
    }

    pub fn reset(&mut self) {
        self.last_action = [0.0; ACTION_LEN];
        self.previous_targets = None;
    }

    pub(crate) fn history(&self) -> PolicyHistory {
        PolicyHistory {
            last_action: self.last_action.to_vec(),
            previous_targets: self.previous_targets.map(|targets| targets.to_vec()),
        }
    }

    pub(crate) fn restore_history(
        &mut self,
        history: &PolicyHistory,
    ) -> Result<(), LoopError> {
        self.last_action = history
            .last_action
            .as_slice()
            .try_into()
            .map_err(|_| LoopError::SensorLength {
                actual: history.last_action.len(),
                expected: ACTION_LEN,
            })?;
        if self.last_action.iter().any(|value| !value.is_finite()) {
            return Err(LoopError::InvalidBackendState {
                field: "policy last action",
            });
        }
        self.previous_targets = history
            .previous_targets
            .as_deref()
            .map(TryInto::try_into)
            .transpose()
            .map_err(|_| LoopError::SensorLength {
                actual: history
                    .previous_targets
                    .as_ref()
                    .map_or(0, Vec::len),
                expected: NUM_JOINTS,
            })?;
        if self
            .previous_targets
            .as_ref()
            .is_some_and(|targets| targets.iter().any(|value| !value.is_finite()))
        {
            return Err(LoopError::InvalidBackendState {
                field: "policy previous targets",
            });
        }
        Ok(())
    }

    pub fn encode_observation(
        &self,
        state: &BackendState,
        command: &Command,
    ) -> Result<(Observation, ImuData, &'static str), LoopError> {
        if state.valid_positions.len() != state.positions.len()
            || state.valid_velocities.len() != state.velocities.len()
            || state.positions.iter().any(|value| !value.is_finite())
            || state.velocities.iter().any(|value| !value.is_finite())
        {
            return Err(LoopError::InvalidBackendState {
                field: "joint state",
            });
        }
        // The canonical display mouth has no policy action or sensor. All driven policy
        // slots require real feedback; its explicitly unactuated placeholder is allowed.
        if state
            .valid_positions
            .iter()
            .enumerate()
            .any(|(i, valid)| i != duck_control::model::MOUTH_INDEX && !valid)
            || state
                .valid_velocities
                .iter()
                .enumerate()
                .any(|(i, valid)| i != duck_control::model::MOUTH_INDEX && !valid)
        {
            return Err(LoopError::InvalidBackendState {
                field: "invalid joint slot",
            });
        }
        if self.config.imu_required && state.imu.is_none() {
            return Err(LoopError::MissingImu);
        }
        let positions: [f64; NUM_JOINTS] =
            state
                .positions
                .clone()
                .try_into()
                .map_err(|_| LoopError::SensorLength {
                    actual: state.positions.len(),
                    expected: NUM_JOINTS,
                })?;
        let velocities: [f64; NUM_JOINTS] =
            state
                .velocities
                .clone()
                .try_into()
                .map_err(|_| LoopError::SensorLength {
                    actual: state.velocities.len(),
                    expected: NUM_JOINTS,
                })?;
        let (imu, imu_source) = state
            .imu
            .map(|sample| {
                (
                    ImuData {
                        gyro: sample.gyro,
                        gravity: sample.gravity,
                        ..ImuData::default()
                    },
                    "backend",
                )
            })
            .unwrap_or_else(|| (ImuData::default(), "default"));
        let observation = self.input_encoder.encode(
            &imu,
            &positions,
            &velocities,
            &self.home,
            &self.last_action,
            command,
        );
        Ok((observation, imu, imu_source))
    }

    pub fn control_tick(
        &mut self,
        input: &ControlTickInput,
    ) -> Result<ControlTickOutput, LoopError> {
        let state = &input.backend_state;
        let command = &input.command;
        let raw_action = &input.raw_action;
        let (observation, imu, imu_source) = self.encode_observation(state, command)?;
        let positions: [f64; NUM_JOINTS] =
            state
                .positions
                .clone()
                .try_into()
                .map_err(|_| LoopError::SensorLength {
                    actual: state.positions.len(),
                    expected: NUM_JOINTS,
                })?;
        let action = self.output_decoder.decode(raw_action)?;
        let previous = self.previous_targets.as_ref().unwrap_or(&positions);
        let (raw_target_array, target_array) = walking_target_trace(
            &action,
            &self.action_codec,
            &self.home,
            Some(previous),
            self.config,
        );
        self.last_action = action;
        self.previous_targets = Some(target_array);
        let mut saturation_reasons = Vec::new();
        for ((raw, target), range) in raw_target_array
            .iter()
            .zip(target_array.iter())
            .zip(self.ranges.iter())
        {
            if *raw < range[0] || *raw > range[1] {
                saturation_reasons.push("body_range_exceeded");
            }
            if (raw - target).abs() > f64::EPSILON {
                saturation_reasons.push("target_filter");
            }
        }
        Ok(ControlTickOutput {
            imu,
            imu_source,
            observation: observation.as_slice().to_vec(),
            action: action.to_vec(),
            raw_targets: raw_target_array.to_vec(),
            canonical_target: target_array.to_vec(),
            saturation_reasons,
        })
    }
}

fn validate_config(config: PolicyConfig) -> Result<(), LoopError> {
    if !config.action_scale.is_finite()
        || config.action_scale <= 0.0
        || [config.head_lowpass, config.legs_lowpass]
            .into_iter()
            .flatten()
            .any(|alpha| !alpha.is_finite() || !(0.0..=1.0).contains(&alpha))
        || matches!(config.net, Net::Skill(index) if index != 0)
    {
        return Err(LoopError::Controller("invalid policy configuration".into()));
    }
    Ok(())
}

pub(crate) fn walking_target_trace(
    action: &[f32; ACTION_LEN],
    codec: &BodyActionCodec,
    home: &[f64; NUM_JOINTS],
    previous: Option<&[f64; NUM_JOINTS]>,
    config: PolicyConfig,
) -> ([f64; NUM_JOINTS], [f64; NUM_JOINTS]) {
    let offsets = codec
        .decode(action)
        .expect("official action layout was validated when the loop opened");
    let raw = std::array::from_fn(|joint| home[joint] + config.action_scale * offsets[joint]);
    // Joint travel limits are enforced by BodyLoop's explicit execution configuration.
    // Clipping here changes the policy controller even when execution limits are disabled.
    let mut targets = raw;
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
mod tests {
    use super::{ControlTickInput, PolicyConfig, PolicyControlCore};
    use duck_body::{BodyProfile, BodyTemplate};
    use duck_control::obs::Command;
    use loop_body::BackendState;

    #[test]
    fn same_backend_frame_and_action_produce_the_same_canonical_target() {
        let profile = BodyProfile::from_template(BodyTemplate::official().unwrap()).unwrap();
        let ranges: Vec<[f64; 2]> = profile
            .template()
            .joints
            .iter()
            .map(|joint| joint.range_rad)
            .collect();
        let mut first =
            PolicyControlCore::new(&profile, ranges.clone(), PolicyConfig::default()).unwrap();
        let mut second = PolicyControlCore::new(&profile, ranges, PolicyConfig::default()).unwrap();
        let state = BackendState {
            positions: profile.template().control.home_pose.clone(),
            velocities: vec![0.0; 15],
            imu: None,
            sequence: 1,
            sampled_at: loop_body::SampleTime(1),
            valid_positions: vec![true; 15],
            valid_velocities: vec![true; 15],
            source: Some("test".into()),
        };
        let action = vec![0.1; 14];
        let input = ControlTickInput {
            backend_state: state,
            command: Command::default(),
            raw_action: action,
        };
        let first = first.control_tick(&input).unwrap();
        let second = second.control_tick(&input).unwrap();
        assert_eq!(first.observation, second.observation);
        assert_eq!(first.action, second.action);
        assert_eq!(first.canonical_target, second.canonical_target);
    }

    #[test]
    fn policy_offsets_beyond_body_ranges_keep_the_requested_canonical_target() {
        let profile = BodyProfile::from_template(BodyTemplate::official().unwrap()).unwrap();
        let ranges: Vec<[f64; 2]> = profile
            .template()
            .joints
            .iter()
            .map(|joint| joint.range_rad)
            .collect();
        let mut core = PolicyControlCore::new(&profile, ranges, PolicyConfig::default()).unwrap();
        let output = core
            .control_tick(&ControlTickInput {
                backend_state: BackendState {
                    positions: profile.template().control.home_pose.clone(),
                    velocities: vec![0.0; 15],
                    imu: None,
                    sequence: 1,
                    sampled_at: loop_body::SampleTime(1),
                    valid_positions: vec![true; 15],
                    valid_velocities: vec![true; 15],
                    source: Some("test".into()),
                },
                command: Command::default(),
                raw_action: vec![2.0; 14],
            })
            .unwrap();
        assert!(output.saturation_reasons.contains(&"body_range_exceeded"));
        assert_eq!(output.canonical_target, output.raw_targets);
        assert!(!output.saturation_reasons.contains(&"body_range_limit"));
    }

    #[test]
    fn policy_history_restores_previous_action_and_targets() {
        let profile = BodyProfile::from_template(BodyTemplate::official().unwrap()).unwrap();
        let ranges: Vec<[f64; 2]> = profile
            .template()
            .joints
            .iter()
            .map(|joint| joint.range_rad)
            .collect();
        let state = BackendState {
            positions: profile.template().control.home_pose.clone(),
            velocities: vec![0.0; 15],
            imu: None,
            sequence: 1,
            sampled_at: loop_body::SampleTime(1),
            valid_positions: vec![true; 15],
            valid_velocities: vec![true; 15],
            source: Some("test".into()),
        };
        let mut first = PolicyControlCore::new(&profile, ranges.clone(), PolicyConfig::default())
            .unwrap();
        let input = ControlTickInput {
            backend_state: state.clone(),
            command: Command::default(),
            raw_action: vec![0.25; 14],
        };
        let expected = first.control_tick(&input).unwrap();
        let history = first.history();

        let mut resumed = PolicyControlCore::new(&profile, ranges, PolicyConfig::default()).unwrap();
        resumed.restore_history(&history).unwrap();
        let (observation, _, _) = resumed
            .encode_observation(&state, &Command::default())
            .unwrap();
        assert_eq!(&observation.as_slice()[34..48], &[0.25; 14]);
        let actual = resumed.control_tick(&input).unwrap();
        assert_eq!(actual.canonical_target, expected.canonical_target);
    }
}
