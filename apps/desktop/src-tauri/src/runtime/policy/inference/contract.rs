//! Policy artifact validation for the desktop runtime.

use std::time::Duration;

use duck_body::BodyProfile;
use duck_control::model::{JOINT_NAMES, MOUTH_INDEX};
use duck_control::obs::{ACTION_LEN, OBS_LEN};
use loop_body::LoopError;

use super::action_space::BodyActionSpace;

pub(super) fn official_layout(profile: &BodyProfile) -> Result<BodyActionSpace, LoopError> {
    let space = BodyActionSpace::official(profile.template())?;
    let expected_observations = (0..JOINT_NAMES.len()).filter(|&index| index != MOUTH_INDEX);
    // The vendor observation encoder receives arrays in its canonical order.
    if !space
        .canonical_slots()
        .iter()
        .map(String::as_str)
        .eq(JOINT_NAMES)
        || !space
            .observation_slots()
            .iter()
            .copied()
            .eq(expected_observations)
    {
        return Err(LoopError::Controller(
            "body joint order does not match the official policy observation layout".to_owned(),
        ));
    }
    Ok(space)
}

/// Static policy artifact contract checked before an ONNX session enters the tick loop.
#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
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
        if self.obs_len != OBS_LEN || self.action_len != ACTION_LEN {
            return Err(LoopError::Controller(format!(
                "policy {} layout is obs={} action={}, expected obs={OBS_LEN} action={ACTION_LEN}",
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
