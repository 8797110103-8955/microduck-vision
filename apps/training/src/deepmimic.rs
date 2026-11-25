//! DeepMimic conversion requests, persisted jobs and portable motion manifests.

use std::collections::BTreeMap;

use serde::{Deserialize, Serialize};
use serde_json::Value;

use crate::artifacts::digest_is_valid;

pub const METHOD: &str = "deepmimic";
pub const TASK_ID: &str = "Microduck-DeepMimic-Velstand-Flat-v1";
pub const SCHEMA_VERSION: &str = "microduck-deepmimic-motion-v1";

#[derive(Debug, Clone, Serialize, Deserialize, specta::Type)]
#[serde(rename_all = "camelCase")]
pub struct TrainingOptions {
    pub bundle_id: String,
    pub seed: u32,
    pub num_envs: u32,
    pub iterations: u32,
    #[serde(default)]
    pub baseline: bool,
}

impl TrainingOptions {
    pub fn validate(&self) -> Result<(), String> {
        if !valid_id(&self.bundle_id)
            || self.num_envs == 0
            || self.iterations == 0
            || self.seed > i32::MAX as u32
        {
            return Err(
                "DeepMimic requires a published bundle and positive environment/iteration counts"
                    .into(),
            );
        }
        Ok(())
    }

    pub fn method(&self) -> &'static str {
        if self.baseline {
            "ppo-baseline"
        } else {
            METHOD
        }
    }

    pub fn arguments(
        &self,
        bundle: &std::path::Path,
        manifest_sha256: &str,
    ) -> Result<Vec<String>, String> {
        self.validate()?;
        if !digest_is_valid(manifest_sha256) {
            return Err("Invalid DeepMimic manifest hash".into());
        }
        let mut args = vec![
            "run".into(),
            "train".into(),
            TASK_ID.into(),
            "--deepmimic-bundle".into(),
            bundle.to_string_lossy().into_owned(),
            "--deepmimic-manifest-sha256".into(),
            manifest_sha256.into(),
            "--seed".into(),
            self.seed.to_string(),
            "--num-envs".into(),
            self.num_envs.to_string(),
            "--iterations".into(),
            self.iterations.to_string(),
            "--domain-randomization".into(),
            "off".into(),
        ];
        if self.baseline {
            args.push("--deepmimic-baseline".into());
        }
        Ok(args)
    }
}

#[derive(Debug, Clone, Serialize, Deserialize, specta::Type)]
#[serde(rename_all = "camelCase")]
pub struct TrainingProvenance {
    pub training_method: String,
    pub bundle_id: String,
    pub bundle_manifest_sha256: String,
    pub clip_sha256: String,
    pub source_recording_id: String,
    pub seed: u32,
    pub num_envs: u32,
    pub iterations: u32,
    pub metadata_file: String,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, specta::Type)]
#[serde(rename_all = "lowercase")]
pub enum Playback {
    Finite,
    Cyclic,
}

#[derive(Debug, Clone, Serialize, Deserialize, specta::Type)]
pub struct ConversionRequest {
    pub recording_id: String,
    pub interval_id: String,
    pub start_s: f64,
    pub end_s: f64,
    pub name: String,
    pub playback: Playback,
}

impl ConversionRequest {
    pub fn validate(&self) -> Result<(), String> {
        if !valid_id(&self.recording_id)
            || !valid_id(&self.interval_id)
            || self.name.trim().is_empty()
            || !self.start_s.is_finite()
            || !self.end_s.is_finite()
            || self.start_s < 0.0
            || self.end_s <= self.start_s
        {
            return Err("Invalid DeepMimic conversion request".into());
        }
        Ok(())
    }
}

#[derive(Debug, Clone, Serialize, Deserialize, specta::Type)]
pub struct IntervalSummary {
    pub id: String,
    pub body_id: String,
    pub model_kind: String,
    pub control_s: f64,
    pub segment_id: u32,
    #[specta(type = f64)]
    pub execution_generation: u64,
    pub first_frame: u32,
    pub last_frame: u32,
    pub first_simulation_s: f64,
    pub last_simulation_s: f64,
    pub frames: u32,
    pub excluded_reasons: BTreeMap<String, u32>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, specta::Type)]
#[serde(rename_all = "lowercase")]
pub enum JobStatus {
    Queued,
    Running,
    Succeeded,
    Failed,
    Cancelled,
    Interrupted,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, specta::Type)]
#[serde(rename_all = "lowercase")]
pub enum ConversionStage {
    Reading,
    Validating,
    Converting,
    Saving,
}

#[derive(Debug, Clone, Serialize, Deserialize, specta::Type)]
pub struct ConversionJob {
    pub id: String,
    pub bundle_id: String,
    pub request: ConversionRequest,
    pub status: JobStatus,
    pub stage: ConversionStage,
    pub processed_frames: u32,
    pub total_frames: Option<u32>,
    pub error: Option<String>,
}

#[derive(Debug, Clone, Serialize, Deserialize, specta::Type)]
pub struct BundleSummary {
    pub id: String,
    pub name: String,
    pub recording_id: String,
    pub body_id: String,
    pub control_s: f64,
    pub frames: u32,
    pub duration_s: f64,
    pub playback: Playback,
}

#[derive(Debug, Clone, Serialize, Deserialize, specta::Type)]
pub struct ReadFailure {
    #[specta(type = f64)]
    pub bytes_read: u64,
    pub last_reliable_frame: Option<u32>,
    pub reason: String,
}

#[derive(Debug, Clone, Serialize, Deserialize, specta::Type)]
pub struct ScanReport {
    pub recording_id: String,
    pub mcap_sha256: String,
    pub intervals: Vec<IntervalSummary>,
    pub frames_read: u32,
    pub excluded_reasons: BTreeMap<String, u32>,
    pub read_failure: Option<ReadFailure>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MotionManifest {
    pub training_method: String,
    pub schema_version: String,
    pub bundle_id: String,
    pub name: String,
    pub created_at: String,
    pub converter_version: String,
    pub source: MotionSource,
    pub body: MotionBody,
    pub timing: MotionTiming,
    pub joint_names: Vec<String>,
    pub feet: [FootSite; 2],
    pub clip: MotionClip,
    pub processing: MotionProcessing,
    pub quality: MotionQuality,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MotionSource {
    pub recording_id: String,
    pub segment_id: u32,
    pub interval_id: String,
    pub first_frame: u32,
    pub last_frame: u32,
    pub first_simulation_s: f64,
    pub last_simulation_s: f64,
    pub mcap_sha256: String,
    pub onnx_sha256: String,
    pub execution_profile: Value,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MotionBody {
    pub id: String,
    pub version: u32,
    pub source_hash: String,
    pub model_kind: String,
    pub robot_xml: String,
    pub robot_sha256: String,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MotionTiming {
    pub control_s: f64,
    pub physics_s: f64,
    pub units: BTreeMap<String, String>,
    pub world_basis: String,
    pub quaternion_order: String,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FootSite {
    pub site: String,
    pub body: String,
    pub local_position: [f64; 3],
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MotionClip {
    pub file: String,
    pub sha256: String,
    pub frames: u32,
    pub duration_s: f64,
    pub playback: Playback,
    pub arrays: BTreeMap<String, ArrayLayout>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ArrayLayout {
    pub dtype: String,
    pub shape: Vec<u32>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MotionProcessing {
    pub velocity_method: String,
    pub angular_velocity_method: String,
    pub state_timebase: String,
    pub teacher_semantics: String,
    pub teacher_command_encoding: String,
    pub tolerances: BTreeMap<String, f64>,
    pub cyclic_thresholds: BTreeMap<String, f64>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MotionQuality {
    pub excluded_frames: u32,
    pub target_range_exceeded_frames: u32,
    pub contact_available: bool,
    pub applied_targets_available: bool,
    pub fk_max_error: BTreeMap<String, f64>,
    pub seam_errors: BTreeMap<String, f64>,
    pub warnings: Vec<String>,
}

pub fn valid_id(value: &str) -> bool {
    !value.is_empty()
        && value != "."
        && value != ".."
        && value
            .bytes()
            .all(|b| b.is_ascii_alphanumeric() || b"-_.".contains(&b))
}

pub fn valid_bundle_path(value: &str) -> bool {
    !value.is_empty() && value.split('/').all(valid_id) && !value.contains(['\\', ':'])
}

pub fn array_layouts(frames: u32, applied_targets: bool) -> BTreeMap<String, ArrayLayout> {
    let mut shapes = BTreeMap::from([("time_s", vec![frames])]);
    for name in [
        "motor_qpos",
        "passive_qpos",
        "output_qpos",
        "motor_qvel",
        "passive_qvel",
        "output_qvel",
        "teacher_action",
    ] {
        shapes.insert(name, vec![frames, 14]);
    }
    for name in ["root_pos", "root_lin_vel_world", "root_ang_vel_world"] {
        shapes.insert(name, vec![frames, 3]);
    }
    shapes.insert("root_quat_wxyz", vec![frames, 4]);
    shapes.insert("foot_pos_world", vec![frames, 2, 3]);
    shapes.insert("teacher_observation", vec![frames, 61]);
    shapes.insert("teacher_command", vec![frames, 13]);
    if applied_targets {
        shapes.insert("teacher_applied_targets", vec![frames, 14]);
    }
    shapes
        .into_iter()
        .map(|(name, shape)| {
            (
                name.into(),
                ArrayLayout {
                    dtype: "float64".into(),
                    shape,
                },
            )
        })
        .collect()
}

impl MotionManifest {
    pub fn validate(&self) -> Result<(), String> {
        let source = &self.source;
        let duration = source.last_simulation_s - source.first_simulation_s;
        if self.training_method != METHOD || self.schema_version != SCHEMA_VERSION {
            return Err("Unsupported DeepMimic motion method or schema version".into());
        }
        if !valid_id(&self.bundle_id)
            || !valid_id(&source.recording_id)
            || !valid_id(&source.interval_id)
            || self.name.trim().is_empty()
            || self.created_at.is_empty()
            || self.converter_version.is_empty()
            || !digest_is_valid(&source.mcap_sha256)
            || !digest_is_valid(&source.onnx_sha256)
            || !digest_is_valid(&self.clip.sha256)
            || !digest_is_valid(&self.body.robot_sha256)
            || !source.execution_profile.is_object()
        {
            return Err("Invalid DeepMimic identity or source provenance".into());
        }
        if self.body.id != "official"
            || self.body.version == 0
            || self.body.source_hash.is_empty()
            || self.body.model_kind != "xl330"
            || self.body.robot_xml != "robot_groundcontact_backlash.xml"
            || self.timing.control_s != 0.02
            || self.timing.physics_s != 0.005
            || self.timing.world_basis != "right-handed-z-up"
            || self.timing.quaternion_order != "wxyz"
            || self.timing.units
                != BTreeMap::from([
                    ("angle".into(), "rad".into()),
                    ("length".into(), "m".into()),
                    ("time".into(), "s".into()),
                ])
        {
            return Err("Unsupported DeepMimic body, timing or units".into());
        }
        let profile = &source.execution_profile;
        let source_joints = profile["joint_order"]
            .as_array()
            .ok_or("Missing source joint order")?;
        let selected_joints: Vec<_> = source_joints
            .iter()
            .filter_map(Value::as_str)
            .filter(|name| *name != "mouth")
            .collect();
        if selected_joints != self.joint_names
            || profile["format_version"] != 1
            || profile["execution_kind"] != "simulation"
            || profile["model_kind"] != self.body.model_kind
            || profile["body"]["id"] != self.body.id
            || profile["body"]["version"] != self.body.version
            || profile["body"]["source_hash"] != self.body.source_hash
            || profile["periods"]["control_s"].as_f64() != Some(self.timing.control_s)
            || profile["periods"]["physics_s"].as_f64() != Some(self.timing.physics_s)
            || profile["simulation"]["scenario"]["robot_xml"] != self.body.robot_xml
            || profile["simulation"]["response"]["encoder_through_backlash"] != true
            || profile["execution_fingerprint"]
                .as_str()
                .is_none_or(|value| !digest_is_valid(value))
            || !["motor", "nominal_motor", "response", "scenario"]
                .iter()
                .all(|key| profile["simulation"][key].is_object())
            || profile["simulation"]["source_assets"]
                .as_array()
                .is_none_or(|assets| {
                    !assets.iter().any(|asset| {
                        asset["path"] == self.body.robot_xml
                            && asset["sha256"] == self.body.robot_sha256
                    })
                })
        {
            return Err("DeepMimic source profile disagrees with motion metadata".into());
        }
        let names: std::collections::BTreeSet<_> = self.joint_names.iter().collect();
        if self.joint_names.len() != 14
            || names.len() != 14
            || self
                .joint_names
                .iter()
                .any(|n| n.is_empty() || n == "mouth")
            || self.feet[0].site != "left_foot"
            || self.feet[1].site != "right_foot"
            || self
                .feet
                .iter()
                .any(|f| f.body.is_empty() || f.local_position.iter().any(|v| !v.is_finite()))
        {
            return Err("Invalid DeepMimic joint or foot layout".into());
        }
        if !valid_bundle_path(&self.clip.file)
            || !self.clip.file.ends_with(".npz")
            || self.clip.frames < 3
            || source.last_frame < source.first_frame
            || source.last_frame - source.first_frame + 1 != self.clip.frames
            || !source.first_simulation_s.is_finite()
            || source.first_simulation_s < 0.0
            || !duration.is_finite()
            || duration <= 0.0
            || !self.clip.duration_s.is_finite()
            || (duration - self.clip.duration_s).abs() > 1e-8
            || (duration - f64::from(self.clip.frames - 1) * self.timing.control_s).abs() > 1e-8
            || self.clip.arrays
                != array_layouts(self.clip.frames, self.quality.applied_targets_available)
        {
            return Err("Invalid DeepMimic clip path, timing or array layout".into());
        }
        if self.processing.state_timebase != "simulation.time"
            || self.processing.velocity_method != "central-difference-one-sided-endpoints"
            || self.processing.angular_velocity_method != "quaternion-log-world"
            || self.processing.teacher_semantics != "pre-step-policy-for-transition-to-state"
            || !["constant", "phase", "posture_flag"]
                .contains(&self.processing.teacher_command_encoding.as_str())
            || self.quality.contact_available
        {
            return Err("Unsupported DeepMimic processing semantics".into());
        }
        for values in [
            &self.processing.tolerances,
            &self.processing.cyclic_thresholds,
            &self.quality.fk_max_error,
            &self.quality.seam_errors,
        ] {
            if values
                .values()
                .any(|value| !value.is_finite() || *value < 0.0)
            {
                return Err("Invalid DeepMimic quality measurements".into());
            }
        }
        Ok(())
    }
}
