//! Shared manifest fields and local ONNX file resolution.

use std::{
    fs, io,
    path::{Path, PathBuf},
};

use serde::{Deserialize, Serialize, Serializer, de::DeserializeOwned};

use crate::types::PolicyCommand;

pub(crate) const MANIFEST_FILE: &str = "manifest.json";

#[derive(Deserialize, Serialize)]
pub(crate) struct PolicyManifest<Entry> {
    #[serde(flatten)]
    pub defaults: ManifestDefaults,
    pub policies: Vec<Entry>,
}

#[derive(Default, Deserialize, Serialize)]
pub(crate) struct ManifestDefaults {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub schema_version: Option<u32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub model_api: Option<u32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub obs_len: Option<usize>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub action_len: Option<usize>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub robot: Option<ManifestRobot>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub body_id: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub body_version: Option<u32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub control_hz: Option<u32>,
}

#[derive(Deserialize, Serialize)]
pub(crate) struct ManifestRobot {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub model: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub hw_rev: Option<u32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub servos: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub control_hz: Option<u32>,
}

#[derive(Default, Deserialize, Serialize)]
pub(crate) struct ManifestPolicy {
    pub file: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub name: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub kind: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub slot: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub entry_pose: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub duration_s: Option<f64>,
    #[serde(
        skip_serializing_if = "Option::is_none",
        serialize_with = "serialize_command"
    )]
    pub command: Option<PolicyCommand>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub action_scale: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub obs_len: Option<usize>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub action_len: Option<usize>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub body_id: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub body_version: Option<u32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub control_hz: Option<u32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub command_encoding: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub recurrent: Option<bool>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub head_lowpass: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub legs_lowpass: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub joint_velocity_delay_ticks: Option<u32>,
    #[serde(default, skip_serializing_if = "std::ops::Not::not")]
    pub imu_required: bool,
}

impl ManifestPolicy {
    pub(crate) fn apply_defaults(&mut self, defaults: &ManifestDefaults) {
        self.obs_len = self.obs_len.or(defaults.obs_len);
        self.action_len = self.action_len.or(defaults.action_len);
        self.body_id = self.body_id.take().or_else(|| defaults.body_id.clone());
        self.body_version = self.body_version.or(defaults.body_version);
        self.control_hz = self
            .control_hz
            .or(defaults.control_hz)
            .or_else(|| defaults.robot.as_ref().and_then(|robot| robot.control_hz));
        if self.command_encoding.is_none() {
            self.command_encoding = self
                .command
                .as_ref()
                .and_then(|command| command.encoding.clone());
        }
    }
}

fn serialize_command<S: Serializer>(
    command: &Option<PolicyCommand>,
    serializer: S,
) -> Result<S::Ok, S::Error> {
    let mut fields = serde_json::to_value(command).map_err(serde::ser::Error::custom)?;
    if let Some(fields) = fields.as_object_mut() {
        fields.retain(|_, value| !value.is_null());
        for (camel, snake) in [("periodS", "period_s"), ("endPhase", "end_phase")] {
            if let Some(value) = fields.remove(camel) {
                fields.insert(snake.into(), value);
            }
        }
    }
    fields.serialize(serializer)
}

pub(crate) fn read_manifest<Entry: DeserializeOwned>(
    path: &Path,
) -> Result<Option<PolicyManifest<Entry>>, String> {
    let bytes = match fs::read(path) {
        Ok(bytes) => bytes,
        Err(error) if error.kind() == io::ErrorKind::NotFound => return Ok(None),
        Err(error) => return Err(format!("读取策略清单失败（{}）：{error}", path.display())),
    };
    serde_json::from_slice(&bytes)
        .map(Some)
        .map_err(|error| format!("解析策略清单失败（{}）：{error}", path.display()))
}

pub(crate) fn is_onnx_filename(file: &str) -> bool {
    !file.starts_with('.')
        && !file.chars().any(|character| {
            character.is_control()
                || matches!(
                    character,
                    '<' | '>' | ':' | '"' | '/' | '\\' | '|' | '?' | '*'
                )
        })
        && Path::new(file)
            .extension()
            .is_some_and(|extension| extension.eq_ignore_ascii_case("onnx"))
}

pub(crate) fn model_file(directory: &Path, file: &str) -> Result<(PathBuf, u64), String> {
    if !is_onnx_filename(file) {
        return Err("策略清单须使用 ONNX 文件名".into());
    }
    let directory = directory
        .canonicalize()
        .map_err(|error| error.to_string())?;
    let path = directory
        .join(file)
        .canonicalize()
        .map_err(|error| format!("读取 ONNX 文件 {file} 失败：{error}"))?;
    if path.parent() != Some(directory.as_path()) {
        return Err(format!("ONNX 模型文件超出策略目录：{file}"));
    }
    let metadata = path.metadata().map_err(|error| error.to_string())?;
    if !metadata.is_file() || metadata.len() == 0 {
        return Err(format!("ONNX 模型文件无效：{file}"));
    }
    Ok((path, metadata.len()))
}
