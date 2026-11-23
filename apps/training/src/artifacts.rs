//! Read training artifacts and resolve their model paths within each run.

use std::path::{Component, Path, PathBuf};

use serde::{Deserialize, de::DeserializeOwned};

pub const ID_PREFIX: &str = "training:";

#[derive(Deserialize)]
pub struct Artifact<Tensor, Contract> {
    format_version: u32,
    onnx_path: PathBuf,
    pub onnx_sha256: String,
    pub inputs: Vec<Tensor>,
    pub outputs: Vec<Tensor>,
    #[serde(flatten)]
    pub contract: Contract,
}

pub fn digest_is_valid(hash: &str) -> bool {
    hash.len() == 64
        && hash
            .bytes()
            .all(|b| b.is_ascii_hexdigit() && !b.is_ascii_uppercase())
}

pub fn read_artifact<Tensor: DeserializeOwned, Contract: DeserializeOwned>(
    run: &Path,
) -> Result<(Artifact<Tensor, Contract>, PathBuf), String> {
    let root = run.canonicalize().map_err(|error| error.to_string())?;
    let manifest: Artifact<Tensor, Contract> = serde_json::from_slice(
        &std::fs::read(root.join("artifact.json")).map_err(|error| error.to_string())?,
    )
    .map_err(|error| format!("训练产物清单无效：{error}"))?;
    if manifest.format_version != 1 || !digest_is_valid(&manifest.onnx_sha256) {
        return Err("训练产物的版本或 ONNX 摘要无效".into());
    }
    if manifest
        .onnx_path
        .extension()
        .is_none_or(|ext| ext != "onnx")
        || !manifest
            .onnx_path
            .components()
            .all(|component| matches!(component, Component::Normal(_)))
    {
        return Err("训练模型必须是任务目录中的相对 ONNX 路径".into());
    }
    let model = root
        .join(&manifest.onnx_path)
        .canonicalize()
        .map_err(|error| error.to_string())?;
    if !model.starts_with(&root) || !model.is_file() {
        return Err("训练模型路径超出任务目录".into());
    }
    Ok((manifest, model))
}
