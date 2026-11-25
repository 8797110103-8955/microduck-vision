//! Resolve published references before preparing the normal PPO process.

use std::path::PathBuf;

use microduck_training::deepmimic::{MotionManifest, TASK_ID, TrainingOptions, TrainingProvenance};
use sha2::{Digest, Sha256};

use crate::runtime::session::state::DesktopState;
use crate::types::{ModelKindKey, TrainingStartInput};

pub(super) struct Input {
    pub options: TrainingOptions,
    pub directory: PathBuf,
    pub manifest: MotionManifest,
    pub manifest_sha256: String,
}

impl Input {
    pub(super) fn arguments(&self) -> Result<Vec<String>, String> {
        self.options
            .arguments(&self.directory, &self.manifest_sha256)
    }

    pub(super) fn provenance(&self) -> TrainingProvenance {
        TrainingProvenance {
            training_method: self.options.method().into(),
            bundle_id: self.options.bundle_id.clone(),
            bundle_manifest_sha256: self.manifest_sha256.clone(),
            clip_sha256: self.manifest.clip.sha256.clone(),
            source_recording_id: self.manifest.source.recording_id.clone(),
            seed: self.options.seed,
            num_envs: self.options.num_envs,
            iterations: self.options.iterations,
            metadata_file: "training-metadata.json".into(),
        }
    }
}

pub(super) fn resolve(
    state: &DesktopState,
    input: &TrainingStartInput,
) -> Result<Option<Input>, String> {
    let Some(options) = &input.deepmimic else {
        if input.args.get(2).is_some_and(|task| task == TASK_ID) {
            return Err("DeepMimic 训练必须选择已发布的动作包".into());
        }
        return Ok(None);
    };
    options.validate()?;
    if input.dataset_path.is_some()
        || input
            .policy_id
            .as_deref()
            .is_some_and(|id| !id.trim().is_empty())
        || !input.args.is_empty()
    {
        return Err(
            "DeepMimic 训练使用独立动作包与参数，不能同时指定 BC 数据集、参考策略或自定义命令参数"
                .into(),
        );
    }
    let machine = crate::runtime::machines::machine_resolution::virtual_machine(
        state,
        &input.virtual_machine_id,
    )?;
    let body = crate::runtime::machines::machine_resolution::body_profile(&machine.body_id)?;
    let directory = state.deepmimic_jobs.bundle_path(&options.bundle_id)?;
    let bytes =
        std::fs::read(directory.join("manifest.json")).map_err(|error| error.to_string())?;
    let manifest: MotionManifest =
        serde_json::from_slice(&bytes).map_err(|error| error.to_string())?;
    manifest.validate()?;
    let identity = body.identity();
    if machine.model_kind != ModelKindKey::Xl330
        || manifest.body.id != identity.id
        || manifest.body.version != identity.version
        || manifest.body.source_hash != identity.source_hash
        || manifest.body.model_kind != "xl330"
    {
        return Err("首轮 DeepMimic 动作包需要匹配的 official XL330 模拟机".into());
    }
    Ok(Some(Input {
        options: options.clone(),
        directory,
        manifest,
        manifest_sha256: format!("{:x}", Sha256::digest(&bytes)),
    }))
}
