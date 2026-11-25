//! Experiment rounds, checkpoint lineage and explicit immutable policy publication.

use crate::{
    runtime::policy::{TRAINED_POLICIES, policy_directory},
    runtime::session::state::DesktopState,
    types::{
        Checkpoint, PolicyEntry, PublicationTarget, RoundRecord, RoundSummary, ServoModelKey,
        TrainingProgress, TrainingStartInput,
    },
};
use microduck_training::{
    deepmimic::{MotionManifest, valid_id},
    progress::training_runs_root,
};
use serde::Serialize;
use sha2::{Digest, Sha256};
use std::{
    fs,
    path::{Path, PathBuf},
};

fn read<T: serde::de::DeserializeOwned>(path: &Path) -> Result<T, String> {
    serde_json::from_slice(&fs::read(path).map_err(|e| e.to_string())?).map_err(|e| e.to_string())
}

fn write(path: &Path, value: &impl Serialize) -> Result<(), String> {
    fs::write(
        path,
        serde_json::to_vec_pretty(value).map_err(|e| e.to_string())?,
    )
    .map_err(|e| e.to_string())
}

pub(crate) fn records(config: Option<&Path>) -> Result<Vec<(PathBuf, RoundRecord)>, String> {
    let root = training_runs_root(config);
    if !root.is_dir() {
        return Ok(vec![]);
    }
    let mut result = vec![];
    for experiment in fs::read_dir(&root).map_err(|e| e.to_string())? {
        let experiment = experiment.map_err(|e| e.to_string())?;
        if !experiment.file_type().map_err(|e| e.to_string())?.is_dir() {
            continue;
        }
        for round in fs::read_dir(experiment.path()).map_err(|e| e.to_string())? {
            let round = round.map_err(|e| e.to_string())?;
            if !round.file_type().map_err(|e| e.to_string())?.is_dir() {
                continue;
            }
            let path = round.path().canonicalize().map_err(|e| e.to_string())?;
            if !path.starts_with(root.canonicalize().map_err(|e| e.to_string())?) {
                return Err("训练轮次路径超出结果目录".into());
            }
            if let Ok(record) = read::<RoundRecord>(&path.join("round.json")) {
                if record.experiment_id != experiment.file_name().to_string_lossy()
                    || round.file_name().to_string_lossy() != format!("round-{:04}", record.round)
                {
                    return Err("训练轮次记录与目录不一致".into());
                }
                result.push((path, record));
            }
        }
    }
    result.sort_by(|a, b| b.1.run_id.cmp(&a.1.run_id));
    Ok(result)
}

pub(crate) fn directory(config: Option<&Path>, run: &str) -> Result<PathBuf, String> {
    records(config)?
        .into_iter()
        .find(|(_, r)| r.run_id == run)
        .map(|(p, _)| p)
        .ok_or("训练轮次不存在".into())
}

pub(crate) fn checkpoint(
    directory: &Path,
    id: &str,
) -> Result<(PathBuf, Checkpoint, PathBuf), String> {
    if !valid_id(id) {
        return Err("无效的 checkpoint ID".into());
    }
    let root = directory.canonicalize().map_err(|e| e.to_string())?;
    let snapshot = root
        .join("checkpoints")
        .join(id)
        .canonicalize()
        .map_err(|e| e.to_string())?;
    if !snapshot.starts_with(&root) {
        return Err("Checkpoint 路径超出轮次目录".into());
    }
    let info: Checkpoint = read(&snapshot.join("checkpoint.json"))?;
    if info.id != id
        || !Path::new(&info.file)
            .components()
            .all(|p| matches!(p, std::path::Component::Normal(_)))
    {
        return Err("Checkpoint 清单无效".into());
    }
    let file = root
        .join(&info.file)
        .canonicalize()
        .map_err(|e| e.to_string())?;
    if !file.starts_with(&root)
        || format!(
            "{:x}",
            Sha256::digest(fs::read(&file).map_err(|e| e.to_string())?)
        ) != info.sha256
    {
        return Err("Checkpoint 路径或摘要无效".into());
    }
    Ok((file, info, snapshot))
}

pub(crate) fn list(config: Option<&Path>) -> Result<Vec<RoundSummary>, String> {
    records(config)?
        .into_iter()
        .map(|(path, record)| {
            let mut checkpoints = vec![];
            if let Ok(entries) = fs::read_dir(path.join("checkpoints")) {
                for entry in entries.flatten() {
                    let id = entry.file_name().to_string_lossy().into_owned();
                    if let Ok((_, info, _)) = checkpoint(&path, &id) {
                        checkpoints.push(info);
                    }
                }
            }
            checkpoints.sort_by_key(|c| c.iteration);
            Ok(RoundSummary {
                record,
                progress: read(&path.join("progress.json")).ok(),
                checkpoints,
            })
        })
        .collect()
}

fn task_id(input: &TrainingStartInput) -> Result<&str, String> {
    if input.deepmimic.is_some() {
        Ok(microduck_training::deepmimic::TASK_ID)
    } else {
        input
            .args
            .get(2)
            .filter(|s| !s.is_empty())
            .map(String::as_str)
            .ok_or_else(|| "缺少训练任务".into())
    }
}

pub(super) struct ExperimentReplacement {
    original: PathBuf,
    backup: PathBuf,
    replacement: PathBuf,
}

impl ExperimentReplacement {
    pub(super) fn finish(self) {
        if let Err(error) = fs::remove_dir_all(&self.backup) {
            eprintln!("删除同名训练备份失败（{}）：{error}", self.backup.display());
        }
    }

    pub(super) fn restore(self) -> Result<(), String> {
        let restore = || -> std::io::Result<()> {
            if self.replacement.try_exists()? {
                let failed = self
                    .backup
                    .with_file_name(format!(".failed-{}", uuid::Uuid::new_v4()));
                fs::rename(&self.replacement, failed)?;
            }
            fs::rename(&self.backup, &self.original)
        };
        restore().map_err(|error| format!("恢复旧训练失败（{}）：{error}", self.backup.display()))
    }
}

pub(super) fn stage_fresh_experiment(
    config: Option<&Path>,
    input: &TrainingStartInput,
    servo_model: ServoModelKey,
) -> Result<Option<ExperimentReplacement>, String> {
    let id = input
        .experiment_id
        .as_deref()
        .map(str::trim)
        .filter(|id| !id.is_empty());
    if input.replace_existing_experiment && (input.initialization.is_some() || id.is_none()) {
        return Err("删除同名训练只适用于已指定训练 ID 的从头训练".into());
    }
    let Some(id) = id.filter(|_| input.initialization.is_none()) else {
        return Ok(None);
    };
    if !valid_id(id) || id.ends_with('.') {
        return Err("训练 ID 只能包含字母、数字、横线、下划线和点，且不能以点结尾".into());
    }
    let root = training_runs_root(config);
    let find = || -> std::io::Result<_> {
        let entries = match fs::read_dir(&root) {
            Ok(entries) => entries,
            Err(error) if error.kind() == std::io::ErrorKind::NotFound => return Ok(None),
            Err(error) => return Err(error),
        };
        for entry in entries {
            let entry = entry?;
            if entry.file_name().to_string_lossy().eq_ignore_ascii_case(id) {
                return Ok(Some(entry));
            }
        }
        Ok(None)
    };
    let Some(entry) = find().map_err(|error| error.to_string())? else {
        return if input.replace_existing_experiment {
            Err("同名训练已不存在，请刷新训练历史".into())
        } else {
            Ok(None)
        };
    };
    if !input.replace_existing_experiment {
        return Err("训练 ID 已存在，请更换 ID 或勾选删除同名训练".into());
    }
    let root = root.canonicalize().map_err(|error| error.to_string())?;
    let original = root.join(entry.file_name());
    if !entry
        .file_type()
        .map_err(|error| error.to_string())?
        .is_dir()
        || original.canonicalize().map_err(|error| error.to_string())? != original
    {
        return Err("同名训练目录无效".into());
    }
    let task = task_id(input)?;
    let old_rounds = records(config)?;
    let old_rounds: Vec<_> = old_rounds
        .iter()
        .filter(|(path, _)| path.parent() == Some(&original))
        .collect();
    let entries = fs::read_dir(&original).map_err(|error| error.to_string())?;
    if old_rounds.is_empty()
        || entries.count() != old_rounds.len()
        || old_rounds.iter().any(|(path, record)| {
            record.task_id != task
                || !read::<TrainingProgress>(&path.join("progress.json"))
                    .is_ok_and(|progress| progress.metadata.servo_profile == servo_model)
        })
    {
        return Err("同名训练包含不匹配或无法识别的任务或电机，无法替换".into());
    }
    let backup = root
        .parent()
        .ok_or("训练目录无效")?
        .join(format!(".replaced-{}", uuid::Uuid::new_v4()));
    fs::rename(&original, &backup).map_err(|error| format!("暂存同名训练失败：{error}"))?;
    Ok(Some(ExperimentReplacement {
        original,
        backup,
        replacement: root.join(id),
    }))
}

pub(crate) fn allocate(
    config: Option<&Path>,
    input: &mut TrainingStartInput,
    run_id: &str,
) -> Result<PathBuf, String> {
    let task_id = task_id(input)?.to_owned();
    let all = records(config)?;
    let parent = input
        .initialization
        .as_ref()
        .map(|init| {
            let (path, record) = all
                .iter()
                .find(|(_, r)| r.run_id == init.parent_run_id)
                .ok_or("父轮次不存在")?;
            if record.task_id != task_id {
                return Err("只能继承相同 task 的 checkpoint".to_owned());
            }
            checkpoint(path, &init.checkpoint)?;
            Ok(record.experiment_id.clone())
        })
        .transpose()?;
    let experiment_id = input
        .experiment_id
        .as_deref()
        .map(str::trim)
        .filter(|s| !s.is_empty())
        .map(str::to_owned)
        .or(parent)
        .unwrap_or_else(|| format!("experiment-{}", uuid::Uuid::new_v4()));
    if !valid_id(&experiment_id) {
        return Err("实验 ID 只能包含字母、数字、横线、下划线和点".into());
    }
    if all
        .iter()
        .any(|(_, r)| r.experiment_id == experiment_id && r.task_id != task_id)
    {
        return Err("同一实验只能使用一个 task".into());
    }
    let round = all
        .iter()
        .filter(|(_, r)| r.experiment_id == experiment_id)
        .map(|(_, r)| r.round)
        .max()
        .unwrap_or(0)
        .checked_add(1)
        .ok_or("轮次溢出")?;
    let directory = training_runs_root(config)
        .join(&experiment_id)
        .join(format!("round-{round:04}"));
    fs::create_dir_all(directory.parent().ok_or("实验目录无效")?).map_err(|e| e.to_string())?;
    fs::create_dir(&directory).map_err(|e| e.to_string())?;
    write(
        &directory.join("round.json"),
        &RoundRecord {
            run_id: run_id.into(),
            experiment_id: experiment_id.clone(),
            round,
            task_id,
            initialization: input.initialization.clone(),
        },
    )?;
    input.experiment_id = Some(experiment_id);
    Ok(directory)
}

pub(crate) fn install_initialization(
    config: Option<&Path>,
    input: &TrainingStartInput,
    result: &Path,
    command: &mut std::process::Command,
) -> Result<(), String> {
    if let Some(init) = &input.initialization {
        if init.reset_normalization && init.mode == crate::types::InitializationMode::Resume {
            return Err("重置归一化统计只适用于适配模式".into());
        }
        let parent = directory(config, &init.parent_run_id)?;
        let (file, info, _) = checkpoint(&parent, &init.checkpoint)?;
        let path = result.join("initialization.json");
        write(
            &path,
            &serde_json::json!({"mode":init.mode,"parent_directory":parent,"checkpoint_path":file,"checkpoint_sha256":info.sha256,"reset_normalization":init.reset_normalization}),
        )?;
        command.arg("--initialization").arg(path);
    }
    Ok(())
}

fn publication_reference(
    config: Option<&Path>,
    directory: &Path,
    progress: Option<&TrainingProgress>,
) -> super::policy_artifacts::PublicationReference {
    #[derive(serde::Deserialize)]
    struct MotionReference {
        bundle_manifest: MotionManifest,
    }

    let motion = read::<MotionReference>(&directory.join("training-metadata.json"))
        .ok()
        .map(|reference| reference.bundle_manifest);
    let training_policy_name = if let Some(motion) = &motion {
        crate::runtime::recordings::recording_name(config, &motion.source.recording_id).ok()
    } else {
        progress
            .and_then(|progress| progress.metadata.policy_id.clone())
            .map(|id| {
                crate::runtime::policy::catalog::list(config)
                    .ok()
                    .and_then(|policies| policies.into_iter().find(|policy| policy.id == id))
                    .map_or(id, |policy| policy.name)
            })
    };
    super::policy_artifacts::PublicationReference {
        task_id: None,
        motion_name: motion.map(|motion| motion.name),
        training_policy_name,
    }
}

pub(crate) fn publication_targets(config: Option<&Path>) -> Vec<PublicationTarget> {
    super::policy_artifacts::publication_targets(&policy_directory(config, TRAINED_POLICIES))
}

pub(crate) fn publish(
    state: &DesktopState,
    run_id: &str,
    checkpoint_id: &str,
    name: &str,
    append_to: Option<&str>,
) -> Result<PolicyEntry, String> {
    let _guard = state.training_start.lock().map_err(|e| e.to_string())?;
    if name.trim().is_empty() {
        return Err("请填写发布名称".into());
    }
    let config = state.config_dir.as_deref();
    let directory = directory(config, run_id)?;
    let (_, _, snapshot) = checkpoint(&directory, checkpoint_id)?;
    let progress = read::<TrainingProgress>(&directory.join("progress.json")).ok();
    let body = crate::runtime::machines::body_catalog::official()?;
    let selection = super::policy_artifacts::resolve(&snapshot, "candidate".into(), body)?;
    let _publication_lock = if append_to.is_some() {
        // Serialize appends across desktop instances while keeping lock files out of policy bundles.
        let lock = fs::File::options()
            .write(true)
            .create(true)
            .truncate(false)
            .open(training_runs_root(config).with_file_name(".publication.lock"))
            .map_err(|error| format!("打开发布锁失败：{error}"))?;
        lock.lock()
            .map_err(|error| format!("锁定发布目录失败：{error}"))?;
        Some(lock)
    } else {
        None
    };
    super::policy_artifacts::publish(
        &policy_directory(config, TRAINED_POLICIES),
        name.trim(),
        body,
        selection,
        publication_reference(config, &directory, progress.as_ref()),
        progress.map(|progress| progress.metadata.servo_profile),
        append_to,
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    fn scratch() -> PathBuf {
        crate::platform::helpers::test_directory().join(format!("rounds-{}", uuid::Uuid::new_v4()))
    }

    fn input(state: &DesktopState) -> TrainingStartInput {
        TrainingStartInput {
            initialization: None,
            replace_existing_experiment: false,
            program: "uv".into(),
            args: [
                "run",
                "train",
                "Mjlab-Velocity-Flat-MicroDuck",
                "--num-envs",
                "2",
                "--seed",
                "0",
                "--agent.max-iterations",
                "1",
                "--agent.save-interval",
                "1",
                "--agent.num-steps-per-env",
                "8",
                "--agent.logger",
                "tensorboard",
                "--agent.upload-model",
                "False",
            ]
            .map(str::to_owned)
            .to_vec(),
            virtual_machine_id: state.virtual_machines.lock().unwrap()[0].id.clone(),
            policy_id: None,
            experiment_id: Some("rehearsal".into()),
            recipe: "round-test".into(),
            teacher_motion_provenance: "task".into(),
            dataset_path: None,
            deepmimic: None,
        }
    }

    #[test]
    fn rounds_have_stable_lineage_and_never_become_policies_automatically() {
        let config = scratch().join("config");
        let state = DesktopState::new(Some(config.clone()));
        let mut request = input(&state);
        let first = allocate(Some(&config), &mut request, "run-one").unwrap();
        let second = allocate(Some(&config), &mut request, "run-two").unwrap();
        assert!(first.ends_with("rehearsal/round-0001"));
        assert!(second.ends_with("rehearsal/round-0002"));
        assert_eq!(list(Some(&config)).unwrap().len(), 2);
        assert!(!policy_directory(Some(&config), TRAINED_POLICIES).exists());
        assert!(directory(Some(&config), "../outside").is_err());
        request.experiment_id = Some("../outside".into());
        assert!(allocate(Some(&config), &mut request, "run-three").is_err());
        request.experiment_id = Some("rehearsal".into());
        request.args[2] = "different-task".into();
        assert!(allocate(Some(&config), &mut request, "run-three").is_err());
        assert!(checkpoint(&first, "../outside").is_err());
    }
}
