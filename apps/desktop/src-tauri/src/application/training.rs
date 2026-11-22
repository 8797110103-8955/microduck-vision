//! Application service for training process orchestration and progress events.

use std::io::{BufRead, BufReader};
use std::process::Command as ProcessCommand;
use std::process::Stdio;
use std::sync::{Arc, Mutex};

use duck_body::BodyProfile;
use tauri::{AppHandle, Emitter, Manager, Wry};

use crate::application::policy;
use crate::state::DesktopState;
use crate::types::{
    TrainingPhase, TrainingPolicyLayout, TrainingProgress, TrainingRunMetadata, TrainingStartInput,
};

fn training_runs_root(config_dir: Option<&std::path::Path>) -> std::path::PathBuf {
    config_dir
        .and_then(std::path::Path::parent)
        .map(|root| root.join("training").join("runs"))
        .unwrap_or_else(|| {
            std::env::temp_dir()
                .join("microduck-vision")
                .join("training")
                .join("runs")
        })
}
fn new_training_run_id() -> String {
    let stamp = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|duration| duration.as_nanos())
        .unwrap_or_default();
    format!("run-{stamp}-{}", std::process::id())
}
pub(crate) fn training_metadata(
    state: &DesktopState,
    input: &TrainingStartInput,
    run_id: &str,
) -> Result<TrainingRunMetadata, String> {
    let machine = state
        .virtual_machines
        .lock()
        .map_err(|error| error.to_string())?
        .iter()
        .find(|machine| machine.id == input.virtual_machine_id)
        .cloned()
        .ok_or_else(|| format!("unknown virtual machine: {}", input.virtual_machine_id))?;
    let profile = BodyProfile::official().map_err(|error| error.to_string())?;
    let identity = profile.identity();
    if machine.body_id != identity.id {
        return Err(format!(
            "virtual machine body {} does not match training body {}",
            machine.body_id, identity.id
        ));
    }
    let (policy_layout, policy_id) = if let Some(dataset) = &input.dataset_path {
        let dataset: serde_json::Value = serde_json::from_slice(
            &std::fs::read(std::path::Path::new(dataset).join("dataset.json"))
                .map_err(|e| e.to_string())?,
        )
        .map_err(|e| e.to_string())?;
        let contract = &dataset["policy_contract"];
        if contract["body_id"].as_str() != Some(identity.id.as_str())
            || contract["body_version"].as_u64() != Some(u64::from(identity.version))
        {
            return Err("教师数据的身体定义与所选模拟机不匹配".into());
        }
        let dimension = |name: &str| -> Result<usize, String> {
            contract[name]
                .as_u64()
                .and_then(|value| usize::try_from(value).ok())
                .filter(|value| *value > 0)
                .ok_or_else(|| format!("教师数据缺少有效的 {name}"))
        };
        (
            TrainingPolicyLayout {
                obs_len: dimension("obs_len")?,
                action_len: dimension("action_len")?,
                control_hz: u32::try_from(dimension("control_hz")?).map_err(|e| e.to_string())?,
                command_encoding: contract["command_encoding"].as_str().map(str::to_owned),
            },
            contract["id"].as_str().map(str::to_owned),
        )
    } else if let Some(policy_id) = input
        .policy_id
        .as_deref()
        .map(str::trim)
        .filter(|id| !id.is_empty())
    {
        let entry = policy::list(state.config_dir.as_deref())?
            .into_iter()
            .find(|entry| entry.id == policy_id)
            .ok_or_else(|| format!("unknown policy: {policy_id}"))?;
        let descriptor = policy::descriptor(state.config_dir.as_deref(), &entry)?;
        (
            TrainingPolicyLayout {
                obs_len: descriptor.obs_len,
                action_len: descriptor.action_len,
                control_hz: descriptor.control_hz,
                command_encoding: descriptor.command_encoding,
            },
            Some(policy_id.to_owned()),
        )
    } else {
        (
            TrainingPolicyLayout {
                obs_len: 61,
                action_len: 14,
                control_hz: 50,
                command_encoding: None,
            },
            None,
        )
    };
    Ok(TrainingRunMetadata {
        experiment_id: input
            .experiment_id
            .as_deref()
            .map(str::trim)
            .filter(|id| !id.is_empty())
            .unwrap_or(run_id)
            .to_owned(),
        body_template_id: identity.id,
        body_version: identity.version,
        body_source_hash: identity.source_hash,
        policy_id,
        policy_layout,
        servo_profile: machine.servo_model,
        recipe: input.recipe.trim().to_owned(),
        teacher_motion_provenance: input.teacher_motion_provenance.trim().to_owned(),
    })
}
fn persist_training_progress(progress: &TrainingProgress) {
    let result_dir = std::path::Path::new(&progress.result_dir);
    if std::fs::create_dir_all(result_dir).is_err() {
        return;
    }
    let run = serde_json::json!({        "runId": progress.run_id,        "resultDir": progress.result_dir,        "metadata": progress.metadata,    });
    if let Ok(bytes) = serde_json::to_vec_pretty(&run) {
        let _ = std::fs::write(result_dir.join("metadata.json"), bytes);
    }
    if let Ok(bytes) = serde_json::to_vec_pretty(progress) {
        let _ = std::fs::write(result_dir.join("progress.json"), bytes);
    }
}
fn training_number(line: &str, names: &[&str]) -> Option<f64> {
    for token in line.split_whitespace() {
        let normalized = token.trim_matches(|character: char| {
            matches!(character, ',' | ';' | '(' | ')' | '[' | ']' | '{' | '}')
        });
        let lower = normalized.to_ascii_lowercase();
        for name in names {
            for separator in ["=", ":"] {
                let prefix = format!("{name}{separator}");
                if let Some(value) = lower.strip_prefix(&prefix)
                    && let Ok(value) = value.trim_end_matches('%').parse::<f64>()
                {
                    return Some(value);
                }
            }
        }
    }
    None
}
fn parse_training_line(progress: &mut TrainingProgress, line: &str) {
    progress.phase = TrainingPhase::Running;
    progress.line = line.to_owned();
    progress.message = line.to_owned();
    progress.step = training_number(line, &["step", "steps", "iteration", "iter"])
        .map(|value| value.max(0.0) as u64)
        .or(progress.step);
    progress.total_steps = training_number(
        line,
        &[
            "total_steps",
            "total",
            "max_steps",
            "max_iter",
            "iterations",
        ],
    )
    .map(|value| value.max(0.0) as u64)
    .or(progress.total_steps);
    progress.reward =
        training_number(line, &["reward", "mean_reward", "episode_reward"]).or(progress.reward);
    progress.loss = training_number(line, &["loss", "policy_loss", "value_loss"]).or(progress.loss);
    if let Some((step, total)) = line.split_whitespace().find_map(|token| {
        let token = token.trim_matches(|character: char| {
            matches!(character, ',' | ';' | '(' | ')' | '[' | ']' | '{' | '}')
        });
        let (step, total) = token.split_once('/')?;
        Some((step.parse::<u64>().ok()?, total.parse::<u64>().ok()?))
    }) {
        progress.step = Some(step);
        progress.total_steps = Some(total);
    }
    if let Some(percent) = line
        .split_whitespace()
        .find_map(|token| token.trim_end_matches('%').parse::<f64>().ok())
        && (0.0..=100.0).contains(&percent)
    {
        progress.percent = Some(percent);
    }
    if progress.percent.is_none()
        && let (Some(step), Some(total)) = (progress.step, progress.total_steps)
        && total > 0
    {
        progress.percent = Some((step as f64 / total as f64 * 100.0).clamp(0.0, 100.0));
    }
}
fn emit_training_progress(state: &DesktopState, progress: TrainingProgress) {
    persist_training_progress(&progress);
    if let Ok(mut current) = state.training_progress.lock() {
        if current.as_ref().is_none_or(|current| {
            current.run_id != progress.run_id || current.phase != progress.phase
        }) {
            crate::logging::diagnostic(
                state.config_dir.as_deref(),
                "training",
                if progress.phase == TrainingPhase::Failed {
                    "error"
                } else {
                    "info"
                },
                "training_phase_changed",
                serde_json::json!({
                    "training_run_id":progress.run_id,"phase":progress.phase,
                    "path":progress.result_dir,"message":progress.message,"exit_code":progress.exit_code,
                }),
            );
        }
        *current = Some(progress.clone());
    }
    if let Ok(app) = state.app_handle.lock()
        && let Some(app) = app.as_ref()
    {
        let _ = app.emit("training-progress", progress);
    }
}
fn spawn_training_reader(
    app: AppHandle<Wry>,
    reader: impl std::io::Read + Send + 'static,
    channel: &'static str,
    initial: TrainingProgress,
) {
    std::thread::spawn(move || {
        use std::io::Write;
        let path = std::path::Path::new(&initial.result_dir).join(format!("{channel}.log"));
        let mut output = std::fs::OpenOptions::new()
            .create(true)
            .append(true)
            .open(path)
            .ok();
        for line in BufReader::new(reader).lines().map_while(Result::ok) {
            if let Some(output) = &mut output {
                let _ = writeln!(output, "{line}");
            }
            let state = app.state::<DesktopState>();
            let Some(mut progress) = state
                .training_progress
                .lock()
                .ok()
                .and_then(|current| current.clone())
            else {
                continue;
            };
            if progress.run_id != initial.run_id {
                continue;
            }
            parse_training_line(&mut progress, &line);
            emit_training_progress(&state, progress);
        }
    });
}
pub(crate) fn start(
    state: &DesktopState,
    input: TrainingStartInput,
) -> Result<TrainingProgress, String> {
    if input.program.trim().is_empty() {
        return Err("训练程序不能为空".into());
    }
    if input.working_dir.trim().is_empty() {
        return Err("训练工作目录不能为空".into());
    }
    if input.virtual_machine_id.trim().is_empty() {
        return Err("训练必须选择模拟机".into());
    }
    if input.recipe.trim().is_empty() {
        return Err("训练 recipe 不能为空".into());
    }
    if input.teacher_motion_provenance.trim().is_empty() {
        return Err("训练 teacher/motion provenance 不能为空".into());
    }
    {
        let mut process = state
            .training_process
            .lock()
            .map_err(|error| error.to_string())?;
        if let Some(existing) = process.as_ref() {
            let running = existing
                .lock()
                .map_err(|error| error.to_string())?
                .try_wait()
                .map_err(|error| error.to_string())?
                .is_none();
            if running {
                return Err("已有训练正在运行".into());
            }
            *process = None;
        }
    }
    let working_dir = std::path::absolute(&input.working_dir)
        .map_err(|error| format!("解析训练目录失败：{error}"))?;
    if !working_dir.is_dir() {
        return Err(format!("训练目录不存在：{}", working_dir.display()));
    }
    let run_id = new_training_run_id();
    let metadata = training_metadata(state, &input, &run_id)?;
    let result_dir = training_runs_root(state.config_dir.as_deref()).join(&run_id);
    std::fs::create_dir_all(&result_dir)
        .map_err(|error| format!("创建训练结果目录失败：{error}"))?;
    let app = state
        .app_handle
        .lock()
        .map_err(|error| error.to_string())?
        .clone()
        .ok_or_else(|| "训练事件通道尚未初始化".to_owned())?;
    let mut args = input.args.clone();
    if let Some(dataset) = &input.dataset_path {
        let dataset = std::path::absolute(dataset).map_err(|e| e.to_string())?;
        let manifest: serde_json::Value = serde_json::from_slice(
            &std::fs::read(dataset.join("dataset.json")).map_err(|e| e.to_string())?,
        )
        .map_err(|e| e.to_string())?;
        if manifest["kind"] != "policy" || manifest["samples"].as_u64().unwrap_or(0) < 2 {
            return Err("请选择至少包含两个有效样本的策略模仿数据集".into());
        }
        let script = result_dir.join("teacher_train.py");
        std::fs::write(&script, include_str!("../../../training/teacher_train.py"))
            .map_err(|e| e.to_string())?;
        args = vec![
            "run".into(),
            "python".into(),
            script.to_string_lossy().into_owned(),
            "--dataset".into(),
            dataset.to_string_lossy().into_owned(),
            "--output".into(),
            result_dir.to_string_lossy().into_owned(),
        ];
        std::fs::write(
            result_dir.join("teacher-dataset.json"),
            serde_json::to_vec_pretty(&manifest).map_err(|e| e.to_string())?,
        )
        .map_err(|e| e.to_string())?;
    }
    let mut command = ProcessCommand::new(&input.program);
    command
        .args(&args)
        .current_dir(&working_dir)
        .env("MICRODUCK_TRAINING_RUN_ID", &run_id)
        .env("MICRODUCK_TRAINING_RESULT_DIR", &result_dir)
        .env("MICRODUCK_TRAINING_EXPERIMENT_ID", &metadata.experiment_id)
        .stdout(Stdio::piped())
        .stderr(Stdio::piped());
    let mut child = command
        .spawn()
        .map_err(|error| format!("启动本地训练失败：{error}"))?;
    let stdout = child.stdout.take();
    let stderr = child.stderr.take();
    let process = Arc::new(Mutex::new(child));
    *state
        .training_process
        .lock()
        .map_err(|error| error.to_string())? = Some(process.clone());
    *state
        .training_stop_requested
        .lock()
        .map_err(|error| error.to_string())? = false;
    let starting = TrainingProgress {
        run_id,
        result_dir: result_dir.to_string_lossy().into_owned(),
        metadata,
        phase: TrainingPhase::Starting,
        step: None,
        total_steps: None,
        percent: None,
        reward: None,
        loss: None,
        message: format!("正在启动：{} {}", input.program, input.args.join(" ")),
        line: String::new(),
        exit_code: None,
    };
    emit_training_progress(state, starting.clone());
    if let Some(stdout) = stdout {
        spawn_training_reader(app.clone(), stdout, "stdout", starting.clone());
    }
    if let Some(stderr) = stderr {
        spawn_training_reader(app.clone(), stderr, "stderr", starting.clone());
    }
    let initial_progress = starting.clone();
    std::thread::spawn(move || {
        loop {
            let result = process
                .lock()
                .ok()
                .and_then(|mut child| child.try_wait().ok().flatten());
            let Some(status) = result else {
                std::thread::sleep(std::time::Duration::from_millis(100));
                continue;
            };
            let state = app.state::<DesktopState>();
            let same_process = state
                .training_process
                .lock()
                .ok()
                .and_then(|current| current.clone())
                .is_some_and(|current| Arc::ptr_eq(&current, &process));
            if !same_process {
                break;
            }
            let stopped = state
                .training_stop_requested
                .lock()
                .map(|requested| *requested)
                .unwrap_or(false);
            let phase = if stopped {
                TrainingPhase::Stopped
            } else if status.success() {
                TrainingPhase::Finished
            } else {
                TrainingPhase::Failed
            };
            let mut progress = state
                .training_progress
                .lock()
                .ok()
                .and_then(|current| current.clone())
                .unwrap_or_else(|| initial_progress.clone());
            progress.phase = phase;
            progress.exit_code = status.code();
            progress.percent = (phase == TrainingPhase::Finished)
                .then_some(100.0)
                .or(progress.percent);
            progress.message = match phase {
                TrainingPhase::Finished => "训练完成".into(),
                TrainingPhase::Stopped => "训练已停止".into(),
                TrainingPhase::Failed => format!("训练退出（代码 {:?}）", status.code()),
                _ => progress.message,
            };
            emit_training_progress(&state, progress);
            if let Ok(mut current) = state.training_process.lock()
                && current
                    .as_ref()
                    .is_some_and(|current| Arc::ptr_eq(current, &process))
            {
                *current = None;
            }
            break;
        }
    });
    Ok(starting)
}
pub(crate) fn stop(state: &DesktopState) -> Result<(), String> {
    let process = state
        .training_process
        .lock()
        .map_err(|error| error.to_string())?
        .clone()
        .ok_or_else(|| "当前没有运行中的训练".to_owned())?;
    *state
        .training_stop_requested
        .lock()
        .map_err(|error| error.to_string())? = true;
    process
        .lock()
        .map_err(|error| error.to_string())?
        .kill()
        .map_err(|error| format!("停止训练失败：{error}"))
}
pub(crate) fn status(state: &DesktopState) -> Result<Option<TrainingProgress>, String> {
    state
        .training_progress
        .lock()
        .map(|progress| progress.clone())
        .map_err(|error| error.to_string())
}
