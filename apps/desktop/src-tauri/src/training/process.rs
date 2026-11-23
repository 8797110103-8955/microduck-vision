//! Application service for training process orchestration and progress events.

use std::io::{BufRead, BufReader};
use std::path::{Path, PathBuf};
use std::process::Command as ProcessCommand;
use std::process::Stdio;
use std::sync::{Arc, Mutex};

use tauri::{AppHandle, Manager, Wry};

use crate::platform::lifecycle::training_events;
use crate::runtime::policy::catalog as policy;
use crate::runtime::session::state::DesktopState;
use crate::types::{
    ModelKindKey, SceneInfo, TeacherEvaluationInput, TrainingFrame, TrainingPhase,
    TrainingPolicyLayout, TrainingProgress, TrainingRunMetadata, TrainingStartInput,
};
use microduck_body_runtime::models::assets::OfficialAssets;
use microduck_body_runtime::training::{TaskBodyDescription, resolve_training_body};
use microduck_training::launch::{MjlabRun, install_evaluation, teacher_command};
use microduck_training::progress::{
    TrainingMetrics, new_training_run_id, parse_training_metrics, save_evaluation_error,
    training_runs_root,
};

struct TrainingFrameSource {
    model_kind: ModelKindKey,
    execution_fingerprint: String,
    active_slots: Vec<bool>,
    body_count: usize,
    scene: Option<SceneInfo>,
}

impl TrainingFrameSource {
    fn decode(&self, json: &str, run_id: &str, after: u64) -> Option<TrainingFrame> {
        let frame: TrainingFrame = serde_json::from_str(json).ok()?;
        (frame.run_id == run_id
            && frame.sequence > after
            && frame.environment_index == 0
            && frame.model_kind == self.model_kind
            && frame.execution_fingerprint == self.execution_fingerprint
            && frame.positions.len() == self.active_slots.len()
            && frame
                .positions
                .iter()
                .zip(&self.active_slots)
                .all(|(position, active)| position.is_some() == *active)
            && frame.frame.bodies.len() == self.body_count
            && frame.frame.bodies.first().is_some_and(|pose| {
                pose[..3] == frame.trunk_position && pose[3..] == frame.trunk_quaternion
            }))
        .then_some(frame)
    }
}

fn prepare_rl_command(
    state: &DesktopState,
    input: &TrainingStartInput,
    working_dir: &Path,
    result_dir: &Path,
) -> Result<(ProcessCommand, Option<TrainingFrameSource>), String> {
    prepare_mjlab_command(state, input, working_dir, result_dir, "run")
}

fn prepare_mjlab_command(
    state: &DesktopState,
    input: &TrainingStartInput,
    working_dir: &Path,
    result_dir: &Path,
    operation: &str,
) -> Result<(ProcessCommand, Option<TrainingFrameSource>), String> {
    let mjlab = MjlabRun::new(&input.program, &input.args, working_dir, result_dir)?;
    let machine = crate::runtime::machines::machine_resolution::virtual_machine(
        state,
        &input.virtual_machine_id,
    )?;
    let body = crate::runtime::machines::machine_resolution::body_profile(&machine.body_id)?;
    let selection = crate::runtime::machines::machine_resolution::simulation_selection(&machine)?;
    let environment = mjlab.prepare(state.config_dir.as_deref())?;
    let described = mjlab.describe(&environment, &body.template().control.joint_order)?;
    let task_description: TaskBodyDescription = serde_json::from_slice(&described)
        .map_err(|error| format!("MJLab 任务描述无效：{error}"))?;
    if task_description.task_id != mjlab.task() {
        return Err("MJLab 返回了不同的训练任务".into());
    }
    let assets = OfficialAssets::Embedded;
    let execution = resolve_training_body(
        selection,
        body,
        &assets,
        crate::runtime::adapters::simulation::tuning(&machine.simulation),
        task_description,
    )?;
    mjlab.install_profile(&execution.export(&assets)?)?;
    let preview = match execution
        .display_scene(body, &assets)
        .map_err(|error| error.to_string())
        .and_then(|scene| {
            crate::runtime::scene::project_scene(&scene, &execution.task.home_pose_rad)
        }) {
        Ok(scene) => Some(TrainingFrameSource {
            model_kind: machine.model_kind,
            execution_fingerprint: execution.profile.execution_fingerprint,
            active_slots: execution
                .task
                .robot_joints
                .iter()
                .map(Option::is_some)
                .collect(),
            body_count: scene.model.bodies.len(),
            scene: Some(scene),
        }),
        Err(error) => {
            crate::platform::logging::diagnostic(
                state.config_dir.as_deref(),
                "training",
                "warn",
                "training_preview_unavailable",
                serde_json::json!({"task_id": mjlab.task(), "message": error}),
            );
            None
        }
    };
    let body_names = preview
        .as_ref()
        .and_then(|preview| preview.scene.as_ref())
        .map(|scene| {
            scene
                .model
                .bodies
                .iter()
                .map(|body| body.name.as_str())
                .collect::<Vec<_>>()
        });
    let command = mjlab.command(operation, &environment, body_names.as_deref())?;
    Ok((command, preview))
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
    let profile = crate::runtime::machines::machine_resolution::body_profile(&machine.body_id)?;
    let identity = profile.identity();
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
    let run = serde_json::json!({
        "runId": progress.run_id,
        "resultDir": progress.result_dir,
        "metadata": progress.metadata,
    });
    microduck_training::progress::persist_training_progress(
        Path::new(&progress.result_dir),
        &run,
        progress,
    );
}

fn parse_training_line(progress: &mut TrainingProgress, line: &str) {
    progress.phase = TrainingPhase::Running;
    progress.line = line.to_owned();
    progress.message = line.to_owned();
    let mut metrics = TrainingMetrics {
        step: progress.step,
        total_steps: progress.total_steps,
        percent: progress.percent,
        reward: progress.reward,
        loss: progress.loss,
    };
    parse_training_metrics(&mut metrics, line);
    progress.step = metrics.step;
    progress.total_steps = metrics.total_steps;
    progress.percent = metrics.percent;
    progress.reward = metrics.reward;
    progress.loss = metrics.loss;
}

fn update_training_progress(
    state: &DesktopState,
    update: impl FnOnce(Option<&TrainingProgress>) -> Option<TrainingProgress>,
) {
    let Ok(mut current) = state.training_progress.lock() else {
        return;
    };
    let Some(progress) = update(current.as_ref()) else {
        return;
    };
    persist_training_progress(&progress);
    if current
        .as_ref()
        .is_none_or(|current| current.run_id != progress.run_id || current.phase != progress.phase)
    {
        crate::platform::logging::diagnostic(
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
    if let Ok(app) = state.app_handle.lock()
        && let Some(app) = app.as_ref()
    {
        training_events::progress(app, progress);
    }
}

fn is_active_run(progress: &TrainingProgress, run_id: &str) -> bool {
    progress.run_id == run_id
        && matches!(
            progress.phase,
            TrainingPhase::Starting | TrainingPhase::Running
        )
}

fn spawn_training_reader(
    app: AppHandle<Wry>,
    reader: impl std::io::Read + Send + 'static,
    channel: &'static str,
    initial: TrainingProgress,
    mut preview: Option<TrainingFrameSource>,
) -> std::thread::JoinHandle<()> {
    std::thread::spawn(move || {
        use std::io::Write;
        let path = std::path::Path::new(&initial.result_dir).join(format!("{channel}.log"));
        let mut output = std::fs::OpenOptions::new()
            .create(true)
            .append(true)
            .open(path)
            .ok();
        let mut last_sequence = 0;
        for line in BufReader::new(reader).lines().map_while(Result::ok) {
            let state = app.state::<DesktopState>();
            if let Some(json) = line.strip_prefix("MICRODUCK_TRAINING_FRAME ") {
                if let Some(source) = preview.as_mut()
                    && let Some(mut frame) = source.decode(json, &initial.run_id, last_sequence)
                    && let Ok(current) = state.training_progress.lock()
                    && current
                        .as_ref()
                        .is_some_and(|progress| is_active_run(progress, &initial.run_id))
                {
                    last_sequence = frame.sequence;
                    frame.preview_scene = source.scene.take();
                    training_events::frame(&app, frame);
                }
                continue;
            }
            if let Some(output) = &mut output {
                let _ = writeln!(output, "{line}");
            }
            update_training_progress(&state, |current| {
                let mut progress = current
                    .filter(|progress| is_active_run(progress, &initial.run_id))?
                    .clone();
                parse_training_line(&mut progress, &line);
                Some(progress)
            });
        }
    })
}
fn ensure_process_idle(state: &DesktopState) -> Result<(), String> {
    let process = state
        .training_process
        .lock()
        .map_err(|error| error.to_string())?;
    if process.is_some() {
        return Err("已有训练或评估正在运行或收尾".into());
    }
    Ok(())
}

pub(crate) fn start(
    state: &DesktopState,
    input: TrainingStartInput,
) -> Result<TrainingProgress, String> {
    let _starting = state
        .training_start
        .try_lock()
        .map_err(|_| "已有训练正在启动")?;
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
    ensure_process_idle(state)?;
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
    let (command, preview) = if let Some(dataset) = &input.dataset_path {
        (
            teacher_command(&input.program, dataset, &working_dir, &result_dir)?,
            None,
        )
    } else {
        prepare_rl_command(state, &input, &working_dir, &result_dir)?
    };
    let starting = initial_progress(
        run_id,
        &result_dir,
        metadata,
        format!("正在启动：{} {}", input.program, input.args.join(" ")),
    );
    launch(state, app, command, preview, starting, false)
}

pub(crate) fn start_teacher_evaluation(
    state: &DesktopState,
    input: TeacherEvaluationInput,
) -> Result<TrainingProgress, String> {
    let _starting = state
        .training_start
        .try_lock()
        .map_err(|_| "已有训练或评估正在启动")?;
    ensure_process_idle(state)?;
    if input.task_id.trim().is_empty()
        || input.virtual_machine_id.is_empty()
        || input.working_dir.trim().is_empty()
        || !policy::is_training_policy(&input.teacher_policy_id)
        || !(1..=5000).contains(&input.steps)
        || input.seed > i32::MAX as u32
    {
        return Err("请选择教师产物、模拟机和任务，并填写有效的评估步数与种子".into());
    }
    let app = state
        .app_handle
        .lock()
        .map_err(|error| error.to_string())?
        .clone()
        .ok_or("训练事件通道尚未初始化")?;
    let run_id = new_training_run_id();
    let result_dir = training_runs_root(state.config_dir.as_deref()).join(&run_id);
    std::fs::create_dir_all(&result_dir).map_err(|error| error.to_string())?;
    let result = (|| {
        let (command, preview, metadata) =
            prepare_teacher_evaluation(state, &input, &run_id, &result_dir)?;
        let progress = initial_progress(
            run_id,
            &result_dir,
            metadata,
            format!("正在准备教师仿真评估：{}", input.task_id),
        );
        launch(state, app, command, preview, progress, true)
    })();
    result.map_err(|error| {
        save_evaluation_error(
            &result_dir,
            &input.teacher_policy_id,
            TrainingPhase::Failed,
            &error,
        );
        format!("{error}。评估结果目录：{}", result_dir.display())
    })
}

fn prepare_teacher_evaluation(
    state: &DesktopState,
    input: &TeacherEvaluationInput,
    run_id: &str,
    result_dir: &Path,
) -> Result<
    (
        ProcessCommand,
        Option<TrainingFrameSource>,
        TrainingRunMetadata,
    ),
    String,
> {
    let selection =
        policy::resolve_training_policy(state.config_dir.as_deref(), &input.teacher_policy_id)?;
    let teacher = selection
        .training
        .as_ref()
        .filter(|model| model.source["kind"] == "teacher")
        .ok_or("请选择离线教师训练生成的学生模型")?;
    let machine = crate::runtime::machines::machine_resolution::virtual_machine(
        state,
        &input.virtual_machine_id,
    )?;
    let profile = crate::runtime::machines::machine_resolution::body_profile(&machine.body_id)?;
    let identity = profile.identity();
    let contract = selection.descriptor;
    contract
        .validate(profile, std::time::Duration::from_millis(20))
        .map_err(|error| error.to_string())?;
    let spec_data = serde_json::json!({
        "model_path": selection.path, "onnx_sha256": teacher.sha256, "source": teacher.source,
        "input": teacher.spec.input, "output": teacher.spec.output,
        "home_pose_rad": teacher.spec.home, "policy_contract": contract,
    });
    let spec = install_evaluation(result_dir, &spec_data)?;
    let working_dir = std::path::absolute(&input.working_dir).map_err(|error| error.to_string())?;
    let metadata = TrainingRunMetadata {
        experiment_id: run_id.to_owned(),
        body_template_id: identity.id,
        body_version: identity.version,
        body_source_hash: identity.source_hash,
        policy_id: Some(input.teacher_policy_id.clone()),
        policy_layout: TrainingPolicyLayout {
            obs_len: contract.obs_len,
            action_len: contract.action_len,
            control_hz: contract.control_hz,
            command_encoding: contract.command_encoding,
        },
        servo_profile: machine.servo_model,
        recipe: "teacher-evaluation".into(),
        teacher_motion_provenance: input.teacher_policy_id.clone(),
    };
    let upstream = TrainingStartInput {
        program: "uv".into(),
        args: [
            "run",
            "train",
            input.task_id.trim(),
            "--num-envs",
            "1",
            "--domain-randomization",
            "off",
        ]
        .map(str::to_owned)
        .to_vec(),
        working_dir: input.working_dir.clone(),
        virtual_machine_id: input.virtual_machine_id.clone(),
        policy_id: Some(input.teacher_policy_id.clone()),
        experiment_id: None,
        recipe: metadata.recipe.clone(),
        teacher_motion_provenance: metadata.teacher_motion_provenance.clone(),
        dataset_path: None,
    };
    let (mut command, preview) = prepare_mjlab_command(
        state,
        &upstream,
        &working_dir,
        result_dir,
        "evaluate-teacher",
    )?;
    command
        .arg("--teacher-spec")
        .arg(spec)
        .arg("--evaluation-steps")
        .arg(input.steps.to_string())
        .arg("--evaluation-seed")
        .arg(input.seed.to_string());
    Ok((command, preview, metadata))
}

fn initial_progress(
    run_id: String,
    result_dir: &Path,
    metadata: TrainingRunMetadata,
    message: String,
) -> TrainingProgress {
    TrainingProgress {
        run_id,
        result_dir: result_dir.to_string_lossy().into_owned(),
        metadata,
        phase: TrainingPhase::Starting,
        step: None,
        total_steps: None,
        percent: None,
        reward: None,
        loss: None,
        message,
        line: String::new(),
        exit_code: None,
    }
}

fn launch(
    state: &DesktopState,
    app: AppHandle<Wry>,
    mut command: ProcessCommand,
    preview: Option<TrainingFrameSource>,
    starting: TrainingProgress,
    evaluation: bool,
) -> Result<TrainingProgress, String> {
    command
        .env("MICRODUCK_TRAINING_RUN_ID", &starting.run_id)
        .env("MICRODUCK_TRAINING_RESULT_DIR", &starting.result_dir)
        .env(
            "MICRODUCK_TRAINING_EXPERIMENT_ID",
            &starting.metadata.experiment_id,
        )
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
    update_training_progress(state, |_| Some(starting.clone()));
    let readers = [
        stdout.map(|stdout| {
            spawn_training_reader(app.clone(), stdout, "stdout", starting.clone(), preview)
        }),
        stderr.map(|stderr| {
            spawn_training_reader(app.clone(), stderr, "stderr", starting.clone(), None)
        }),
    ];
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
            for reader in readers.into_iter().flatten() {
                let _ = reader.join();
            }
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
            update_training_progress(&state, |current| {
                let mut progress = current
                    .filter(|progress| is_active_run(progress, &initial_progress.run_id))?
                    .clone();
                progress.phase = phase;
                progress.exit_code = status.code();
                progress.percent = (phase == TrainingPhase::Finished)
                    .then_some(100.0)
                    .or(progress.percent);
                progress.message = if evaluation {
                    let result_dir = Path::new(&progress.result_dir);
                    if !result_dir.join("evaluation.json").exists() {
                        save_evaluation_error(
                            result_dir,
                            &progress.metadata.teacher_motion_provenance,
                            phase,
                            &progress.line,
                        );
                    }
                    match phase {
                        TrainingPhase::Finished => "教师仿真评估完成".into(),
                        TrainingPhase::Stopped => "教师仿真评估已停止".into(),
                        _ => format!("教师仿真评估退出（代码 {:?}）", status.code()),
                    }
                } else {
                    match phase {
                        TrainingPhase::Finished => "训练完成".into(),
                        TrainingPhase::Stopped => "训练已停止".into(),
                        TrainingPhase::Failed => format!("训练退出（代码 {:?}）", status.code()),
                        _ => progress.message,
                    }
                };
                Some(progress)
            });
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
    let mut child = process.lock().map_err(|error| error.to_string())?;
    stop_child(&mut child).map_err(|error| format!("停止训练失败：{error}"))?;
    *state
        .training_stop_requested
        .lock()
        .map_err(|error| error.to_string())? = true;
    Ok(())
}

#[cfg(windows)]
fn stop_child(child: &mut std::process::Child) -> std::io::Result<()> {
    use std::os::windows::process::CommandExt;

    if child.try_wait()?.is_some() {
        return Err(std::io::Error::other("训练进程已退出"));
    }
    let system_root = std::env::var_os("SystemRoot").unwrap_or_else(|| "C:\\Windows".into());
    let result = ProcessCommand::new(PathBuf::from(system_root).join("System32/taskkill.exe"))
        .args(["/T", "/F", "/PID"])
        .arg(child.id().to_string())
        .creation_flags(0x08000000) // CREATE_NO_WINDOW
        .output()?;
    if !result.status.success() {
        return Err(std::io::Error::other(format!(
            "终止训练进程树失败：{}",
            result.status
        )));
    }
    Ok(())
}

#[cfg(not(windows))]
fn stop_child(child: &mut std::process::Child) -> std::io::Result<()> {
    child.kill()
}
pub(crate) fn status(state: &DesktopState) -> Result<Option<TrainingProgress>, String> {
    state
        .training_progress
        .lock()
        .map(|progress| progress.clone())
        .map_err(|error| error.to_string())
}

#[cfg(test)]
#[path = "process_tests.rs"]
mod tests;
