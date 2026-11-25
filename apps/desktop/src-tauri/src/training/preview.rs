//! Replay an unpublished checkpoint in its own native simulator and log directory.

use std::fs::{self, File};
use std::io::{BufWriter, Write};
use std::path::Path;

use serde::Serialize;

use crate::runtime::session::execution::{PolicyCache, PolicyExecution};
use crate::types::{
    PreviewInput, TrainingFrame, TrainingPreview, TrainingPreviewInput, TrainingPreviewReport,
    VirtualMachine,
};
use microduck_training::progress::training_runs_root;

pub(crate) const DEFAULT_STEPS: u32 = 500;
pub(crate) const MAX_STEPS: u32 = 1500;
const SAMPLE_STRIDE: u32 = 2;

fn write_json(path: &Path, value: &impl Serialize) -> Result<(), String> {
    let file = File::create(path).map_err(|error| error.to_string())?;
    serde_json::to_writer_pretty(file, value).map_err(|error| error.to_string())
}

pub(crate) fn run(
    config: Option<&Path>,
    machine: &VirtualMachine,
    input: &TrainingPreviewInput,
) -> Result<TrainingPreview, String> {
    let steps = input.steps.unwrap_or(DEFAULT_STEPS);
    if !(1..=MAX_STEPS).contains(&steps) {
        return Err(format!("预览步数须为 1 到 {MAX_STEPS}"));
    }
    let root = training_runs_root(config).with_file_name("previews");
    fs::create_dir_all(&root).map_err(|error| error.to_string())?;
    let output = root.join(format!("preview-{}", uuid::Uuid::new_v4()));
    fs::create_dir(&output).map_err(|error| error.to_string())?;
    write_json(&output.join("request.json"), input)?;
    rollout(config, machine, input, steps, &output).map_err(|error| {
        let _ = write_json(
            &output.join("error.json"),
            &serde_json::json!({ "error": error }),
        );
        format!("{error}。预览日志：{}", output.display())
    })
}

fn rollout(
    config: Option<&Path>,
    machine: &VirtualMachine,
    input: &TrainingPreviewInput,
    steps: u32,
    output: &Path,
) -> Result<TrainingPreview, String> {
    let round = super::rounds::directory(config, &input.run_id)?;
    let (_, _, snapshot) = super::rounds::checkpoint(&round, &input.checkpoint_id)?;
    let body = crate::runtime::machines::machine_resolution::body_profile(&machine.body_id)?;
    let candidate = super::policy_artifacts::resolve(&snapshot, "preview-candidate".into(), body)?;
    let machine_selection =
        crate::runtime::machines::machine_resolution::simulation_selection(machine)?;
    let policy_sha256 = candidate
        .training
        .as_ref()
        .ok_or("预览策略缺少训练合同")?
        .sha256
        .clone();
    let scene = crate::runtime::scene::scene_model()?;
    write_json(
        &output.join("layout.json"),
        &serde_json::json!({
            "jointNames": body.template().control.joint_order,
            "bodyNames": scene.model.bodies.iter().map(|body| body.name.as_str()).collect::<Vec<_>>(),
        }),
    )?;
    // The separate registry has no physical connection or app runtime state. Native controller
    // diagnostics go into this preview's logs/ alongside the structured samples and report.
    let log_config = output.join("config");
    let mut registry = loop_body::BackendRegistry::default();
    let mut execution = PolicyExecution::open(
        machine_selection,
        machine.simulation.clone(),
        PolicyCache {
            config_dir: Some(&log_config),
            resolve: |_| Ok(None),
        },
        &machine.id,
        &machine.name,
        &mut registry,
        body.template(),
    )?;
    execution.select_training_policy(candidate)?;
    let command = PreviewInput {
        twist: [0.15, 0.0, 0.0],
        head: [0.0; 4],
        body: [0.0; 3],
        target_distance_m: None,
    };
    execution.set_input(command);
    let initial = execution
        .status()
        .simulation
        .ok_or("预览模拟机没有初始姿态")?
        .trunk_position;
    let fingerprint = execution
        .simulation_fingerprint()
        .ok_or("预览模拟机缺少执行标识")?
        .to_owned();
    let period = execution.control_period();
    let mut log = BufWriter::new(
        File::create(output.join("samples.jsonl")).map_err(|error| error.to_string())?,
    );
    let mut frames = Vec::new();
    let mut last = initial;
    let mut min_height = initial[2];
    let mut max_tilt: f64 = 0.0;
    let mut upright = 0;
    let mut policy_steps = 0;
    for step in 1..=steps {
        let state = execution.tick()?;
        let simulation = state.simulation.ok_or("预览模拟机没有姿态")?;
        let position = simulation.trunk_position;
        let orientation = simulation.trunk_quaternion;
        if !position
            .iter()
            .chain(orientation.iter())
            .all(|v| v.is_finite())
        {
            return Err(format!("第 {step} 步的模拟姿态出现非有限数值"));
        }
        let tilt = (1.0 - 2.0 * (orientation[1].powi(2) + orientation[2].powi(2)))
            .clamp(-1.0, 1.0)
            .acos()
            .to_degrees();
        last = position;
        min_height = min_height.min(position[2]);
        max_tilt = max_tilt.max(tilt);
        upright += u32::from(tilt <= 45.0 && position[2] >= initial[2] * 0.5);
        let trace = execution.runner().policy_trace();
        policy_steps += u32::from(trace.is_some());
        serde_json::to_writer(
            &mut log,
            &serde_json::json!({
                "step": step,
                "phase": state.phase,
                "seconds": period.as_secs_f64() * f64::from(step),
                "trunkPosition": position,
                "trunkQuaternion": orientation,
                "baseVelocity": simulation.base_velocity,
                "motorPositions": simulation.motor_positions,
                "passivePositions": simulation.passive_positions,
                "bodyPoses": simulation.frame.bodies,
                "positions": state.positions,
                "targets": state.targets,
                "observation": trace.map(|trace| &trace.observation),
                "action": trace.map(|trace| &trace.action),
                "tiltDeg": tilt,
                "lastError": state.last_error,
            }),
        )
        .map_err(|error| error.to_string())?;
        log.write_all(b"\n").map_err(|error| error.to_string())?;
        if (step - 1) % SAMPLE_STRIDE == 0 {
            if simulation.frame.bodies.len() != scene.model.bodies.len() {
                return Err("预览姿态和显示模型的刚体数量不一致".into());
            }
            frames.push(TrainingFrame {
                run_id: input.run_id.clone(),
                sequence: u64::from(step),
                environment_index: 0,
                model_kind: machine.model_kind,
                execution_fingerprint: fingerprint.clone(),
                positions: state.positions.into_iter().map(Some).collect(),
                trunk_position: position,
                trunk_quaternion: orientation,
                frame: simulation.frame,
                preview_scene: None,
            });
        }
    }
    log.flush().map_err(|error| error.to_string())?;
    if policy_steps == 0 {
        return Err("预览未执行任何策略推理".into());
    }
    let report = TrainingPreviewReport {
        directory: output.display().to_string(),
        run_id: input.run_id.clone(),
        checkpoint_id: input.checkpoint_id.clone(),
        policy_sha256,
        execution_fingerprint: fingerprint,
        command,
        steps,
        policy_steps,
        seconds: period.as_secs_f64() * f64::from(steps),
        displacement_m: [last[0] - initial[0], last[1] - initial[1]],
        min_height_m: min_height,
        max_tilt_deg: max_tilt,
        upright_fraction: f64::from(upright) / f64::from(steps),
    };
    write_json(&output.join("report.json"), &report)?;
    Ok(TrainingPreview {
        report,
        scene,
        sample_interval_ms: (period * SAMPLE_STRIDE).as_millis() as u32,
        frames,
    })
}

pub(crate) fn run_cli(config: &Path, input: &TrainingPreviewInput) -> Result<(), String> {
    let config = config.canonicalize().map_err(|error| error.to_string())?;
    let machines = crate::platform::state::machine_store::load_machines(Some(&config))?;
    let machine = machines
        .virtual_machines
        .iter()
        .find(|machine| machine.id == input.virtual_machine_id)
        .ok_or("预览模拟机不存在")?;
    let result = run(Some(&config), machine, input);
    crate::platform::logging::flush();
    let result = result?;
    println!(
        "{}",
        serde_json::to_string(&result.report).map_err(|error| error.to_string())?
    );
    Ok(())
}
