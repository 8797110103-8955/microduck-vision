//! Training run paths, progress snapshots, and worker output parsing.

use std::path::Path;

use serde::Serialize;

pub fn training_runs_root(config_dir: Option<&std::path::Path>) -> std::path::PathBuf {
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
pub fn new_training_run_id() -> String {
    let stamp = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|duration| duration.as_nanos())
        .unwrap_or_default();
    format!("run-{stamp}-{}", std::process::id())
}

pub fn persist_training_progress(
    result_dir: &Path,
    run: &impl Serialize,
    progress: &impl Serialize,
) {
    if std::fs::create_dir_all(result_dir).is_err() {
        return;
    }
    // The progress lock serializes writers. Rename lets readers open a complete snapshot
    // even while the next update is being written.
    for (name, bytes) in [
        ("metadata.json", serde_json::to_vec_pretty(run)),
        ("progress.json", serde_json::to_vec_pretty(progress)),
    ] {
        let temporary = result_dir.join(name).with_extension("json.tmp");
        if let Ok(bytes) = bytes
            && std::fs::write(&temporary, bytes).is_ok()
        {
            let _ = std::fs::rename(temporary, result_dir.join(name));
        }
    }
}

#[derive(Default)]
pub struct TrainingMetrics {
    pub step: Option<u64>,
    pub total_steps: Option<u64>,
    pub percent: Option<f64>,
    pub reward: Option<f64>,
    pub loss: Option<f64>,
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
pub fn parse_training_metrics(progress: &mut TrainingMetrics, line: &str) {
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

pub fn save_evaluation_error(dir: &Path, teacher: &str, phase: impl Serialize, error: &str) {
    let report = serde_json::json!({"format_version": 1, "teacher_policy_id": teacher, "status": phase, "error": error});
    let _ = std::fs::write(dir.join("evaluation.json"), report.to_string());
}
