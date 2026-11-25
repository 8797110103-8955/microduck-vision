//! Training run paths, progress snapshots, and worker output parsing.

use std::path::Path;

use serde::Serialize;

pub fn training_runs_root(config_dir: Option<&std::path::Path>) -> std::path::PathBuf {
    config_dir
        .and_then(std::path::Path::parent)
        .map(|root| root.join("trainings").join("runs"))
        .unwrap_or_else(|| {
            std::env::temp_dir()
                .join("microduck-vision")
                .join("trainings")
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
    let mut tokens = line.split_whitespace().peekable();
    while let Some(token) = tokens.next() {
        let normalized = token.trim_matches(|character: char| {
            matches!(character, ',' | ';' | '(' | ')' | '[' | ']' | '{' | '}')
        });
        let lower = normalized.to_ascii_lowercase();
        for name in names {
            for separator in ["=", ":"] {
                let prefix = format!("{name}{separator}");
                if let Some(value) = lower.strip_prefix(&prefix) {
                    let value = if value.is_empty() {
                        tokens.peek().copied().unwrap_or_default()
                    } else {
                        value
                    };
                    if let Ok(value) = value.trim_end_matches('%').parse::<f64>()
                        && value.is_finite()
                    {
                        return Some(value);
                    }
                }
            }
        }
    }
    None
}
pub fn parse_training_metrics(progress: &mut TrainingMetrics, line: &str) {
    // Native MJLab reports both optimizer iterations and total environment
    // steps. Keep the displayed progress in iterations; setup tables and
    // reward/loss numbers must never become progress percentages.
    let native_total = line.trim().to_ascii_lowercase().starts_with("total steps:");
    let old_counts = (progress.step, progress.total_steps);
    progress.step = (!native_total)
        .then(|| training_number(line, &["step", "steps", "iteration", "iter"]))
        .flatten()
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
    let lower = line.trim().to_ascii_lowercase();
    if !lower.starts_with("mean ") || lower.starts_with("mean value loss:") {
        progress.loss =
            training_number(line, &["loss", "policy_loss", "value_loss"]).or(progress.loss);
    }
    let has_progress_label = line.to_ascii_lowercase().split_whitespace().any(|token| {
        matches!(token, "iteration" | "step" | "steps" | "progress") || token.starts_with("step=")
    });
    if let Some((step, total)) = has_progress_label
        .then(|| {
            line.split_whitespace().find_map(|token| {
                let token = token.trim_matches(|character: char| {
                    matches!(character, ',' | ';' | '(' | ')' | '[' | ']' | '{' | '}')
                });
                let (step, total) = token.split_once('/')?;
                Some((step.parse::<u64>().ok()?, total.parse::<u64>().ok()?))
            })
        })
        .flatten()
    {
        progress.step = Some(step);
        progress.total_steps = Some(total);
    }
    if let Some(percent) = line
        .split_whitespace()
        .find_map(|token| token.strip_suffix('%')?.parse::<f64>().ok())
        && (0.0..=100.0).contains(&percent)
    {
        progress.percent = Some(percent);
    } else if (progress.percent.is_none() || old_counts != (progress.step, progress.total_steps))
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn native_iterations_rewards_and_losses_have_distinct_meanings() {
        let mut value = TrainingMetrics::default();
        for line in [
            "Resolving joints 14/14",
            "action scale: 1.0",
            "Total steps: 76800",
        ] {
            parse_training_metrics(&mut value, line);
        }
        assert_eq!(value.step, None);
        assert_eq!(value.percent, None);
        parse_training_metrics(&mut value, "Learning iteration 19/50");
        assert_eq!(
            (value.step, value.total_steps, value.percent),
            (Some(19), Some(50), Some(38.0))
        );
        parse_training_metrics(&mut value, "Mean reward: 1.09");
        parse_training_metrics(&mut value, "Mean value loss: 0.0073");
        parse_training_metrics(&mut value, "Mean warmup_transitions loss: 119.0000");
        parse_training_metrics(&mut value, "Mean entropy loss: 19.6801");
        parse_training_metrics(&mut value, "Total steps: 76800");
        assert_eq!(value.reward, Some(1.09));
        assert_eq!(value.loss, Some(0.0073));
        assert_eq!(value.percent, Some(38.0));
        parse_training_metrics(&mut value, "Learning iteration 49/50");
        assert_eq!(value.percent, Some(98.0));
    }

    #[test]
    fn explicit_worker_metrics_and_percentages_remain_supported() {
        let mut value = TrainingMetrics::default();
        parse_training_metrics(&mut value, "step=5 total=10 reward=1.2 loss=0.3");
        assert_eq!(
            (value.step, value.total_steps, value.percent),
            (Some(5), Some(10), Some(50.0))
        );
        parse_training_metrics(&mut value, "progress 75%");
        assert_eq!(value.percent, Some(75.0));
        parse_training_metrics(&mut value, "loss=NaN");
        assert_eq!(value.loss, Some(0.3));
    }
}
