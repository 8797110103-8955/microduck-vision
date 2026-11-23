//! Prepare the packaged Python entry points in a run directory.

use std::path::{Path, PathBuf};
use std::process::Command;

use serde_json::Value;

use crate::adapters::PYTHON_ADAPTERS;
use crate::progress::training_runs_root;

pub struct MjlabRun<'a> {
    program: &'a str,
    working_dir: &'a Path,
    result_dir: &'a Path,
    task: &'a str,
    options: &'a [String],
}

impl<'a> MjlabRun<'a> {
    pub fn new(
        program: &'a str,
        args: &'a [String],
        working_dir: &'a Path,
        result_dir: &'a Path,
    ) -> Result<Self, String> {
        let [run, train, task, options @ ..] = args else {
            return Err("训练需要指定 MJLab 任务".into());
        };
        if run != "run" || train != "train" || task.trim().is_empty() {
            return Err("训练参数应指定 uv run train <task>".into());
        }
        let upstream_lock = std::fs::read(working_dir.join("uv.lock"))
            .map_err(|error| format!("读取训练项目的 uv.lock 失败：{error}"))?;
        if !working_dir.join("pyproject.toml").is_file()
            || upstream_lock.as_slice() != include_bytes!("../../../vendor/microduck_rl/uv.lock")
        {
            return Err("训练项目须使用与桌面版本对应的 vendor/microduck_rl 锁定环境".into());
        }
        Ok(Self {
            program,
            working_dir,
            result_dir,
            task,
            options,
        })
    }

    pub fn task(&self) -> &str {
        self.task
    }

    pub fn prepare(&self, config_dir: Option<&Path>) -> Result<PathBuf, String> {
        std::fs::write(
            self.result_dir.join("train.py"),
            include_str!("../python/train.py"),
        )
        .map_err(|error| format!("准备训练入口失败：{error}"))?;
        std::fs::write(
            self.result_dir.join("training_preview.py"),
            include_str!("../python/training_preview.py"),
        )
        .map_err(|error| format!("准备训练预览入口失败：{error}"))?;
        Ok(std::env::var_os("UV_PROJECT_ENVIRONMENT")
            .map(PathBuf::from)
            .map(std::path::absolute)
            .transpose()
            .map_err(|error| error.to_string())?
            .unwrap_or_else(|| training_runs_root(config_dir).with_file_name("mjlab-venv")))
    }

    fn python_command(&self, operation: &str, environment: &Path) -> Command {
        let mut command = Command::new(self.program);
        command
            .args(["run", "--locked", "--project"])
            .arg(self.working_dir)
            .arg("python")
            .arg(self.result_dir.join("train.py"))
            .args([operation, self.task])
            .env("UV_PROJECT_ENVIRONMENT", environment)
            .env("PYTHONDONTWRITEBYTECODE", "1")
            .env("PYTHONUTF8", "1");
        command
    }

    pub fn describe(&self, environment: &Path, joint_order: &[String]) -> Result<Vec<u8>, String> {
        let joint_order = serde_json::to_string(joint_order).map_err(|error| error.to_string())?;
        let described = self
            .python_command("describe", environment)
            .args(["--joint-order", &joint_order])
            .args(self.options)
            .current_dir(self.result_dir)
            .output()
            .map_err(|error| format!("读取 MJLab 任务描述失败：{error}"))?;
        std::fs::write(
            self.result_dir.join("describe.stderr.log"),
            &described.stderr,
        )
        .map_err(|error| error.to_string())?;
        if !described.status.success() {
            let details = String::from_utf8_lossy(&described.stderr);
            let last = details
                .lines()
                .rev()
                .find(|line| !line.trim().is_empty())
                .unwrap_or_default();
            return Err(format!(
                "MJLab 任务描述失败（{}）：{last}",
                described.status
            ));
        }
        Ok(described.stdout)
    }

    pub fn install_profile(&self, profile: &[u8]) -> Result<(), String> {
        std::fs::write(self.result_dir.join("simulation-profile.json"), profile)
            .map_err(|error| error.to_string())?;
        let adapters = self.result_dir.join("body-adapters");
        for (path, source) in PYTHON_ADAPTERS {
            let path = adapters.join(path);
            std::fs::create_dir_all(path.parent().expect("adapter destination"))
                .map_err(|error| error.to_string())?;
            std::fs::write(path, source).map_err(|error| error.to_string())?;
        }
        Ok(())
    }

    pub fn command(
        &self,
        operation: &str,
        environment: &Path,
        preview_bodies: Option<&[&str]>,
    ) -> Result<Command, String> {
        let mut command = self.python_command(operation, environment);
        if let Some(names) = preview_bodies {
            command
                .arg("--preview-bodies")
                .arg(serde_json::to_string(names).map_err(|error| error.to_string())?);
        }
        command
            .arg("--profile")
            .arg(self.result_dir.join("simulation-profile.json"))
            .arg("--run-dir")
            .arg(self.result_dir)
            .arg("--adapters")
            .arg(self.result_dir.join("body-adapters"))
            .args(self.options)
            .current_dir(self.result_dir);
        Ok(command)
    }
}

pub fn teacher_command(
    program: &str,
    dataset: &str,
    working_dir: &Path,
    result_dir: &Path,
) -> Result<Command, String> {
    let dataset = std::path::absolute(dataset).map_err(|error| error.to_string())?;
    let manifest: Value = serde_json::from_slice(
        &std::fs::read(dataset.join("dataset.json")).map_err(|error| error.to_string())?,
    )
    .map_err(|error| error.to_string())?;
    if manifest["kind"] != "policy" || manifest["samples"].as_u64().unwrap_or(0) < 2 {
        return Err("请选择至少包含两个有效样本的策略模仿数据集".into());
    }
    let script = result_dir.join("teacher_train.py");
    std::fs::write(&script, include_str!("../python/teacher_train.py"))
        .map_err(|error| error.to_string())?;
    std::fs::write(
        result_dir.join("teacher-dataset.json"),
        serde_json::to_vec_pretty(&manifest).map_err(|error| error.to_string())?,
    )
    .map_err(|error| error.to_string())?;
    let mut command = Command::new(program);
    command
        .args(["run", "python"])
        .arg(script)
        .arg("--dataset")
        .arg(dataset)
        .arg("--output")
        .arg(result_dir)
        .current_dir(working_dir);
    Ok(command)
}

pub fn install_evaluation(result_dir: &Path, spec: &Value) -> Result<PathBuf, String> {
    let path = result_dir.join("teacher-evaluation-spec.json");
    std::fs::write(
        &path,
        serde_json::to_vec_pretty(spec).map_err(|error| error.to_string())?,
    )
    .map_err(|error| error.to_string())?;
    std::fs::write(
        result_dir.join("teacher_evaluation.py"),
        include_str!("../python/teacher_evaluation.py"),
    )
    .map_err(|error| error.to_string())?;
    Ok(path)
}
