//! Launch the compiled Python bundle with per-job inputs and outputs.

use std::path::{Path, PathBuf};
use std::process::Command;

use serde_json::Value;
use sha2::{Digest, Sha256};

use crate::progress::training_runs_root;

fn validate_training_project(working_dir: &Path) -> Result<(), String> {
    let lock = std::fs::read(working_dir.join("uv.lock"))
        .map_err(|error| format!("读取训练项目的 uv.lock 失败：{error}"))?;
    if !working_dir.join("pyproject.toml").is_file()
        || format!("{:x}", Sha256::digest(&lock)) != crate::bundle::lock_hash()
    {
        return Err("训练项目须使用与桌面版本对应的锁定环境".into());
    }
    Ok(())
}

fn environment_path(config_dir: Option<&Path>) -> Result<PathBuf, String> {
    std::env::var_os("UV_PROJECT_ENVIRONMENT")
        .map(PathBuf::from)
        .map(std::path::absolute)
        .transpose()
        .map_err(|error| error.to_string())
        .map(|path| {
            path.unwrap_or_else(|| training_runs_root(config_dir).with_file_name("mjlab-venv"))
        })
}

fn pinned_python(program: &str, working_dir: &Path, environment: &Path) -> Command {
    let mut command = Command::new(program);
    #[cfg(windows)]
    {
        use std::os::windows::process::CommandExt;
        command.creation_flags(0x08000000); // CREATE_NO_WINDOW
    }
    command
        .args(["run", "--frozen", "--project"])
        .arg(working_dir)
        .arg("python")
        .env("UV_PROJECT_ENVIRONMENT", environment)
        .env("PYTHONDONTWRITEBYTECODE", "1")
        .env("PYTHONUTF8", "1")
        .env("PYTHONPATH", crate::bundle::modules(working_dir));
    command
}

/// Query the Python registry using the same environment as training.
pub fn registered_tasks(
    working_dir: &Path,
    config_dir: Option<&Path>,
) -> Result<Vec<String>, String> {
    let output = query_tasks(working_dir, config_dir, None)?;
    serde_json::from_slice(&output).map_err(|error| format!("Python 任务列表无效：{error}"))
}

pub fn task_configuration(
    working_dir: &Path,
    config_dir: Option<&Path>,
    task_id: &str,
) -> Result<crate::tasks::TaskConfiguration, String> {
    let output = query_tasks(working_dir, config_dir, Some(task_id))?;
    serde_json::from_slice(&output).map_err(|error| format!("Python 任务配置无效：{error}"))
}

fn query_tasks(
    working_dir: &Path,
    config_dir: Option<&Path>,
    task_id: Option<&str>,
) -> Result<Vec<u8>, String> {
    let working_dir = std::path::absolute(working_dir).map_err(|error| error.to_string())?;
    validate_training_project(&working_dir)?;
    let environment = environment_path(config_dir)?;
    let root = environment.join("task-discovery");
    // Configuration requests can overlap when the selected task changes.
    let modules = if task_id.is_some() {
        root.join(crate::progress::new_training_run_id())
    } else {
        root
    };
    let mut command = pinned_python("uv", &working_dir, &environment);
    command.args(["-m", "training.tasks"]);
    if let Some(task_id) = task_id {
        command.args(["--task", task_id]);
    }
    let output = command
        .current_dir(&working_dir)
        .output()
        .map_err(|error| format!("启动 Python 任务查询失败：{error}"))?;
    // Let uv create the environment before writing anything inside it.
    if environment.join("pyvenv.cfg").is_file() {
        std::fs::create_dir_all(&modules).map_err(|error| error.to_string())?;
        std::fs::write(modules.join("stderr.log"), &output.stderr)
            .map_err(|error| error.to_string())?;
    }
    if !output.status.success() {
        return Err(format!(
            "Python 任务查询失败（{}）：{}",
            output.status,
            String::from_utf8_lossy(&output.stderr).trim()
        ));
    }
    Ok(output.stdout)
}

/// Prepare a converter in the job's private directory using the same pinned
/// project check and environment selection as normal RL training.
pub fn deepmimic_conversion_command(
    program: &str,
    working_dir: &Path,
    job_dir: &Path,
    config_dir: Option<&Path>,
    parameters: &Value,
) -> Result<Command, String> {
    let working_dir = std::path::absolute(working_dir).map_err(|error| error.to_string())?;
    validate_training_project(&working_dir)?;
    let job_dir = std::path::absolute(job_dir).map_err(|error| error.to_string())?;
    let parameters_path = job_dir.join("conversion-parameters.json");
    std::fs::write(
        &parameters_path,
        serde_json::to_vec(parameters).map_err(|e| e.to_string())?,
    )
    .map_err(|error| error.to_string())?;
    let mut command = pinned_python(program, &working_dir, &environment_path(config_dir)?);
    command
        .args(["-m", "deepmimic.convert", "--source"])
        .arg(&job_dir)
        .arg("--parameters")
        .arg(parameters_path)
        .arg("--output")
        .arg(job_dir.join("bundle"))
        .current_dir(&job_dir);
    Ok(command)
}

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
        validate_training_project(working_dir)?;
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
        environment_path(config_dir)
    }

    fn python_command(&self, operation: &str, environment: &Path) -> Command {
        let mut command = pinned_python(self.program, self.working_dir, environment);
        command
            .arg(crate::bundle::modules(self.working_dir).join("train.py"))
            .args([operation, self.task]);
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
        let document: Value = serde_json::from_slice(profile).map_err(|error| error.to_string())?;
        let backend = &document["backend"];
        let fit = &backend["bam_fit"];
        if document["format_version"] != 2
            || backend["kind"] != "mjlab-bam"
            || backend["version"] != 1
            || fit["file"] != "bam-fit.json"
            || backend["execution_fingerprint"] != document["profile"]["execution_fingerprint"]
        {
            return Err("训练身体配置或 BAM 文件路径无效".into());
        }
        let content = fit["content"].as_str().ok_or("缺少 BAM fit 内容")?;
        std::fs::write(self.result_dir.join("bam-fit.json"), content)
            .map_err(|error| error.to_string())?;
        std::fs::write(self.result_dir.join("simulation-profile.json"), profile)
            .map_err(|error| error.to_string())?;
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
    config_dir: Option<&Path>,
) -> Result<Command, String> {
    let dataset = std::path::absolute(dataset).map_err(|error| error.to_string())?;
    let manifest: Value = serde_json::from_slice(
        &std::fs::read(dataset.join("dataset.json")).map_err(|error| error.to_string())?,
    )
    .map_err(|error| error.to_string())?;
    if manifest["kind"] != "policy" || manifest["samples"].as_u64().unwrap_or(0) < 2 {
        return Err("请选择至少包含两个有效样本的策略模仿数据集".into());
    }
    let script = crate::bundle::modules(working_dir).join("teacher_train.py");
    std::fs::write(
        result_dir.join("teacher-dataset.json"),
        serde_json::to_vec_pretty(&manifest).map_err(|error| error.to_string())?,
    )
    .map_err(|error| error.to_string())?;
    let mut command = pinned_python(program, working_dir, &environment_path(config_dir)?);
    command
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
    Ok(path)
}
