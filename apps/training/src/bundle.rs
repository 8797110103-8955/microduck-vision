//! Use repository Python sources during development and adjacent resources in distributions.

use std::path::{Path, PathBuf};

pub fn identity() -> &'static str {
    env!("MICRODUCK_PYTHON_BUNDLE_ID")
}

pub(crate) fn lock_hash() -> &'static str {
    env!("MICRODUCK_PYTHON_LOCK_SHA256")
}

#[cfg(not(feature = "packaged-python"))]
pub fn project() -> Result<PathBuf, String> {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .canonicalize()
        .map_err(|e| format!("读取仓库训练工程失败：{e}"))
}

#[cfg(feature = "packaged-python")]
pub fn project() -> Result<PathBuf, String> {
    let executable = std::env::current_exe().map_err(|e| e.to_string())?;
    let directory = executable
        .parent()
        .ok_or("无法解析程序所在目录")?
        .join("python");
    let id = std::fs::read_to_string(directory.join(".bundle-id")).map_err(|e| {
        format!(
            "读取 Python 资源失败（{}）：{e}，请完整复制发布目录",
            directory.display()
        )
    })?;
    if id != identity() {
        return Err("Python 资源版本与程序不匹配，请重新构建或完整复制发布目录".into());
    }
    let project = directory.join("apps").join("training");
    if !project.join("pyproject.toml").is_file() || !modules(&project).join("train.py").is_file() {
        return Err(format!("Python 资源不完整：{}", directory.display()));
    }
    Ok(project)
}

pub fn modules(project: &Path) -> PathBuf {
    project.join("python")
}
