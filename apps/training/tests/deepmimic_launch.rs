#[path = "../../../.cargo/test_directory.rs"]
mod test_paths;

use test_paths::test_directory;

use std::path::PathBuf;

use microduck_training::launch::deepmimic_conversion_command;
use serde_json::json;

fn job_directory() -> PathBuf {
    let directory = test_directory()
        .join("deepmimic-jobs")
        .join(microduck_training::progress::new_training_run_id());
    std::fs::create_dir_all(&directory).unwrap();
    directory
}

#[test]
fn converter_is_installed_and_uses_the_verified_project_lock() {
    let job = job_directory();
    let project = microduck_training::bundle::project().unwrap();
    let command = deepmimic_conversion_command(
        "uv",
        &project,
        &job,
        None,
        &json!({
            "bundle_id":"fixture-bundle", "name":"Fixture", "playback":"finite",
        }),
    )
    .unwrap();
    let arguments: Vec<_> = command
        .get_args()
        .map(|value| value.to_string_lossy().into_owned())
        .collect();
    assert_eq!(&arguments[..3], &["run", "--frozen", "--project"]);
    assert!(
        arguments
            .windows(2)
            .any(|pair| pair == ["-m", "deepmimic.convert"])
    );
    for module in [
        "__init__.py",
        "schema.py",
        "convert.py",
        "motion.py",
        "recipe.py",
        "task.py",
        "rewards.py",
        "state.py",
        "reset.py",
    ] {
        assert!(
            microduck_training::bundle::modules(&project)
                .join("deepmimic")
                .join(module)
                .is_file()
        );
    }
    let parameters: serde_json::Value =
        serde_json::from_slice(&std::fs::read(job.join("conversion-parameters.json")).unwrap())
            .unwrap();
    assert_eq!(parameters["bundle_id"], "fixture-bundle");
    assert!(
        command
            .get_envs()
            .any(|(key, value)| key == "PYTHONPATH" && value.is_some())
    );
}

#[test]
fn changed_project_lock_fails_before_installing_python() {
    let job = job_directory();
    let project = job.join("project");
    std::fs::create_dir(&project).unwrap();
    std::fs::write(
        project.join("pyproject.toml"),
        "[project]\nname='changed'\n",
    )
    .unwrap();
    std::fs::write(project.join("uv.lock"), "version = 1\n").unwrap();
    let error = deepmimic_conversion_command("uv", &project, &job, None, &json!({})).unwrap_err();
    assert!(error.contains("锁定环境"));
    assert!(!job.join("python").exists());
}
