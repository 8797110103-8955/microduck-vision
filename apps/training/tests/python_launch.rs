#[path = "../../../.cargo/test_directory.rs"]
mod test_paths;

use test_paths::test_directory;

use microduck_training::launch::MjlabRun;
use serde_json::json;
use std::path::PathBuf;

fn directory() -> PathBuf {
    let path = test_directory()
        .join("training-refactor")
        .join(microduck_training::progress::new_training_run_id());
    std::fs::create_dir_all(&path).unwrap();
    path
}

#[test]
fn launch_materializes_only_the_fixed_bam_path_and_uses_profile_input() {
    let output = directory();
    let project = microduck_training::bundle::project().unwrap();
    let args = vec![
        "run".into(),
        "train".into(),
        "Mjlab-Velocity-Flat-MicroDuck".into(),
    ];
    let run = MjlabRun::new("uv", &args, &project, &output).unwrap();
    let environment = run.prepare(None).unwrap();
    let mut document = json!({"format_version":2,"profile":{"execution_fingerprint":"a"},
        "backend":{"kind":"mjlab-bam","version":1,"execution_fingerprint":"a",
            "bam_fit":{"file":"bam-fit.json","content":"{}"}}});
    for file in ["../bam-fit.json", "/bam-fit.json", "C:/bam-fit.json"] {
        document["backend"]["bam_fit"]["file"] = file.into();
        assert!(
            run.install_profile(&serde_json::to_vec(&document).unwrap())
                .is_err()
        );
        assert!(!output.join("simulation-profile.json").exists());
    }
    document["backend"]["bam_fit"]["file"] = "bam-fit.json".into();
    run.install_profile(&serde_json::to_vec(&document).unwrap())
        .unwrap();
    assert_eq!(
        std::fs::read_to_string(output.join("bam-fit.json")).unwrap(),
        "{}"
    );
    assert!(
        microduck_training::bundle::modules(&project)
            .join("training/body.py")
            .is_file()
    );
    assert!(!output.join("body-adapters").exists());
    let command = run.command("run", &environment, None).unwrap();
    assert!(command.get_args().any(|arg| arg == "--profile"));
    assert!(!command.get_args().any(|arg| arg == "--adapters"));
}
