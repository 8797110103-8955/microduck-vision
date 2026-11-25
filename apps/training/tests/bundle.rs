#![cfg(not(feature = "packaged-python"))]

use microduck_training::bundle;
use std::path::Path;

#[test]
fn development_and_tests_load_python_directly_from_the_checkout() {
    let root = Path::new(env!("CARGO_MANIFEST_DIR"))
        .canonicalize()
        .unwrap();
    let project = bundle::project().unwrap();
    assert_eq!(project, root);
    assert!(project.join("uv.lock").is_file());
    let modules = bundle::modules(&project);
    assert_eq!(modules, root.join("python"));
    for file in [
        "train.py",
        "training_preview.py",
        "teacher_train.py",
        "teacher_evaluation.py",
        "training/tasks.py",
        "training/runtime.py",
        "deepmimic/convert.py",
    ] {
        assert!(modules.join(file).is_file(), "missing {file}");
    }
}
