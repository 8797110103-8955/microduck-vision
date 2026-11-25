use std::path::{Component, Path, PathBuf};

pub fn output_root(repo: &Path, target_dir: &Path) -> PathBuf {
    let canonical_repo = repo.canonicalize().expect("repository directory");
    let target = target_dir.canonicalize().expect("Cargo target directory");
    if let Ok(relative) = target.strip_prefix(canonical_repo)
        && let Some(Component::Normal(first)) = relative.components().next()
    {
        return repo.join(first);
    }
    // An external target directory already provides an explicit output boundary.
    target_dir.to_path_buf()
}

pub fn export_test_directory(repo: &Path, output: &Path) {
    println!(
        "cargo:rerun-if-changed={}",
        repo.join(".cargo/output_paths.rs").display()
    );
    println!(
        "cargo:rustc-env=MICRODUCK_TEST_DIR={}",
        output.join("tests").display()
    );
}
