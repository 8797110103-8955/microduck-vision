use sha2::{Digest, Sha256};
use std::{
    env, fs,
    path::{Path, PathBuf},
};

#[allow(dead_code)]
mod package;

#[path = "../../.cargo/output_paths.rs"]
mod output_paths;

fn collect(root: &Path, directory: &Path, files: &mut Vec<(String, PathBuf)>) {
    println!("cargo:rerun-if-changed={}", directory.display());
    let mut entries: Vec<_> = fs::read_dir(directory)
        .unwrap()
        .map(Result::unwrap)
        .collect();
    entries.sort_by_key(|entry| entry.file_name());
    for entry in entries {
        let path = entry.path();
        if entry.file_type().unwrap().is_dir() {
            if !matches!(
                entry.file_name().to_str(),
                Some("__pycache__" | "tests" | ".pytest_cache")
            ) {
                collect(root, &path, files);
            }
        } else if !matches!(
            path.extension().and_then(|value| value.to_str()),
            Some("pyc" | "pyo")
        ) {
            files.push((
                path.strip_prefix(root)
                    .unwrap()
                    .to_str()
                    .unwrap()
                    .replace('\\', "/"),
                path,
            ));
        }
    }
}

fn main() {
    let root = PathBuf::from(env::var_os("CARGO_MANIFEST_DIR").unwrap());
    let repo = root.ancestors().nth(2).expect("repository directory");
    let out = PathBuf::from(env::var_os("OUT_DIR").expect("OUT_DIR"));
    let profile = out.ancestors().nth(3).expect("Cargo profile directory");
    let target = profile.parent().expect("Cargo target directory");
    output_paths::export_test_directory(repo, &output_paths::output_root(repo, target));
    let upstream_project = repo.join("vendor/microduck_rl");
    let mut application = Vec::new();
    collect(&root, &root.join("python"), &mut application);
    for name in ["pyproject.toml", "uv.lock"] {
        let path = root.join(name);
        println!("cargo:rerun-if-changed={}", path.display());
        application.push((name.to_owned(), path));
    }
    let mut upstream = Vec::new();
    collect(
        &upstream_project,
        &upstream_project.join("src"),
        &mut upstream,
    );
    for name in [
        "pyproject.toml",
        "README.md",
        "LICENSE",
        "LICENSE-MODELS.md",
    ] {
        let path = upstream_project.join(name);
        println!("cargo:rerun-if-changed={}", path.display());
        upstream.push((name.to_owned(), path));
    }
    let mut hash = Sha256::new();
    let mut resources = Vec::new();
    // Preserve the local dependency path from the application's pyproject.toml.
    for (name, files) in [
        ("apps/training", application),
        ("vendor/microduck_rl", upstream),
    ] {
        hash.update(name);
        for (relative, path) in files {
            let bytes = fs::read(&path).unwrap();
            hash.update((relative.len() as u64).to_le_bytes());
            hash.update(relative.as_bytes());
            hash.update((bytes.len() as u64).to_le_bytes());
            hash.update(&bytes);
            resources.push((PathBuf::from(name).join(relative), path));
        }
    }
    let id = format!("{:x}", hash.finalize());
    let lock_hash = format!(
        "{:x}",
        Sha256::digest(fs::read(root.join("uv.lock")).unwrap())
    );
    println!("cargo:rustc-env=MICRODUCK_PYTHON_BUNDLE_ID={id}");
    println!("cargo:rustc-env=MICRODUCK_PYTHON_LOCK_SHA256={lock_hash}");
    if env::var_os("CARGO_FEATURE_PACKAGED_PYTHON").is_some() {
        package::write_bundle(&profile.join("python"), &resources, &id)
            .expect("stage Python resources");
    }
}
