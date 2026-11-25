//! Policy selection and ONNX inference.

use std::path::{Path, PathBuf};

pub(crate) mod catalog;
pub(crate) mod inference;
pub(crate) mod manifest;

pub(crate) const OFFICIAL_POLICIES: &str = "policies/official";
pub(crate) const TRAINED_POLICIES: &str = "policies/trained";

pub(crate) fn policy_directory(config: Option<&Path>, relative: &str) -> PathBuf {
    match config.and_then(Path::parent) {
        Some(root) => root.join(relative),
        None => std::env::temp_dir().join("microduck-vision").join(relative),
    }
}

pub(crate) fn publication_directories(root: &Path) -> Vec<PathBuf> {
    let Ok(root) = root.canonicalize() else {
        return Vec::new();
    };
    let mut directories: Vec<_> = std::fs::read_dir(&root)
        .into_iter()
        .flatten()
        .filter_map(Result::ok)
        .filter(|entry| entry.file_type().is_ok_and(|kind| kind.is_dir()))
        .filter(|entry| {
            entry
                .file_name()
                .to_str()
                .is_some_and(|name| !name.starts_with('.'))
        })
        .filter_map(|entry| entry.path().canonicalize().ok())
        .filter(|directory| directory.parent() == Some(root.as_path()))
        .collect();
    directories.sort_unstable();
    directories
}
