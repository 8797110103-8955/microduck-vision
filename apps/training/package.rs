//! Build-time staging for the Python sidecar directory.

use std::{
    fs, io,
    path::{Path, PathBuf},
};

pub fn write_bundle(destination: &Path, files: &[(PathBuf, PathBuf)], id: &str) -> io::Result<()> {
    // This directory is generated output owned exclusively by this packager.
    fs::create_dir_all(destination)?;
    let destination = destination.canonicalize()?;
    if destination.file_name().and_then(|name| name.to_str()) != Some("python") {
        return Err(io::Error::other(
            "Python output directory must be named python",
        ));
    }
    // Separate Cargo invocations may build the same resources while Python is
    // reading them. Reuse complete, identical output without touching its files.
    if fs::read_to_string(destination.join(".bundle-id")).is_ok_and(|value| value == id)
        && files.iter().all(|(relative, source)| {
            fs::read(source)
                .ok()
                .zip(fs::read(destination.join(relative)).ok())
                .is_some_and(|(source, staged)| source == staged)
        })
    {
        return Ok(());
    }
    fs::remove_dir_all(&destination)?;
    fs::create_dir_all(&destination)?;
    for (relative, source) in files {
        let target = destination.join(relative);
        fs::create_dir_all(target.parent().expect("Python resource parent"))?;
        fs::copy(source, target)?;
    }
    fs::write(destination.join(".bundle-id"), id)
}

pub fn copy_bundle(source: &Path, destination: &Path) -> io::Result<()> {
    fn collect(root: &Path, path: &Path, files: &mut Vec<(PathBuf, PathBuf)>) -> io::Result<()> {
        for entry in fs::read_dir(path)? {
            let entry = entry?;
            let path = entry.path();
            if entry.file_type()?.is_dir() {
                if entry.file_name() != "__pycache__" {
                    collect(root, &path, files)?;
                }
            } else if entry.file_name() != ".bundle-id" {
                files.push((
                    path.strip_prefix(root)
                        .expect("Python bundle file")
                        .to_path_buf(),
                    path,
                ));
            }
        }
        Ok(())
    }
    let id = fs::read_to_string(source.join(".bundle-id"))?;
    let mut files = Vec::new();
    collect(source, source, &mut files)?;
    write_bundle(destination, &files, &id)
}
