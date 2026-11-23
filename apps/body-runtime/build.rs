use std::path::{Path, PathBuf};

#[cfg(feature = "native")]
#[path = "../../.cargo/output_paths.rs"]
mod output_paths;

#[cfg(feature = "native")]
mod mujoco {
    use std::env;
    use std::fs;
    use std::io;
    use std::path::{Path, PathBuf};
    use std::process::Command;

    use sha2::{Digest, Sha256};

    use super::output_paths::output_root;

    const VERSION: &str = "3.10.0";
    const HEADER_VERSION: &str = "3010000";
    const WINDOWS_ARCHIVE: &str = "mujoco-3.10.0-windows-x86_64.zip";
    const WINDOWS_SHA256: &str = "5d938c7558490d6f7332307212a41c9c5d5bf39009590e4ca5cf1ff0762da91c";
    const HEADER: &str = "include/mujoco/mujoco.h";
    const WINDOWS_FILES: &[&str] = &[
        HEADER,
        "lib/mujoco.lib",
        "bin/mujoco.dll",
        "LICENSE",
        "THIRD_PARTY_NOTICES.txt",
    ];

    pub(crate) fn prepare_and_link(out: &Path) {
        println!("cargo:rerun-if-env-changed=DUCK_MUJOCO_SDK");
        println!("cargo:rerun-if-env-changed=DUCK_MUJOCO_ZIP");
        let windows = env::var("CARGO_CFG_TARGET_OS").as_deref() == Ok("windows");
        let sdk = if let Some(custom) = env::var_os("DUCK_MUJOCO_SDK") {
            let custom = PathBuf::from(custom);
            assert!(custom.is_absolute(), "DUCK_MUJOCO_SDK must be absolute");
            assert!(
                !windows || complete_windows_sdk(&custom),
                "DUCK_MUJOCO_SDK must point to a complete MuJoCo {VERSION} SDK: {}",
                custom.display()
            );
            custom
        } else {
            assert!(
                cfg!(windows)
                    && windows
                    && env::var("CARGO_CFG_TARGET_ARCH").as_deref() == Ok("x86_64"),
                "set DUCK_MUJOCO_SDK to the absolute path of a MuJoCo {VERSION} SDK for this target"
            );
            let repo = Path::new(env!("CARGO_MANIFEST_DIR"))
                .ancestors()
                .nth(2)
                .expect("repository directory");
            // Cargo places OUT_DIR under <target>/<profile>/build/<package>/out.
            let target = out.ancestors().nth(4).expect("Cargo target directory");
            let cache = output_root(repo, target).join("mujoco").join(VERSION);
            let sdk = cache.join("sdk");
            if !complete_windows_sdk(&sdk) {
                install_windows_sdk(&cache, &sdk).unwrap_or_else(|error| {
                    panic!("prepare MuJoCo {VERSION} in {}: {error}", cache.display())
                });
            }
            sdk
        };

        let header = sdk.join(HEADER);
        let contents = fs::read_to_string(&header).unwrap_or_else(|error| {
            panic!("read MuJoCo {VERSION} header {}: {error}", header.display())
        });
        assert!(
            contents.lines().any(|line| line.split_whitespace().eq([
                "#define",
                "mjVERSION_HEADER",
                HEADER_VERSION
            ])),
            "the Rust FFI bindings require the MuJoCo {VERSION} SDK"
        );
        if windows {
            for file in WINDOWS_FILES {
                println!("cargo:rerun-if-changed={}", sdk.join(file).display());
            }
        } else {
            println!("cargo:rerun-if-changed={}", header.display());
        }
        println!(
            "cargo:rustc-link-search=native={}",
            sdk.join("lib").display()
        );
        println!("cargo:rustc-link-lib=dylib=mujoco");
        println!("cargo::metadata=sdk={}", sdk.display());
    }

    fn complete_windows_sdk(sdk: &Path) -> bool {
        WINDOWS_FILES.iter().all(|file| sdk.join(file).is_file())
    }

    fn install_windows_sdk(cache: &Path, sdk: &Path) -> io::Result<()> {
        fs::create_dir_all(cache)?;
        let staging = tempfile::Builder::new()
            .prefix("install-")
            .tempdir_in(cache)?;
        let cached_archive = cache.join("download.zip");
        let archive = if let Some(custom) = env::var_os("DUCK_MUJOCO_ZIP") {
            PathBuf::from(custom)
        } else if cached_archive.is_file() {
            cached_archive
        } else {
            let archive = staging.path().join(WINDOWS_ARCHIVE);
            let url = format!(
                "https://github.com/google-deepmind/mujoco/releases/download/{VERSION}/{WINDOWS_ARCHIVE}"
            );
            run(
                Command::new(system_tool("curl.exe"))
                    .args(["--fail", "--location", "--retry", "3", "--output"])
                    .arg(&archive)
                    .arg(url),
                "download pinned MuJoCo SDK",
            )?;
            archive
        };
        let hash = format!("{:x}", Sha256::digest(fs::read(&archive)?));
        if hash != WINDOWS_SHA256 {
            return Err(io::Error::other(format!(
                "MuJoCo archive checksum mismatch: {}",
                archive.display()
            )));
        }
        let extracted = staging.path().join("sdk");
        fs::create_dir(&extracted)?;
        run(
            Command::new(system_tool("tar.exe"))
                .arg("-xf")
                .arg(&archive)
                .arg("-C")
                .arg(&extracted),
            "extract MuJoCo SDK",
        )?;
        if !complete_windows_sdk(&extracted) {
            return Err(io::Error::other(format!(
                "incomplete MuJoCo SDK archive: {}",
                archive.display()
            )));
        }
        match fs::rename(extracted, sdk) {
            Ok(()) => Ok(()),
            Err(_) if complete_windows_sdk(sdk) => Ok(()),
            Err(error) => Err(io::Error::other(format!(
                "install SDK in {}: {error}",
                sdk.display()
            ))),
        }
    }

    fn system_tool(name: &str) -> PathBuf {
        env::var_os("SystemRoot")
            .map(PathBuf::from)
            .unwrap_or_else(|| PathBuf::from(r"C:\Windows"))
            .join("System32")
            .join(name)
    }

    fn run(command: &mut Command, operation: &str) -> io::Result<()> {
        let status = command.status()?;
        if !status.success() {
            return Err(io::Error::other(format!("{operation}: {status}")));
        }
        Ok(())
    }
}

fn gather(root: &Path, directory: &Path, entries: &mut Vec<(String, PathBuf)>) {
    println!("cargo:rerun-if-changed={}", directory.display());
    for item in std::fs::read_dir(directory).expect("read pinned MicroDuck assets") {
        let item = item.expect("read pinned MicroDuck asset entry");
        let path = item.path();
        if item.file_type().expect("asset file type").is_dir() {
            gather(root, &path, entries);
        } else if path
            .extension()
            .is_some_and(|ext| ext == "xml" || ext == "stl")
        {
            let id = path
                .strip_prefix(root)
                .expect("asset under source directory")
                .to_str()
                .expect("UTF-8 resource name")
                .replace('\\', "/");
            println!("cargo:rerun-if-changed={}", path.display());
            entries.push((id, path));
        }
    }
}

fn main() {
    let out = PathBuf::from(std::env::var_os("OUT_DIR").expect("OUT_DIR"));
    #[cfg(feature = "native")]
    mujoco::prepare_and_link(&out);
    let root = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../vendor/microduck_rl/src/mjlab_microduck/robot/microduck");
    let mut entries = Vec::new();
    gather(&root, &root, &mut entries);
    entries.sort_unstable_by(|a, b| a.0.cmp(&b.0));
    let rows = entries
        .into_iter()
        .map(|(id, path)| {
            format!(
                "    ({id:?}, include_bytes!({:?})),\n",
                path.to_str().unwrap()
            )
        })
        .collect::<String>();
    std::fs::write(out.join("official_assets.rs"), format!("&[\n{rows}]\n"))
        .expect("write embedded MicroDuck asset table");
}
