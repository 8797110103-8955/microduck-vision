use std::fs::File;
use std::io;
use std::path::{Path, PathBuf};
use std::process::Command;
use std::thread;
use std::time::Duration;

#[path = "../../../.cargo/output_paths.rs"]
mod output_paths;

use output_paths::output_root;

const ORT_VERSION: &str = "1.28.0";
const ORT_SHA256: &str = "769d1d3ea8ab6cd69f737c9dd4d4462aa4ad0ccfa106eaf506efc40d7bead5db";
const DOWNLOAD_ATTEMPTS: u8 = 3;

fn main() {
    let out = PathBuf::from(std::env::var_os("OUT_DIR").expect("OUT_DIR"));
    let repo = Path::new(env!("CARGO_MANIFEST_DIR"))
        .ancestors()
        .nth(3)
        .expect("repository directory");
    let profile = out.ancestors().nth(3).expect("Cargo profile directory");
    let target = profile.parent().expect("Cargo target directory");
    let root = output_root(repo, target);
    println!(
        "cargo:rustc-env=DUCK_DESKTOP_TEST_DIR={}",
        root.join("tests").display()
    );
    let release_dir = root.join("bin");
    let output = if cfg!(feature = "desktop-distribution") {
        println!(
            "cargo:rustc-env=DUCK_DESKTOP_RELEASE_DIR={}",
            release_dir.display()
        );
        release_dir.as_path()
    } else {
        profile
    };

    let attrs = tauri_build::Attributes::new()
        .windows_attributes(tauri_build::WindowsAttributes::new_without_app_manifest());
    tauri_build::try_build(attrs).expect("tauri build");

    let windows = std::env::var("CARGO_CFG_TARGET_OS").as_deref() == Ok("windows");
    let msvc = std::env::var("CARGO_CFG_TARGET_ENV").as_deref() == Ok("msvc");
    if windows && msvc {
        let assets = Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("..")
            .join("assets");
        let manifest = assets.join("windows-app-manifest.xml");
        println!("cargo:rerun-if-changed={}", manifest.display());
        println!("cargo:rustc-link-arg-bin=microduck-vision=/MANIFEST:EMBED");
        println!(
            "cargo:rustc-link-arg-bin=microduck-vision=/MANIFESTINPUT:{}",
            manifest.display()
        );
    }

    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:rerun-if-env-changed=DUCK_ORT_ZIP");
    if windows {
        deploy_onnxruntime(&out, profile, &root, output);
        deploy_mujoco(profile, output);
    }
}

// The product package links MuJoCo. The desktop deploys the same SDK runtime.
fn deploy_mujoco(profile: &Path, output: &Path) {
    let sdk = PathBuf::from(
        std::env::var_os("DEP_MUJOCO_SDK").expect("body-runtime must provide the MuJoCo SDK"),
    );
    let root = Path::new(env!("CARGO_MANIFEST_DIR"));
    let dll = sdk.join("bin/mujoco.dll");
    println!("cargo:rerun-if-changed={}", dll.display());
    for destination in [
        output.join("mujoco.dll"),
        profile.join("mujoco.dll"),
        profile.join("deps/mujoco.dll"),
    ] {
        println!("cargo:rerun-if-changed={}", destination.display());
        if let Some(parent) = destination.parent() {
            std::fs::create_dir_all(parent).expect("create native output directory");
        }
        if !same_file(&dll, &destination) {
            std::fs::copy(&dll, &destination).expect("deploy MuJoCo runtime");
        }
    }
    let notices = output.join("resources/licenses/mujoco");
    std::fs::create_dir_all(&notices).expect("create native dependency license directory");
    for name in ["LICENSE", "THIRD_PARTY_NOTICES.txt"] {
        std::fs::copy(sdk.join(name), notices.join(name)).expect("deploy MuJoCo license notices");
        println!("cargo:rerun-if-changed={}", notices.join(name).display());
    }
    // Three.js loads STL through the Tauri asset protocol. Rust uses embedded MJCF and meshes.
    let assets =
        root.join("../../../vendor/microduck_rl/src/mjlab_microduck/robot/microduck/assets");
    let model_license = root.join("../../../vendor/microduck_rl/LICENSE-MODELS.md");
    let deployed = output.join("resources/microduck");
    if let Ok(entries) = std::fs::read_dir(&deployed) {
        for entry in entries {
            let entry = entry.expect("read old deployed model resource");
            if entry.file_type().expect("model resource type").is_file()
                && entry
                    .path()
                    .extension()
                    .is_some_and(|extension| extension == "xml")
            {
                std::fs::remove_file(entry.path()).expect("remove old deployed MuJoCo XML");
            }
        }
    }
    copy_display_meshes(&assets, &deployed.join("assets"));
    println!("cargo:rerun-if-changed={}", model_license.display());
    let license_target = deployed.join("LICENSE-MODELS.md");
    if !same_file(&model_license, &license_target) {
        std::fs::copy(model_license, license_target).expect("deploy model license");
    }
}

fn copy_display_meshes(source: &Path, destination: &Path) {
    println!("cargo:rerun-if-changed={}", source.display());
    std::fs::create_dir_all(destination).expect("create model asset directory");
    for entry in std::fs::read_dir(source).expect("read official display meshes") {
        let entry = entry.expect("display mesh");
        let path = entry.path();
        let target = destination.join(entry.file_name());
        if entry.file_type().expect("display mesh type").is_dir() {
            copy_display_meshes(&path, &target);
        } else if path.extension().is_some_and(|extension| extension == "stl") {
            println!("cargo:rerun-if-changed={}", path.display());
            if !same_file(&path, &target) {
                std::fs::copy(path, target).expect("deploy display mesh");
            }
        }
    }
}

fn deploy_onnxruntime(out: &Path, profile: &Path, root: &Path, output: &Path) {
    let cache = root.join("onnxruntime").join(ORT_VERSION);
    let dll = cache.join("onnxruntime.dll");

    if !dll.is_file()
        && let Err(error) = fetch_onnxruntime(&cache, &dll)
    {
        panic!(
            "\n准备 ONNX Runtime {ORT_VERSION} 失败：{error}\n\
             没有网络时，手动下载 {}\n\
             保存到 {}，或用环境变量 DUCK_ORT_ZIP 指向这个 nupkg 重新构建。\n",
            ort_url(),
            cache.join("download.nupkg").display(),
        );
    }

    println!("cargo:rustc-env=DUCK_ORT_VERSION={ORT_VERSION}");
    // Tauri launches the executable from the profile root. Keep a copy there in addition to
    // the build-script and deps directories used by cargo test and direct crate binaries.
    for target in [
        out.join("onnxruntime.dll"),
        output.join("onnxruntime.dll"),
        profile.join("onnxruntime.dll"),
        profile.join("deps").join("onnxruntime.dll"),
    ] {
        let dir = target.parent().expect("target has a parent");
        if !same_file(&dll, &target) {
            std::fs::create_dir_all(dir).expect("create ONNX Runtime output directory");
            std::fs::copy(&dll, &target)
                .unwrap_or_else(|error| panic!("复制 {} 失败：{error}", target.display()));
        }
        println!("cargo:rerun-if-changed={}", target.display());
    }
}

fn same_file(source: &Path, target: &Path) -> bool {
    match (std::fs::read(source), std::fs::read(target)) {
        (Ok(source), Ok(target)) => source == target,
        _ => false,
    }
}

fn ort_url() -> String {
    format!(
        "https://api.nuget.org/v3-flatcontainer/microsoft.ml.onnxruntime/{ORT_VERSION}/microsoft.ml.onnxruntime.{ORT_VERSION}.nupkg"
    )
}

fn system_tool(name: &str) -> PathBuf {
    let root = std::env::var_os("SystemRoot").unwrap_or_else(|| "C:\\Windows".into());
    PathBuf::from(root).join("System32").join(name)
}

fn fetch_onnxruntime(cache: &Path, dll: &Path) -> Result<(), String> {
    std::fs::create_dir_all(cache).map_err(|error| error.to_string())?;
    let package = match std::env::var_os("DUCK_ORT_ZIP") {
        Some(path) => PathBuf::from(path),
        None => {
            let package = cache.join("download.nupkg");
            if !package.is_file() {
                download_file(&ort_url(), &package)?;
            }
            package
        }
    };

    let bytes = std::fs::read(&package)
        .map_err(|error| format!("读不到 {}：{error}", package.display()))?;
    let got = {
        use sha2::Digest;
        sha2::Sha256::digest(&bytes)
            .iter()
            .map(|byte| format!("{byte:02x}"))
            .collect::<String>()
    };
    if got != ORT_SHA256 {
        return Err(format!(
            "{} 的 SHA-256 是 {got}，应为 {ORT_SHA256}",
            package.display()
        ));
    }

    let unpack = cache.join("unpack");
    let _ = std::fs::remove_dir_all(&unpack);
    std::fs::create_dir_all(&unpack).map_err(|error| error.to_string())?;
    let status = Command::new(system_tool("tar.exe"))
        .args(["-xf"])
        .arg(&package)
        .args(["-C"])
        .arg(&unpack)
        .status()
        .map_err(|error| format!("无法运行 tar：{error}"))?;
    if !status.success() {
        return Err(format!("解压失败（tar {status}）"));
    }
    let inner = unpack.join("runtimes/win-x64/native/onnxruntime.dll");
    std::fs::copy(&inner, dll).map_err(|error| format!("取出 onnxruntime.dll 失败：{error}"))?;
    let _ = std::fs::remove_dir_all(&unpack);
    Ok(())
}

fn download_file(url: &str, target: &Path) -> Result<(), String> {
    let client = reqwest::blocking::Client::builder()
        .user_agent("microduck-build/0.1")
        .connect_timeout(Duration::from_secs(30))
        .timeout(Duration::from_secs(600))
        .build()
        .map_err(|error| format!("创建下载客户端失败：{error}"))?;
    let mut last_error = String::from("未知下载错误");

    for attempt in 1..=DOWNLOAD_ATTEMPTS {
        let result = (|| {
            let mut response = client
                .get(url)
                .send()
                .map_err(|error| format!("请求失败：{error}"))?
                .error_for_status()
                .map_err(|error| format!("服务器返回错误：{error}"))?;
            let mut file =
                File::create(target).map_err(|error| format!("创建缓存文件失败：{error}"))?;
            io::copy(&mut response, &mut file)
                .map_err(|error| format!("写入缓存文件失败：{error}"))?;
            Ok::<(), String>(())
        })();

        match result {
            Ok(()) => return Ok(()),
            Err(error) => {
                let _ = std::fs::remove_file(target);
                last_error = error;
                let _ = std::fs::remove_file(target);
                if attempt < DOWNLOAD_ATTEMPTS {
                    thread::sleep(Duration::from_millis(250 * u64::from(attempt)));
                }
            }
        }
    }

    Err(format!(
        "下载失败（尝试 {DOWNLOAD_ATTEMPTS} 次）：{last_error}"
    ))
}
