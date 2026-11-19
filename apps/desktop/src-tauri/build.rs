use std::fs::File;
use std::io;
use std::path::{Path, PathBuf};
use std::process::Command;
use std::thread;
use std::time::Duration;

const ORT_VERSION: &str = "1.28.0";
const ORT_SHA256: &str = "769d1d3ea8ab6cd69f737c9dd4d4462aa4ad0ccfa106eaf506efc40d7bead5db";
const DOWNLOAD_ATTEMPTS: u8 = 3;

fn executable_directory(profile: &Path) -> PathBuf {
    std::env::var_os("DUCK_RELEASE_OUTPUT")
        .map(PathBuf::from)
        .unwrap_or_else(|| profile.to_path_buf())
}

fn workspace_target_directory() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .ancestors()
        .nth(3)
        .expect("desktop crate is under apps/desktop/src-tauri")
        .join("target")
}

fn main() {
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
        println!("cargo:rustc-link-arg=/MANIFEST:EMBED");
        println!("cargo:rustc-link-arg=/MANIFESTINPUT:{}", manifest.display());
    }

    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:rerun-if-env-changed=DUCK_ORT_ZIP");
    println!("cargo:rerun-if-env-changed=DUCK_RELEASE_OUTPUT");
    if windows {
        deploy_onnxruntime();
        deploy_mujoco();
    }
}

// The C binding is built from these exact headers and the matching runtime is deployed with
// the executable. Override only the archive location, never its version or checksum.
fn deploy_mujoco() {
    const VERSION: &str = "3.10.0";
    const SHA: &str = "5d938c7558490d6f7332307212a41c9c5d5bf39009590e4ca5cf1ff0762da91c";
    let out = PathBuf::from(std::env::var_os("OUT_DIR").expect("OUT_DIR"));
    let profile = out.ancestors().nth(3).expect("OUT_DIR layout");
    let cache = workspace_target_directory().join("mujoco").join(VERSION);
    let sdk = cache.join("sdk");
    println!("cargo:rerun-if-env-changed=DUCK_MUJOCO_ZIP");
    println!("cargo:rerun-if-changed=native/mujoco_bridge.c");
    if !sdk.join("include/mujoco/mujoco.h").is_file() {
        std::fs::create_dir_all(&sdk).expect("create MuJoCo SDK cache");
        let archive = match std::env::var_os("DUCK_MUJOCO_ZIP") {
            Some(path) => PathBuf::from(path),
            None => {
                let path = cache.join("download.zip");
                if !path.is_file() {
                    download_file(&format!("https://github.com/google-deepmind/mujoco/releases/download/{VERSION}/mujoco-{VERSION}-windows-x86_64.zip"), &path).expect("download pinned MuJoCo SDK");
                }
                path
            }
        };
        use sha2::Digest;
        let hash = format!(
            "{:x}",
            sha2::Sha256::digest(std::fs::read(&archive).expect("read MuJoCo archive"))
        );
        assert_eq!(hash, SHA, "MuJoCo archive checksum mismatch");
        let status = Command::new(system_tool("tar.exe"))
            .arg("-xf")
            .arg(&archive)
            .arg("-C")
            .arg(&sdk)
            .status()
            .expect("extract MuJoCo SDK");
        assert!(status.success(), "MuJoCo SDK extraction failed");
        assert!(
            sdk.join("include/mujoco/mujoco.h").is_file(),
            "MuJoCo SDK layout mismatch"
        );
    }
    cc::Build::new()
        .file("native/mujoco_bridge.c")
        .include(sdk.join("include"))
        .debug(false)
        .warnings(true)
        .compile("duck_mujoco_bridge");
    println!(
        "cargo:rustc-link-search=native={}",
        sdk.join("lib").display()
    );
    println!("cargo:rustc-link-lib=dylib=mujoco");
    println!("cargo:rustc-env=DUCK_MUJOCO_VERSION={VERSION}");
    let root = Path::new(env!("CARGO_MANIFEST_DIR"));
    let output = executable_directory(profile);
    let dll = sdk.join("bin/mujoco.dll");
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
    // Copy the pinned vendor model at build time. The source of truth stays in vendor;
    // the executable profile receives a runnable copy with its relative include/mesh paths.
    let assets = root.join("../../../vendor/microduck_rl/src/mjlab_microduck/robot/microduck");
    let model_license = root.join("../../../vendor/microduck_rl/LICENSE-MODELS.md");
    let deployed = output.join("resources/microduck");
    println!("cargo:rerun-if-changed={}", deployed.display());
    if deployed.is_dir() {
        std::fs::remove_dir_all(&deployed).expect("clear deployed model resources");
    }
    copy_model_resources(&assets, &deployed, &model_license);
}

fn copy_model_resources(source: &Path, destination: &Path, model_license: &Path) {
    std::fs::create_dir_all(destination).expect("create model asset directory");
    for entry in std::fs::read_dir(source).expect("read official model resources") {
        let entry = entry.expect("model resource");
        let path = entry.path();
        let target = destination.join(entry.file_name());
        if path.is_dir() {
            copy_model_resources(&path, &target, model_license);
        } else if path.extension().is_some_and(|e| e == "xml" || e == "stl") {
            println!("cargo:rerun-if-changed={}", path.display());
            if !same_file(&path, &target) {
                std::fs::copy(path, target).expect("deploy model resource");
            }
        }
    }
    let license_target = destination.join("LICENSE-MODELS.md");
    if source
        == Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../../vendor/microduck_rl/src/mjlab_microduck/robot/microduck")
        && !same_file(model_license, &license_target)
    {
        println!("cargo:rerun-if-changed={}", model_license.display());
        std::fs::copy(model_license, license_target).expect("deploy model license");
    }
}

fn deploy_onnxruntime() {
    let out = PathBuf::from(std::env::var("OUT_DIR").expect("OUT_DIR"));
    let profile = out
        .ancestors()
        .nth(3)
        .expect("OUT_DIR layout")
        .to_path_buf();
    let cache = workspace_target_directory()
        .join("onnxruntime")
        .join(ORT_VERSION);
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
    let output = executable_directory(&profile);
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
