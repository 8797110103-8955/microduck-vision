use std::fs::File;
use std::io;
use std::path::{Path, PathBuf};
use std::process::Command;
use std::thread;
use std::time::Duration;

const ORT_VERSION: &str = "1.28.0";
const ORT_SHA256: &str = "abef733dacbe2f571547a7150b479b5cb9cc0df22f96c24983a42cadb1b4f8bc";
const DOWNLOAD_ATTEMPTS: u8 = 3;

fn main() {
    let attrs = tauri_build::Attributes::new().windows_attributes(
        tauri_build::WindowsAttributes::new_without_app_manifest(),
    );
    tauri_build::try_build(attrs).expect("tauri build");

    let windows = std::env::var("CARGO_CFG_TARGET_OS").as_deref() == Ok("windows");
    let msvc = std::env::var("CARGO_CFG_TARGET_ENV").as_deref() == Ok("msvc");
    if windows && msvc {
        let assets = Path::new(env!("CARGO_MANIFEST_DIR")).join("..").join("assets");
        let manifest = assets.join("windows-app-manifest.xml");
        println!("cargo:rerun-if-changed={}", manifest.display());
        println!("cargo:rustc-link-arg=/MANIFEST:EMBED");
        println!("cargo:rustc-link-arg=/MANIFESTINPUT:{}", manifest.display());
    }

    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:rerun-if-env-changed=DUCK_ORT_ZIP");
    if windows {
        deploy_onnxruntime();
    }
}

fn deploy_onnxruntime() {
    let out = PathBuf::from(std::env::var("OUT_DIR").expect("OUT_DIR"));
    let profile = out
        .ancestors()
        .nth(3)
        .expect("OUT_DIR layout")
        .to_path_buf();
    let cache = profile
        .parent()
        .expect("profile dir has a parent")
        .join("onnxruntime")
        .join(ORT_VERSION);
    let dll = cache.join("onnxruntime.dll");

    if !dll.is_file()
        && let Err(error) = fetch_onnxruntime(&cache, &dll)
    {
        panic!(
            "\n准备 ONNX Runtime {ORT_VERSION} 失败：{error}\n\
             没有网络时，手动下载 {}\n\
             再用环境变量 DUCK_ORT_ZIP 指向这个 zip 重新构建。\n",
            ort_url()
        );
    }

    println!("cargo:rustc-env=DUCK_ORT_VERSION={ORT_VERSION}");
    for target in [out.join("onnxruntime.dll"), profile.join("deps").join("onnxruntime.dll")] {
        let dir = target.parent().expect("target has a parent");
        if !target.is_file()
            || std::fs::metadata(&target).map(|metadata| metadata.len()).ok()
                != std::fs::metadata(&dll).map(|metadata| metadata.len()).ok()
        {
            std::fs::create_dir_all(dir).expect("create ONNX Runtime output directory");
            std::fs::copy(&dll, &target)
                .unwrap_or_else(|error| panic!("复制 {} 失败：{error}", target.display()));
        }
        println!("cargo:rerun-if-changed={}", target.display());
    }
}

fn ort_url() -> String {
    format!(
        "https://github.com/microsoft/onnxruntime/releases/download/v{ORT_VERSION}/onnxruntime-win-x64-{ORT_VERSION}.zip"
    )
}

fn system_tool(name: &str) -> PathBuf {
    let root = std::env::var_os("SystemRoot").unwrap_or_else(|| "C:\\Windows".into());
    PathBuf::from(root).join("System32").join(name)
}

fn fetch_onnxruntime(cache: &Path, dll: &Path) -> Result<(), String> {
    std::fs::create_dir_all(cache).map_err(|error| error.to_string())?;
    let zip = match std::env::var_os("DUCK_ORT_ZIP") {
        Some(path) => PathBuf::from(path),
        None => {
            let zip = cache.join("download.zip");
            download_file(&ort_url(), &zip)?;
            zip
        }
    };

    let bytes = std::fs::read(&zip).map_err(|error| format!("读不到 {}：{error}", zip.display()))?;
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
            zip.display()
        ));
    }

    let unpack = cache.join("unpack");
    let _ = std::fs::remove_dir_all(&unpack);
    std::fs::create_dir_all(&unpack).map_err(|error| error.to_string())?;
    let status = Command::new(system_tool("tar.exe"))
        .args(["-xf"])
        .arg(&zip)
        .args(["-C"])
        .arg(&unpack)
        .status()
        .map_err(|error| format!("无法运行 tar：{error}"))?;
    if !status.success() {
        return Err(format!("解压失败（tar {status}）"));
    }
    let inner = unpack
        .join(format!("onnxruntime-win-x64-{ORT_VERSION}"))
        .join("lib")
        .join("onnxruntime.dll");
    std::fs::copy(&inner, dll).map_err(|error| format!("取出 onnxruntime.dll 失败：{error}"))?;
    let _ = std::fs::remove_dir_all(&unpack);
    if std::env::var_os("DUCK_ORT_ZIP").is_none() {
        let _ = std::fs::remove_file(zip);
    }
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
            let mut file = File::create(target).map_err(|error| format!("创建缓存文件失败：{error}"))?;
            io::copy(&mut response, &mut file).map_err(|error| format!("写入缓存文件失败：{error}"))?;
            Ok::<(), String>(())
        })();

        match result {
            Ok(()) => return Ok(()),
            Err(error) => {
                last_error = error;
                let _ = std::fs::remove_file(target);
                if attempt < DOWNLOAD_ATTEMPTS {
                    thread::sleep(Duration::from_millis(250 * u64::from(attempt)));
                }
            }
        }
    }

    Err(format!("下载失败（尝试 {DOWNLOAD_ATTEMPTS} 次）：{last_error}"))
}
