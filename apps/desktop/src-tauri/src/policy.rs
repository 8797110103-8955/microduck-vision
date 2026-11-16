//! Official ONNX policy files owned by the desktop client.
//!
//! The control loop only receives a selected virtual machine and a servo driver.
//! Downloads and cache layout stay here so a policy can be replaced without
//! changing the loop-body crate.

use std::fs::File;
use std::io;
use std::path::{Path, PathBuf};
use std::thread;
use std::time::{Duration, SystemTime};

use serde::Deserialize;

use crate::types::{PolicyCommand, PolicyEntry};

const CACHE_DIR: &str = "library/policies/official";
const MANIFEST_FILE: &str = "manifest.json";
const MANIFEST_URL: &str =
    "https://huggingface.co/pollen-robotics/microduck-policies/resolve/main/manifest.json";
const SELECTED_FILE: &str = ".selected";
const DOWNLOAD_ATTEMPTS: u8 = 3;
const MANIFEST_MAX_AGE: Duration = Duration::from_secs(60 * 60);

#[derive(Debug, Clone, Deserialize)]
struct OfficialManifest {
    policies: Vec<ManifestPolicy>,
}

#[derive(Debug, Clone, Deserialize)]
struct ManifestPolicy {
    file: String,
    #[serde(default)]
    name: Option<String>,
    #[serde(default)]
    kind: Option<String>,
    #[serde(default)]
    slot: Option<String>,
    #[serde(default)]
    duration_s: Option<f64>,
    #[serde(default)]
    command: Option<PolicyCommand>,
}

#[derive(Debug, Clone)]
struct OfficialSource {
    id: String,
    name: String,
    file: String,
    url: String,
    kind: Option<String>,
    slot: Option<String>,
    duration_s: Option<f64>,
    command: Option<PolicyCommand>,
}

pub(crate) fn cache_root(config_dir: Option<&Path>) -> PathBuf {
    config_dir
        .and_then(Path::parent)
        .map(|root| root.join(CACHE_DIR))
        .unwrap_or_else(|| {
            std::env::temp_dir()
                .join("microduck-vision")
                .join(CACHE_DIR)
        })
}

pub(crate) fn list(config_dir: Option<&Path>) -> Result<Vec<PolicyEntry>, String> {
    let root = cache_root(config_dir);
    let selected = read_selected(&root);
    let sources = sources(config_dir)?;
    Ok(sources
        .iter()
        .map(|source| {
            let path = root.join(&source.file);
            let size = std::fs::metadata(&path)
                .map(|metadata| metadata.len())
                .unwrap_or(0);
            PolicyEntry {
                id: source.id.clone(),
                name: source.name.clone(),
                file: source.file.clone(),
                source_url: source.url.clone(),
                cached: path.is_file() && size > 0,
                selected: selected.as_deref() == Some(&source.id),
                size,
                kind: source.kind.clone(),
                slot: source.slot.clone(),
                duration_s: source.duration_s,
                command: source.command.clone(),
            }
        })
        .collect())
}

pub(crate) fn download(config_dir: Option<&Path>, id: &str) -> Result<PolicyEntry, String> {
    let source = sources(config_dir)?
        .into_iter()
        .find(|source| source.id == id)
        .ok_or_else(|| format!("unknown official policy: {id}"))?;
    let root = cache_root(config_dir);
    std::fs::create_dir_all(&root).map_err(|error| error.to_string())?;
    let temporary = root.join(format!(".{}.download", source.file));
    let _ = std::fs::remove_file(&temporary);

    download_file(&source.url, &temporary)?;

    let size = std::fs::metadata(&temporary)
        .map_err(|error| format!("策略文件不可读：{error}"))?
        .len();
    if size == 0 {
        let _ = std::fs::remove_file(&temporary);
        return Err("下载的 ONNX 文件为空".into());
    }
    let target = root.join(&source.file);
    let _ = std::fs::remove_file(&target);
    std::fs::rename(&temporary, &target).map_err(|error| error.to_string())?;
    select(config_dir, id)
}

fn download_file(url: &str, target: &Path) -> Result<(), String> {
    let client = reqwest::blocking::Client::builder()
        .user_agent("microduck-vision/0.1")
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
                last_error = error;
                let _ = std::fs::remove_file(target);
                if attempt < DOWNLOAD_ATTEMPTS {
                    thread::sleep(Duration::from_millis(250 * u64::from(attempt)));
                }
            }
        }
    }

    Err(format!(
        "下载官方策略失败（尝试 {DOWNLOAD_ATTEMPTS} 次）：{last_error}"
    ))
}

pub(crate) fn select(config_dir: Option<&Path>, id: &str) -> Result<PolicyEntry, String> {
    let sources = sources(config_dir)?;
    let source = sources
        .iter()
        .find(|source| source.id == id)
        .ok_or_else(|| format!("unknown official policy: {id}"))?;
    let root = cache_root(config_dir);
    let path = root.join(&source.file);
    let size = std::fs::metadata(&path)
        .map_err(|_| "请先下载这个官方策略".to_owned())?
        .len();
    if size == 0 {
        return Err("请先下载这个官方策略".into());
    }
    std::fs::create_dir_all(&root).map_err(|error| error.to_string())?;
    std::fs::write(root.join(SELECTED_FILE), &source.id).map_err(|error| error.to_string())?;
    list(config_dir)?
        .into_iter()
        .find(|entry| entry.id == source.id)
        .ok_or_else(|| "策略缓存读取失败".into())
}

pub(crate) fn selected_path(config_dir: Option<&Path>) -> Option<PathBuf> {
    let root = cache_root(config_dir);
    let id = read_selected(&root)?;
    let source = sources(config_dir)
        .ok()?
        .into_iter()
        .find(|source| source.id == id)?;
    let path = root.join(source.file);
    path.is_file().then_some(path)
}

fn sources(config_dir: Option<&Path>) -> Result<Vec<OfficialSource>, String> {
    let manifest = load_manifest(config_dir)?;
    Ok(manifest
        .policies
        .into_iter()
        .filter_map(|policy| {
            let path = Path::new(&policy.file);
            let file_name = path.file_name()?.to_str()?;
            if file_name != policy.file || !file_name.ends_with(".onnx") {
                return None;
            }
            let stem = path.file_stem()?.to_str()?;
            let id = stem.replace('_', "-");
            let name = policy.name.unwrap_or_else(|| stem.replace('_', " "));
            Some(OfficialSource {
                id,
                name,
                file: policy.file.clone(),
                url: format!(
                    "https://huggingface.co/pollen-robotics/microduck-policies/resolve/main/{}",
                    file_name
                ),
                kind: policy.kind,
                slot: policy.slot,
                duration_s: policy.duration_s,
                command: policy.command,
            })
        })
        .collect())
}

fn load_manifest(config_dir: Option<&Path>) -> Result<OfficialManifest, String> {
    let root = cache_root(config_dir);
    std::fs::create_dir_all(&root).map_err(|error| error.to_string())?;
    let cache = root.join(MANIFEST_FILE);
    let cached = std::fs::read(&cache)
        .ok()
        .and_then(|bytes| serde_json::from_slice::<OfficialManifest>(&bytes).ok());
    let cache_is_fresh = std::fs::metadata(&cache)
        .and_then(|metadata| metadata.modified())
        .ok()
        .and_then(|modified| SystemTime::now().duration_since(modified).ok())
        .map(|age| age < MANIFEST_MAX_AGE)
        .unwrap_or(false);

    if let Some(manifest) = cached.as_ref().filter(|_| cache_is_fresh) {
        return Ok(manifest.clone());
    }

    let temporary = root.join(format!(".{MANIFEST_FILE}.download"));
    let _ = std::fs::remove_file(&temporary);
    match download_file(MANIFEST_URL, &temporary).and_then(|_| {
        let bytes = std::fs::read(&temporary)
            .map_err(|error| format!("读取官方 manifest 失败：{error}"))?;
        let manifest = serde_json::from_slice::<OfficialManifest>(&bytes)
            .map_err(|error| format!("解析官方 manifest 失败：{error}"))?;
        std::fs::rename(&temporary, &cache)
            .map_err(|error| format!("缓存官方 manifest 失败：{error}"))?;
        Ok(manifest)
    }) {
        Ok(manifest) => Ok(manifest),
        Err(error) => cached.ok_or(error),
    }
}

fn read_selected(root: &Path) -> Option<String> {
    std::fs::read_to_string(root.join(SELECTED_FILE))
        .ok()
        .map(|value| value.trim().to_owned())
        .filter(|value| !value.is_empty())
}

#[cfg(test)]
mod tests {
    use super::{MANIFEST_FILE, cache_root, list, select};
    use std::path::{Path, PathBuf};

    fn write_manifest(config: &Path) {
        let root = cache_root(Some(config));
        std::fs::create_dir_all(&root).unwrap();
        std::fs::write(
            root.join(MANIFEST_FILE),
            r#"{"policies":[
                {"file":"alpha_walking.onnx"},
                {"file":"velstand.onnx","name":"Velstand","kind":"perpetual","slot":"walk",
                 "command":{"encoding":"constant","idle":[0,0,0]}},
                {"file":"alpha_ground_pick.onnx"}
            ]}"#,
        )
        .unwrap();
    }

    #[test]
    fn policy_cache_is_next_to_config_not_inside_it() {
        let config = PathBuf::from("C:/microduck/config");
        assert_eq!(
            cache_root(Some(&config)),
            PathBuf::from("C:/microduck/library/policies/official")
        );
    }

    #[test]
    fn official_source_is_visible_before_download() {
        let root = std::env::temp_dir().join("microduck-vision-no-policy-config");
        let _ = std::fs::remove_dir_all(&root);
        let config = root.join("config");
        write_manifest(&config);
        let entries = list(Some(&config)).unwrap();
        assert_eq!(entries.len(), 3);
        assert!(entries.iter().all(|entry| !entry.cached));
        assert!(
            entries
                .iter()
                .all(|entry| entry.source_url.starts_with("https://huggingface.co/"))
        );
        assert_eq!(entries[1].name, "Velstand");
        assert_eq!(entries[1].kind.as_deref(), Some("perpetual"));
        assert_eq!(entries[1].slot.as_deref(), Some("walk"));
        assert_eq!(
            entries[1]
                .command
                .as_ref()
                .and_then(|c| c.encoding.as_deref()),
            Some("constant")
        );
        let _ = std::fs::remove_dir_all(root);
    }

    #[test]
    fn selecting_a_missing_policy_is_rejected() {
        let root = std::env::temp_dir().join(format!(
            "microduck-vision-policy-test-{}",
            std::process::id()
        ));
        let config = root.join("config");
        let _ = std::fs::remove_dir_all(&root);
        write_manifest(&config);
        assert!(select(Some(&config), "velstand").is_err());
        let _ = std::fs::remove_dir_all(root);
    }
}
