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

use crate::runtime::machines::body_catalog;
use crate::runtime::policy::inference::{PolicyConfig, PolicyDescriptor};
use crate::runtime::session::execution::PreviewPolicySelection;
use crate::training::policy_artifacts as training_policy;
use duck_control::policy::Net;
use microduck_training::artifacts::ID_PREFIX as TRAINING_ID_PREFIX;
use microduck_training::progress::training_runs_root;
use serde::Deserialize;

use crate::types::{PolicyCommand, PolicyEntry};

const CACHE_DIR: &str = "policies/official";
const MANIFEST_FILE: &str = "manifest.json";
const MANIFEST_URL: &str =
    "https://huggingface.co/pollen-robotics/microduck-policies/resolve/main/manifest.json";
const SELECTED_FILE: &str = ".selected";
const DOWNLOAD_ATTEMPTS: u8 = 3;
const MANIFEST_MAX_AGE: Duration = Duration::from_secs(60 * 60);

#[derive(Debug, Clone, Deserialize)]
struct OfficialManifest {
    policies: Vec<ManifestPolicy>,
    #[serde(default)]
    obs_len: Option<usize>,
    #[serde(default)]
    action_len: Option<usize>,
    #[serde(default)]
    body_id: Option<String>,
    #[serde(default)]
    body_version: Option<u32>,
    #[serde(default)]
    control_hz: Option<u32>,
    #[serde(default)]
    robot: Option<ManifestRobot>,
}

#[derive(Debug, Clone, Deserialize)]
struct ManifestRobot {
    #[serde(default)]
    control_hz: Option<u32>,
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
    #[serde(default)]
    action_scale: Option<f64>,
    #[serde(default)]
    obs_len: Option<usize>,
    #[serde(default)]
    action_len: Option<usize>,
    #[serde(default)]
    body_id: Option<String>,
    #[serde(default)]
    body_version: Option<u32>,
    #[serde(default)]
    control_hz: Option<u32>,
    #[serde(default)]
    command_encoding: Option<String>,
    #[serde(default)]
    recurrent: Option<bool>,
    #[serde(default)]
    head_lowpass: Option<f64>,
    #[serde(default)]
    legs_lowpass: Option<f64>,
    #[serde(default)]
    joint_velocity_delay_ticks: Option<u32>,
    #[serde(default)]
    imu_required: bool,
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
    action_scale: Option<f64>,
    obs_len: Option<usize>,
    action_len: Option<usize>,
    body_id: Option<String>,
    body_version: Option<u32>,
    control_hz: Option<u32>,
    command_encoding: Option<String>,
    recurrent: Option<bool>,
    head_lowpass: Option<f64>,
    legs_lowpass: Option<f64>,
    joint_velocity_delay_ticks: Option<u32>,
    imu_required: bool,
}

pub(crate) fn cached_policy_selections(
    config: &Path,
) -> Result<Option<Vec<PreviewPolicySelection>>, String> {
    let root = cache_root(Some(config));
    if !root.join(MANIFEST_FILE).is_file() {
        return Ok(None);
    }
    official_entries(Some(config))?
        .into_iter()
        .filter(|entry| entry.cached)
        .map(|entry| {
            Ok(PreviewPolicySelection {
                path: root.join(&entry.file),
                command: entry.command.clone(),
                descriptor: descriptor(Some(config), &entry)?,
                config: runtime_config(Some(config), &entry)?,
                episodic: entry.kind.as_deref() == Some("episodic"),
                duration: entry
                    .duration_s
                    .filter(|duration| duration.is_finite() && *duration > 0.0)
                    .and_then(|duration| Duration::try_from_secs_f64(duration).ok()),
                training: None,
            })
        })
        .collect::<Result<Vec<_>, _>>()
        .map(Some)
}

pub(crate) fn descriptor(
    config_dir: Option<&Path>,
    entry: &PolicyEntry,
) -> Result<PolicyDescriptor, String> {
    if is_training_policy(&entry.id) {
        return Ok(resolve_training_policy(config_dir, &entry.id)?.descriptor);
    }
    let source = sources(config_dir)?
        .into_iter()
        .find(|source| source.id == entry.id)
        .ok_or_else(|| format!("unknown official policy: {}", entry.id))?;
    Ok(PolicyDescriptor {
        id: source.id,
        body_id: source.body_id.unwrap_or_else(|| "official".into()),
        body_version: source.body_version.unwrap_or(1),
        obs_len: source.obs_len.unwrap_or(61),
        action_len: source.action_len.unwrap_or(14),
        control_hz: source.control_hz.unwrap_or(50),
        command_encoding: source.command_encoding,
        recurrent: source.recurrent,
        action_scale: source.action_scale,
        head_lowpass: source.head_lowpass,
        legs_lowpass: source.legs_lowpass,
        joint_velocity_delay_ticks: source.joint_velocity_delay_ticks.unwrap_or(1),
    })
}

/// Resolve controller tuning once, identically for preview and physical loops.
pub(crate) fn runtime_config(
    config_dir: Option<&Path>,
    entry: &PolicyEntry,
) -> Result<PolicyConfig, String> {
    let source = sources(config_dir)?
        .into_iter()
        .find(|source| source.id == entry.id)
        .ok_or_else(|| format!("unknown official policy: {}", entry.id))?;
    let name = Path::new(&source.file)
        .file_stem()
        .and_then(|value| value.to_str())
        .unwrap_or_default();
    let net = match source.slot.as_deref() {
        Some("walk") => Net::Walk,
        Some("stand") => Net::Stand,
        Some("sitstand") => Net::SitStand,
        Some("ground_pick") => Net::GroundPick,
        _ => match name {
            "alpha_stand" => Net::Stand,
            "alpha_sitstand" => Net::SitStand,
            "alpha_ground_pick" => Net::GroundPick,
            "roller_crouch" | "roulade" | "ball_kick_left" | "ball_kick_right" => Net::Skill(0),
            _ if source.kind.as_deref() == Some("episodic") => Net::Skill(0),
            _ => Net::Walk,
        },
    };
    // Match the official controller's position-action contract; manifest overrides win.
    let action_scale = source
        .action_scale
        .unwrap_or(PolicyConfig::default().action_scale);
    if !action_scale.is_finite() || action_scale <= 0.0 {
        return Err(format!("invalid action_scale for policy: {}", entry.id));
    }
    Ok(PolicyConfig {
        net,
        action_scale,
        head_lowpass: source.head_lowpass,
        legs_lowpass: source.legs_lowpass,
        imu_required: source.imu_required,
    })
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

pub(crate) fn open_folder(config_dir: Option<&Path>) -> Result<(), String> {
    let folder = std::path::absolute(cache_root(config_dir))
        .map_err(|error| format!("解析策略缓存目录失败：{error}"))?;
    std::fs::create_dir_all(&folder).map_err(|error| error.to_string())?;
    #[cfg(target_os = "windows")]
    let mut command = std::process::Command::new("explorer");
    #[cfg(target_os = "macos")]
    let mut command = std::process::Command::new("open");
    #[cfg(all(unix, not(target_os = "macos")))]
    let mut command = std::process::Command::new("xdg-open");
    command
        .arg(folder)
        .spawn()
        .map(|_| ())
        .map_err(|error| format!("打开策略缓存目录失败：{error}"))
}

fn official_entries(config_dir: Option<&Path>) -> Result<Vec<PolicyEntry>, String> {
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

pub(crate) fn is_training_policy(id: &str) -> bool {
    id.starts_with(TRAINING_ID_PREFIX)
}

fn training_run(config_dir: Option<&Path>, id: &str) -> Result<PathBuf, String> {
    let run = id
        .strip_prefix(TRAINING_ID_PREFIX)
        .ok_or("训练策略来源无效")?;
    if !matches!(
        Path::new(run).components().collect::<Vec<_>>().as_slice(),
        [std::path::Component::Normal(_)]
    ) {
        return Err("训练任务目录无效".into());
    }
    let root = training_runs_root(config_dir)
        .canonicalize()
        .map_err(|error| error.to_string())?;
    let path = root
        .join(run)
        .canonicalize()
        .map_err(|error| error.to_string())?;
    if path.parent() != Some(root.as_path()) {
        return Err("训练任务目录超出工作区".into());
    }
    Ok(path)
}

pub(crate) fn resolve_training_policy(
    config_dir: Option<&Path>,
    id: &str,
) -> Result<PreviewPolicySelection, String> {
    training_policy::resolve(
        &training_run(config_dir, id)?,
        id.to_owned(),
        body_catalog::official()?,
    )
}

pub(in crate::runtime) fn selected_training_policy(
    config_dir: Option<&Path>,
    id: &str,
) -> Result<PreviewPolicySelection, String> {
    if read_selected(&cache_root(config_dir)).as_deref() != Some(id) {
        return Err("请先选择该训练策略".into());
    }
    resolve_training_policy(config_dir, id)
}

pub(crate) fn list(config_dir: Option<&Path>) -> Result<Vec<PolicyEntry>, String> {
    let selected = read_selected(&cache_root(config_dir));
    let mut local: Vec<_> = std::fs::read_dir(training_runs_root(config_dir))
        .into_iter()
        .flatten()
        .filter_map(Result::ok)
        .filter(|run| run.file_type().is_ok_and(|kind| kind.is_dir()))
        .filter_map(|run| {
            let id = format!("{}{}", TRAINING_ID_PREFIX, run.file_name().to_str()?);
            let active = selected.as_deref() == Some(&id);
            training_policy::entry(&run.path(), id, active).ok()
        })
        .collect();
    local.sort_by(|left, right| right.id.cmp(&left.id));
    match official_entries(config_dir) {
        Ok(mut official) => {
            official.extend(local);
            Ok(official)
        }
        Err(_) if !local.is_empty() => Ok(local),
        Err(error) => Err(error),
    }
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
    if is_training_policy(id) {
        let selection = resolve_training_policy(config_dir, id)?;
        let expected = &selection
            .training
            .as_ref()
            .ok_or("训练产物缺少来源")?
            .sha256;
        use sha2::{Digest, Sha256};
        let bytes = std::fs::read(&selection.path).map_err(|error| error.to_string())?;
        if format!("{:x}", Sha256::digest(&bytes)) != *expected {
            return Err("训练 ONNX 文件摘要与产物清单不一致".into());
        }
        let entry = training_policy::entry(&training_run(config_dir, id)?, id.to_owned(), true)?;
        let root = cache_root(config_dir);
        std::fs::create_dir_all(&root).map_err(|error| error.to_string())?;
        std::fs::write(root.join(SELECTED_FILE), id).map_err(|error| error.to_string())?;
        return Ok(entry);
    }
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
    official_entries(config_dir)?
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
    let default_obs_len = manifest.obs_len;
    let default_action_len = manifest.action_len;
    let default_body_id = manifest.body_id;
    let default_body_version = manifest.body_version;
    let default_control_hz = manifest
        .control_hz
        .or_else(|| manifest.robot.and_then(|robot| robot.control_hz));
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
            let command_encoding = policy.command_encoding.or_else(|| {
                policy
                    .command
                    .as_ref()
                    .and_then(|command| command.encoding.clone())
            });
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
                action_scale: policy.action_scale,
                obs_len: policy.obs_len.or(default_obs_len),
                action_len: policy.action_len.or(default_action_len),
                body_id: policy.body_id.or_else(|| default_body_id.clone()),
                body_version: policy.body_version.or(default_body_version),
                control_hz: policy.control_hz.or(default_control_hz),
                command_encoding,
                recurrent: policy.recurrent,
                head_lowpass: policy.head_lowpass,
                legs_lowpass: policy.legs_lowpass,
                joint_velocity_delay_ticks: policy.joint_velocity_delay_ticks,
                imu_required: policy.imu_required,
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
            PathBuf::from("C:/microduck/policies/official")
        );
    }

    #[test]
    fn official_source_is_visible_before_download() {
        let root =
            crate::platform::helpers::test_directory().join("microduck-vision-no-policy-config");
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
        let root = crate::platform::helpers::test_directory().join(format!(
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
