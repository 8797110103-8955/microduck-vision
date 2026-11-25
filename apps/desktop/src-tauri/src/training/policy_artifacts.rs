//! Validate training checkpoints and package the contracts used by published policies.

use std::collections::BTreeMap;
use std::fs;
use std::io::{self, Write};
use std::path::{Path, PathBuf};
use std::time::Duration;

use duck_body::{AssetResolver, BodyIdentity, BodyProfile};
use duck_control::model::{DEFAULT_POSITION, JOINT_NAMES, MOUTH_INDEX, NUM_JOINTS};
use duck_control::obs::{ACTION_LEN, OBS_LEN};
use microduck_body_runtime::models::assets::OfficialAssets;
use serde::{Deserialize, Serialize};
use serde_json::{Value, json};
use sha2::{Digest, Sha256};

use crate::runtime::policy::inference::{
    LocalPolicySpec, PolicyConfig, PolicyDescriptor, TensorSpec,
};
use crate::runtime::policy::manifest::{
    MANIFEST_FILE, ManifestDefaults, ManifestPolicy, ManifestRobot, PolicyManifest,
    is_onnx_filename, model_file, read_manifest,
};
use crate::runtime::policy::publication_directories;
use crate::runtime::session::execution::PreviewPolicySelection;
use crate::types::{ModelKindKey, PolicyEntry, PublicationTarget, ServoModelKey};

use microduck_training::{
    artifacts::{ID_PREFIX, digest_is_valid, read_artifact},
    deepmimic::valid_id,
};

const POLICY_SCHEMA_VERSION: u32 = 2;
const MODEL_API: u32 = 1;
const WALK_ROBOT: &str = "robot_walk.xml";
const BACKLASH_ROBOT: &str = "robot_groundcontact_backlash.xml";

#[derive(Debug, Clone)]
pub(crate) struct TrainingPolicy {
    origin: PolicyOrigin,
    pub(crate) sha256: String,
    pub(crate) spec: LocalPolicySpec,
    pub(crate) source: Value,
}

impl TrainingPolicy {
    pub(crate) fn requires_backlash(&self) -> bool {
        matches!(&self.origin, PolicyOrigin::Rl { robot_xml, .. } if robot_xml == BACKLASH_ROBOT)
    }
}

#[derive(Debug, Clone, Serialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
enum PolicyOrigin {
    Rl {
        robot_xml: String,
        robot_sha256: String,
    },
    Teacher,
}

impl PolicyOrigin {
    fn tensor_names(&self) -> [&'static str; 2] {
        match self {
            Self::Rl { .. } => ["obs", "actions"],
            Self::Teacher => ["observation", "action"],
        }
    }

    fn default_home(&self, body: &BodyProfile) -> Result<[f64; NUM_JOINTS], String> {
        match self {
            Self::Rl { .. } => Ok(DEFAULT_POSITION),
            Self::Teacher => body_home(body),
        }
    }
}

type PublishedManifest = PolicyManifest<PublishedManifestEntry>;

#[derive(Deserialize, Serialize)]
struct PublishedManifestEntry {
    id: String,
    #[serde(flatten)]
    policy: ManifestPolicy,
    #[serde(default, skip_serializing_if = "PublicationReference::is_empty")]
    training: PublicationReference,
    microduck_vision: DesktopPolicyContract,
}

#[derive(Deserialize, Serialize)]
struct DesktopPolicyContract {
    body_source_hash: String,
    onnx_sha256: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    robot_sha256: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    home: Option<[f64; NUM_JOINTS]>,
    #[serde(skip_serializing_if = "Option::is_none")]
    input_batch: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    output_batch: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    head_lowpass: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    legs_lowpass: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    joint_velocity_delay_ticks: Option<u32>,
}

#[derive(Default, Deserialize, Serialize)]
pub(super) struct PublicationReference {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub(super) task_id: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub(super) motion_name: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub(super) training_policy_name: Option<String>,
}

impl PublicationReference {
    fn is_empty(&self) -> bool {
        self.task_id.is_none() && self.motion_name.is_none() && self.training_policy_name.is_none()
    }
}

#[derive(Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
enum ManifestContract {
    Rl(Box<RlContract>),
    Teacher(TeacherContract),
}

#[derive(Deserialize)]
struct RlContract {
    task_id: String,
    model_kind: ModelKindKey,
    execution_fingerprint: String,
    robot_xml: String,
    robot_sha256: String,
    body: BodyIdentity,
    control_s: f64,
    joint_order: Vec<String>,
    home_pose_rad: [f64; NUM_JOINTS],
    policy_layout: Value,
    onnx_metadata: BTreeMap<String, String>,
    domain_randomization: bool,
    encoder_through_backlash: Option<bool>,
    joint_velocity_delay_ticks: Option<u32>,
}

#[derive(Deserialize)]
struct TeacherContract {
    policy_contract: PolicyDescriptor,
    samples_sha256: String,
    source_segments: BTreeMap<String, Value>,
}

fn published_model_name(name: &str) -> Result<String, String> {
    if name.is_empty()
        || name.starts_with('.')
        || name.chars().any(|character| {
            character.is_control()
                || matches!(
                    character,
                    '<' | '>' | ':' | '"' | '/' | '\\' | '|' | '?' | '*'
                )
        })
    {
        return Err("发布名称含有文件名不允许的字符".into());
    }
    let device = name
        .split('.')
        .next()
        .unwrap_or_default()
        .trim_end()
        .to_ascii_uppercase();
    if matches!(device.as_str(), "CON" | "PRN" | "AUX" | "NUL")
        || matches!(
            device
                .strip_prefix("COM")
                .or_else(|| device.strip_prefix("LPT")),
            Some("1" | "2" | "3" | "4" | "5" | "6" | "7" | "8" | "9" | "¹" | "²" | "³")
        )
    {
        return Err("发布名称不能使用 Windows 保留的设备名".into());
    }
    let filename = if name
        .rsplit_once('.')
        .is_some_and(|(_, ext)| ext.eq_ignore_ascii_case("onnx"))
    {
        name.to_owned()
    } else {
        format!("{name}.onnx")
    };
    if filename.encode_utf16().count() > 255 {
        return Err("发布名称过长".into());
    }
    Ok(filename)
}

pub(crate) struct PublishedPolicy {
    entry: PublishedManifestEntry,
    directory: PathBuf,
}

fn check_publication_defaults(defaults: &ManifestDefaults) -> Result<(), String> {
    if defaults.schema_version != Some(POLICY_SCHEMA_VERSION)
        || defaults.model_api != Some(MODEL_API)
        || defaults
            .robot
            .as_ref()
            .and_then(|robot| robot.model.as_deref())
            .is_some_and(|model| !model.eq_ignore_ascii_case("microduck"))
    {
        return Err("发布策略的版本或机体无效".into());
    }
    Ok(())
}

fn publication_motor(defaults: &ManifestDefaults) -> Option<ServoModelKey> {
    match defaults
        .robot
        .as_ref()?
        .servos
        .as_deref()?
        .to_ascii_lowercase()
        .as_str()
    {
        "xl330" => Some(ServoModelKey::Xl330),
        "hd1910" | "hd1910-compatible" => Some(ServoModelKey::Hd1910),
        _ => None,
    }
}

pub(crate) fn publication_targets(root: &Path) -> Vec<PublicationTarget> {
    publication_directories(root)
        .into_iter()
        .filter_map(|directory| {
            let manifest =
                read_manifest::<PublishedManifestEntry>(&directory.join(MANIFEST_FILE)).ok()??;
            check_publication_defaults(&manifest.defaults).ok()?;
            let servo_model = publication_motor(&manifest.defaults)?;
            let first = manifest.policies.iter().find(|entry| entry.is_valid())?;
            Some(PublicationTarget {
                id: directory.file_name()?.to_str()?.to_owned(),
                name: first.name().to_owned(),
                servo_model,
            })
        })
        .collect()
}

impl PublishedPolicy {
    pub(crate) fn from_manifest(directory: &Path) -> Result<Vec<Self>, String> {
        let manifest = read_manifest::<PublishedManifestEntry>(&directory.join(MANIFEST_FILE))?
            .ok_or("缺少发布策略清单")?;
        let defaults = manifest.defaults;
        check_publication_defaults(&defaults)?;
        Ok(manifest
            .policies
            .into_iter()
            .filter_map(|mut entry| {
                if !entry.is_valid() {
                    return None;
                }
                entry.policy.apply_defaults(&defaults);
                Some(Self {
                    entry,
                    directory: directory.to_path_buf(),
                })
            })
            .collect())
    }

    pub(crate) fn id(&self) -> &str {
        &self.entry.id
    }

    pub(crate) fn entry(&self, selected: bool) -> PolicyEntry {
        let size = model_file(&self.directory, &self.entry.policy.file).map_or(0, |(_, size)| size);
        self.entry.entry(selected, size)
    }

    pub(crate) fn resolve(self, body: &BodyProfile) -> Result<PreviewPolicySelection, String> {
        let policy = self.entry.policy;
        let desktop = self.entry.microduck_vision;
        let (path, _) = model_file(&self.directory, &policy.file)?;
        let identity = body.identity();
        if desktop.body_source_hash != identity.source_hash {
            return Err("发布策略的身体标识与当前模型不匹配".into());
        }
        let origin = published_origin(desktop.robot_sha256)?;
        let [input, output] = origin.tensor_names();
        let home = match desktop.home {
            Some(home) => home,
            None => origin.default_home(body)?,
        };
        let id = format!("{ID_PREFIX}{}", self.entry.id);
        let mut source = serde_json::to_value(&origin).map_err(|error| error.to_string())?;
        source["id"] = id.clone().into();
        let descriptor = PolicyDescriptor {
            id,
            body_id: policy.body_id.unwrap_or(identity.id),
            body_version: policy.body_version.unwrap_or(identity.version),
            obs_len: policy.obs_len.unwrap_or(OBS_LEN),
            action_len: policy.action_len.unwrap_or(ACTION_LEN),
            control_hz: policy.control_hz.unwrap_or(50),
            command_encoding: policy.command_encoding,
            action_scale: policy.action_scale,
            recurrent: policy.recurrent.or(Some(false)),
            head_lowpass: policy.head_lowpass.or(desktop.head_lowpass),
            legs_lowpass: policy.legs_lowpass.or(desktop.legs_lowpass),
            joint_velocity_delay_ticks: policy
                .joint_velocity_delay_ticks
                .or(desktop.joint_velocity_delay_ticks)
                .unwrap_or(1),
        };
        let spec = LocalPolicySpec {
            input: TensorSpec::for_policy(input, descriptor.obs_len, desktop.input_batch),
            output: TensorSpec::for_policy(output, descriptor.action_len, desktop.output_batch),
            home,
            // Publishing checks metadata before storing the digest of these exact bytes.
            metadata: BTreeMap::new(),
        };
        let mut selection = resolve_selection(
            path,
            descriptor,
            body,
            TrainingPolicy {
                origin,
                sha256: desktop.onnx_sha256,
                spec,
                source,
            },
        )?;
        selection.command = policy.command;
        selection.episodic = policy.kind.as_deref() == Some("episodic");
        selection.duration = policy
            .duration_s
            .filter(|duration| *duration > 0.0)
            .and_then(|duration| Duration::try_from_secs_f64(duration).ok());
        Ok(selection)
    }
}

impl PublishedManifestEntry {
    fn is_valid(&self) -> bool {
        valid_id(&self.id)
            && is_onnx_filename(&self.policy.file)
            && digest_is_valid(&self.microduck_vision.onnx_sha256)
    }

    fn name(&self) -> &str {
        self.policy
            .name
            .as_deref()
            .filter(|name| !name.trim().is_empty())
            .unwrap_or_else(|| {
                Path::new(&self.policy.file)
                    .file_stem()
                    .and_then(|stem| stem.to_str())
                    .unwrap_or(&self.policy.file)
            })
    }

    fn entry(&self, selected: bool, size: u64) -> PolicyEntry {
        let kind = if self.microduck_vision.robot_sha256.is_some() {
            "training"
        } else {
            "teacher-training"
        };
        PolicyEntry {
            id: format!("{ID_PREFIX}{}", self.id),
            name: self.name().to_owned(),
            file: self.policy.file.clone(),
            source_url: String::new(),
            cached: size > 0,
            selected,
            size,
            kind: Some(kind.into()),
            slot: self.policy.slot.clone(),
            duration_s: self.policy.duration_s,
            command: self.policy.command.clone(),
        }
    }
}

pub(super) fn publish(
    root: &Path,
    name: &str,
    body: &BodyProfile,
    selection: PreviewPolicySelection,
    mut reference: PublicationReference,
    servo_model: Option<ServoModelKey>,
    append_to: Option<&str>,
) -> Result<PolicyEntry, String> {
    let file = published_model_name(name)?;
    let bytes = fs::read(&selection.path).map_err(|error| error.to_string())?;
    let training = selection.training.ok_or("候选策略缺少合同")?;
    if format!("{:x}", Sha256::digest(&bytes)) != training.sha256 {
        return Err("ONNX 摘要无效".into());
    }
    crate::runtime::policy::inference::validate_candidate(&bytes, &training.spec)?;
    let id = format!("published-{}", uuid::Uuid::new_v4());
    let descriptor = selection.descriptor;
    let motor = servo_model.or_else(|| match training.source["model_kind"].as_str() {
        Some("xl330") => Some(ServoModelKey::Xl330),
        Some("hd1910" | "hd1910-compatible") => Some(ServoModelKey::Hd1910),
        _ => None,
    });
    let servos = motor.map(|motor| match motor {
        ServoModelKey::Xl330 => "xl330".into(),
        ServoModelKey::Hd1910 => "hd1910".into(),
    });
    let home =
        (training.spec.home != training.origin.default_home(body)?).then_some(training.spec.home);
    let robot_sha256 = match training.origin {
        PolicyOrigin::Rl { robot_sha256, .. } => Some(robot_sha256),
        PolicyOrigin::Teacher => None,
    };
    reference.task_id = training.source["task"].as_str().map(str::to_owned);
    let manifest = PublishedManifest {
        defaults: ManifestDefaults {
            schema_version: Some(POLICY_SCHEMA_VERSION),
            model_api: Some(MODEL_API),
            obs_len: Some(descriptor.obs_len),
            action_len: Some(descriptor.action_len),
            robot: Some(ManifestRobot {
                model: Some("microduck".into()),
                hw_rev: Some(1),
                servos,
                control_hz: Some(descriptor.control_hz),
            }),
            ..Default::default()
        },
        policies: vec![PublishedManifestEntry {
            id: id.clone(),
            policy: ManifestPolicy {
                name: Some(name.to_owned()),
                file,
                kind: Some("perpetual".into()),
                slot: Some("walk".into()),
                entry_pose: Some("standing".into()),
                action_scale: descriptor
                    .action_scale
                    .filter(|scale| *scale != PolicyConfig::default().action_scale),
                command: selection.command,
                ..Default::default()
            },
            training: reference,
            microduck_vision: DesktopPolicyContract {
                body_source_hash: body.source_hash(),
                onnx_sha256: training.sha256,
                robot_sha256,
                home,
                input_batch: training.spec.input.batch_symbol(),
                output_batch: training.spec.output.batch_symbol(),
                head_lowpass: descriptor.head_lowpass,
                legs_lowpass: descriptor.legs_lowpass,
                joint_velocity_delay_ticks: (descriptor.joint_velocity_delay_ticks != 1)
                    .then_some(descriptor.joint_velocity_delay_ticks),
            },
        }],
    };
    match append_to {
        Some(target) => append_publication(root, target, manifest, &body.identity(), &bytes),
        None => write_publication(root, &id, manifest, &bytes),
    }
}

fn append_publication(
    root: &Path,
    target: &str,
    manifest: PublishedManifest,
    body: &BodyIdentity,
    bytes: &[u8],
) -> Result<PolicyEntry, String> {
    let directory = publication_directories(root)
        .into_iter()
        .find(|directory| directory.file_name().and_then(|name| name.to_str()) == Some(target))
        .ok_or("找不到追加目标，请刷新发布页面")?;
    let path = directory.join(MANIFEST_FILE);
    let mut document: Value =
        serde_json::from_slice(&fs::read(&path).map_err(|error| error.to_string())?)
            .map_err(|error| format!("解析发布清单失败：{error}"))?;
    let existing = PublishedManifest::deserialize(&document).map_err(|error| error.to_string())?;
    check_publication_defaults(&existing.defaults)?;
    let motor = publication_motor(&manifest.defaults).ok_or("无法确定训练使用的电机")?;
    if publication_motor(&existing.defaults) != Some(motor) {
        return Err("追加目标与训练使用的电机不一致".into());
    }
    let [mut entry]: [PublishedManifestEntry; 1] = manifest
        .policies
        .try_into()
        .map_err(|_| "每次发布需要一个 ONNX 策略")?;
    let filename = entry.policy.file.to_lowercase();
    if existing
        .policies
        .iter()
        .any(|old| old.policy.file.to_lowercase() == filename)
    {
        return Err("追加目标已有同名 ONNX 文件".into());
    }

    let mut inherited = ManifestPolicy::default();
    inherited.apply_defaults(&existing.defaults);
    entry.policy.obs_len = manifest
        .defaults
        .obs_len
        .filter(|length| Some(*length) != inherited.obs_len);
    entry.policy.action_len = manifest
        .defaults
        .action_len
        .filter(|length| Some(*length) != inherited.action_len);
    entry.policy.control_hz = manifest
        .defaults
        .robot
        .and_then(|robot| robot.control_hz)
        .filter(|rate| Some(*rate) != inherited.control_hz);
    if inherited.body_id.as_ref().is_some_and(|id| id != &body.id) {
        entry.policy.body_id = Some(body.id.clone());
    }
    if inherited
        .body_version
        .is_some_and(|version| version != body.version)
    {
        entry.policy.body_version = Some(body.version);
    }

    let policies = document["policies"]
        .as_array_mut()
        .ok_or("发布清单缺少策略列表")?;
    policies.push(serde_json::to_value(&entry).map_err(|error| error.to_string())?);
    let json = serde_json::to_vec_pretty(&document).map_err(|error| error.to_string())?;
    let model = directory.join(&entry.policy.file);
    let temporary = directory.join(format!(".manifest-{}.tmp", uuid::Uuid::new_v4()));
    let mut file = fs::File::options()
        .write(true)
        .create_new(true)
        .open(&model)
        .map_err(|error| {
            if error.kind() == io::ErrorKind::AlreadyExists {
                "追加目标已有同名 ONNX 文件".into()
            } else {
                error.to_string()
            }
        })?;
    let result = file.write_all(bytes);
    drop(file);
    // Readers see the new entry only after the model is complete and the manifest is replaced.
    if let Err(error) = result
        .and_then(|()| fs::write(&temporary, json))
        .and_then(|()| fs::rename(&temporary, &path))
    {
        let _ = fs::remove_file(temporary);
        let _ = fs::remove_file(model);
        return Err(format!("追加发布策略失败：{error}"));
    }
    Ok(entry.entry(false, bytes.len() as u64))
}

fn write_publication(
    root: &Path,
    directory: &str,
    manifest: PublishedManifest,
    bytes: &[u8],
) -> Result<PolicyEntry, String> {
    let [entry] = manifest.policies.as_slice() else {
        return Err("每次发布需要一个 ONNX 策略".into());
    };
    fs::create_dir_all(root).map_err(|error| error.to_string())?;
    let pending = root.join(format!(".{directory}"));
    fs::create_dir(&pending).map_err(|error| error.to_string())?;
    let result = (|| {
        fs::write(pending.join(&entry.policy.file), bytes).map_err(|error| error.to_string())?;
        let json = serde_json::to_vec_pretty(&manifest).map_err(|error| error.to_string())?;
        fs::write(pending.join(MANIFEST_FILE), json).map_err(|error| error.to_string())?;
        fs::rename(&pending, root.join(directory)).map_err(|error| error.to_string())?;
        Ok(entry.entry(false, bytes.len() as u64))
    })();
    if result.is_err() {
        let _ = fs::remove_dir_all(pending);
    }
    result
}

pub(crate) fn resolve(
    run: &Path,
    id: String,
    body: &BodyProfile,
) -> Result<PreviewPolicySelection, String> {
    let (manifest, path) = read_artifact::<TensorSpec, ManifestContract>(run)?;
    let (mut descriptor, home, metadata, source, origin) = match manifest.contract {
        ManifestContract::Rl(contract) => {
            let descriptor = contract.descriptor(body)?;
            let source = json!({
                "kind": "rl", "id": id, "task": contract.task_id,
                "model_kind": contract.model_kind, "execution_fingerprint": contract.execution_fingerprint,
                "robot_xml": contract.robot_xml, "robot_sha256": contract.robot_sha256,
                "domain_randomization": contract.domain_randomization,
                "encoder_through_backlash": contract.encoder_through_backlash,
            });
            (
                descriptor,
                contract.home_pose_rad,
                contract.onnx_metadata,
                source,
                PolicyOrigin::Rl {
                    robot_xml: contract.robot_xml,
                    robot_sha256: contract.robot_sha256,
                },
            )
        }
        ManifestContract::Teacher(contract) => {
            let home = contract.home(body)?;
            let source = json!({
                "kind": "teacher", "id": id, "teacher_policy_id": contract.policy_contract.id,
                "samples_sha256": contract.samples_sha256,
                "recorded_backend_contracts": contract.source_segments.values().map(|segment| &segment["backend_contract"]).collect::<Vec<_>>(),
            });
            (
                contract.policy_contract,
                home,
                BTreeMap::new(),
                source,
                PolicyOrigin::Teacher,
            )
        }
    };
    let [input] = manifest
        .inputs
        .try_into()
        .map_err(|_| "训练模型必须有一个输入")?;
    let [output] = manifest
        .outputs
        .try_into()
        .map_err(|_| "训练模型必须有一个输出")?;
    descriptor.id = id;
    resolve_selection(
        path,
        descriptor,
        body,
        TrainingPolicy {
            origin,
            sha256: manifest.onnx_sha256,
            spec: LocalPolicySpec {
                input,
                output,
                home,
                metadata,
            },
            source,
        },
    )
}

fn resolve_selection(
    path: PathBuf,
    descriptor: PolicyDescriptor,
    body: &BodyProfile,
    training: TrainingPolicy,
) -> Result<PreviewPolicySelection, String> {
    let [input, output] = training.origin.tensor_names();
    if !training.spec.input.validate(input, OBS_LEN)
        || !training.spec.output.validate(output, ACTION_LEN)
    {
        return Err("训练模型张量名称、类型或维度与合同不一致".into());
    }
    descriptor
        .validate(body, Duration::from_millis(20))
        .map_err(|error| error.to_string())?;
    let config = PolicyConfig {
        action_scale: descriptor
            .action_scale
            .unwrap_or(PolicyConfig::default().action_scale),
        head_lowpass: descriptor.head_lowpass,
        legs_lowpass: descriptor.legs_lowpass,
        ..PolicyConfig::default()
    };
    Ok(PreviewPolicySelection {
        path,
        command: None,
        descriptor,
        config,
        episodic: false,
        duration: None,
        training: Some(training),
    })
}

fn validate_robot(xml: &str, sha256: &str) -> Result<(), String> {
    if !matches!(xml, WALK_ROBOT | BACKLASH_ROBOT) {
        return Err("桌面预览需要支持的机器人模型".into());
    }
    let bytes = OfficialAssets::Embedded
        .read(Path::new(xml))
        .map_err(|error| error.to_string())?;
    if format!("{:x}", Sha256::digest(bytes)) != sha256 {
        return Err("训练机器人 XML 与当前 pinned 资源不一致".into());
    }
    Ok(())
}

fn published_origin(robot_sha256: Option<String>) -> Result<PolicyOrigin, String> {
    let Some(robot_sha256) = robot_sha256 else {
        return Ok(PolicyOrigin::Teacher);
    };
    let robot_xml = [WALK_ROBOT, BACKLASH_ROBOT]
        .into_iter()
        .find(|xml| validate_robot(xml, &robot_sha256).is_ok())
        .ok_or("发布策略的机器人摘要与当前 pinned 资源不匹配")?;
    Ok(PolicyOrigin::Rl {
        robot_xml: robot_xml.into(),
        robot_sha256,
    })
}

fn body_home(body: &BodyProfile) -> Result<[f64; NUM_JOINTS], String> {
    body.template()
        .control
        .home_pose
        .as_slice()
        .try_into()
        .map_err(|_| "身体 home 槽位不匹配".into())
}

const WALK_TERMS: [(&str, &str, [u64; 2]); 8] = [
    (
        "base_ang_vel",
        "mjlab_microduck.tasks.mdp.base_ang_vel_imu_misaligned",
        [0, 1],
    ),
    (
        "projected_gravity",
        "mjlab_microduck.tasks.mdp.projected_gravity_imu_misaligned",
        [0, 1],
    ),
    (
        "joint_pos",
        "mjlab.envs.mdp.observations.joint_pos_rel",
        [0, 0],
    ),
    (
        "joint_vel",
        "mjlab.envs.mdp.observations.joint_vel_rel",
        [1, 1],
    ),
    ("actions", "mjlab.envs.mdp.observations.last_action", [0, 0]),
    (
        "command",
        "mjlab.envs.mdp.observations.generated_commands",
        [0, 0],
    ),
    (
        "head_command",
        "mjlab.envs.mdp.observations.generated_commands",
        [0, 0],
    ),
    (
        "body_command",
        "mjlab.envs.mdp.observations.generated_commands",
        [0, 0],
    ),
];

impl RlContract {
    fn descriptor(&self, body: &BodyProfile) -> Result<PolicyDescriptor, String> {
        let joints = JOINT_NAMES
            .iter()
            .copied()
            .enumerate()
            .filter_map(|(slot, name)| (slot != MOUTH_INDEX).then_some(name));
        if self.body != body.identity()
            || !self.joint_order.iter().map(String::as_str).eq(JOINT_NAMES)
        {
            return Err("训练产物的身体标识或规范槽位不匹配".into());
        }
        let backlash = self.robot_xml == BACKLASH_ROBOT;
        if (self.control_s - 0.02).abs() > 1e-9
            || self.task_id.trim().is_empty()
            || !digest_is_valid(&self.execution_fingerprint)
        {
            return Err("桌面预览需要支持的机器人模型、执行来源和 50 Hz 控制周期".into());
        }
        if backlash
            && (self.model_kind != ModelKindKey::Xl330
                || self.encoder_through_backlash != Some(true)
                || self.joint_velocity_delay_ticks != Some(1)
                || self.policy_layout["action_len"] != ACTION_LEN)
        {
            return Err(
                "Backlash 学生需要 XL330、输出侧 encoder、单步速度延迟和 14D action".into(),
            );
        }
        validate_robot(&self.robot_xml, &self.robot_sha256)?;
        let [action] = self.policy_layout["actions"]
            .as_array()
            .map(Vec::as_slice)
            .unwrap_or_default()
        else {
            return Err("桌面预览需要单一位置动作".into());
        };
        let cfg = &action["configuration"];
        let scale = cfg["scale"]
            .as_f64()
            .filter(|scale| *scale > 0.0)
            .ok_or("训练位置动作比例无效")?;
        if action["name"] != "joint_pos"
            || action["joints"] != json!(joints.clone().collect::<Vec<_>>())
            || cfg["type"] != "mjlab.envs.mdp.actions.actions.JointPositionActionCfg"
            || cfg["entity_name"] != "robot"
            || cfg["transmission_type"] != "joint"
            || cfg["use_default_offset"] != true
            || cfg["offset"].as_f64() != Some(0.0)
            || !cfg["clip"].is_null()
        {
            return Err("训练动作关节顺序、偏置或限幅与桌面位置动作不匹配".into());
        }
        self.validate_observations()?;
        let meta = |key: &str| {
            self.onnx_metadata
                .get(key)
                .map(String::as_str)
                .unwrap_or_default()
        };
        if backlash
            && (meta("microduck_encoder_through_backlash") != "true"
                || meta("microduck_joint_velocity_delay_ticks") != "1"
                || meta("microduck_robot_sha256") != self.robot_sha256)
        {
            return Err("ONNX 的 backlash encoder、速度延迟或模型摘要不匹配".into());
        }
        let home = meta("default_joint_pos")
            .split(',')
            .map(str::parse::<f64>)
            .collect::<Result<Vec<_>, _>>()
            .map_err(|_| "ONNX 缺少有效的关节 home 信息")?;
        // The upstream exporter rounds this metadata to three decimal places. The manifest
        // retains the full training home for use by both the observation and action codecs.
        if !meta("joint_names").split(',').eq(joints)
            || !meta("observation_names")
                .split(',')
                .eq(WALK_TERMS.map(|term| term.0))
            || meta("command_names") != "twist,head_pose,body_pose"
            || meta("action_scale").parse::<f64>().ok() != Some(scale)
            || home.len() != ACTION_LEN
            || !home
                .iter()
                .zip(
                    self.home_pose_rad
                        .iter()
                        .enumerate()
                        .filter_map(|(slot, home)| (slot != MOUTH_INDEX).then_some(home)),
                )
                .all(|(rounded, precise)| (rounded - precise).abs() <= 0.000501)
        {
            return Err("ONNX 的关节、观测、命令或 home 与训练清单不匹配".into());
        }
        Ok(PolicyDescriptor {
            id: String::new(),
            body_id: self.body.id.clone(),
            body_version: self.body.version,
            obs_len: OBS_LEN,
            action_len: ACTION_LEN,
            control_hz: 50,
            command_encoding: None,
            recurrent: Some(false),
            action_scale: Some(scale),
            head_lowpass: None,
            legs_lowpass: None,
            joint_velocity_delay_ticks: 1,
        })
    }

    fn validate_observations(&self) -> Result<(), String> {
        let terms = self.policy_layout["observations"]
            .as_array()
            .ok_or("训练产物缺少观测项")?;
        if self.policy_layout["concatenate_observations"] != true
            || (!self.policy_layout["observation_history_length"].is_null()
                && self.policy_layout["observation_history_length"].as_u64() != Some(0))
            || terms.len() != WALK_TERMS.len()
        {
            return Err("训练观测拼接或历史长度与桌面预览不匹配".into());
        }
        let backlash = self.robot_xml == BACKLASH_ROBOT;
        for (term, (name, mut func, mut delay)) in terms.iter().zip(WALK_TERMS) {
            if backlash {
                match name {
                    "base_ang_vel" => {
                        func = "mjlab.envs.mdp.observations.base_ang_vel";
                        delay = [0, 0];
                    }
                    "projected_gravity" => {
                        func = "mjlab.envs.mdp.observations.projected_gravity";
                        delay = [0, 0];
                    }
                    "joint_pos" => func = "mjlab_microduck.tasks.mdp.joint_pos_rel_backlash",
                    "joint_vel" => func = "mjlab_microduck.tasks.mdp.joint_vel_rel_backlash",
                    _ => {}
                }
            }
            let cfg = &term["configuration"];
            if term["name"] != name
                || cfg["func"] != func
                || cfg["history_length"].as_u64() != Some(0)
                || !cfg["clip"].is_null()
                || !cfg["scale"].is_null()
                || [cfg["delay_min_lag"].as_u64(), cfg["delay_max_lag"].as_u64()] != delay.map(Some)
            {
                return Err(format!("训练观测项 {name} 的顺序、缩放或采样延迟不匹配"));
            }
            let params = &cfg["params"];
            if backlash && name == "joint_pos" && params["biased"] != false {
                return Err("Backlash encoder 不能包含未部署的编码器偏置".into());
            }
            if matches!(name, "joint_pos" | "joint_vel")
                && (params["asset_cfg"]["name"] != "robot"
                    || params["asset_cfg"]["joint_names"] != json!(["^(?!passive_).*"]))
            {
                return Err(format!("训练观测项 {name} 的关节来源不匹配"));
            }
        }
        if !terms[5..]
            .iter()
            .map(|term| term["configuration"]["params"]["command_name"].as_str())
            .eq([Some("twist"), Some("head_pose"), Some("body_pose")])
        {
            return Err("训练命令块顺序与桌面预览不匹配".into());
        }
        Ok(())
    }
}

impl TeacherContract {
    fn home(&self, body: &BodyProfile) -> Result<[f64; NUM_JOINTS], String> {
        if self.policy_contract.recurrent == Some(true)
            || self
                .policy_contract
                .command_encoding
                .as_deref()
                .is_some_and(|encoding| encoding != "constant")
            || !digest_is_valid(&self.samples_sha256)
            || self.source_segments.is_empty()
        {
            return Err("教师产物缺少可用的前馈策略合同或录制片段".into());
        }
        let mut home = None;
        for segment in self.source_segments.values() {
            if segment["joint_order"] != json!(JOINT_NAMES)
                || serde_json::from_value::<PolicyDescriptor>(segment["policy_contract"].clone())
                    .ok()
                    .as_ref()
                    != Some(&self.policy_contract)
            {
                return Err("教师产物与录制策略的关节顺序或合同不匹配".into());
            }
            let pose: [f64; NUM_JOINTS] = if segment["training_policy_home_rad"].is_null() {
                body_home(body)?
            } else {
                serde_json::from_value(segment["training_policy_home_rad"].clone())
                    .map_err(|_| "录制的训练策略 home 无效")?
            };
            if home.is_some_and(|previous| previous != pose) {
                return Err("教师产物的片段使用了不同的策略 home".into());
            }
            home = Some(pose);
        }
        home.ok_or_else(|| "教师产物缺少录制 home".into())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::runtime::{
        machines::body_catalog,
        policy::{TRAINED_POLICIES, catalog, policy_directory},
    };

    #[test]
    fn publication_name_is_a_portable_onnx_filename() {
        assert_eq!(
            published_model_name("Walk v2.ONNX").unwrap(),
            "Walk v2.ONNX"
        );
        assert!(published_model_name("../outside").is_err());
        assert!(published_model_name("NUL.v2").is_err());
    }

    fn manifest(id: &str, name: &str, model: &[u8]) -> PublishedManifest {
        let body = body_catalog::official().unwrap();
        let robot = OfficialAssets::Embedded
            .read(Path::new(BACKLASH_ROBOT))
            .unwrap();
        PublishedManifest {
            defaults: ManifestDefaults {
                schema_version: Some(POLICY_SCHEMA_VERSION),
                model_api: Some(MODEL_API),
                obs_len: Some(OBS_LEN),
                action_len: Some(ACTION_LEN),
                robot: Some(ManifestRobot {
                    model: Some("microduck".into()),
                    hw_rev: Some(1),
                    servos: Some("xl330".into()),
                    control_hz: Some(50),
                }),
                ..Default::default()
            },
            policies: vec![PublishedManifestEntry {
                id: id.into(),
                policy: ManifestPolicy {
                    name: Some(name.into()),
                    file: published_model_name(name).unwrap(),
                    kind: Some("perpetual".into()),
                    slot: Some("walk".into()),
                    entry_pose: Some("standing".into()),
                    ..Default::default()
                },
                training: PublicationReference {
                    task_id: Some("Microduck-DeepMimic-Velstand-Flat-v1".into()),
                    motion_name: Some("向前走".into()),
                    training_policy_name: Some("Velstand".into()),
                },
                microduck_vision: DesktopPolicyContract {
                    body_source_hash: body.source_hash(),
                    onnx_sha256: format!("{:x}", Sha256::digest(model)),
                    robot_sha256: Some(format!("{:x}", Sha256::digest(robot))),
                    home: None,
                    input_batch: None,
                    output_batch: None,
                    head_lowpass: None,
                    legs_lowpass: None,
                    joint_velocity_delay_ticks: None,
                },
            }],
        }
    }

    #[test]
    fn published_policy_loads_from_two_files_and_latest_in_load_order_wins() {
        let scratch = crate::platform::helpers::test_directory()
            .join(format!("policy-publication-{}", uuid::Uuid::new_v4()));
        let config = scratch.join("config");
        let root = policy_directory(Some(&config), TRAINED_POLICIES);
        let manifest_id = "published-example";
        let id = format!("{ID_PREFIX}{manifest_id}");
        let name = "向前走 v2";
        let model = b"model loaded second";
        let directory = "z-import";
        let published =
            write_publication(&root, directory, manifest(manifest_id, name, model), model).unwrap();
        let newer_model = b"newer file loaded first";
        write_publication(
            &root,
            "a-import",
            manifest(manifest_id, "Other", newer_model),
            newer_model,
        )
        .unwrap();

        let mut files: Vec<_> = fs::read_dir(root.join(directory))
            .unwrap()
            .map(|entry| entry.unwrap().file_name())
            .collect();
        files.sort();
        assert_eq!(files, ["manifest.json", "向前走 v2.onnx"]);
        let document: Value =
            serde_json::from_slice(&fs::read(root.join(directory).join("manifest.json")).unwrap())
                .unwrap();
        assert_eq!(document["schema_version"], 2);
        assert_eq!(document["model_api"], 1);
        assert_eq!(document["robot"]["control_hz"], 50);
        assert!(document.get("file").is_none());
        let official_shape =
            read_manifest::<ManifestPolicy>(&root.join(directory).join(MANIFEST_FILE))
                .unwrap()
                .unwrap();
        assert_eq!(official_shape.policies[0].file, "向前走 v2.onnx");
        let [entry] = document["policies"].as_array().unwrap().as_slice() else {
            panic!("expected one policy")
        };
        assert_eq!(entry["kind"], "perpetual");
        assert_eq!(entry["file"], "向前走 v2.onnx");
        assert_eq!(entry["training"]["motion_name"], "向前走");
        assert_eq!(entry["training"]["training_policy_name"], "Velstand");
        assert!(entry.get("command").is_none());
        let desktop = entry["microduck_vision"].as_object().unwrap();
        assert_eq!(desktop.len(), 3);
        assert!(desktop.contains_key("body_source_hash"));
        assert!(desktop.contains_key("onnx_sha256"));
        assert!(desktop.contains_key("robot_sha256"));
        let mut entries = catalog::list(Some(&config)).unwrap();
        assert_eq!(entries.len(), 1);
        let listed = entries.pop().unwrap();
        assert_eq!(listed.id, id);
        assert_eq!(listed.name, published.name);
        assert_eq!(listed.file, "向前走 v2.onnx");
        let selected = catalog::select(Some(&config), &id).unwrap();
        assert!(selected.selected);
        assert_eq!(selected.file, listed.file);
        let selection = catalog::resolve_training_policy(Some(&config), &id).unwrap();
        assert!(selection.path.ends_with("z-import/向前走 v2.onnx"));
        let training = selection.training.unwrap();
        assert!(training.requires_backlash());
        assert_eq!(training.spec.home, DEFAULT_POSITION);
        assert!(training.spec.input.validate("obs", OBS_LEN));
        assert!(training.spec.output.validate("actions", ACTION_LEN));
        fs::remove_dir_all(scratch).unwrap();
    }

    #[test]
    fn appends_same_motor_policy_to_the_existing_manifest() {
        let scratch = crate::platform::helpers::test_directory()
            .join(format!("policy-append-{}", uuid::Uuid::new_v4()));
        let config = scratch.join("config");
        let root = policy_directory(Some(&config), TRAINED_POLICIES);
        let directory = root.join("bundle");
        let path = directory.join(MANIFEST_FILE);
        let original = b"original ONNX";
        let model = b"another ONNX";
        write_publication(
            &root,
            "bundle",
            manifest("original", "Original", original),
            original,
        )
        .unwrap();
        let mut document: Value = serde_json::from_slice(&fs::read(&path).unwrap()).unwrap();
        document["description"] = "A set of policies".into();
        document["policies"][0]["eval"] = serde_json::json!({ "score": 1 });
        let initial = serde_json::to_vec(&document).unwrap();
        fs::write(&path, &initial).unwrap();
        let body = body_catalog::official().unwrap().identity();

        let mut incompatible = manifest("incompatible", "Incompatible", model);
        incompatible.defaults.robot.as_mut().unwrap().servos = Some("hd1910".into());
        assert!(append_publication(&root, "bundle", incompatible, &body, model).is_err());
        let duplicate = manifest("duplicate", "original", model);
        assert!(append_publication(&root, "bundle", duplicate, &body, model).is_err());
        assert_eq!(fs::read(&path).unwrap(), initial);

        let mut next = manifest("next", "Another round", model);
        next.policies[0].training.training_policy_name = Some("Another run".into());
        let added = append_publication(&root, "bundle", next, &body, model).unwrap();
        assert_eq!(added.id, "training:next");
        let result: Value = serde_json::from_slice(&fs::read(&path).unwrap()).unwrap();
        assert_eq!(result["description"], document["description"]);
        assert_eq!(result["policies"][0], document["policies"][0]);
        assert_eq!(result["policies"].as_array().unwrap().len(), 2);
        assert_eq!(
            result["policies"][1]["training"]["training_policy_name"],
            "Another run"
        );
        assert_eq!(fs::read(directory.join("Original.onnx")).unwrap(), original);
        assert_eq!(
            fs::read(directory.join("Another round.onnx")).unwrap(),
            model
        );
        assert_eq!(fs::read_dir(&directory).unwrap().count(), 3);
        assert_eq!(catalog::list(Some(&config)).unwrap().len(), 2);
        let loaded = catalog::resolve_training_policy(Some(&config), &added.id).unwrap();
        assert!(loaded.path.ends_with("bundle/Another round.onnx"));
        let [target]: [PublicationTarget; 1] = publication_targets(&root).try_into().ok().unwrap();
        assert_eq!(target.id, "bundle");
        assert_eq!(target.name, "Original");
        assert_eq!(target.servo_model, ServoModelKey::Xl330);
        fs::remove_dir_all(scratch).unwrap();
    }

    #[test]
    fn published_manifest_selects_entries_before_checking_their_files() {
        let scratch = crate::platform::helpers::test_directory()
            .join(format!("policy-manifest-{}", uuid::Uuid::new_v4()));
        let config = scratch.join("config");
        let directory = policy_directory(Some(&config), TRAINED_POLICIES).join("bundle");
        fs::create_dir_all(&directory).unwrap();
        let bytes = b"example ONNX bytes";
        let mut set = manifest("duplicate", "original", bytes);
        set.policies
            .extend(manifest("working", "available", bytes).policies);
        set.policies
            .extend(manifest("duplicate", "missing", bytes).policies);
        fs::write(
            directory.join(MANIFEST_FILE),
            serde_json::to_vec(&set).unwrap(),
        )
        .unwrap();
        for file in ["original.onnx", "available.onnx", "unlisted.onnx"] {
            fs::write(directory.join(file), bytes).unwrap();
        }

        let entries = catalog::list(Some(&config)).unwrap();
        assert_eq!(entries.len(), 2);
        assert!(
            entries
                .iter()
                .any(|policy| policy.file == "available.onnx" && policy.cached)
        );
        assert!(
            entries
                .iter()
                .any(|policy| policy.file == "missing.onnx" && !policy.cached)
        );
        assert!(catalog::resolve_training_policy(Some(&config), "training:duplicate").is_err());
        let available =
            catalog::resolve_training_policy(Some(&config), "training:working").unwrap();
        assert!(available.path.ends_with("bundle/available.onnx"));
        fs::remove_dir_all(scratch).unwrap();
    }
}
