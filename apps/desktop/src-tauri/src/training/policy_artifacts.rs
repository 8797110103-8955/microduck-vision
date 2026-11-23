//! Resolve a completed local training run into a preview-only policy contract.

use std::collections::BTreeMap;
use std::path::Path;
use std::time::Duration;

use duck_body::{AssetResolver, BodyIdentity, BodyProfile};
use duck_control::model::{JOINT_NAMES, MOUTH_INDEX, NUM_JOINTS};
use duck_control::obs::{ACTION_LEN, OBS_LEN};
use microduck_body_runtime::models::assets::OfficialAssets;
use serde::Deserialize;
use serde_json::{Value, json};
use sha2::{Digest, Sha256};

use crate::runtime::policy::inference::{
    LocalPolicySpec, PolicyConfig, PolicyDescriptor, TensorSpec,
};
use crate::runtime::session::execution::PreviewPolicySelection;
use crate::types::{ModelKindKey, PolicyEntry};

use microduck_training::artifacts::{digest_is_valid, read_artifact};

#[derive(Debug, Clone)]
pub(crate) struct TrainingPolicy {
    pub(crate) sha256: String,
    pub(crate) spec: LocalPolicySpec,
    pub(crate) source: Value,
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
}

#[derive(Deserialize)]
struct TeacherContract {
    policy_contract: PolicyDescriptor,
    samples_sha256: String,
    source_segments: BTreeMap<String, Value>,
}

pub(crate) fn entry(run: &Path, id: String, selected: bool) -> Result<PolicyEntry, String> {
    let (manifest, model) = read_artifact::<TensorSpec, ManifestContract>(run)?;
    let file = model
        .file_name()
        .and_then(|name| name.to_str())
        .ok_or("训练模型文件名无效")?;
    let size = model.metadata().map_err(|error| error.to_string())?.len();
    let (name, kind) = match &manifest.contract {
        ManifestContract::Rl(contract) => (
            format!("{} · {:?}", contract.task_id, contract.model_kind),
            "training",
        ),
        ManifestContract::Teacher(contract) => (
            format!("教师训练 · {}", contract.policy_contract.id),
            "teacher-training",
        ),
    };
    Ok(PolicyEntry {
        id,
        name,
        file: file.to_owned(),
        source_url: String::new(),
        cached: size > 0,
        selected,
        size,
        kind: Some(kind.into()),
        slot: Some("walk".into()),
        duration_s: None,
        command: None,
    })
}

pub(crate) fn resolve(
    run: &Path,
    id: String,
    body: &BodyProfile,
) -> Result<PreviewPolicySelection, String> {
    let (manifest, path) = read_artifact::<TensorSpec, ManifestContract>(run)?;
    let (names, mut descriptor, home, metadata, source) = match manifest.contract {
        ManifestContract::Rl(contract) => {
            let descriptor = contract.descriptor(body)?;
            let source = json!({
                "kind": "rl", "id": id, "task": contract.task_id,
                "model_kind": contract.model_kind, "execution_fingerprint": contract.execution_fingerprint,
                "robot_xml": contract.robot_xml, "robot_sha256": contract.robot_sha256,
                "domain_randomization": contract.domain_randomization,
            });
            (
                ["obs", "actions"],
                descriptor,
                contract.home_pose_rad,
                contract.onnx_metadata,
                source,
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
                ["observation", "action"],
                contract.policy_contract,
                home,
                BTreeMap::new(),
                source,
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
    let [input, output]: [TensorSpec; 2] = [input, output];
    if !input.validate(names[0], OBS_LEN) || !output.validate(names[1], ACTION_LEN) {
        return Err("训练模型张量名称、类型或维度与合同不一致".into());
    }
    descriptor.id = id;
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
        training: Some(TrainingPolicy {
            sha256: manifest.onnx_sha256,
            spec: LocalPolicySpec {
                input,
                output,
                home,
                metadata,
            },
            source,
        }),
    })
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
        if self.robot_xml != "robot_walk.xml"
            || (self.control_s - 0.02).abs() > 1e-9
            || self.task_id.trim().is_empty()
            || !digest_is_valid(&self.execution_fingerprint)
        {
            return Err("桌面预览需要支持的 walk 身体、执行来源和 50 Hz 控制周期".into());
        }
        let bytes = OfficialAssets::Embedded
            .read(Path::new(&self.robot_xml))
            .map_err(|error| error.to_string())?;
        if format!("{:x}", Sha256::digest(bytes)) != self.robot_sha256 {
            return Err("训练机器人 XML 与当前 pinned 资源不一致".into());
        }
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
        for (term, (name, func, delay)) in terms.iter().zip(WALK_TERMS) {
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
                body.template()
                    .control
                    .home_pose
                    .as_slice()
                    .try_into()
                    .map_err(|_| "身体 home 槽位不匹配")?
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
