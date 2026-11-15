pub use duck_body::ServoModelKey;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, specta::Type)]
#[serde(rename_all = "snake_case")]
pub enum LoopPhase {
    Limp,
    Homing,
    Ready,
    Driving,
    Holding,
    Stopped,
    Reconnecting,
    Estop,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, specta::Type)]
#[serde(rename_all = "lowercase")]
pub enum ActionKind {
    Pose,
    Sequence,
    Policy,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, specta::Type)]
#[serde(rename_all = "lowercase")]
pub enum ActionSource {
    Builtin,
    User,
}

#[derive(Debug, Clone, Serialize, Deserialize, specta::Type)]
#[serde(rename_all = "camelCase")]
pub struct SceneGeom {
    pub mesh: String,
    pub pos: [f64; 3],
    pub quat: [f64; 4],
    pub rgba: [f32; 4],
}

#[derive(Debug, Clone, Serialize, Deserialize, specta::Type)]
#[serde(rename_all = "camelCase")]
pub struct SceneBody {
    pub name: String,
    pub geoms: Vec<SceneGeom>,
}

#[derive(Debug, Clone, Serialize, Deserialize, specta::Type)]
#[serde(rename_all = "camelCase")]
pub struct SceneJoint {
    pub name: String,
    pub body: u32,
}

#[derive(Debug, Clone, Serialize, Deserialize, specta::Type)]
#[serde(rename_all = "camelCase")]
pub struct SceneSite {
    pub body: u32,
    pub pos: [f64; 3],
}

#[derive(Debug, Clone, Serialize, Deserialize, specta::Type)]
#[serde(rename_all = "camelCase")]
pub struct SceneModel {
    pub bodies: Vec<SceneBody>,
    pub joints: Vec<SceneJoint>,
    pub trunk_height: f64,
    pub head_camera: Option<SceneSite>,
}

#[derive(Debug, Clone, Serialize, Deserialize, specta::Type)]
#[serde(rename_all = "camelCase")]
pub struct SceneFrame {
    pub bodies: Vec<[f64; 7]>,
    pub markers: Vec<[f64; 3]>,
}

#[derive(Debug, Clone, Serialize, Deserialize, specta::Type)]
#[serde(rename_all = "camelCase")]
pub struct SceneInfo {
    pub model: SceneModel,
    pub mesh_dir: String,
}

#[derive(Debug, Clone, Serialize, Deserialize, specta::Type)]
#[serde(rename_all = "camelCase")]
pub struct SimulationParameters {
    pub damping: f64,
    pub friction_loss: f64,
    pub armature: f64,
    pub kp: f64,
    pub force_range: f64,
    #[serde(default)]
    pub world_position: [f64; 3],
}

#[derive(Debug, Clone, Serialize, Deserialize, specta::Type)]
#[serde(rename_all = "camelCase")]
pub struct VirtualMachine {
    pub id: String,
    pub name: String,
    pub body_id: String,
    pub servo_model: ServoModelKey,
    pub simulation: SimulationParameters,
    pub startup_selected: bool,
    pub referenced_by: Vec<String>,
    #[serde(default)]
    pub joint_bindings: Vec<ServoJointBinding>,
}

#[derive(Debug, Clone, Serialize, Deserialize, specta::Type)]
#[serde(rename_all = "camelCase")]
pub struct PhysicalMachine {
    pub id: String,
    pub name: String,
    /// USB metadata is a reconnect hint; machine_code is a derived display value.
    #[serde(default)]
    pub usb_port: String,
    #[serde(default)]
    pub machine_code: String,
    #[serde(default)]
    pub census: Vec<ServoIdentity>,
    #[serde(default)]
    pub reconnect_hint: Option<ReconnectHint>,
    pub virtual_machine_id: Option<String>,
    pub online_servo_ids: Vec<u8>,
    pub connected: bool,
    #[serde(default)]
    pub last_seen: Option<String>,
    #[serde(default)]
    pub torque_enabled: bool,
}

#[derive(Debug, Clone, Serialize, Deserialize, specta::Type)]
#[serde(rename_all = "camelCase")]
pub struct PhysicalServoRegister {
    pub servo_id: u8,
    pub address: u8,
    pub name: String,
    pub size: u8,
    pub default: bool,
    pub writable: bool,
    pub value: Option<i32>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, specta::Type)]
#[serde(rename_all = "camelCase")]
pub struct JointRange {
    pub slot_id: String,
    pub min_rad: f64,
    pub max_rad: f64,
}

/// The physical Dynamixel assigned to one body actuator slot.  The slot name is
/// the stable body-side identity; the ID is allowed to vary per robot.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, specta::Type)]
#[serde(rename_all = "camelCase")]
pub struct ServoJointBinding {
    pub slot_id: String,
    pub physical_id: u8,
    pub direction: i8,
    pub zero_offset_rad: f64,
    #[serde(default)]
    pub min_rad: Option<f64>,
    #[serde(default)]
    pub max_rad: Option<f64>,
}

impl ServoJointBinding {
    pub fn official(census: &[ServoIdentity]) -> Vec<Self> {
        duck_control::model::JOINT_NAMES
            .iter()
            .copied()
            .zip(duck_control::model::JOINT_IDS)
            .filter(|(_, id)| census.iter().any(|servo| servo.servo_id == *id))
            .map(|(slot_id, physical_id)| Self {
                slot_id: slot_id.to_owned(),
                physical_id,
                direction: 1,
                zero_offset_rad: 0.0,
                min_rad: None,
                max_rad: None,
            })
            .collect()
    }
}

#[cfg(test)]
mod joint_binding_tests {
    use super::{ServoIdentity, ServoJointBinding};

    #[test]
    fn official_census_is_ordered_by_body_slots() {
        let census = [
            ServoIdentity {
                servo_id: 14,
                model: "hd1910".into(),
            },
            ServoIdentity {
                servo_id: 20,
                model: "hd1910".into(),
            },
            ServoIdentity {
                servo_id: 10,
                model: "hd1910".into(),
            },
        ];
        let bindings = ServoJointBinding::official(&census);
        assert_eq!(
            bindings
                .iter()
                .map(|binding| binding.slot_id.as_str())
                .collect::<Vec<_>>(),
            ["left_hip_yaw", "right_hip_yaw", "right_ankle",]
        );
        assert_eq!(
            bindings
                .iter()
                .map(|binding| binding.physical_id)
                .collect::<Vec<_>>(),
            [20, 10, 14]
        );
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize, specta::Type)]
#[serde(rename_all = "camelCase")]
pub struct ServoIdentity {
    pub servo_id: u8,
    pub model: String,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize, specta::Type)]
#[serde(rename_all = "camelCase")]
pub struct ReconnectHint {
    pub port_name: String,
    pub vid: Option<u16>,
    pub pid: Option<u16>,
    pub adapter_serial: Option<String>,
    pub product: Option<String>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize, specta::Type)]
#[serde(rename_all = "camelCase")]
pub struct SerialPortCandidate {
    pub port_name: String,
    pub vid: Option<u16>,
    pub pid: Option<u16>,
    pub adapter_serial: Option<String>,
    pub product: Option<String>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize, specta::Type)]
#[serde(rename_all = "camelCase")]
pub enum MatchResult {
    New,
    Unique { physical_machine_id: String },
    IdentityConflict,
    IncompleteCensus,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, specta::Type)]
#[serde(rename_all = "PascalCase")]
pub enum DiagnosticCode {
    PortEnumerateFailed,
    OpenFailed,
    ProtocolMismatch,
    Timeout,
    ShortRead,
    CrcError,
    IncompleteCensus,
    IdentityConflict,
}

#[derive(Debug, Clone, Serialize, Deserialize, specta::Type)]
#[serde(rename_all = "camelCase")]
pub struct DetectedPhysicalMachine {
    pub census: Vec<ServoIdentity>,
    pub md5: Option<String>,
    pub reconnect_hint: Option<ReconnectHint>,
    pub match_result: MatchResult,
    pub diagnostics: Vec<DiagnosticCode>,
}

#[derive(Debug, Clone, Serialize, Deserialize, specta::Type)]
#[serde(rename_all = "camelCase")]
pub struct ActionRecord {
    pub id: String,
    pub name: String,
    pub kind: ActionKind,
    pub updated_at: String,
    pub source: ActionSource,
}

#[derive(Debug, Clone, Serialize, Deserialize, specta::Type)]
#[serde(rename_all = "camelCase")]
pub struct LoopStatus {
    pub phase: LoopPhase,
    #[specta(type = specta_typescript::Number)]
    pub session: u64,
    #[specta(type = specta_typescript::Number)]
    pub tick: u64,
    pub torque: bool,
    #[specta(type = Vec<specta_typescript::Number>)]
    pub positions: Vec<f64>,
    #[specta(type = Vec<specta_typescript::Number>)]
    pub targets: Vec<f64>,
    pub world: WorldState,
    pub sensor_failures: u32,
    pub last_error: Option<String>,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, specta::Type)]
#[serde(rename_all = "camelCase")]
pub struct WorldState {
    pub position: [f64; 2],
    pub yaw: f64,
    pub distance_m: f64,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, specta::Type)]
#[serde(rename_all = "camelCase")]
pub struct LoopCommandInput {
    pub twist: [f64; 3],
    pub head: [f64; 4],
    pub body: [f64; 3],
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, specta::Type)]
#[serde(rename_all = "camelCase")]
pub struct PreviewInput {
    pub twist: [f64; 3],
    pub head: [f64; 4],
    pub body: [f64; 3],
    pub target_distance_m: Option<f64>,
}

#[derive(Debug, Clone, Serialize, Deserialize, specta::Type)]
#[serde(rename_all = "camelCase", tag = "kind")]
pub enum LoopAction {
    Power {
        on: bool,
    },
    Drive {
        on: bool,
    },
    Stop,
    Estop,
    Reconnect {
        #[specta(type = specta_typescript::Number)]
        session: u64,
    },
}
