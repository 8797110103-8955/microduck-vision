use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize, specta::Type)]
#[serde(rename_all = "camelCase")]
pub struct RecordingInfo {
    pub id: String,
    pub app_session_id: String,
    pub duck_id: String,
    pub name: String,
    pub path: String,
    pub started_at_ms: f64,
    pub ended_at_ms: Option<f64>,
    pub state: String,
    pub frames: u32,
    pub dropped_frames: u32,
    pub error: Option<String>,
}

#[derive(Debug, Clone, Serialize, Deserialize, specta::Type)]
#[serde(rename_all = "camelCase")]
pub struct RecordingPolicy {
    pub observation: Vec<f32>,
    pub action: Vec<f32>,
    pub raw_targets: Option<Vec<f64>>,
    pub applied_targets: Option<Vec<f64>>,
    pub twist: [f64; 3],
    pub head: [f64; 4],
    pub body: [f64; 3],
    pub positions: Vec<f64>,
    pub velocities: Vec<f64>,
    pub valid: Vec<bool>,
    pub gyro: [f64; 3],
    pub gravity: [f64; 3],
    pub imu_source: String,
    pub saturation_reasons: Vec<String>,
}

#[derive(Debug, Clone, Serialize, Deserialize, specta::Type)]
#[serde(rename_all = "camelCase")]
pub struct RecordingFrame {
    pub recording_id: String,
    pub duck_id: String,
    pub segment_id: u32,
    pub episode_id: u32,
    pub elapsed_s: f64,
    pub control_started_s: f64,
    pub status: LoopStatus,
    pub policy: Option<RecordingPolicy>,
}

#[derive(Debug, Clone, Serialize, specta::Type)]
#[serde(rename_all = "camelCase")]
pub struct RecordingPreviewFrame {
    #[serde(flatten)]
    pub frame: RecordingFrame,
    pub execution: Option<RecordingExecution>,
}

#[derive(Debug, Clone, Serialize, specta::Type)]
#[serde(rename_all = "camelCase")]
pub struct RecordingExecution {
    pub simulation: Option<RecordedModel>,
    pub physical: Option<RecordedModel>,
    pub slots: Vec<RecordedSlot>,
}

#[derive(Debug, Clone, Deserialize, Serialize, specta::Type)]
#[serde(rename_all = "snake_case")]
pub struct RecordedModel {
    pub execution_kind: ExecutionKindKey,
    pub model_kind: ModelKindKey,
    pub servo_model: ServoModelKey,
    pub execution_fingerprint: String,
}

#[derive(Debug, Clone, Deserialize, Serialize, specta::Type)]
#[serde(rename_all = "snake_case")]
pub struct RecordedSlot {
    pub slot_id: String,
    pub physical_id: Option<u8>,
    pub source: String,
}

#[derive(Debug, Clone, Serialize, Deserialize, specta::Type)]
#[serde(rename_all = "camelCase")]
pub struct RecordingWindow {
    pub id: String,
    pub segment_id: u32,
    pub start_s: f64,
    pub end_s: f64,
}

#[derive(Debug, Clone, Serialize, specta::Type)]
#[serde(rename_all = "camelCase")]
pub struct RecordingMetric {
    pub field: String,
    pub samples: u32,
    pub rmse: f64,
    pub max_error: f64,
}

#[derive(Debug, Clone, Serialize, specta::Type)]
#[serde(rename_all = "camelCase")]
pub struct RecordingComparison {
    pub metrics: Vec<RecordingMetric>,
    pub matched_frames: u32,
    pub unmatched_frames: u32,
    pub path: String,
    pub warnings: Vec<String>,
}

#[derive(Debug, Clone, Serialize, specta::Type)]
#[serde(rename_all = "camelCase")]
pub struct RecordingExport {
    pub path: String,
    pub samples: u32,
    pub excluded_frames: u32,
}

#[derive(Debug, Clone, Deserialize, specta::Type)]
#[serde(rename_all = "camelCase")]
pub struct ActionInput {
    pub name: String,
    pub kind: String,
}

#[derive(Debug, Clone, Deserialize, specta::Type)]
#[serde(rename_all = "camelCase")]
pub struct TrainingStartInput {
    /// Executable to run without going through a shell (for example `uv`).
    pub program: String,
    pub args: Vec<String>,
    pub working_dir: String,
    pub virtual_machine_id: String,
    pub policy_id: Option<String>,
    pub experiment_id: Option<String>,
    pub recipe: String,
    pub teacher_motion_provenance: String,
    #[serde(default)]
    pub dataset_path: Option<String>,
}

#[derive(Debug, Clone, Deserialize, specta::Type)]
#[serde(rename_all = "camelCase")]
pub struct TeacherEvaluationInput {
    pub working_dir: String,
    pub virtual_machine_id: String,
    pub teacher_policy_id: String,
    pub task_id: String,
    pub steps: u32,
    pub seed: u32,
}

#[derive(Debug, Clone, Serialize, specta::Type)]
#[serde(rename_all = "camelCase")]
pub struct TrainingPolicyLayout {
    #[specta(type = specta_typescript::Number)]
    pub obs_len: usize,
    #[specta(type = specta_typescript::Number)]
    pub action_len: usize,
    pub control_hz: u32,
    pub command_encoding: Option<String>,
}

#[derive(Debug, Clone, Serialize, specta::Type)]
#[serde(rename_all = "camelCase")]
pub struct TrainingRunMetadata {
    pub experiment_id: String,
    pub body_template_id: String,
    pub body_version: u32,
    pub body_source_hash: String,
    pub policy_id: Option<String>,
    pub policy_layout: TrainingPolicyLayout,
    pub servo_profile: ServoModelKey,
    pub recipe: String,
    pub teacher_motion_provenance: String,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, specta::Type)]
#[serde(rename_all = "lowercase")]
pub enum TrainingPhase {
    Starting,
    Running,
    Finished,
    Failed,
    Stopped,
}

#[derive(Debug, Clone, Serialize, specta::Type)]
#[serde(rename_all = "camelCase")]
pub struct TrainingProgress {
    pub run_id: String,
    pub result_dir: String,
    pub metadata: TrainingRunMetadata,
    pub phase: TrainingPhase,
    #[specta(type = Option<specta_typescript::Number>)]
    pub step: Option<u64>,
    #[specta(type = Option<specta_typescript::Number>)]
    pub total_steps: Option<u64>,
    pub percent: Option<f64>,
    pub reward: Option<f64>,
    pub loss: Option<f64>,
    pub message: String,
    pub line: String,
    pub exit_code: Option<i32>,
}

#[derive(Debug, Clone, Serialize, Deserialize, specta::Type)]
#[serde(rename_all = "camelCase")]
pub struct TrainingFrame {
    pub run_id: String,
    #[specta(type = specta_typescript::Number)]
    pub sequence: u64,
    pub environment_index: u32,
    pub model_kind: ModelKindKey,
    pub execution_fingerprint: String,
    pub positions: Vec<Option<f64>>,
    pub trunk_position: [f64; 3],
    pub trunk_quaternion: [f64; 4],
    pub frame: SceneFrame,
    pub preview_scene: Option<SceneInfo>,
}

#[derive(Debug, Clone, Deserialize, specta::Type)]
#[serde(rename_all = "camelCase")]
pub struct VirtualMachineInput {
    pub name: String,
    pub body_id: String,
    pub execution_kind: ExecutionKindKey,
    pub model_kind: ModelKindKey,
    pub servo_model: String,
    pub simulation: SimulationParameters,
}

#[derive(Debug, Clone, Deserialize, specta::Type)]
#[serde(rename_all = "camelCase")]
pub struct PhysicalMachineInput {
    pub name: String,
    pub usb_port: String,
    pub machine_code: String,
}

#[derive(Debug, Clone, Deserialize, specta::Type)]
#[serde(rename_all = "camelCase")]
pub struct PhysicalMachineDetectInput {
    pub census: Vec<ServoIdentity>,
    pub reconnect_hint: Option<ReconnectHint>,
}

#[derive(Debug, Clone, Deserialize, specta::Type)]
#[serde(rename_all = "camelCase")]
pub struct PhysicalMachineSaveInput {
    pub census: Vec<ServoIdentity>,
    pub reconnect_hint: Option<ReconnectHint>,
    pub name: Option<String>,
}

#[derive(Debug, Clone, Deserialize, specta::Type)]
#[serde(rename_all = "camelCase")]
pub struct PhysicalMachineProbeInput {
    pub port_name: String,
    pub baud: u32,
    pub reconnect_hint: Option<ReconnectHint>,
}

#[derive(Debug, Clone, Deserialize, specta::Type)]
#[serde(rename_all = "camelCase")]
pub struct PhysicalMachineReconnectInput {
    pub id: String,
    pub census: Vec<ServoIdentity>,
    pub reconnect_hint: Option<ReconnectHint>,
}

#[derive(Debug, Clone, Deserialize, specta::Type)]
#[serde(rename_all = "snake_case")]
pub enum LoopAccess {
    ReadOnly,
    Control,
}

#[derive(Debug, Clone, Deserialize, specta::Type)]
#[serde(rename_all = "camelCase")]
pub struct LoopOpenInput {
    pub virtual_machine_id: String,
    pub physical_machine_id: String,
    pub access: LoopAccess,
    #[serde(default)]
    pub policy_id: Option<String>,
}

#[derive(Debug, Clone, Deserialize, specta::Type)]
#[serde(rename_all = "camelCase")]
pub struct RuntimeOpenInput {
    pub virtual_machine_id: String,
    pub physical_machine_id: Option<String>,
    pub access: LoopAccess,
    pub policy_id: Option<String>,
}

#[derive(Debug, Clone, Copy, Deserialize, specta::Type)]
#[serde(rename_all = "camelCase")]
pub enum RuntimeCloseScope {
    Runtime,
    ReadOnlySource,
}

#[derive(Debug, Clone, Deserialize, specta::Type)]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
pub struct PhysicalMachineBindingInput {
    pub physical_machine_id: String,
    pub slot_id: String,
    pub direction: i8,
}

#[derive(Debug, Clone, Deserialize, specta::Type)]
#[serde(rename_all = "camelCase")]
pub struct VirtualMachineBindingInput {
    pub virtual_machine_id: String,
    pub slot_id: String,
    pub physical_id: Option<u8>,
}

#[derive(Debug, Clone, Deserialize, specta::Type)]
#[serde(rename_all = "camelCase")]
pub struct VirtualJointBindingInput {
    pub slot_id: String,
    pub physical_id: Option<u8>,
}

#[derive(Debug, Clone, Deserialize, specta::Type)]
#[serde(rename_all = "camelCase")]
pub struct PhysicalMachineRegisterWriteInput {
    pub physical_machine_id: String,
    pub servo_id: u8,
    pub name: String,
    pub value: i32,
}

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
pub struct SimulationState {
    #[specta(type = specta_typescript::Number)]
    pub physics_step: u64,
    pub time: f64,
    pub trunk_position: [f64; 3],
    /// MuJoCo quaternion order is w,x,y,z. SceneFrame uses the same world basis.
    pub trunk_quaternion: [f64; 4],
    pub motor_positions: Vec<f64>,
    pub passive_positions: Vec<f64>,
    /// Freejoint qvel: world linear velocity followed by local angular velocity.
    pub base_velocity: [f64; 6],
    pub contacts: Vec<[i32; 2]>,
    pub motor_torques: Vec<f64>,
    pub friction_budgets: Vec<f64>,
    pub effective_voltage: f64,
    pub frame: SceneFrame,
}

#[derive(Debug, Clone, Serialize, Deserialize, specta::Type)]
#[serde(rename_all = "camelCase")]
pub struct SceneInfo {
    pub model: SceneModel,
    pub mesh_dir: String,
    pub default_frame: SceneFrame,
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
    /// Scene placement offset in metres, using the MuJoCo right-handed Z-up basis.
    pub world_position: [f64; 3],
}

#[derive(Debug, Clone, Serialize, Deserialize, specta::Type)]
#[serde(rename_all = "camelCase")]
pub struct VirtualMachine {
    pub id: String,
    pub name: String,
    pub body_id: String,
    pub execution_kind: ExecutionKindKey,
    pub model_kind: ModelKindKey,
    pub servo_model: ServoModelKey,
    pub simulation: SimulationParameters,
    pub startup_selected: bool,
    pub referenced_by: Vec<String>,
    #[serde(default)]
    pub joint_bindings: Vec<VirtualJointBinding>,
}

#[derive(Debug, Clone, Serialize, Deserialize, specta::Type)]
#[serde(rename_all = "camelCase")]
pub struct PhysicalMachine {
    pub id: String,
    pub name: String,
    pub execution_kind: ExecutionKindKey,
    pub model_kind: Option<ModelKindKey>,
    pub servo_model: Option<ServoModelKey>,
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
    /// Installation direction overrides, independent of the virtual body's servo IDs.
    #[serde(default)]
    pub joint_directions: Vec<ServoJointDirection>,
    /// Installation zero and limits keyed by body slot; bus IDs belong to the virtual machine.
    #[serde(default)]
    pub joint_bindings: Vec<ServoJointCalibration>,
}

#[derive(Debug, Clone, Serialize, specta::Type)]
#[serde(rename_all = "camelCase")]
pub struct EncoderCalibrationResult {
    pub machine: PhysicalMachine,
    pub calibrated_servo_ids: Vec<u8>,
    pub error: Option<String>,
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

/// A merged body-to-servo calibration used by the loop boundary. The virtual
/// machine supplies the ID; the selected physical machine supplies direction
/// and installation calibration.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, specta::Type)]
#[serde(rename_all = "camelCase")]
pub struct ServoJointBinding {
    pub slot_id: String,
    pub physical_id: u8,
    pub direction: i8,
    /// Raw servo angle at canonical joint zero; changing direction preserves this angle.
    pub zero_offset_rad: f64,
    #[serde(default)]
    pub min_rad: Option<f64>,
    #[serde(default)]
    pub max_rad: Option<f64>,
}

/// A virtual machine's body slot to the physical servo ID used when a physical
/// machine is attached. Direction and installation calibration belong to the
/// selected physical machine and are merged at body construction time.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, specta::Type)]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
pub struct VirtualJointBinding {
    pub slot_id: String,
    pub physical_id: u8,
}

/// Direction override for a physical robot installation.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, specta::Type)]
#[serde(rename_all = "camelCase")]
pub struct ServoJointDirection {
    pub slot_id: String,
    pub direction: i8,
}

/// Installation zero and limits belong to a physical limb, independently of bus IDs.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, specta::Type)]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
pub struct ServoJointCalibration {
    pub slot_id: String,
    pub zero_offset_rad: f64,
    pub min_rad: Option<f64>,
    pub max_rad: Option<f64>,
}

impl From<&ServoJointBinding> for ServoJointCalibration {
    fn from(binding: &ServoJointBinding) -> Self {
        Self {
            slot_id: binding.slot_id.clone(),
            zero_offset_rad: binding.zero_offset_rad,
            min_rad: binding.min_rad,
            max_rad: binding.max_rad,
        }
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
pub struct JointRangeState {
    pub slot_id: String,
    pub target_rad: f64,
    pub min_rad: f64,
    pub max_rad: f64,
    pub below_min: bool,
    pub above_max: bool,
}

#[derive(Debug, Clone, Serialize, Deserialize, specta::Type)]
#[serde(rename_all = "camelCase")]
pub struct ActiveBody {
    pub virtual_machine_id: Option<String>,
    pub physical_machine_id: Option<String>,
}

#[derive(Debug, Clone, Serialize, Deserialize, specta::Type)]
#[serde(rename_all = "camelCase")]
pub struct LoopStatus {
    pub has_execution: bool,
    pub active_body: Option<ActiveBody>,
    pub runtime_state: RuntimeState,
    pub simulation: Option<SimulationState>,
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
    pub joint_range_states: Vec<JointRangeState>,
    #[specta(type = specta_typescript::Number)]
    pub runtime_generation: u64,
    #[specta(type = specta_typescript::Number)]
    pub execution_generation: u64,
    #[specta(type = specta_typescript::Number)]
    pub config_revision: u64,
    pub read_sources: Vec<ReadSourceStatus>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, specta::Type)]
#[serde(rename_all = "camelCase")]
pub enum RuntimeState {
    Running,
    Faulted,
}

#[derive(Debug, Clone, Serialize, Deserialize, specta::Type)]
#[serde(rename_all = "camelCase")]
pub struct ReadSourceStatus {
    pub valid_positions: Vec<bool>,
    pub valid_velocities: Vec<bool>,
    pub id: String,
    #[specta(type = specta_typescript::Number)]
    pub generation: u64,
    #[specta(type = specta_typescript::Number)]
    pub sequence: u64,
    #[specta(type = specta_typescript::Number)]
    pub sampled_at: u64,
    pub age_ms: Option<f64>,
    pub health: ReadSourceHealth,
    #[specta(type = Vec<specta_typescript::Number>)]
    pub positions: Vec<f64>,
    #[specta(type = Vec<specta_typescript::Number>)]
    pub velocities: Vec<f64>,
    pub last_error: Option<String>,
}

#[derive(Debug, Clone, Serialize, Deserialize, specta::Type)]
#[serde(rename_all = "camelCase")]
pub enum ReadSourceHealth {
    Starting,
    Fresh,
    Stale,
    Offline,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, specta::Type)]
#[serde(rename_all = "camelCase")]
pub struct WorldState {
    /// Accumulated scene displacement in the MuJoCo X/Y plane, in metres.
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
    #[specta(type = [specta_typescript::Number; 3])]
    pub twist: [f64; 3],
    #[specta(type = [specta_typescript::Number; 4])]
    pub head: [f64; 4],
    #[specta(type = [specta_typescript::Number; 3])]
    pub body: [f64; 3],
    pub target_distance_m: Option<f64>,
}

/// Mutations of the selected execution session. Sampling and physics advancement are owned by
/// the Rust clock and deliberately have no frontend command variant.
#[derive(Debug, Clone, Serialize, Deserialize, specta::Type)]
#[serde(rename_all = "camelCase", tag = "kind")]
pub enum RuntimeCommand {
    Targets {
        #[specta(type = Vec<specta_typescript::Number>)]
        targets: Vec<f64>,
    },
    Input {
        input: PreviewInput,
    },
    Policy {
        policy_id: String,
        input: PreviewInput,
    },
    Action {
        action: LoopAction,
    },
    ResetWorld,
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

#[derive(Debug, Clone, Serialize, Deserialize, specta::Type)]
#[serde(rename_all = "camelCase")]
pub struct PolicyCommand {
    pub slot: Option<String>,
    pub encoding: Option<String>,
    pub idle: Option<[f64; 3]>,
    pub twist: Option<String>,
    pub head: Option<String>,
    pub body: Option<String>,
    #[serde(alias = "period_s")]
    pub period_s: Option<f64>,
    #[serde(alias = "end_phase")]
    pub end_phase: Option<f64>,
    pub sit: Option<f64>,
    pub stand: Option<f64>,
}

#[derive(Debug, Clone, Serialize, Deserialize, specta::Type)]
#[serde(rename_all = "camelCase")]
pub struct PolicyEntry {
    pub id: String,
    pub name: String,
    pub file: String,
    pub source_url: String,
    pub cached: bool,
    pub selected: bool,
    #[specta(type = specta_typescript::Number)]
    pub size: u64,
    #[serde(default)]
    pub kind: Option<String>,
    #[serde(default)]
    pub slot: Option<String>,
    #[serde(default)]
    pub duration_s: Option<f64>,
    #[serde(default)]
    pub command: Option<PolicyCommand>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, specta::Type)]
#[serde(rename_all = "camelCase")]
pub enum ServoModelKey {
    #[serde(rename = "hd1910")]
    Hd1910,
    #[serde(rename = "xl330")]
    Xl330,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, specta::Type)]
#[serde(rename_all = "kebab-case")]
pub enum ModelKindKey {
    Xl330,
    Hd1910,
    Hd1910Compatible,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, specta::Type)]
#[serde(rename_all = "snake_case")]
pub enum ExecutionKindKey {
    Simulation,
    Physical,
}

#[derive(Debug, Clone, Serialize, Deserialize, specta::Type)]
#[serde(rename_all = "camelCase")]
pub struct ServoCatalogEntry {
    pub key: ServoModelKey,
    pub label: String,
    pub protocol: String,
    pub default_binding: bool,
    pub capabilities: Vec<String>,
    pub model_number: u16,
    pub resolution: u32,
    pub kp_divisor: Option<u32>,
    pub pwm_limit: Option<u32>,
    pub default_baud: u32,
    pub baud_rates: Vec<u32>,
    pub register_count: u32,
    pub supports_fast_sync: bool,
    pub supports_center_calibration: bool,
    pub reset_needs_unlock: bool,
    pub supports_reboot: bool,
    pub protocol_definition: String,
    pub velocity_rad_per_count: f64,
    pub volts_per_count: f64,
    pub milliamps_per_count: f64,
    pub move_block: Vec<String>,
    pub mass_g: f64,
    pub baseline_mass_g: f64,
    pub hardware_mass_g: Option<f64>,
    pub hardware_gear_ratio: Option<String>,
    pub hardware_backlash_deg: Option<f64>,
    pub hardware_voltage_range: Option<[f64; 2]>,
    pub datasheet_torque_constant: Option<f64>,
    pub no_load_speed_rpm: Option<f64>,
    pub peak_stall_torque_kg_cm: Option<f64>,
    pub bam_motor: String,
    pub bam_model: String,
    pub bam_vin: f64,
    pub bam_kp: f64,
    pub bam_error_gain: f64,
    pub bam_max_pwm: f64,
    pub bam_kt: f64,
    pub bam_resistance: f64,
    pub bam_armature: f64,
    pub bam_q_offset: f64,
    pub bam_command_delay: f64,
    pub bam_friction_base: f64,
    pub bam_friction_stribeck: f64,
    pub bam_friction_viscous: f64,
    pub bam_load_friction_motor: f64,
    pub bam_load_friction_external: f64,
    pub bam_load_friction_motor_stribeck: f64,
    pub bam_load_friction_external_stribeck: f64,
    pub bam_load_friction_motor_quad: f64,
    pub bam_load_friction_external_quad: f64,
    pub bam_dtheta_stribeck: f64,
    pub bam_alpha: f64,
    pub kp_fw: f64,
    pub vin_range: [f64; 2],
    pub vin_drop_gain_range: [f64; 2],
    pub vin_min: f64,
    pub delay_steps: [u32; 2],
    pub backlash_deg: f64,
    pub p_at_kp_fw: Option<u8>,
    pub sim_damping: f64,
    pub sim_friction_loss: f64,
    pub sim_armature: f64,
    pub sim_kp: f64,
    pub sim_force_range: f64,
}
