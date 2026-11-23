//! Application contract shared by native simulation, physical deployment, and training.

use duck_body::{BodyIdentity, BodyPhysical};
use serde::Serialize;
use servo::{ServoHardwareSpecs, ServoModel};
use sha2::{Digest, Sha256};

use crate::models::assets::SourceAsset;
use crate::models::parameter_sources::ParameterSource;
use crate::models::scene_recipe::SceneRecipe;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum ExecutionKind {
    Simulation,
    Physical,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
#[serde(rename_all = "kebab-case")]
pub enum ModelKind {
    Xl330,
    Hd1910,
    Hd1910Compatible,
}

impl ModelKind {
    pub const fn servo_model(self) -> ServoModel {
        match self {
            Self::Xl330 => ServoModel::Xl330M288,
            Self::Hd1910 | Self::Hd1910Compatible => ServoModel::Hd1910,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Selection {
    pub execution_kind: ExecutionKind,
    pub model_kind: ModelKind,
    pub servo_model: ServoModel,
}

impl Selection {
    pub fn validate(self) -> Result<(), String> {
        if self.servo_model != self.model_kind.servo_model() {
            return Err(format!(
                "model {:?} cannot use device {}",
                self.model_kind,
                self.servo_model.key()
            ));
        }
        if self.execution_kind == ExecutionKind::Physical
            && self.model_kind == ModelKind::Hd1910Compatible
        {
            return Err(super::hd1910_compatible::physical::UNSUPPORTED.into());
        }
        Ok(())
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
#[serde(tag = "protocol", rename_all = "snake_case")]
pub enum PhysicalGains {
    Xl330 { p: u16, i: u16, d: u16 },
    Hd1910 { p: u8 },
}

impl PhysicalGains {
    pub const fn position_p(self) -> u16 {
        match self {
            Self::Xl330 { p, .. } => p,
            Self::Hd1910 { p } => p as u16,
        }
    }

    pub const fn servo_model(self) -> ServoModel {
        match self {
            Self::Xl330 { .. } => ServoModel::Xl330M288,
            Self::Hd1910 { .. } => ServoModel::Hd1910,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, serde::Deserialize)]
pub struct BodyPeriods {
    pub control_s: f64,
    pub physics_s: Option<f64>,
    pub physics_substeps: Option<u32>,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize)]
pub struct DeviceFacts {
    pub encoder_resolution: u32,
    pub velocity_rad_s_per_count: f64,
    pub protocol: &'static str,
    pub hardware: Option<ServoHardwareSpecs>,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize)]
pub struct SimulationTuning {
    pub damping: f64,
    pub friction_loss: f64,
    pub armature: f64,
    pub kp: f64,
    pub force_range: f64,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize)]
pub struct BamParameters {
    pub motor_name: &'static str,
    pub model: &'static str,
    pub vin: f64,
    pub kp: f64,
    pub error_gain: f64,
    pub max_pwm: f64,
    pub kt: f64,
    pub resistance: f64,
    pub armature: f64,
    pub identified_position_offset_rad: f64,
    pub identified_delay_s: f64,
    pub friction_base: f64,
    pub friction_stribeck: f64,
    pub friction_viscous: f64,
    pub load_friction_motor: f64,
    pub load_friction_external: f64,
    pub load_friction_motor_stribeck: f64,
    pub load_friction_external_stribeck: f64,
    pub load_friction_motor_quad: f64,
    pub load_friction_external_quad: f64,
    pub dtheta_stribeck: f64,
    pub alpha: f64,
}

impl BamParameters {
    pub(crate) fn position_stiffness(self) -> f64 {
        self.vin * self.kt / self.resistance * self.kp * self.error_gain
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize)]
pub struct ActuatorResponse {
    pub delay_steps: [usize; 2],
    pub encoder_through_backlash: bool,
    pub position_gain_scale: f64,
    pub back_emf_scale: f64,
    pub friction_scale: f64,
}

#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct ResolvedSimulationProfile {
    pub id: &'static str,
    pub scenario: SceneRecipe,
    pub source_assets: Vec<SourceAsset>,
    pub nominal_motor: BamParameters,
    pub motor: BamParameters,
    pub response: ActuatorResponse,
    pub root_home_pose: [f64; 7],
    pub delay_seed: u64,
    pub voltage_drop_gain: f64,
    pub minimum_voltage_v: f64,
    pub compatibility_revision: Option<u32>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum GainMode {
    FixedModel,
    Configurable,
}

#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct BodyExecutionProfile {
    pub format_version: u32,
    pub body: BodyIdentity,
    pub execution_kind: ExecutionKind,
    pub model_kind: ModelKind,
    pub servo_model: &'static str,
    pub device_facts: DeviceFacts,
    pub periods: BodyPeriods,
    pub joint_order: Vec<String>,
    pub home_pose_rad: Vec<f64>,
    pub body_physical: BodyPhysical,
    pub gain_mode: GainMode,
    pub physical_gains: Option<PhysicalGains>,
    pub simulation: Option<ResolvedSimulationProfile>,
    pub parameter_sources: Vec<ParameterSource>,
    pub execution_fingerprint: String,
}

impl BodyExecutionProfile {
    pub(crate) fn finalize(mut self) -> Result<Self, String> {
        self.execution_fingerprint = format!(
            "{:x}",
            Sha256::digest(serde_json::to_vec(&self).map_err(|error| error.to_string())?,)
        );
        Ok(self)
    }
}
