//! Servo capabilities and measured or documented device specifications.

use serde::{Deserialize, Serialize};

use crate::{hd1910, xl330};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum ServoModel {
    Xl330M288,
    Hd1910,
    Sts3215,
    Scs0009,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ServoCapability {
    PositionControl,
    PresentPosition,
    PresentVelocity,
    TorqueEnable,
    SetGain,
    PresentCurrent,
    PresentVoltage,
    PresentTemperature,
    Reboot,
    FastSyncRead,
}

#[derive(Debug, Clone, Copy)]
pub struct ServoCapabilities(pub &'static [ServoCapability]);

const FEETECH_CAPABILITIES: ServoCapabilities = ServoCapabilities(&[
    ServoCapability::PositionControl,
    ServoCapability::PresentPosition,
    ServoCapability::PresentVelocity,
    ServoCapability::TorqueEnable,
    ServoCapability::SetGain,
    ServoCapability::PresentCurrent,
    ServoCapability::PresentVoltage,
    ServoCapability::PresentTemperature,
    ServoCapability::Reboot,
]);

pub const XL330_CAPABILITIES: ServoCapabilities = ServoCapabilities(&[
    ServoCapability::PositionControl,
    ServoCapability::PresentPosition,
    ServoCapability::PresentVelocity,
    ServoCapability::TorqueEnable,
    ServoCapability::SetGain,
    ServoCapability::PresentCurrent,
    ServoCapability::PresentVoltage,
    ServoCapability::PresentTemperature,
    ServoCapability::Reboot,
    ServoCapability::FastSyncRead,
]);

#[derive(Debug, Clone, Copy, PartialEq, Serialize)]
pub struct ServoHardwareSpecs {
    pub mass_g: f64,
    pub gear_ratio: &'static str,
    pub backlash_deg: f64,
    pub voltage_range: [f64; 2],
    pub torque_constant: f64,
    pub no_load_speed_rpm: f64,
    pub peak_stall_torque_kg_cm: f64,
}

#[derive(Debug, Clone, Copy)]
pub struct ServoModelFacts {
    pub model_number: u16,
    pub resolution: u32,
    pub kp_divisor: Option<u32>,
    pub pwm_limit: Option<u32>,
    pub default_baud: u32,
    pub baud_rates: &'static [u32],
    pub register_count: usize,
    pub supports_fast_sync: bool,
    pub supports_center_calibration: bool,
    pub reset_needs_unlock: bool,
    pub supports_reboot: bool,
    pub protocol_definition: &'static str,
    pub velocity_rad_per_count: f64,
    pub volts_per_count: f64,
    pub milliamps_per_count: f64,
    pub move_block: &'static [&'static str],
    pub hardware: Option<ServoHardwareSpecs>,
}

impl ServoModel {
    pub fn key(self) -> &'static str {
        match self {
            Self::Xl330M288 => "xl330",
            Self::Hd1910 => "hd1910",
            Self::Sts3215 => "sts3215",
            Self::Scs0009 => "scs0009",
        }
    }

    pub fn capabilities(self) -> ServoCapabilities {
        match self {
            Self::Xl330M288 => XL330_CAPABILITIES,
            Self::Hd1910 | Self::Sts3215 | Self::Scs0009 => FEETECH_CAPABILITIES,
        }
    }

    pub fn hardware_specs(self) -> Option<ServoHardwareSpecs> {
        (self == Self::Hd1910).then_some(ServoHardwareSpecs {
            mass_g: 21.0,
            gear_ratio: "1:320",
            backlash_deg: 0.5,
            voltage_range: [4.0, 8.4],
            torque_constant: 7.5,
            no_load_speed_rpm: 73.0,
            peak_stall_torque_kg_cm: 9.0,
        })
    }

    pub fn facts(self) -> Option<ServoModelFacts> {
        Some(match self {
            Self::Xl330M288 => ServoModelFacts {
                model_number: xl330::XL330_MODEL_NUMBER,
                resolution: xl330::XL330_RESOLUTION as u32,
                kp_divisor: Some(xl330::XL330_KP_DIVISOR),
                pwm_limit: Some(xl330::XL330_PWM_LIMIT),
                default_baud: xl330::XL330_DEFAULT_BAUD,
                baud_rates: &xl330::XL330_BAUD_RATES,
                register_count: xl330::registers().len(),
                supports_fast_sync: true,
                supports_center_calibration: false,
                reset_needs_unlock: false,
                supports_reboot: true,
                protocol_definition: "XL330",
                velocity_rad_per_count: xl330::XL330_VELOCITY_RAD_PER_SEC_PER_COUNT,
                volts_per_count: xl330::XL330_VOLTAGE_PER_COUNT,
                milliamps_per_count: xl330::XL330_CURRENT_MA_PER_COUNT,
                move_block: &["goal_position"],
                hardware: self.hardware_specs(),
            },
            Self::Hd1910 => ServoModelFacts {
                model_number: hd1910::HD1910_MODEL_NUMBER,
                resolution: hd1910::HD1910_RESOLUTION,
                kp_divisor: None,
                pwm_limit: None,
                default_baud: hd1910::HD1910_BAUD_RATES[0],
                baud_rates: &hd1910::HD1910_BAUD_RATES,
                // The HLS table extends the STS table by five net entries.
                register_count: hd1910::registers().len() + 5,
                supports_fast_sync: false,
                supports_center_calibration: true,
                reset_needs_unlock: true,
                supports_reboot: false,
                protocol_definition: "STS3215 / HD1910",
                velocity_rad_per_count: hd1910::HD1910_VELOCITY_RAD_PER_SEC_PER_COUNT,
                volts_per_count: hd1910::HD1910_VOLTAGE_PER_COUNT,
                milliamps_per_count: hd1910::HD1910_CURRENT_MA_PER_COUNT,
                move_block: hd1910::HD1910_MOVE_BLOCK,
                hardware: self.hardware_specs(),
            },
            Self::Sts3215 | Self::Scs0009 => return None,
        })
    }
}
