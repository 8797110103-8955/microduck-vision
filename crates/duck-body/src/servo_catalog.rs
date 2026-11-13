use serde::{Deserialize, Serialize};
use servo::ServoModel;
/// Read-only parameter projection used by the machine settings UI.  The values are the
/// built-in body/training facts; bus protocol details remain owned by `servo`.
#[derive(Debug, Clone, PartialEq)]
pub struct ServoProfile {
    pub model: ServoModel,
    pub model_number: u16,
    pub resolution: u32,
    pub kp_divisor: Option<u32>,
    pub pwm_limit: Option<u32>,
    pub default_baud: u32,
    pub baud_rates: Vec<u32>,
    pub register_count: usize,
    pub supports_fast_sync: bool,
    pub supports_center_calibration: bool,
    pub reset_needs_unlock: bool,
    pub supports_reboot: bool,
    pub protocol_definition: &'static str,
    pub velocity_rad_per_count: f64,
    pub volts_per_count: f64,
    pub milliamps_per_count: f64,
    pub move_block: Vec<&'static str>,
    pub mass_g: f64,
    pub baseline_mass_g: f64,
    pub hardware_mass_g: Option<f64>,
    pub hardware_gear_ratio: Option<&'static str>,
    pub hardware_backlash_deg: Option<f64>,
    pub hardware_voltage_range: Option<[f64; 2]>,
    pub datasheet_torque_constant: Option<f64>,
    pub no_load_speed_rpm: Option<f64>,
    pub peak_stall_torque_kg_cm: Option<f64>,
    pub bam_motor: &'static str,
    pub bam_model: &'static str,
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

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, specta::Type)]
#[serde(rename_all = "camelCase")]
pub enum ServoModelKey {
    #[serde(rename = "hd1910")]
    Hd1910,
    #[serde(rename = "xl330")]
    Xl330,
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

pub fn servo_catalog() -> Vec<ServoCatalogEntry> {
    servo_profiles()
        .into_iter()
        .map(|profile| {
            let is_hd1910 = profile.model == ServoModel::Hd1910;
            ServoCatalogEntry {
                key: if is_hd1910 {
                    ServoModelKey::Hd1910
                } else {
                    ServoModelKey::Xl330
                },
                label: if is_hd1910 {
                    "飞特 HD1910".to_owned()
                } else {
                    "XL330".to_owned()
                },
                protocol: if is_hd1910 {
                    "Feetech Protocol 1".to_owned()
                } else {
                    "Dynamixel Protocol 2".to_owned()
                },
                default_binding: is_hd1910,
                capabilities: if is_hd1910 {
                    vec!["position", "velocity", "torque", "gain", "reboot"]
                } else {
                    vec![
                        "position",
                        "velocity",
                        "torque",
                        "gain",
                        "reboot",
                        "fast_sync_read",
                    ]
                }
                .into_iter()
                .map(String::from)
                .collect(),
                model_number: profile.model_number,
                resolution: profile.resolution,
                kp_divisor: profile.kp_divisor,
                pwm_limit: profile.pwm_limit,
                default_baud: profile.default_baud,
                baud_rates: profile.baud_rates,
                register_count: profile.register_count as u32,
                supports_fast_sync: profile.supports_fast_sync,
                supports_center_calibration: profile.supports_center_calibration,
                reset_needs_unlock: profile.reset_needs_unlock,
                supports_reboot: profile.supports_reboot,
                protocol_definition: profile.protocol_definition.to_owned(),
                velocity_rad_per_count: profile.velocity_rad_per_count,
                volts_per_count: profile.volts_per_count,
                milliamps_per_count: profile.milliamps_per_count,
                move_block: profile.move_block.into_iter().map(String::from).collect(),
                mass_g: profile.mass_g,
                baseline_mass_g: profile.baseline_mass_g,
                hardware_mass_g: profile.hardware_mass_g,
                hardware_gear_ratio: profile.hardware_gear_ratio.map(String::from),
                hardware_backlash_deg: profile.hardware_backlash_deg,
                hardware_voltage_range: profile.hardware_voltage_range,
                datasheet_torque_constant: profile.datasheet_torque_constant,
                no_load_speed_rpm: profile.no_load_speed_rpm,
                peak_stall_torque_kg_cm: profile.peak_stall_torque_kg_cm,
                bam_motor: profile.bam_motor.to_owned(),
                bam_model: profile.bam_model.to_owned(),
                bam_vin: profile.bam_vin,
                bam_kp: profile.bam_kp,
                bam_error_gain: profile.bam_error_gain,
                bam_max_pwm: profile.bam_max_pwm,
                bam_kt: profile.bam_kt,
                bam_resistance: profile.bam_resistance,
                bam_armature: profile.bam_armature,
                bam_q_offset: profile.bam_q_offset,
                bam_command_delay: profile.bam_command_delay,
                bam_friction_base: profile.bam_friction_base,
                bam_friction_stribeck: profile.bam_friction_stribeck,
                bam_friction_viscous: profile.bam_friction_viscous,
                bam_load_friction_motor: profile.bam_load_friction_motor,
                bam_load_friction_external: profile.bam_load_friction_external,
                bam_load_friction_motor_stribeck: profile.bam_load_friction_motor_stribeck,
                bam_load_friction_external_stribeck: profile.bam_load_friction_external_stribeck,
                bam_load_friction_motor_quad: profile.bam_load_friction_motor_quad,
                bam_load_friction_external_quad: profile.bam_load_friction_external_quad,
                bam_dtheta_stribeck: profile.bam_dtheta_stribeck,
                bam_alpha: profile.bam_alpha,
                kp_fw: profile.kp_fw,
                vin_range: profile.vin_range,
                vin_drop_gain_range: profile.vin_drop_gain_range,
                vin_min: profile.vin_min,
                delay_steps: profile.delay_steps,
                backlash_deg: profile.backlash_deg,
                p_at_kp_fw: profile.p_at_kp_fw,
                sim_damping: profile.sim_damping,
                sim_friction_loss: profile.sim_friction_loss,
                sim_armature: profile.sim_armature,
                sim_kp: profile.sim_kp,
                sim_force_range: profile.sim_force_range,
            }
        })
        .collect()
}

/// Built-in profiles corresponding to the old servo library's complete read-only view.
pub fn servo_profiles() -> Vec<ServoProfile> {
    let common = |model: ServoModel,
                  model_number: u16,
                  register_count: usize,
                  supports_fast_sync: bool,
                  p_at_kp_fw: Option<u8>,
                  baud_rates: Vec<u32>,
                  supports_center_calibration: bool,
                  reset_needs_unlock: bool,
                  supports_reboot: bool,
                  protocol_definition: &'static str,
                  velocity_rad_per_count: f64,
                  milliamps_per_count: f64,
                  move_block: Vec<&'static str>| ServoProfile {
        model,
        model_number,
        resolution: 4096,
        kp_divisor: None,
        pwm_limit: None,
        default_baud: 1_000_000,
        baud_rates,
        register_count,
        supports_fast_sync,
        supports_center_calibration,
        reset_needs_unlock,
        supports_reboot,
        protocol_definition,
        velocity_rad_per_count,
        volts_per_count: 0.1,
        milliamps_per_count,
        move_block,
        mass_g: 18.0,
        baseline_mass_g: 18.0,
        hardware_mass_g: None,
        hardware_gear_ratio: None,
        hardware_backlash_deg: None,
        hardware_voltage_range: None,
        datasheet_torque_constant: None,
        no_load_speed_rpm: None,
        peak_stall_torque_kg_cm: None,
        bam_motor: "xl330",
        bam_model: "m6",
        bam_vin: 0.0,
        bam_kp: 0.0,
        bam_error_gain: 0.0,
        bam_max_pwm: 0.0,
        bam_kt: 0.0,
        bam_resistance: 0.0,
        bam_armature: 0.0,
        bam_q_offset: 0.0,
        bam_command_delay: 0.0,
        bam_friction_base: 0.0,
        bam_friction_stribeck: 0.0,
        bam_friction_viscous: 0.0,
        bam_load_friction_motor: 0.0,
        bam_load_friction_external: 0.0,
        bam_load_friction_motor_stribeck: 0.0,
        bam_load_friction_external_stribeck: 0.0,
        bam_load_friction_motor_quad: 0.0,
        bam_load_friction_external_quad: 0.0,
        bam_dtheta_stribeck: 0.0,
        bam_alpha: 0.0,
        kp_fw: 200.0,
        vin_range: [6.5, 8.2],
        vin_drop_gain_range: [0.0, 0.2],
        vin_min: 6.0,
        delay_steps: [3, 6],
        backlash_deg: 2.0,
        p_at_kp_fw,
        sim_damping: 0.053,
        sim_friction_loss: 0.0048,
        sim_armature: 0.0018,
        sim_kp: 0.55,
        sim_force_range: 0.96,
    };

    let mut profiles = vec![
        common(
            ServoModel::Xl330M288,
            1200,
            servo::xl330::registers().len(),
            true,
            None,
            vec![
                9_600, 57_600, 115_200, 1_000_000, 2_000_000, 3_000_000, 4_000_000,
            ],
            false,
            false,
            true,
            "XL330",
            0.229 * 2.0 * std::f64::consts::PI / 60.0,
            1.0,
            vec!["goal_position"],
        ),
        common(
            ServoModel::Hd1910,
            7946,
            servo::hd1910::registers().len() + 4,
            false,
            Some(32),
            vec![
                1_000_000, 500_000, 250_000, 128_000, 115_200, 76_800, 57_600, 38_400,
            ],
            true,
            true,
            false,
            "STS3215 / HD1910",
            2.0 * std::f64::consts::PI / 4096.0,
            6.5,
            vec!["acceleration", "goal_position", "goal_time", "goal_speed"],
        ),
    ];

    let xl = &mut profiles[0];
    xl.kp_divisor = Some(256);
    xl.pwm_limit = Some(885);
    xl.hardware_mass_g = None;
    xl.hardware_gear_ratio = None;
    xl.hardware_backlash_deg = None;
    xl.hardware_voltage_range = None;
    xl.datasheet_torque_constant = None;
    xl.no_load_speed_rpm = None;
    xl.peak_stall_torque_kg_cm = None;
    xl.bam_vin = 7.5;
    xl.bam_kp = 400.0;
    xl.bam_error_gain = 0.00287737750222636;
    xl.bam_max_pwm = 1.0;
    xl.bam_kt = 0.3459739511711113;
    xl.bam_resistance = 2.501880848390462;
    xl.bam_armature = 0.0015732226779339757;
    xl.bam_q_offset = 0.01499659309559756;
    xl.bam_command_delay = 0.010220594809412403;
    xl.bam_friction_base = 0.011919956825702578;
    xl.bam_friction_stribeck = 0.0008513719673776765;
    xl.bam_friction_viscous = 0.005788445053875673;
    xl.bam_load_friction_motor = 0.22781736172050673;
    xl.bam_load_friction_external = 0.10651211228971481;
    xl.bam_load_friction_motor_stribeck = 1.4725366831870136e-8;
    xl.bam_load_friction_external_stribeck = 0.14201808120858034;
    xl.bam_load_friction_motor_quad = 0.00526632315329998;
    xl.bam_load_friction_external_quad = 0.00298581585293041;
    xl.bam_dtheta_stribeck = 0.2606668812858925;
    xl.bam_alpha = 8.528815753151498;

    let hd = &mut profiles[1];
    hd.hardware_mass_g = Some(21.0);
    hd.hardware_gear_ratio = Some("1:320");
    hd.hardware_backlash_deg = Some(0.5);
    hd.hardware_voltage_range = Some([4.0, 8.4]);
    hd.datasheet_torque_constant = Some(7.5);
    hd.no_load_speed_rpm = Some(73.0);
    hd.peak_stall_torque_kg_cm = Some(9.0);
    hd.bam_motor = "feetech_hd1910_7_4V";
    hd.bam_vin = 7.4;
    hd.bam_kp = 32.0;
    hd.bam_error_gain = 0.163;
    hd.bam_max_pwm = 0.97;
    hd.bam_kt = 0.5644993059031307;
    hd.bam_resistance = 2.9664762312357853;
    hd.bam_armature = 0.0018273861091867637;
    hd.bam_q_offset = 0.008824155147854507;
    hd.bam_command_delay = 0.0009781055842204172;
    hd.bam_friction_base = 0.011950659296477161;
    hd.bam_friction_stribeck = 0.0;
    hd.bam_friction_viscous = 0.004767196924056633;
    hd.bam_load_friction_motor = 0.3534653049182988;
    hd.bam_load_friction_external = 0.04335300722280222;
    hd.bam_load_friction_motor_stribeck = 1.6996676628765351e-6;
    hd.bam_load_friction_external_stribeck = 0.14250972390583813;
    hd.bam_load_friction_motor_quad = 0.006861907376104702;
    hd.bam_load_friction_external_quad = 0.009999951209775423;
    hd.bam_dtheta_stribeck = 0.716192604078371;
    hd.bam_alpha = 4.248291402139783;

    profiles
}
