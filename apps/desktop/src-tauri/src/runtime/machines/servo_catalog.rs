//! Device and simulation settings projected into the desktop servo catalog.

use microduck_body_runtime::{ModelKind, PhysicalGains, models::resolve};
use servo::ServoModel;

use crate::types::{ServoCatalogEntry, ServoModelKey};

pub(crate) fn entries() -> Vec<ServoCatalogEntry> {
    [ModelKind::Xl330, ModelKind::Hd1910]
        .into_iter()
        .map(|kind| {
            let model = kind.servo_model();
            let is_hd1910 = model == ServoModel::Hd1910;
            let device = model.facts().expect("catalog model has device facts");
            let hardware = device.hardware;
            let motor = resolve::nominal_motor(kind);
            let defaults = resolve::defaults(kind);
            let reference = resolve::XL330_REFERENCE;
            let physical_p = match resolve::physical_gains(kind) {
                Some(PhysicalGains::Hd1910 { p }) => Some(p),
                _ => None,
            };
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
                model_number: device.model_number,
                resolution: device.resolution,
                kp_divisor: device.kp_divisor,
                pwm_limit: device.pwm_limit,
                default_baud: device.default_baud,
                baud_rates: device.baud_rates.to_vec(),
                register_count: device.register_count as u32,
                supports_fast_sync: device.supports_fast_sync,
                supports_center_calibration: device.supports_center_calibration,
                reset_needs_unlock: device.reset_needs_unlock,
                supports_reboot: device.supports_reboot,
                protocol_definition: device.protocol_definition.to_owned(),
                velocity_rad_per_count: device.velocity_rad_per_count,
                volts_per_count: device.volts_per_count,
                milliamps_per_count: device.milliamps_per_count,
                move_block: device
                    .move_block
                    .iter()
                    .copied()
                    .map(String::from)
                    .collect(),
                mass_g: reference.baseline_mass_g,
                baseline_mass_g: reference.baseline_mass_g,
                hardware_mass_g: hardware.map(|spec| spec.mass_g),
                hardware_gear_ratio: hardware.map(|spec| spec.gear_ratio).map(String::from),
                hardware_backlash_deg: hardware.map(|spec| spec.backlash_deg),
                hardware_voltage_range: hardware.map(|spec| spec.voltage_range),
                datasheet_torque_constant: hardware.map(|spec| spec.torque_constant),
                no_load_speed_rpm: hardware.map(|spec| spec.no_load_speed_rpm),
                peak_stall_torque_kg_cm: hardware.map(|spec| spec.peak_stall_torque_kg_cm),
                bam_motor: motor.motor_name.to_owned(),
                bam_model: motor.model.to_owned(),
                bam_vin: motor.vin,
                bam_kp: motor.kp,
                bam_error_gain: motor.error_gain,
                bam_max_pwm: motor.max_pwm,
                bam_kt: motor.kt,
                bam_resistance: motor.resistance,
                bam_armature: motor.armature,
                bam_q_offset: motor.identified_position_offset_rad,
                bam_command_delay: motor.identified_delay_s,
                bam_friction_base: motor.friction_base,
                bam_friction_stribeck: motor.friction_stribeck,
                bam_friction_viscous: motor.friction_viscous,
                bam_load_friction_motor: motor.load_friction_motor,
                bam_load_friction_external: motor.load_friction_external,
                bam_load_friction_motor_stribeck: motor.load_friction_motor_stribeck,
                bam_load_friction_external_stribeck: motor.load_friction_external_stribeck,
                bam_load_friction_motor_quad: motor.load_friction_motor_quad,
                bam_load_friction_external_quad: motor.load_friction_external_quad,
                bam_dtheta_stribeck: motor.dtheta_stribeck,
                bam_alpha: motor.alpha,
                kp_fw: reference.kp_fw,
                vin_range: reference.vin_range,
                vin_drop_gain_range: reference.vin_drop_gain_range,
                vin_min: reference.vin_min,
                delay_steps: reference.delay_steps,
                backlash_deg: reference.backlash_deg,
                p_at_kp_fw: physical_p,
                sim_damping: defaults.damping,
                sim_friction_loss: defaults.friction_loss,
                sim_armature: defaults.armature,
                sim_kp: defaults.kp,
                sim_force_range: defaults.force_range,
            }
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn servo_profiles_expose_the_complete_builtin_library_view() {
        let profiles = entries();
        let body = crate::runtime::adapters::body_assets::test_template();
        let xl_drive = crate::runtime::adapters::simulation::resolve(
            crate::test_support::simulation_selection(ModelKind::Xl330),
            &crate::platform::helpers::default_simulation(ServoModelKey::Xl330),
            &body,
        )
        .unwrap()
        .simulation
        .unwrap();
        let hd_drive = crate::runtime::adapters::simulation::resolve(
            crate::test_support::simulation_selection(ModelKind::Hd1910),
            &crate::platform::helpers::default_simulation(ServoModelKey::Hd1910),
            &body,
        )
        .unwrap()
        .simulation
        .unwrap();
        let compatible_drive = crate::runtime::adapters::simulation::resolve(
            crate::test_support::simulation_selection(ModelKind::Hd1910Compatible),
            &crate::platform::helpers::default_simulation(ServoModelKey::Hd1910),
            &body,
        )
        .unwrap()
        .simulation
        .unwrap();
        let xl = profiles
            .iter()
            .find(|profile| profile.key == ServoModelKey::Xl330)
            .expect("XL330 profile");
        assert_eq!(xl.model_number, 1200);
        assert_eq!(xl.vin_range, [6.5, 8.2]);
        assert_eq!(xl.delay_steps, [3, 6]);
        assert_eq!(xl.bam_motor, "xl330");
        assert!((xl.bam_vin - 7.4).abs() < 1e-12);
        assert!((xl.bam_kp - 200.0).abs() < 1e-12);
        assert!((xl.bam_kt - 0.36601349688984386).abs() < 1e-12);
        assert!((xl.bam_resistance - 2.8113923539223227).abs() < 1e-12);
        assert!((xl.bam_armature - 0.0018077432831600838).abs() < 1e-15);
        assert!((xl.bam_q_offset - 0.0271132870444849).abs() < 1e-12);
        assert!((xl.bam_friction_base - 0.004771183165566).abs() < 1e-12);
        assert!((xl.bam_dtheta_stribeck - 2.890372094130307).abs() < 1e-12);
        assert!(xl.supports_fast_sync);
        assert_eq!(xl_drive.response.delay_steps, [3, 6]);
        assert!(xl_drive.response.encoder_through_backlash);
        assert!((xl_drive.response.position_gain_scale - 1.0).abs() < 1e-12);

        let hd = profiles
            .iter()
            .find(|profile| profile.key == ServoModelKey::Hd1910)
            .expect("HD1910 profile");
        assert_eq!(hd.model_number, 7946);
        assert_eq!(hd.p_at_kp_fw, Some(32));
        assert_eq!(hd.hardware_mass_g, Some(21.0));
        assert_eq!(hd.hardware_backlash_deg, Some(0.5));
        assert_eq!(hd.bam_motor, "feetech_hd1910_7_4V");
        assert!((hd.bam_kp - 32.0).abs() < 1e-12);
        assert!((hd.bam_kt - 0.5644993059031307).abs() < 1e-12);
        assert!((hd.bam_command_delay - 0.0009781055842204172).abs() < 1e-12);
        assert!(!hd.supports_fast_sync);
        assert!(hd.register_count > 0);
        assert_eq!(hd_drive.response.delay_steps, [0, 0]);
        assert!(!hd_drive.response.encoder_through_backlash);
        assert_eq!(compatible_drive.response.delay_steps, [3, 6]);
        assert!(compatible_drive.response.encoder_through_backlash);
        assert!(
            (compatible_drive.response.position_gain_scale
                - (xl.bam_vin * xl.bam_kt / xl.bam_resistance * xl.bam_kp * xl.bam_error_gain)
                    / (hd.bam_vin * hd.bam_kt / hd.bam_resistance * hd.bam_kp * hd.bam_error_gain)
                    * 0.989)
                .abs()
                < 1e-12
        );
    }
}
