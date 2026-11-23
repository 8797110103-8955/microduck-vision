//! Resolve product model selection and tuning once for execution consumers.

use duck_body::{AssetResolver, BodyProfile};

use super::{hd1910, hd1910_compatible, xl330};
use crate::models::parameter_sources::{ParameterClass, ParameterSource, RealMachineMapping};
use crate::models::profile::{
    BamParameters, BodyExecutionProfile, BodyPeriods, DeviceFacts, ExecutionKind, GainMode,
    ModelKind, PhysicalGains, ResolvedSimulationProfile, Selection, SimulationTuning,
};
use crate::models::scene_recipe::SceneOperation;
use crate::training::TaskBodyDescription;

pub use super::xl330::profile::{TRAINING_REFERENCE as XL330_REFERENCE, TrainingReference};

/// Values that leave the fitted model unchanged in desktop controls.
pub fn defaults(model: ModelKind) -> SimulationTuning {
    match model {
        ModelKind::Xl330 => xl330::profile::SLIDER_DEFAULTS,
        ModelKind::Hd1910 | ModelKind::Hd1910Compatible => hd1910::profile::SLIDER_DEFAULTS,
    }
}

pub fn nominal_motor(model: ModelKind) -> BamParameters {
    match model {
        ModelKind::Xl330 => xl330::profile::MOTOR,
        ModelKind::Hd1910 | ModelKind::Hd1910Compatible => hd1910::profile::MOTOR,
    }
}

pub fn physical_gains(model: ModelKind) -> Option<PhysicalGains> {
    match model {
        ModelKind::Xl330 => Some(xl330::physical::GAINS),
        ModelKind::Hd1910 => Some(hd1910::physical::GAINS),
        ModelKind::Hd1910Compatible => None,
    }
}

pub fn resolve_simulation(
    selection: Selection,
    body: &BodyProfile,
    source: &dyn AssetResolver,
    tuning: SimulationTuning,
) -> Result<BodyExecutionProfile, String> {
    resolve_simulation_profile(selection, body, source, tuning, None)
}

pub(crate) fn resolve_simulation_profile(
    selection: Selection,
    body: &BodyProfile,
    source: &dyn AssetResolver,
    tuning: SimulationTuning,
    task: Option<&TaskBodyDescription>,
) -> Result<BodyExecutionProfile, String> {
    selection.validate()?;
    if selection.execution_kind != ExecutionKind::Simulation {
        return Err("expected a simulation selection".into());
    }
    let nominal = nominal_motor(selection.model_kind);
    let (mut scenario, mut response, compatibility_revision) = match selection.model_kind {
        ModelKind::Xl330 => (xl330::scene::scene(), xl330::profile::RESPONSE, None),
        ModelKind::Hd1910 => (hd1910::scene::scene(), hd1910::profile::RESPONSE, None),
        ModelKind::Hd1910Compatible => (
            hd1910_compatible::scene::scene(),
            hd1910_compatible::profile::response(xl330::profile::MOTOR, nominal),
            Some(hd1910_compatible::profile::REVISION),
        ),
    };
    let mut profile = base(selection, body);
    profile.periods.physics_s = Some(0.005);
    profile.periods.physics_substeps = Some(4);
    let mut root_home_pose = [0.0, 0.0, 0.125, 1.0, 0.0, 0.0, 0.0];
    if let Some(task) = task {
        scenario.scene_xml = None;
        scenario.robot_xml = task.robot_xml.clone();
        scenario.task_id = Some(task.task_id.clone());
        if task.name_servo_collision_geoms {
            scenario
                .derived_scene_operations
                .push(SceneOperation::NameServoCollisionGeoms);
        }
        profile.home_pose_rad.clone_from(&task.home_pose_rad);
        profile.periods = task.periods;
        root_home_pose = task.root_home_pose;
        if selection.model_kind == ModelKind::Xl330 {
            response.delay_steps = task.target_delay_steps;
            response.encoder_through_backlash = task.requires_backlash;
        }
    }
    let motor = apply_tuning(nominal, defaults(selection.model_kind), tuning)?;
    let resources = scenario
        .resources(source)
        .map_err(|error| error.to_string())?;
    profile
        .parameter_sources
        .extend(simulation_sources(selection.model_kind, task.is_some()));
    profile.gain_mode = GainMode::FixedModel;
    profile.simulation = Some(ResolvedSimulationProfile {
        id: match selection.model_kind {
            ModelKind::Xl330 => "official-xl330-m6",
            ModelKind::Hd1910 => "hd1910-native",
            ModelKind::Hd1910Compatible => "body-drive-hd1910",
        },
        scenario,
        source_assets: resources.manifest(),
        nominal_motor: nominal,
        motor,
        response,
        root_home_pose,
        delay_seed: 1,
        voltage_drop_gain: 0.1,
        minimum_voltage_v: XL330_REFERENCE.vin_min,
        compatibility_revision,
    });
    profile.finalize()
}

pub fn resolve_physical(
    selection: Selection,
    body: &BodyProfile,
) -> Result<BodyExecutionProfile, String> {
    selection.validate()?;
    if selection.execution_kind != ExecutionKind::Physical {
        return Err("expected a physical selection".into());
    }
    let source = match selection.model_kind {
        ModelKind::Xl330 => xl330::physical::SOURCE,
        ModelKind::Hd1910 => hd1910::physical::SOURCE,
        ModelKind::Hd1910Compatible => {
            return Err(hd1910_compatible::physical::UNSUPPORTED.into());
        }
    };
    let mut profile = base(selection, body);
    profile.physical_gains = physical_gains(selection.model_kind);
    profile.parameter_sources.push(ParameterSource {
        parameter: "physical_gains",
        class: ParameterClass::Fact,
        source,
        assumption: Some(
            "Configured through the device driver. Dynamic response has not been measured",
        ),
        real_machine_mapping: RealMachineMapping::Implemented,
    });
    profile.finalize()
}

fn base(selection: Selection, body: &BodyProfile) -> BodyExecutionProfile {
    let template = body.template();
    let device = selection
        .servo_model
        .facts()
        .expect("selected model has servo facts");
    BodyExecutionProfile {
        format_version: 1,
        body: body.identity(),
        execution_kind: selection.execution_kind,
        model_kind: selection.model_kind,
        servo_model: selection.servo_model.key(),
        device_facts: DeviceFacts {
            encoder_resolution: device.resolution,
            velocity_rad_s_per_count: device.velocity_rad_per_count,
            protocol: device.protocol_definition,
            hardware: device.hardware,
        },
        periods: BodyPeriods {
            control_s: 0.02,
            physics_s: None,
            physics_substeps: None,
        },
        joint_order: template.control.joint_order.clone(),
        home_pose_rad: template.control.home_pose.clone(),
        body_physical: template.physical.clone(),
        gain_mode: GainMode::Configurable,
        physical_gains: None,
        simulation: None,
        parameter_sources: [
            "encoder_resolution",
            "velocity_per_count",
            "device_position_domain",
        ]
        .into_iter()
        .map(|parameter| ParameterSource {
            parameter,
            class: ParameterClass::Fact,
            source: "servo protocol",
            assumption: None,
            real_machine_mapping: RealMachineMapping::Implemented,
        })
        .collect(),
        execution_fingerprint: String::new(),
    }
}

fn simulation_sources(model: ModelKind, training: bool) -> Vec<ParameterSource> {
    use ParameterClass::{CompatibilityMeasure, Fact, SimulationApproximation};
    use RealMachineMapping::{PhysicalEffect, SimulationOnly, Unverified};

    let xl = model == ModelKind::Xl330;
    let fit = if xl {
        xl330::profile::FIT_SOURCE
    } else {
        hd1910::profile::FIT_SOURCE
    };
    let mut sources: Vec<_> = [
        "motor.vin",
        "motor.kp",
        "motor.error_gain",
        "motor.max_pwm",
        "motor.kt",
        "motor.resistance",
        "motor.armature",
        "motor.friction_base",
        "motor.friction_stribeck",
        "motor.friction_viscous",
        "motor.load_friction_motor",
        "motor.load_friction_external",
        "motor.load_friction_motor_stribeck",
        "motor.load_friction_external_stribeck",
        "motor.load_friction_motor_quad",
        "motor.load_friction_external_quad",
        "motor.dtheta_stribeck",
        "motor.alpha",
    ]
    .into_iter()
    .map(|parameter| ParameterSource {
        parameter,
        class: SimulationApproximation,
        source: fit,
        assumption: None,
        real_machine_mapping: PhysicalEffect,
    })
    .collect();
    for parameter in [
        "motor.identified_position_offset_rad",
        "motor.identified_delay_s",
    ] {
        sources.push(ParameterSource {
            parameter,
            class: SimulationApproximation,
            source: fit,
            assumption: Some(
                "Identified testbench value, unused by the deployed MuJoCo control equation",
            ),
            real_machine_mapping: Unverified,
        });
    }
    for parameter in [
        "physics_s",
        "physics_substeps",
        "delay_seed",
        "voltage_drop_gain",
        "minimum_voltage_v",
    ] {
        sources.push(ParameterSource {
            parameter,
            class: SimulationApproximation,
            source: if training && matches!(parameter, "physics_s" | "physics_substeps") {
                "MJLab task simulation timestep and decimation"
            } else {
                "existing native MuJoCo controller"
            },
            assumption: None,
            real_machine_mapping: SimulationOnly,
        });
    }
    let source_selection = if training {
        "pinned vendor robot entity selected by the MJLab task"
    } else {
        "pinned vendor scene and robot"
    };
    if !training {
        sources.push(ParameterSource {
            parameter: "scene_xml",
            class: Fact,
            source: source_selection,
            assumption: None,
            real_machine_mapping: SimulationOnly,
        });
    }
    sources.push(ParameterSource {
        parameter: "robot_xml",
        class: Fact,
        source: source_selection,
        assumption: None,
        real_machine_mapping: SimulationOnly,
    });
    for parameter in ["link_mass", "link_inertia"] {
        sources.push(ParameterSource {
            parameter,
            class: SimulationApproximation,
            source: "pinned vendor MJCF used by the solver",
            assumption: Some(if xl {
                "Pinned MJCF inertials enter the solver. Assembled hardware inertia is unverified"
            } else {
                hd1910::profile::SCENE_ASSUMPTION
            }),
            real_machine_mapping: Unverified,
        });
    }
    for parameter in [
        "passive_backlash",
        "encoder_through_backlash",
        "position_feedback",
        "velocity_feedback",
        "delay_steps",
    ] {
        sources.push(ParameterSource {
            parameter,
            class: SimulationApproximation,
            source: if xl {
                if training {
                    "vendor task actuator and robot entity"
                } else {
                    "vendor ground contact backlash model and native delay schedule"
                }
            } else {
                "existing native HD1910 MJCF and delay configuration"
            },
            assumption: (!xl).then_some(hd1910::profile::RESPONSE_ASSUMPTION),
            real_machine_mapping: if parameter == "delay_steps" {
                SimulationOnly
            } else {
                PhysicalEffect
            },
        });
    }
    for parameter in ["position_gain_scale", "back_emf_scale", "friction_scale"] {
        let compensation =
            model == ModelKind::Hd1910Compatible && parameter == "position_gain_scale";
        sources.push(ParameterSource {
            parameter,
            class: if compensation {
                CompatibilityMeasure
            } else {
                SimulationApproximation
            },
            source: if compensation {
                "ratio of nominal XL330 and HD1910 BAM position stiffness"
            } else {
                "native model multiplier 1"
            },
            assumption: None,
            real_machine_mapping: SimulationOnly,
        });
    }
    if !xl {
        for parameter in ["device_mass", "device_gear_ratio", "device_backlash"] {
            sources.push(ParameterSource {
                parameter,
                class: Fact,
                source: "servo HD1910 hardware specifications",
                assumption: Some(
                    "Device specification is not an assembled link inertial or a passive joint",
                ),
                real_machine_mapping: PhysicalEffect,
            });
        }
    }
    sources
}

fn apply_tuning(
    mut motor: BamParameters,
    defaults: SimulationTuning,
    tuning: SimulationTuning,
) -> Result<BamParameters, String> {
    fn ratio(value: f64, base: f64) -> Result<f64, String> {
        if !value.is_finite() || value < 0.0 || base <= 0.0 {
            return Err("native simulation parameters must be finite and nonnegative".into());
        }
        Ok(value / base)
    }
    motor.friction_viscous *= ratio(tuning.damping, defaults.damping)?;
    motor.friction_base *= ratio(tuning.friction_loss, defaults.friction_loss)?;
    motor.armature *= ratio(tuning.armature, defaults.armature)?;
    motor.kp *= ratio(tuning.kp, defaults.kp)?;
    motor.max_pwm = (motor.max_pwm * ratio(tuning.force_range, defaults.force_range)?).min(1.0);
    Ok(motor)
}
