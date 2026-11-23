"""Configure the pinned MJLab/BAM adapters with a resolved product profile."""

from __future__ import annotations

import json
from dataclasses import dataclass, fields, replace
from pathlib import Path

from mjlab_microduck.actuator import BacklashEncoderBamActuatorCfg, FrictionDRBamActuatorCfg
from mjlab_microduck.actuator.friction_dr_bam import (
    BacklashEncoderBamActuator,
    FrictionDRBamActuator,
)

from simulation_profile import MOTOR_NUMBERS, SimulationProfile


class _ResolvedMotor:
    """Set the electrical coefficients that BAM's fit JSON does not load."""

    def __init__(self, cfg, entity, target_ids, target_names):
        super().__init__(cfg, entity, target_ids, target_names)
        self._bam_model.actuator.error_gain = cfg.error_gain
        self._bam_model.actuator.max_pwm = cfg.max_pwm
        self._bam_model.actuator.max_current = None


class ResolvedBamActuator(_ResolvedMotor, FrictionDRBamActuator):
    pass


class ResolvedBacklashActuator(_ResolvedMotor, BacklashEncoderBamActuator):
    pass


@dataclass(kw_only=True)
class ResolvedBamCfg(FrictionDRBamActuatorCfg):
    error_gain: float
    max_pwm: float

    def build(self, entity, target_ids, target_names):
        return ResolvedBamActuator(self, entity, target_ids, target_names)


@dataclass(kw_only=True)
class ResolvedBacklashCfg(BacklashEncoderBamActuatorCfg):
    error_gain: float
    max_pwm: float

    def build(self, entity, target_ids, target_names):
        return ResolvedBacklashActuator(self, entity, target_ids, target_names)


def _write_bam_fit(profile: SimulationProfile, run_dir: Path) -> Path:
    motor = profile.simulation["motor"]
    electrical_options = {"vin", "kp", "error_gain", "max_pwm", "identified_delay_s"}
    aliases = {"resistance": "R", "identified_position_offset_rad": "q_offset"}
    fit = {
        aliases.get(key, key): motor[key]
        for key in MOTOR_NUMBERS
        if key not in electrical_options
    }
    # The pinned BAM loader uses this entry to construct its voltage-control class.
    # The effective motor constants and electrical command gains come from the profile.
    fit.update(model=motor["model"], actuator="xl330")
    path = run_dir.resolve() / "bam-fit.json"
    path.write_text(json.dumps(fit, allow_nan=False), encoding="utf-8")
    return path


def configure_robot(
    env_cfg, profile: SimulationProfile, run_dir: Path, *, encoder_through_backlash: bool,
    domain_randomization: bool,
) -> None:
    if profile.task is None:
        raise ValueError("MJLab needs a task robot profile")
    robot = env_cfg.scene.entities["robot"]
    if robot.articulation is None or len(robot.articulation.actuators) != 1:
        raise ValueError("task requires one BAM actuator group")
    original = robot.articulation.actuators[0]
    if not isinstance(original, FrictionDRBamActuatorCfg):
        raise ValueError("task requires a BAM actuator group")
    if isinstance(original, BacklashEncoderBamActuatorCfg) != encoder_through_backlash:
        raise ValueError("task encoder feedback and resolved model disagree")
    simulation = profile.simulation
    motor = simulation["motor"]
    response = simulation["response"]
    if response["encoder_through_backlash"] != encoder_through_backlash:
        raise ValueError("resolved feedback does not match the model adapter")
    if response["back_emf_scale"] != 1.0 or response["friction_scale"] != 1.0:
        raise ValueError("the current MJLab adapter expects unit back-EMF and friction multipliers")
    cfg_type = ResolvedBacklashCfg if encoder_through_backlash else ResolvedBamCfg
    options = {field.name: getattr(original, field.name) for field in fields(original) if field.init}
    options.update(
        motor_name=None,
        model=None,
        json_path=str(_write_bam_fit(profile, run_dir)),
        vin=motor["vin"],
        kp_fw=motor["kp"] * response["position_gain_scale"],
        error_gain=motor["error_gain"],
        max_pwm=motor["max_pwm"],
        delay_min_lag=response["delay_steps"][0],
        delay_max_lag=response["delay_steps"][1],
        vin_min=simulation["minimum_voltage_v"],
    )
    if not domain_randomization:
        options["vin_range"] = None
        options["vin_drop_gain_range"] = (simulation["voltage_drop_gain"],) * 2
    actuator = cfg_type(**options)
    env_cfg.scene.entities["robot"] = replace(
        robot,
        articulation=replace(robot.articulation, actuators=(actuator,)),
        spec_fn=profile.robot_spec,
    )


def adapter(model_kind: str):
    from hd1910.mjlab_body import install as hd1910
    from hd1910_compatible.mjlab_body import install as compatible
    from xl330.mjlab_body import install as xl330

    return {"xl330": xl330, "hd1910": hd1910, "hd1910-compatible": compatible}[model_kind]
