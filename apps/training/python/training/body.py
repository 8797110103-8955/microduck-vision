"""Build official MJLab/BAM objects from parameters resolved by Rust."""

from __future__ import annotations

import base64
import binascii
import hashlib
import json
import math

import torch
from dataclasses import dataclass, fields, replace
from pathlib import Path, PurePosixPath

from mjlab_microduck.actuator import BacklashEncoderBamActuatorCfg, FrictionDRBamActuatorCfg
from mjlab_microduck.actuator.friction_dr_bam import BacklashEncoderBamActuator, FrictionDRBamActuator


UNITS = {"angle": "rad", "angular_velocity": "rad/s", "length": "m", "time": "s", "voltage": "V"}


def _number(value):
    return type(value) in (int, float) and math.isfinite(value)


def _digest(value):
    return isinstance(value, str) and len(value) == 64 and all(char in "0123456789abcdef" for char in value)


def _resource_id(value):
    if not isinstance(value, str) or not value or "\\" in value or ":" in value:
        raise ValueError(f"invalid resource name {value!r}")
    path = PurePosixPath(value)
    if path.is_absolute() or ".." in path.parts or path.as_posix() != value:
        raise ValueError(f"invalid resource name {value!r}")
    return value


def decode_assets(envelope):
    manifest = {}
    for entry in envelope["profile"]["simulation"]["source_assets"]:
        name = _resource_id(entry["path"])
        if name in manifest or not _digest(entry["sha256"]):
            raise ValueError("invalid resource manifest entry")
        manifest[name] = entry["sha256"]
    assets = {}
    for entry in envelope["assets"]:
        name = _resource_id(entry["path"])
        data = base64.b64decode(entry["base64"], validate=True)
        if name in assets or hashlib.sha256(data).hexdigest() != manifest.get(name):
            raise ValueError(f"resource digest mismatch: {name}")
        assets[name] = data
    if assets.keys() != manifest.keys():
        raise ValueError("simulation export has incomplete resources")
    return assets


def validate_backend(backend, execution, fit_path):
    if backend["kind"] != "mjlab-bam" or backend["version"] != 1:
        raise ValueError("unsupported MJLab backend")
    if backend["execution_fingerprint"] != execution["execution_fingerprint"]:
        raise ValueError("backend execution fingerprint differs from the profile")
    for name in ("encoder_through_backlash", "name_servo_collision_geoms"):
        if type(backend[name]) is not bool:
            raise ValueError(f"invalid backend {name}")
    actuator = backend["actuator"]
    numeric = {"vin", "kp_fw", "error_gain", "max_pwm", "vin_min", "voltage_drop_gain"}
    if set(actuator) != numeric | {"max_current", "delay_min_lag", "delay_max_lag"}:
        raise ValueError("invalid backend actuator fields")
    if any(not _number(actuator[key]) or actuator[key] < 0 for key in numeric):
        raise ValueError("invalid backend actuator number")
    if not 0 <= actuator["max_pwm"] <= 1:
        raise ValueError("invalid backend PWM limit")
    current = actuator["max_current"]
    if current is not None and (not _number(current) or current < 0):
        raise ValueError("invalid backend current limit")
    lo, hi = actuator["delay_min_lag"], actuator["delay_max_lag"]
    if type(lo) is not int or type(hi) is not int or not 0 <= lo <= hi:
        raise ValueError("invalid backend delay")
    fit = backend["bam_fit"]
    if fit["file"] != "bam-fit.json" or not _digest(fit["sha256"]):
        raise ValueError("invalid BAM fit path or digest")
    if not isinstance(fit["content"], str) or hashlib.sha256(fit["content"].encode()).hexdigest() != fit["sha256"]:
        raise ValueError("BAM fit content digest mismatch")
    if hashlib.sha256(fit_path.read_bytes()).hexdigest() != fit["sha256"]:
        raise ValueError("BAM fit file digest mismatch")


@dataclass(frozen=True)
class SimulationProfile:
    execution: dict
    task: dict | None
    assets: dict[str, bytes]
    backend: dict
    fit_path: Path

    @property
    def simulation(self):
        return self.execution["simulation"]

    @property
    def model_kind(self):
        return self.execution["model_kind"]

    def robot_spec(self):
        import mujoco

        robot = self.simulation["scenario"]["robot_xml"]
        spec = mujoco.MjSpec.from_string(self.assets[robot].decode("utf-8"),
            assets={name: data for name, data in self.assets.items() if name != robot})
        if self.backend["name_servo_collision_geoms"]:
            from mjlab_microduck.robot.microduck_constants import name_servo_collision_geoms

            name_servo_collision_geoms(spec)
        return spec


def load_profile(path):
    path = Path(path).resolve(strict=True)
    try:
        envelope = json.loads(path.read_bytes())
        if envelope["format_version"] != 2 or envelope["units"] != UNITS:
            raise ValueError("unsupported training profile version or units")
        execution, task, backend = envelope["profile"], envelope["task"], envelope["backend"]
        if (execution["format_version"] != 1 or execution["execution_kind"] != "simulation"
                or execution["gain_mode"] != "fixed_model" or execution["physical_gains"] is not None
                or not _digest(execution["execution_fingerprint"])):
            raise ValueError("invalid simulation execution profile")
        layout, home = execution["joint_order"], execution["home_pose_rad"]
        if (not layout or any(not isinstance(name, str) or not name for name in layout)
                or len(layout) != len(set(layout)) or len(layout) != len(home) or not all(map(_number, home))):
            raise ValueError("invalid canonical joint layout or home pose")
        periods = execution["periods"]
        physics, decimation, control = periods["physics_s"], periods["physics_substeps"], periods["control_s"]
        if (not _number(physics) or physics <= 0 or type(decimation) is not int or decimation < 1
                or not _number(control) or not math.isclose(control, physics * decimation, abs_tol=1e-12)):
            raise ValueError("invalid simulation periods")
        fit_path = path.parent / "bam-fit.json"
        validate_backend(backend, execution, fit_path)
        assets = decode_assets(envelope)
        source = execution["simulation"]["scenario"]
        robot = _resource_id(source["robot_xml"])
        root_home = execution["simulation"]["root_home_pose"]
        if len(root_home) != 7 or not all(map(_number, root_home)):
            raise ValueError("invalid root home pose")
        if (task is None or source["scene_xml"] is not None or source["task_id"] != task["task_id"]
                or robot not in assets or task["robot_xml"] != robot
                or task["robot_sha256"] != hashlib.sha256(assets[robot]).hexdigest()
                or task["joint_order"] != layout or task["home_pose_rad"] != home
                or task["periods"] != periods or task["root_home_pose"] != root_home
                or task["requires_backlash"] != backend["encoder_through_backlash"]):
            raise ValueError("training description and resolved robot profile disagree")
        return SimulationProfile(execution, task, assets, backend, fit_path)
    except (KeyError, TypeError, binascii.Error, OSError) as error:
        raise ValueError(f"invalid training profile: {error}") from error


class _ResolvedMotor:
    def __init__(self, cfg, entity, target_ids, target_names):
        super().__init__(cfg, entity, target_ids, target_names)
        for name in ("error_gain", "max_pwm", "max_current"):
            setattr(self._bam_model.actuator, name, getattr(cfg, name))


class ResolvedBamActuator(_ResolvedMotor, FrictionDRBamActuator):
    pass


class ResolvedBacklashActuator(_ResolvedMotor, BacklashEncoderBamActuator):
    pass


@dataclass(kw_only=True)
class ResolvedBamCfg(FrictionDRBamActuatorCfg):
    error_gain: float
    max_pwm: float
    max_current: float | None = None

    def build(self, entity, target_ids, target_names):
        return ResolvedBamActuator(self, entity, target_ids, target_names)


@dataclass(kw_only=True)
class ResolvedBacklashCfg(BacklashEncoderBamActuatorCfg):
    error_gain: float
    max_pwm: float
    max_current: float | None = None

    def build(self, entity, target_ids, target_names):
        return ResolvedBacklashActuator(self, entity, target_ids, target_names)


def configure_robot(env_cfg, profile, run_dir, *, domain_randomization):
    robot = env_cfg.scene.entities["robot"]
    if robot.articulation is None or len(robot.articulation.actuators) != 1:
        raise ValueError("task requires one BAM actuator group")
    original = robot.articulation.actuators[0]
    if not isinstance(original, FrictionDRBamActuatorCfg):
        raise ValueError("task requires a BAM actuator group")
    backlash = profile.backend["encoder_through_backlash"]
    if isinstance(original, BacklashEncoderBamActuatorCfg) != backlash:
        raise ValueError("task encoder feedback and resolved model disagree")
    options = {field.name: getattr(original, field.name) for field in fields(original) if field.init}
    effective = dict(profile.backend["actuator"])
    voltage_drop = effective.pop("voltage_drop_gain")
    options.update(effective, motor_name=None, model=None, json_path=str(profile.fit_path))
    if not domain_randomization:
        options.update(vin_range=None, vin_drop_gain_range=(voltage_drop,) * 2)
    cfg_type = ResolvedBacklashCfg if backlash else ResolvedBamCfg
    env_cfg.scene.entities["robot"] = replace(robot,
        articulation=replace(robot.articulation, actuators=(cfg_type(**options),)), spec_fn=profile.robot_spec)


class PendingTargets:
    def __init__(self, num_envs, max_lag, num_targets, device):
        self.max_lag = max_lag
        self.values = torch.zeros((num_envs, max_lag, num_targets), device=device)
        self.age = torch.full((num_envs,), max_lag, dtype=torch.long, device=device)

    def reset(self, env_ids=None):
        self.age[slice(None) if env_ids is None else env_ids] = self.max_lag

    def seed(self, env_ids, values):
        if values.shape != (len(env_ids), self.max_lag, self.values.shape[-1]):
            raise ValueError("Incorrect reference actuator history shape")
        self.values[env_ids] = values
        self.age[env_ids] = 0

    def apply(self, current_target, lags):
        if self.max_lag == 0:
            return current_target
        active = self.age < lags
        index = (self.max_lag - lags + self.age).clamp(0, self.max_lag - 1)
        rows = torch.arange(len(self.age), device=self.age.device)
        result = torch.where(active[:, None], self.values[rows, index], current_target)
        self.age.clamp_max_(self.max_lag)
        self.age += (self.age < self.max_lag).long()
        return result



class HistoryBacklashActuator(ResolvedBacklashActuator):
    @property
    def history_target_names(self):
        return tuple(self._target_names)

    def initialize(self, mj_model, model, data, device):
        super().initialize(mj_model, model, data, device)
        self.target_history = PendingTargets(data.nworld, self.cfg.delay_max_lag, len(self._target_names), device)
        self.physics_pending = False
        self._physics_advance = False
        self._last_torque = None

    def reset(self, env_ids=None):
        super().reset(env_ids)
        self.target_history.reset(env_ids)
        if self._last_torque is not None:
            self._last_torque[slice(None) if env_ids is None else env_ids] = 0

    def seed_targets(self, env_ids, values):
        self.target_history.seed(env_ids, values)

    def apply_delay(self, cmd):
        self._physics_advance = self.physics_pending
        self.physics_pending = False
        if not self._physics_advance:
            # MJLab also writes controls after a subset reset, without a physics
            # step. That write must not advance other environments' lag queues.
            return cmd
        delayed = super().apply_delay(cmd)
        if self._delay_buffer is None:
            return delayed
        # The normal pinned queue is still appended exactly once. Only rows that
        # need pre-reset inputs use the seed; all other rows retain its output.
        target = self.target_history.apply(delayed.position_target, self._delay_buffer.current_lags)
        return replace(delayed, position_target=target)

    def compute(self, cmd):
        if not self._physics_advance and self._last_torque is not None:
            return self._last_torque
        self._last_torque = super().compute(cmd)
        return self._last_torque


@dataclass(kw_only=True)
class HistoryBacklashCfg(ResolvedBacklashCfg):
    def build(self, entity, target_ids, target_names):
        return HistoryBacklashActuator(self, entity, target_ids, target_names)
