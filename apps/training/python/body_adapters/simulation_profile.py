"""Read the shared Rust simulation export and verify its resource bytes."""

from __future__ import annotations

import base64
import binascii
import hashlib
import json
import math
from dataclasses import dataclass
from pathlib import Path, PurePosixPath
from typing import Any


MODELS = {"xl330": "xl330", "hd1910": "hd1910", "hd1910-compatible": "hd1910"}
UNITS = {"angle": "rad", "angular_velocity": "rad/s", "length": "m", "time": "s", "voltage": "V"}
OPERATIONS = {
    "configure_bam_actuators",
    "match_xl330_position_stiffness",
    "name_servo_collision_geoms",
}
MOTOR_NUMBERS = (
    "vin", "kp", "error_gain", "max_pwm", "kt", "resistance", "armature",
    "identified_position_offset_rad", "identified_delay_s",
    "friction_base", "friction_stribeck", "friction_viscous",
    "load_friction_motor", "load_friction_external",
    "load_friction_motor_stribeck", "load_friction_external_stribeck",
    "load_friction_motor_quad", "load_friction_external_quad", "dtheta_stribeck", "alpha",
)


def _number(value: Any) -> bool:
    return type(value) in (int, float) and math.isfinite(value)


def _digest(value: Any) -> bool:
    return isinstance(value, str) and len(value) == 64 and all(
        character in "0123456789abcdef" for character in value
    )


def _resource_id(name: Any) -> str:
    if not isinstance(name, str) or not name or "\\" in name:
        raise ValueError(f"invalid resource name {name!r}")
    path = PurePosixPath(name)
    if path.is_absolute() or ".." in path.parts or path.as_posix() != name or ":" in name:
        raise ValueError(f"invalid resource name {name!r}")
    return name


@dataclass(frozen=True)
class SimulationProfile:
    execution: dict
    task: dict | None
    assets: dict[str, bytes]

    @property
    def simulation(self) -> dict:
        return self.execution["simulation"]

    @property
    def model_kind(self) -> str:
        return self.execution["model_kind"]

    def robot_spec(self):
        import mujoco

        source = self.simulation["scenario"]
        robot = source["robot_xml"]
        resources = {name: data for name, data in self.assets.items() if name != robot}
        spec = mujoco.MjSpec.from_string(self.assets[robot].decode("utf-8"), assets=resources)
        if "name_servo_collision_geoms" in source["derived_scene_operations"]:
            from mjlab_microduck.robot.microduck_constants import name_servo_collision_geoms

            name_servo_collision_geoms(spec)
        return spec


def load_profile(path: Path) -> SimulationProfile:
    envelope = json.loads(path.read_bytes())
    try:
        return _parse(envelope)
    except (KeyError, TypeError, binascii.Error) as error:
        raise ValueError(f"invalid simulation profile: {error}") from error


def _parse(envelope: dict) -> SimulationProfile:
    if envelope["format_version"] != 1 or envelope["units"] != UNITS:
        raise ValueError("unsupported simulation profile version or units")
    execution = envelope["profile"]
    model = execution["model_kind"]
    if (execution["format_version"] != 1 or execution["execution_kind"] != "simulation"
            or execution["gain_mode"] != "fixed_model" or execution["physical_gains"] is not None
            or model not in MODELS or execution["servo_model"] != MODELS[model]):
        raise ValueError("invalid simulation model or execution kind")
    if not _digest(execution["execution_fingerprint"]):
        raise ValueError("invalid execution fingerprint")

    layout = execution["joint_order"]
    home = execution["home_pose_rad"]
    if (not layout or not all(isinstance(name, str) and name for name in layout)
            or len(set(layout)) != len(layout) or len(layout) != len(home)
            or not all(_number(value) for value in home)):
        raise ValueError("invalid canonical joint layout or home pose")
    periods = execution["periods"]
    step = periods["physics_s"]
    substeps = periods["physics_substeps"]
    control = periods["control_s"]
    if (not _number(step) or step <= 0 or type(substeps) is not int or substeps < 1
            or not _number(control) or not math.isclose(control, step * substeps, abs_tol=1e-12)):
        raise ValueError("invalid simulation periods")

    simulation = execution["simulation"]
    for motor in (simulation["nominal_motor"], simulation["motor"]):
        if motor["model"] != "m6" or not all(_number(motor[key]) for key in MOTOR_NUMBERS):
            raise ValueError("unsupported BAM model or effective parameters")
        if (not 0 <= motor["max_pwm"] <= 1 or motor["resistance"] <= 0
                or motor["kt"] <= 0 or motor["dtheta_stribeck"] <= 0):
            raise ValueError("invalid motor limits")
    for key in ("minimum_voltage_v", "voltage_drop_gain"):
        if not _number(simulation[key]) or simulation[key] < 0:
            raise ValueError(f"invalid {key}")
    root_home = simulation["root_home_pose"]
    if len(root_home) != 7 or not all(_number(value) for value in root_home):
        raise ValueError("invalid root home pose")
    response = simulation["response"]
    lo, hi = response["delay_steps"]
    if type(lo) is not int or type(hi) is not int or not 0 <= lo <= hi:
        raise ValueError("invalid actuator delay range")
    for key in ("position_gain_scale", "back_emf_scale", "friction_scale"):
        if not _number(response[key]) or response[key] < 0:
            raise ValueError(f"invalid {key}")
    if type(response["encoder_through_backlash"]) is not bool:
        raise ValueError("invalid backlash feedback flag")
    if model != "xl330" and (response["encoder_through_backlash"] or [lo, hi] != [0, 0]):
        raise ValueError("HD1910 simulation requires native encoder feedback and zero-step target delay")

    source = simulation["scenario"]
    robot = _resource_id(source["robot_xml"])
    operations = set(source["derived_scene_operations"])
    if operations - OPERATIONS or "configure_bam_actuators" not in operations:
        raise ValueError("unsupported scene derivation")
    if ("match_xl330_position_stiffness" in operations) != (model == "hd1910-compatible"):
        raise ValueError("compatibility operation does not match the model")
    for entry in execution["parameter_sources"]:
        if entry["class"] not in {"fact", "simulation_approximation", "compatibility_measure"}:
            raise ValueError("unknown simulation parameter classification")
        if entry["real_machine_mapping"] not in {
            "implemented", "physical_effect", "simulation_only", "unverified"
        }:
            raise ValueError("unknown real machine mapping")

    manifest = {}
    for entry in simulation["source_assets"]:
        name = _resource_id(entry["path"])
        if name in manifest or not _digest(entry["sha256"]):
            raise ValueError(f"invalid resource manifest entry {name}")
        manifest[name] = entry["sha256"]
    assets = {}
    for entry in envelope["assets"]:
        name = _resource_id(entry["path"])
        data = base64.b64decode(entry["base64"], validate=True)
        if name in assets or hashlib.sha256(data).hexdigest() != manifest.get(name):
            raise ValueError(f"resource digest mismatch: {name}")
        assets[name] = data
    if assets.keys() != manifest.keys() or robot not in assets:
        raise ValueError("simulation export has incomplete resources")

    task = envelope["task"]
    if task is None:
        if _resource_id(source["scene_xml"]) not in assets or "task_id" in source:
            raise ValueError("native simulation requires a complete scene")
    elif (source["scene_xml"] is not None or source["task_id"] != task["task_id"]
            or task["robot_xml"] != robot or task["robot_sha256"] != manifest[robot]
            or task["joint_order"] != layout or task["home_pose_rad"] != home
            or task["periods"] != periods or task["root_home_pose"] != root_home
            or task["requires_backlash"] != response["encoder_through_backlash"]):
        raise ValueError("training description and resolved robot profile disagree")
    return SimulationProfile(execution, task, assets)
