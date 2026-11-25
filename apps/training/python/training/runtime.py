"""Resolve and describe upstream tasks without choosing a training method."""

import hashlib
import json
import math
from dataclasses import fields, is_dataclass
from enum import Enum
from functools import partial
from pathlib import Path


def _name(value):
    return f"{value.__module__}.{value.__qualname__}"


def describe_value(value):
    if value is None or type(value) in (bool, int, str):
        return value
    if isinstance(value, float):
        return value if math.isfinite(value) else repr(value)
    if isinstance(value, Enum):
        return describe_value(value.value)
    if isinstance(value, dict):
        return {str(key): describe_value(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [describe_value(item) for item in value]
    if isinstance(value, slice):
        return {"slice": [value.start, value.stop, value.step]}
    if isinstance(value, partial):
        return {"function": _name(value.func), "args": describe_value(value.args),
                "kwargs": describe_value(value.keywords)}
    if isinstance(value, Path):
        return value.as_posix()
    if callable(value):
        return _name(value)
    if is_dataclass(value):
        return {"type": _name(type(value)), **{
            field.name: describe_value(getattr(value, field.name)) for field in fields(value)
        }}
    if type(value).__module__.startswith("numpy"):
        return describe_value(value.tolist())
    raise ValueError(f"unsupported task description field {_name(type(value))}")


def describe_task(task_id, env_cfg, joint_order):
    from mjlab.envs.mdp.actions import JointPositionActionCfg
    from mjlab.utils.lab_api.string import resolve_matching_names
    from mjlab.utils.spec import get_non_free_joints
    from mjlab.utils.string import resolve_expr
    from mjlab_microduck.actuator import BacklashEncoderBamActuatorCfg, FrictionDRBamActuatorCfg
    from mjlab_microduck.robot import microduck_constants as robot_sources

    variants = {
        robot_sources.get_walk_spec: robot_sources.MICRODUCK_WALK_XML,
        robot_sources.get_standup_spec: robot_sources.MICRODUCK_GROUNDCONTACT_XML,
        robot_sources.get_ground_pick_spec: robot_sources.MICRODUCK_GROUNDCONTACT_XML,
        robot_sources.get_allcollisions_spec: robot_sources.MICRODUCK_ALLCOLLISIONS_XML,
        robot_sources.get_walk_rollers_spec: robot_sources.MICRODUCK_GROUNDCONTACT_ROLLERS_XML,
        robot_sources.get_backlash_spec: robot_sources.MICRODUCK_GROUNDCONTACT_BACKLASH_XML,
        robot_sources.get_walk_backlash_spec: robot_sources.MICRODUCK_WALK_BACKLASH_XML,
        robot_sources.get_allcollisions_backlash_spec: robot_sources.MICRODUCK_ALLCOLLISIONS_BACKLASH_XML,
        robot_sources.get_rollers_backlash_spec: robot_sources.MICRODUCK_GROUNDCONTACT_ROLLERS_BACKLASH_XML,
    }
    robot = env_cfg.scene.entities.get("robot")
    if robot is None or robot.spec_fn not in variants:
        raise ValueError(f"task {task_id} has no supported official robot entity")
    source = variants[robot.spec_fn]
    if robot.articulation is None or len(robot.articulation.actuators) != 1:
        raise ValueError(f"task {task_id} does not define one MicroDuck actuator group")
    actuator = robot.articulation.actuators[0]
    if not isinstance(actuator, FrictionDRBamActuatorCfg):
        raise ValueError(f"task {task_id} does not use a MicroDuck BAM actuator")
    spec = robot.spec_fn()
    names = tuple(joint.name for joint in get_non_free_joints(spec))
    _, driven = resolve_matching_names(actuator.target_names_expr, names)
    driven = list(driven)
    robot_joints = [name if name in driven else None for name in joint_order]
    if set(driven) != {name for name in robot_joints if name is not None}:
        raise ValueError(f"task {task_id} actuates joint names outside the official body")
    if robot.init_state.joint_pos is None:
        raise ValueError(f"task {task_id} needs explicit robot home poses")
    home = resolve_expr(robot.init_state.joint_pos, names, 0.0)
    home_by_name = dict(zip(names, home, strict=True))
    actions = []
    for name, term in env_cfg.actions.items():
        if term is None:
            continue
        if not isinstance(term, JointPositionActionCfg) or term.entity_name != "robot":
            raise ValueError(f"task {task_id} has unsupported action term {name}")
        _, joints = resolve_matching_names(term.actuator_names, driven)
        actions.append({"name": name, "joints": list(joints), "configuration": describe_value(term)})
    actor = env_cfg.observations["actor"]
    observations = [
        {"name": name, "configuration": describe_value(term)}
        for name, term in actor.terms.items() if term is not None
    ]
    physics_s = env_cfg.sim.mujoco.timestep
    terrain = env_cfg.scene.terrain
    return {
        "task_id": task_id,
        "robot_xml": source.name,
        "robot_sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
        "joint_order": joint_order,
        "robot_joints": robot_joints,
        "home_pose_rad": [home_by_name.get(name, 0.0) for name in robot_joints],
        "root_home_pose": [*robot.init_state.pos, *robot.init_state.rot],
        "periods": {
            "control_s": physics_s * env_cfg.decimation,
            "physics_s": physics_s,
            "physics_substeps": env_cfg.decimation,
        },
        "requires_backlash": isinstance(actuator, BacklashEncoderBamActuatorCfg),
        "name_servo_collision_geoms": source.name.startswith("robot_allcollisions"),
        "target_delay_steps": [actuator.delay_min_lag, actuator.delay_max_lag],
        "policy_layout": {
            "actions": actions,
            "action_len": sum(len(term["joints"]) for term in actions),
            "observations": observations,
            "concatenate_observations": actor.concatenate_terms,
            "observation_history_length": actor.history_length,
        },
        "assembly": {
            "terrain": terrain.terrain_type if terrain is not None else None,
            "num_envs": env_cfg.scene.num_envs,
            "entities": list(env_cfg.scene.entities),
            "sensors": [describe_value(sensor) for sensor in env_cfg.scene.sensors],
            "collisions": describe_value(robot.collisions),
        },
    }


def task_config(task_id, trailing, *, num_envs=None, terrain=None, seed=None, iterations=None):
    import mjlab
    import mjlab_microduck.tasks  # noqa: F401 - registers the pinned tasks
    import tyro
    from mjlab.scripts.train import TrainConfig

    cfg = TrainConfig.from_task(task_id)
    if trailing:
        cfg = tyro.cli(TrainConfig, args=trailing, default=cfg, config=mjlab.TYRO_FLAGS)
    if num_envs is not None:
        if num_envs < 1:
            raise ValueError("num-envs must be positive")
        cfg.env.scene.num_envs = num_envs
    if seed is not None:
        if not 0 <= seed <= 2**31 - 1:
            raise ValueError("seed must be between 0 and 2147483647")
        cfg.env.seed = cfg.agent.seed = seed
    if iterations is not None:
        if iterations < 1:
            raise ValueError("iterations must be positive")
        cfg.agent.max_iterations = cfg.agent.save_interval = iterations
    if terrain is not None:
        actual = cfg.env.scene.terrain
        if actual is None or actual.terrain_type != terrain:
            raise ValueError(f"task {task_id} does not use terrain {terrain!r}")
    return cfg


def install_profile(task_id, cfg, profile, run_dir, *, domain_randomization):
    from training.body import configure_robot

    description = describe_task(task_id, cfg.env, profile.execution["joint_order"])
    if description != profile.task:
        raise ValueError("task description or source changed after resolving its body")
    configure_robot(cfg.env, profile, run_dir, domain_randomization=domain_randomization)
    if not domain_randomization:
        removed = {
            name for name in cfg.env.events
            if name.startswith("randomize_") or name in {"encoder_bias", "push_robot"}
        }
        cfg.env.events = {
            name: event for name, event in cfg.env.events.items() if name not in removed
        }
        cfg.env.curriculum = {
            name: term for name, term in cfg.env.curriculum.items()
            if term is None or term.params.get("event_name") not in removed
        }
        cfg.env.observations["actor"].enable_corruption = False



def write_snapshot(path, value):
    Path(path).write_text(json.dumps(value, indent=2, allow_nan=False), encoding="utf-8")


def verify_snapshot(path, value):
    if json.loads(Path(path).read_bytes()) != value:
        raise ValueError("Training inputs changed between describe and run")
