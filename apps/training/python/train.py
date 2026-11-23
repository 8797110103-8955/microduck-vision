"""Describe a pinned MJLab task, then execute it with the resolved robot profile."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import sys
from contextlib import redirect_stdout
from dataclasses import fields, is_dataclass
from enum import Enum
from functools import partial
from pathlib import Path


def _name(value):
    return f"{value.__module__}.{value.__qualname__}"


def _describe_value(value):
    if value is None or type(value) in (bool, int, str):
        return value
    if isinstance(value, float):
        return value if math.isfinite(value) else repr(value)
    if isinstance(value, Enum):
        return _describe_value(value.value)
    if isinstance(value, dict):
        return {str(key): _describe_value(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [_describe_value(item) for item in value]
    if isinstance(value, slice):
        return {"slice": [value.start, value.stop, value.step]}
    if isinstance(value, partial):
        return {"function": _name(value.func), "args": _describe_value(value.args),
                "kwargs": _describe_value(value.keywords)}
    if isinstance(value, Path):
        return value.as_posix()
    if callable(value):
        return _name(value)
    if is_dataclass(value):
        return {"type": _name(type(value)), **{
            field.name: _describe_value(getattr(value, field.name)) for field in fields(value)
        }}
    if type(value).__module__.startswith("numpy"):
        return _describe_value(value.tolist())
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
        actions.append({"name": name, "joints": list(joints), "configuration": _describe_value(term)})
    actor = env_cfg.observations["actor"]
    observations = [
        {"name": name, "configuration": _describe_value(term)}
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
            "sensors": [_describe_value(sensor) for sensor in env_cfg.scene.sensors],
            "collisions": _describe_value(robot.collisions),
        },
    }


def task_config(task_id, trailing, *, num_envs=None, terrain=None):
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
    if terrain is not None:
        actual = cfg.env.scene.terrain
        if actual is None or actual.terrain_type != terrain:
            raise ValueError(f"task {task_id} does not use terrain {terrain!r}")
    return cfg


def install_profile(task_id, cfg, profile, run_dir, *, domain_randomization):
    from mjlab_adapter import adapter

    description = describe_task(task_id, cfg.env, profile.execution["joint_order"])
    if description != profile.task:
        raise ValueError("task description or source changed after resolving its body")
    adapter(profile.model_kind)(cfg.env, profile, run_dir, domain_randomization=domain_randomization)
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


def _write_artifact(run_dir, profile, *, domain_randomization):
    import onnx

    models = sorted((run_dir / "logs" / "rsl_rl").rglob("*.onnx"))
    if len(models) != 1:
        raise ValueError("training must produce one ONNX artifact for the desktop run")
    path = models[0]
    model = onnx.load(path, load_external_data=False)

    def tensor(value):
        tensor_type = value.type.tensor_type
        return {
            "name": value.name,
            "element_type": onnx.TensorProto.DataType.Name(tensor_type.elem_type),
            "dimensions": [
                length.dim_value if length.HasField("dim_value") else length.dim_param or None
                for length in tensor_type.shape.dim
            ],
        }

    manifest = {
        "format_version": 1,
        "kind": "rl",
        "onnx_path": path.relative_to(run_dir).as_posix(),
        "onnx_sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
        "task_id": profile.task["task_id"],
        "robot_xml": profile.task["robot_xml"],
        "robot_sha256": profile.task["robot_sha256"],
        "joint_order": profile.execution["joint_order"],
        "home_pose_rad": profile.execution["home_pose_rad"],
        "policy_layout": profile.task["policy_layout"],
        "inputs": [tensor(value) for value in model.graph.input],
        "outputs": [tensor(value) for value in model.graph.output],
        "onnx_metadata": {item.key: item.value for item in model.metadata_props},
        "control_s": profile.execution["periods"]["control_s"],
        "body": profile.execution["body"],
        "model_kind": profile.model_kind,
        "domain_randomization": domain_randomization,
        "execution_fingerprint": profile.execution["execution_fingerprint"],
    }
    pending = run_dir / "artifact.json.tmp"
    pending.write_text(json.dumps(manifest, allow_nan=False, indent=2), encoding="utf-8")
    pending.replace(run_dir / "artifact.json")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("operation", choices=("describe", "run", "evaluate-teacher"))
    parser.add_argument("task")
    parser.add_argument("--joint-order", type=json.loads)
    parser.add_argument("--profile", type=Path)
    parser.add_argument("--run-dir", type=Path)
    parser.add_argument("--adapters", type=Path)
    parser.add_argument("--preview-bodies", type=json.loads)
    parser.add_argument("--num-envs", type=int)
    parser.add_argument("--terrain")
    parser.add_argument("--domain-randomization", choices=("on", "off"), default="on")
    parser.add_argument("--teacher-spec", type=Path)
    parser.add_argument("--evaluation-steps", type=int, default=100)
    parser.add_argument("--evaluation-seed", type=int, default=0)
    args, trailing = parser.parse_known_args(argv)
    if trailing and trailing[0] == "--":
        trailing = trailing[1:]
    if any(flag.split("=", 1)[0] == "--hf-jobs" for flag in trailing):
        parser.error("desktop training runs locally")
    from mjlab_microduck.checkpoints import consume_init_flag

    trailing = consume_init_flag(trailing)
    if args.operation == "describe":
        if not isinstance(args.joint_order, list) or not args.joint_order:
            parser.error("describe requires --joint-order")
        with redirect_stdout(sys.stderr):
            cfg = task_config(args.task, trailing, num_envs=args.num_envs, terrain=args.terrain)
            result = describe_task(args.task, cfg.env, args.joint_order)
        print(json.dumps(result, allow_nan=False))
        return
    if args.profile is None or args.run_dir is None or args.adapters is None:
        parser.error("run requires --profile, --run-dir and --adapters")
    sys.path.insert(0, str(args.adapters.resolve()))
    from simulation_profile import load_profile

    profile = load_profile(args.profile)
    cfg = task_config(args.task, trailing, num_envs=args.num_envs, terrain=args.terrain)
    run_dir = args.run_dir.resolve()
    run_dir.mkdir(parents=True, exist_ok=True)
    domain_randomization = args.domain_randomization == "on"
    install_profile(
        args.task, cfg, profile, run_dir,
        domain_randomization=domain_randomization,
    )
    run_id = os.environ.get("MICRODUCK_TRAINING_RUN_ID")
    if run_id and args.preview_bodies:
        from training_preview import install_training_preview

        install_training_preview(cfg.env, profile, run_id, args.preview_bodies)
    if args.operation == "evaluate-teacher":
        if args.teacher_spec is None or cfg.env.scene.num_envs != 1 or domain_randomization:
            parser.error("teacher evaluation requires a teacher spec, one environment and domain randomization off")
        if not 1 <= args.evaluation_steps <= 5000 or not 0 <= args.evaluation_seed <= 2**31 - 1:
            parser.error("invalid teacher evaluation steps or seed")
        from teacher_evaluation import evaluate

        evaluate(cfg, profile, args.teacher_spec, run_dir, args.evaluation_steps, args.evaluation_seed)
        return
    from mjlab.scripts.train import launch_training

    os.chdir(run_dir)
    launch_training(args.task, cfg)
    _write_artifact(run_dir, profile, domain_randomization=domain_randomization)


if __name__ == "__main__":
    main()
