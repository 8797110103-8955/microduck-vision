"""Evaluate a recorded-policy student in the chosen, resolved MJLab body."""

import hashlib
import json
import math
from dataclasses import replace
from pathlib import Path


_WALK_OBSERVATIONS = (
    "base_ang_vel", "projected_gravity", "joint_pos", "joint_vel", "actions",
    "command", "head_command", "body_command",
)


def _recorded_joint_position(env, asset_cfg, home_rad):
    robot = env.scene[asset_cfg.name]
    return robot.data.joint_pos[:, asset_cfg.joint_ids] - home_rad


def _prepare_contract(cfg, profile, spec):
    import torch
    from mjlab.envs.mdp.observations import base_ang_vel, projected_gravity

    contract = spec["policy_contract"]
    expected_body = profile.execution["body"]
    if (
        contract["body_id"] != expected_body["id"]
        or contract["body_version"] != expected_body["version"]
        or not math.isclose(contract["control_hz"] * profile.execution["periods"]["control_s"], 1.0, abs_tol=1e-9)
        or contract["obs_len"] != 61 or contract["action_len"] != 14
        or contract["recurrent"] is True
        or contract["command_encoding"] not in (None, "constant")
        or contract["head_lowpass"] is not None or contract["legs_lowpass"] is not None
        or contract["joint_velocity_delay_ticks"] not in (0, 1)
    ):
        raise ValueError("evaluation requires a matching feed-forward walk policy, period and unfiltered action contract")
    if profile.task["robot_xml"] != "robot_walk.xml":
        raise ValueError("teacher evaluation requires the canonical walk robot and joint axes")
    layout = profile.task["policy_layout"]
    if (
        tuple(term["name"] for term in layout["observations"]) != _WALK_OBSERVATIONS
        or not layout["concatenate_observations"] or layout["observation_history_length"] not in (0, None)
    ):
        raise ValueError("teacher observation order or history differs from the chosen task")
    observations = cfg.env.observations["actor"].terms
    for term in layout["observations"]:
        config = term["configuration"]
        if config["history_length"] or config["scale"] is not None or config["clip"] is not None:
            raise ValueError(f"unsupported teacher observation processing for {term['name']}")
    for name, function in {"joint_pos": "joint_pos_rel", "joint_vel": "joint_vel_rel", "actions": "last_action"}.items():
        if observations[name].func.__name__ != function:
            raise ValueError(f"unsupported teacher observation source {name}")
    for name, command in {"command": "twist", "head_command": "head_pose", "body_command": "body_pose"}.items():
        if observations[name].params.get("command_name") != command:
            raise ValueError(f"unsupported teacher evaluation command term {name}")
    joints = profile.task["robot_joints"]
    driven = [joint for joint in joints if joint is not None]
    (action,) = layout["actions"]
    if action["name"] != "joint_pos" or action["joints"] != driven:
        raise ValueError("teacher action order differs from the chosen task")

    # The dataset holds the deployment encoder's nominal IMU and exact joint home.
    # The task remains responsible for its world, contact model and command generation.
    for name, function in {"base_ang_vel": base_ang_vel, "projected_gravity": projected_gravity}.items():
        observations[name] = replace(observations[name], func=function, params={}, delay_min_lag=0, delay_max_lag=0)
    lag = contract["joint_velocity_delay_ticks"]
    observations["joint_vel"] = replace(observations["joint_vel"], delay_min_lag=lag, delay_max_lag=lag)
    home = dict(zip(joints, spec["home_pose_rad"], strict=True))
    asset = observations["joint_pos"].params["asset_cfg"]
    preferences = observations["joint_vel"].params["asset_cfg"]
    if any(source.name != "robot" or list(source.joint_names) != ["^(?!passive_).*"] for source in (asset, preferences)):
        raise ValueError("teacher sensing needs the canonical driven robot joints")
    observations["joint_pos"] = replace(observations["joint_pos"], func=_recorded_joint_position, params={
        "asset_cfg": asset, "home_rad": torch.tensor([home[joint] for joint in driven], dtype=torch.float32),
    })
    cfg.env.actions["joint_pos"] = replace(
        cfg.env.actions["joint_pos"], scale=contract["action_scale"] or 1.0,
        offset={joint: home[joint] for joint in driven}, use_default_offset=False, clip=None,
    )


def _policy_rollout(cfg, profile, spec, steps, seed):
    import numpy as np
    import onnxruntime as ort
    import torch
    from mjlab.envs import ManagerBasedRlEnv

    _prepare_contract(cfg, profile, spec)
    model = Path(spec["model_path"]).read_bytes()
    if hashlib.sha256(model).hexdigest() != spec["onnx_sha256"]:
        raise ValueError("teacher ONNX digest changed before evaluation")
    session_options = ort.SessionOptions()
    session_options.intra_op_num_threads = 1
    session = ort.InferenceSession(model, sess_options=session_options, providers=["CPUExecutionProvider"])
    for actual, declared in ((session.get_inputs(), spec["input"]), (session.get_outputs(), spec["output"])):
        if len(actual) != 1 or actual[0].name != declared["name"] or actual[0].shape != declared["dimensions"] or actual[0].type != "tensor(float)":
            raise ValueError("teacher ONNX graph differs from the declared tensor contract")
    cfg.env.seed = seed
    env = ManagerBasedRlEnv(cfg=cfg.env, device="cpu", render_mode=None)
    try:
        observations, _ = env.reset()
        reward_sum = 0.0
        completed_episodes = 0
        for step in range(steps):
            actor = observations["actor"].detach().cpu().numpy()
            if actor.shape != (1, 61) or actor.dtype != np.float32 or not np.isfinite(actor).all():
                raise ValueError("evaluation generated invalid teacher observations")
            (action,) = session.run([spec["output"]["name"]], {spec["input"]["name"]: actor})
            if action.shape != (1, 14) or not np.isfinite(action).all():
                raise ValueError("teacher produced invalid evaluation actions")
            observations, reward, terminated, truncated, _ = env.step(torch.from_numpy(action))
            reward_value = float(reward[0])
            if not np.isfinite(reward_value):
                raise ValueError("evaluation produced a non-finite reward")
            reward_sum += reward_value
            completed_episodes += int((terminated | truncated)[0])
            if (step + 1) % 25 == 0 or step + 1 == steps:
                print(json.dumps({"step": step + 1, "total_steps": steps, "reward": reward_sum / (step + 1)}), flush=True)
        return {
            "completed_steps": steps,
            "completed_episodes": completed_episodes,
            "reward_sum": reward_sum,
            "mean_step_reward": reward_sum / steps,
        }
    finally:
        env.close()


def evaluate(cfg, profile, spec_path, run_dir, steps, seed):
    spec = json.loads(spec_path.read_bytes())
    report = {
        "format_version": 1,
        "teacher_policy_id": spec["policy_contract"]["id"],
        "onnx_sha256": spec["onnx_sha256"],
        "teacher_source": spec["source"],
        "task_id": profile.task["task_id"],
        "robot_sha256": profile.task["robot_sha256"],
        "terrain": profile.task["assembly"]["terrain"],
        "model_kind": profile.model_kind,
        "execution_fingerprint": profile.execution["execution_fingerprint"],
        "control_s": profile.execution["periods"]["control_s"],
        "policy_home_pose_rad": spec["home_pose_rad"],
        "policy_contract": spec["policy_contract"],
        "requested_steps": steps,
        "seed": seed,
        "domain_randomization": False,
    }
    try:
        report.update(_policy_rollout(cfg, profile, spec, steps, seed), status="finished")
    except Exception as error:
        report.update(status="failed", error=str(error))
        raise
    finally:
        pending = run_dir / "evaluation.json.tmp"
        pending.write_text(json.dumps(report, allow_nan=False, indent=2), encoding="utf-8")
        pending.replace(run_dir / "evaluation.json")
