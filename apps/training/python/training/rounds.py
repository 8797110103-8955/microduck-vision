"""Controlled checkpoint inheritance and independently publishable save points."""

import hashlib
import json
from pathlib import Path


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def signature(task_id, cfg, profile, run_dir=None, reference=None):
    from training.runtime import describe_value

    agent = describe_value(cfg.agent)
    for key in ("max_iterations", "save_interval", "logger", "upload_model", "run_name", "experiment_name"):
        agent.pop(key, None)
    result = {
        "task_id": task_id,
        "layout": profile.task["policy_layout"],
        "joint_order": profile.execution["joint_order"],
        "control_s": profile.execution["periods"]["control_s"],
        "execution_fingerprint": profile.execution["execution_fingerprint"],
        "environment": describe_value(cfg.env),
        "agent": agent,
        "reference": reference,
    }
    def normalize(value):
        if isinstance(value, dict):
            return {key: normalize(item) for key, item in value.items()}
        if isinstance(value, list):
            return [normalize(item) for item in value]
        if isinstance(value, str) and run_dir is not None:
            return value.replace(str(Path(run_dir).resolve()), "$ROUND")
        return value
    return normalize(result)


def validate_parent(current, parent, mode):
    for key in ("task_id", "layout", "joint_order", "control_s"):
        if current[key] != parent[key]:
            raise ValueError(f"parent checkpoint has incompatible {key}")
    if mode == "resume" and current != parent:
        raise ValueError("resume requires the same environment and agent configuration; choose adaptation for changed settings")
    if mode not in ("resume", "adapt"):
        raise ValueError("unknown initialization mode")


def load_parent(runner, parent):
    import torch

    if digest(parent["checkpoint_path"]) != parent["checkpoint_sha256"]:
        raise ValueError("parent checkpoint digest changed before loading")
    if parent["mode"] == "resume":
        if parent.get("reset_normalization"):
            raise ValueError("normalization reset requires adaptation")
        runner.load(parent["checkpoint_path"])
    else:
        state = torch.load(parent["checkpoint_path"], map_location=runner.device, weights_only=False)
        if parent.get("reset_normalization"):
            for key, value in runner.alg.actor.state_dict().items():
                if key.startswith("obs_normalizer."):
                    state["actor_state_dict"][key] = value
        runner.alg.load(state, {"actor": True}, strict=True)


def install(task_id, cfg, profile, run_dir, initialization, *, domain_randomization, reference=None, walking=None):
    import os
    from mjlab.tasks.registry import load_runner_cls
    from mjlab.rl.runner import MjlabOnPolicyRunner

    run_dir = Path(run_dir)
    os.environ.pop("MICRODUCK_INIT_CHECKPOINT", None)
    os.environ.pop("MICRODUCK_WARM_START", None)
    current = signature(task_id, cfg, profile, run_dir, reference)
    parent = None
    if initialization is not None:
        parent = json.loads(Path(initialization).read_bytes())
        saved = json.loads((Path(parent["parent_directory"]) / "round-input.json").read_bytes())
        validate_parent(current, saved, parent["mode"])
        if digest(parent["checkpoint_path"]) != parent["checkpoint_sha256"]:
            raise ValueError("parent checkpoint digest changed")
    (run_dir / "round-input.json").write_text(json.dumps(current, indent=2, allow_nan=False), encoding="utf-8")
    runner_cls = load_runner_cls(task_id) or MjlabOnPolicyRunner
    original_learn, original_save = runner_cls.learn, runner_cls.save

    def learn(self, *args, **kwargs):
        if parent is not None:
            load_parent(self, parent)
            print(f"[round] initialized via {parent['mode']} from {parent['checkpoint_sha256']}", flush=True)
        if walking is None:
            return original_learn(self, *args, **kwargs)
        return walking.learn(self, lambda: original_learn(self, *args, **kwargs), run_dir,
                             cfg.agent.max_iterations, from_teacher=parent is None)

    def save(self, path, infos=None):
        original_save(self, path, infos)
        from mjlab.rl.exporter_utils import attach_metadata_to_onnx, get_base_metadata
        from training.artifacts import write_artifact

        checkpoint = Path(path).resolve()
        snapshot = run_dir / "checkpoints" / checkpoint.stem
        snapshot.mkdir(parents=True, exist_ok=True)
        self.export_policy_to_onnx(str(snapshot), "policy.onnx")
        attach_metadata_to_onnx(str(snapshot / "policy.onnx"), get_base_metadata(self.env.unwrapped, "local"))
        write_artifact(run_dir, profile, domain_randomization=domain_randomization,
                       model_path=snapshot / "policy.onnx", destination=snapshot)
        description = {
            "id": checkpoint.stem,
            "file": checkpoint.relative_to(run_dir).as_posix(),
            "sha256": digest(checkpoint),
            "iteration": self.current_learning_iteration,
        }
        temporary = snapshot / "checkpoint.json.tmp"
        temporary.write_text(json.dumps(description, indent=2), encoding="utf-8")
        temporary.replace(snapshot / "checkpoint.json")

    runner_cls.learn, runner_cls.save = learn, save
