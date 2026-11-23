"""Read-only desktop preview of the first environment on the primary worker."""

import json
import math
import os
import time

import torch
from mjlab.managers.recorder_manager import RecorderTerm, RecorderTermCfg


class TrainingPreview(RecorderTerm):
    def __init__(self, cfg, env):
        super().__init__(cfg, env)
        self._primary_worker = int(os.environ.get("RANK", "0")) == 0
        self._last_sent = -math.inf
        self._metadata = cfg.params["metadata"]
        self._robot = env.scene["robot"]
        self._joint_slots = cfg.params["joint_slots"]
        driven = [name for name in self._joint_slots if name is not None]
        self._joint_indices = [self._robot.joint_names.index(name) for name in driven]
        self._passive_indices = (
            [self._robot.joint_names.index(f"passive_{name}_backlash") for name in driven]
            if cfg.params["encoder_through_backlash"] else []
        )
        self._body_indices = [self._robot.body_names.index(name) for name in cfg.params["body_names"]]

    def record_post_step(self):
        if not self._primary_worker:
            return
        now = time.monotonic()
        if now - self._last_sent < 0.1:
            return
        self._last_sent = now

        data = self._robot.data
        positions = data.joint_pos[0, self._joint_indices]
        if self._passive_indices:
            positions = positions + data.joint_pos[0, self._passive_indices]
        origin = self._env.scene.env_origins[0]
        body_poses = data.body_link_pose_w[0, self._body_indices]
        local_poses = torch.cat((body_poses[:, :3] - origin, body_poses[:, 3:]), dim=-1)
        values = torch.cat((positions, local_poses.flatten())).tolist()
        if not all(math.isfinite(value) for value in values):
            return
        count = len(self._joint_indices)
        angles = iter(values[:count])
        bodies = [values[index:index + 7] for index in range(count, len(values), 7)]
        payload = {
            **self._metadata,
            "sequence": self._env.common_step_counter,
            "environmentIndex": 0,
            "positions": [None if name is None else next(angles) for name in self._joint_slots],
            "trunkPosition": bodies[0][:3],
            "trunkQuaternion": bodies[0][3:],
            "frame": {"bodies": bodies, "markers": []},
        }
        print("MICRODUCK_TRAINING_FRAME " + json.dumps(payload, separators=(",", ":")), flush=True)


def install_training_preview(env_cfg, profile, run_id, body_names):
    env_cfg.recorders["desktop_preview"] = RecorderTermCfg(
        func=TrainingPreview,
        params={
            "metadata": {
                "runId": run_id,
                "modelKind": profile.model_kind,
                "executionFingerprint": profile.execution["execution_fingerprint"],
            },
            "body_names": body_names,
            "joint_slots": profile.task["robot_joints"],
            "encoder_through_backlash": profile.simulation["response"]["encoder_through_backlash"],
        },
    )
