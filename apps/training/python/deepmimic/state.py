"""Per-environment reference clocks and MJLab's pre-reset reward integration."""

import json
import math
from pathlib import Path

import torch
from mjlab.managers import EventTermCfg, RewardTermCfg, TerminationTermCfg
from mjlab.managers.metrics_manager import MetricsTermCfg
from mjlab.managers.recorder_manager import RecorderTerm, RecorderTermCfg

from .motion import ReferenceMotion, rotate_heading, yaw
from .recipe import default_recipe, resolved_recipe
from .rewards import exponentials, reference_complete, reference_error, reference_reward, squared_errors


class ReferenceClock:
    def __init__(self, num_envs, step_s, *, device="cpu", step_counter=0):
        self.phase_s = torch.zeros(num_envs, dtype=torch.float64, device=device)
        self.elapsed_steps = torch.zeros(num_envs, dtype=torch.int64, device=device)
        self.step_s = step_s
        self.step_counter = step_counter
        self.revision = 0

    @property
    def times(self):
        # Integer * Python float otherwise promotes through torch's float32
        # default, which can miss a finite clip's terminal sample by one step.
        return self.phase_s + self.elapsed_steps.to(torch.float64) * self.step_s

    def advance(self, step_counter):
        if step_counter < self.step_counter:
            raise ValueError("Reference clock cannot move backwards")
        if step_counter == self.step_counter:
            return
        self.elapsed_steps += step_counter - self.step_counter
        self.step_counter = step_counter

    def reset(self, env_ids, phase_s=0.0):
        phase = torch.as_tensor(phase_s, dtype=torch.float64, device=self.phase_s.device)
        if not torch.isfinite(phase).all() or (phase < 0).any():
            raise ValueError("Reference reset phases must be finite and nonnegative")
        ids = slice(None) if env_ids is None else env_ids
        self.phase_s[ids] = phase
        self.elapsed_steps[ids] = 0
        self.revision += 1


class ReferenceState:
    def __init__(self, env, motion, recipe):
        self.env, self.motion, self.recipe = env, motion, recipe
        self.robot = env.scene["robot"]
        names = self.robot.joint_names
        self.motor_ids = [names.index(name) for name in motion.joint_names]
        self.passive_ids = [names.index(f"passive_{name}_backlash") for name in motion.joint_names]
        self.foot_ids = [self.robot.site_names.index(foot["site"]) for foot in motion.manifest["feet"]]
        self.clock = ReferenceClock(env.num_envs, env.step_dt, device=env.device, step_counter=env.common_step_counter)
        self.heading = torch.zeros(env.num_envs, dtype=motion.dtype, device=env.device)
        self._prepared_key = None
        self.last_errors, self.last_rewards, self.last_sample = {}, {}, {}
        self.last_times = self.clock.times.clone()

    def prepare(self):
        self.clock.advance(self.env.common_step_counter)
        key = (self.clock.step_counter, self.clock.revision)
        if key == self._prepared_key:
            return
        # Pinned MJLab rewards otherwise read derived poses from before the last
        # physics integration. Refresh once before termination/reward evaluation.
        self.env.sim.forward()
        self.env._training_forward_step = self.env.common_step_counter
        reference = self.motion.sample(self.clock.times)
        student = self.student()
        self.last_errors = squared_errors(student, reference, self.heading)
        if "forward_velocity" in self.recipe["reference"]:
            angle = -(self.heading + yaw(self.motion.arrays["root_quat_wxyz"][0]))
            student_speed = rotate_heading(self.robot.data.root_link_lin_vel_w, angle)[:, 0]
            target_speed = rotate_heading(reference["root_lin_vel_world"], angle + self.heading)[:, 0]
            self.last_errors["forward_velocity"] = (student_speed - target_speed).square()
        self.last_rewards = exponentials(self.last_errors, self.recipe)
        self.last_sample = reference
        self.last_times = self.clock.times.clone()
        self._prepared_key = key

    def student(self, env_ids=None):
        ids = slice(None) if env_ids is None else env_ids
        data, origins = self.robot.data, self.env.scene.env_origins[ids]
        return {
            "output_qpos": data.joint_pos[ids][:, self.motor_ids] + data.joint_pos[ids][:, self.passive_ids],
            "output_qvel": data.joint_vel[ids][:, self.motor_ids] + data.joint_vel[ids][:, self.passive_ids],
            "root_pos": data.root_link_pos_w[ids] - origins,
            "root_quat_wxyz": data.root_link_quat_w[ids],
            "foot_pos_world": data.site_pos_w[ids][:, self.foot_ids] - origins[:, None, :],
        }


def initialize_reference(env, env_ids, bundle, manifest_sha256, recipe, reset_enabled=False):
    del env_ids
    motion = ReferenceMotion(bundle, device=env.device)
    if motion.manifest_sha256 != manifest_sha256:
        raise ValueError("Reference manifest changed after training input validation")
    env._deepmimic_reference = ReferenceState(env, motion, recipe)
    if reset_enabled:
        from .reset import ResetController

        env._deepmimic_reference.reset_controller = ResetController(env._deepmimic_reference)


def reset_reference_clock(env, env_ids):
    state = env._deepmimic_reference
    state.clock.reset(env_ids)
    ids = slice(None) if env_ids is None else env_ids
    initial = state.motion.arrays["root_quat_wxyz"][0]
    state.heading[ids] = yaw(state.robot.data.default_root_state[ids, 3:7]) - yaw(initial)


class ReferenceMetrics(RecorderTerm):
    def __init__(self, cfg, env):
        super().__init__(cfg, env)
        path = Path(cfg.params["path"])
        path.parent.mkdir(parents=True, exist_ok=True)
        self.stream = path.open("a", encoding="utf-8")

    def record_post_step(self):
        step = self._env.common_step_counter
        if step != 1 and step % 24 != 0:
            return
        state = self._env._deepmimic_reference

        def scalar(value):
            result = float(value.mean().item())
            return result if math.isfinite(result) else None

        # These values were captured before automatic resets changed the phases.
        result = {
            "step": step,
            "reference_reward": {name: scalar(value) for name, value in state.last_rewards.items()},
            "reference_squared_error": {name: scalar(value) for name, value in state.last_errors.items()},
            "reference_time_min_s": float(state.last_times.min().item()),
            "reference_time_max_s": float(state.last_times.max().item()),
            "reference_complete_fraction": scalar(state.last_sample["completed"].float()),
        }
        controller = getattr(state, "reset_controller", None)
        if controller is not None:
            valid = ~controller.current_warmup
            result["valid_reference_reward"] = {
                name: scalar(value[valid]) if valid.any() else None for name, value in state.last_rewards.items()
            }
            result["valid_reference_squared_error"] = {
                name: scalar(value[valid]) if valid.any() else None for name, value in state.last_errors.items()
            }
            result["valid_transition_counts"] = dict(controller.valid_counts)
            result["reset_modes"] = {
                name: {
                    "valid_envs": int((mask & ~controller.current_warmup).sum()),
                    "warmup_envs": int((mask & controller.current_warmup).sum()),
                    "squared_error": {key: scalar(value[mask & valid]) if (mask & valid).any() else None
                                      for key, value in state.last_errors.items()},
                    "warmup_squared_error": {
                        key: scalar(value[mask & ~valid]) if (mask & ~valid).any() else None
                        for key, value in state.last_errors.items()
                    },
                }
                for name, mask in (("reference", controller.last_reference), ("ordinary", ~controller.last_reference))
            }
        self.stream.write(json.dumps(result, allow_nan=False) + "\n")
        self.stream.flush()

    def close(self):
        self.stream.close()


def install_reference(cfg, bundle, run_dir, *, recipe=None, with_reset=True):
    recipe = default_recipe() if recipe is None else recipe
    motion = ReferenceMotion(bundle)
    if motion.manifest["body"]["model_kind"] != "xl330":
        raise ValueError("DeepMimic task requires an XL330 source")
    parameters = {
        "bundle": str(Path(bundle).resolve()), "manifest_sha256": motion.manifest_sha256,
        "recipe": recipe,
    }
    cfg.env.events["initialize_reference"] = EventTermCfg(func=initialize_reference, mode="startup", params=parameters)
    cfg.env.events["reset_reference_clock"] = EventTermCfg(func=reset_reference_clock, mode="reset")
    cfg.env.terminations = {
        "reference_complete": TerminationTermCfg(func=reference_complete, time_out=False),
        **cfg.env.terminations,
    }
    for name, options in recipe["reference"].items():
        if name == "contact":
            continue
        if options["scale"] <= 0 or options["weight"] < 0:
            raise ValueError(f"Invalid reference reward configuration: {name}")
        cfg.env.rewards[f"reference_{name}"] = RewardTermCfg(
            func=reference_reward, weight=options["weight"], params={"component": name},
        )
        cfg.env.metrics[f"reference_{name}_squared_error"] = MetricsTermCfg(
            func=reference_error, params={"component": name},
        )
    cfg.env.recorders["reference_metrics"] = RecorderTermCfg(
        func=ReferenceMetrics, params={"path": str(Path(run_dir).resolve() / "reference-metrics.jsonl")},
    )
    if with_reset:
        from .reset import install_reset

        install_reset(cfg, motion, run_dir, recipe)
    snapshot = {
        "bundle_id": motion.manifest["bundle_id"], "manifest_sha256": motion.manifest_sha256,
        "clip_sha256": motion.manifest["clip"]["sha256"], "recipe": resolved_recipe(cfg, recipe),
        "reward_state_timing": "post_control_integration_forward_kinematics_before_reset",
    }
    (Path(run_dir) / "reference-configuration.json").write_text(json.dumps(snapshot, indent=2, allow_nan=False), encoding="utf-8")
    return motion
