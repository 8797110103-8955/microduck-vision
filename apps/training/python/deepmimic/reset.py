"""Reference state initialization, delayed command history and excluded warmup."""

import json
import math
from dataclasses import dataclass, fields, replace
from pathlib import Path

import torch
from mjlab.envs.mdp.actions import JointPositionAction, JointPositionActionCfg
from mjlab.managers import EventTermCfg
from mjlab.managers.recorder_manager import RecorderTerm, RecorderTermCfg

from .motion import rotate_heading, yaw
from .rewards import align_world, squared_errors


def history_indices(frames, max_lag, decimation):
    """Frame k is post-control: its target was held for the last decimation ticks."""
    offsets = torch.div(torch.arange(-max_lag, 0, device=frames.device), decimation, rounding_mode="floor") + 1
    return frames[:, None] + offsets[None, :]


def validate_history(motion, max_lag, decimation, recipe):
    settings = recipe["reset"]
    probability, warmup = settings["reference_probability"], settings["warmup_steps"]
    if not 0 <= probability <= 1 or type(warmup) is not int or warmup < 1:
        raise ValueError("Invalid reference reset probability or warmup length")
    if warmup < math.ceil(max_lag / decimation):
        raise ValueError("Reference warmup must cover the full actuator delay")
    if (not motion.manifest["quality"]["applied_targets_available"]
            or "teacher_applied_targets" not in motion.arrays or "teacher_action" not in motion.arrays):
        raise ValueError(f"DeepMimic reset requires teacher_action and teacher_applied_targets in {motion.manifest['bundle_id']}")
    history_frames = max(2, math.ceil(max_lag / decimation))
    if len(motion.time) < history_frames + warmup + 2:
        raise ValueError(f"DeepMimic bundle {motion.manifest['bundle_id']} is too short for {history_frames} history frames and {warmup} warmup steps")
    return history_frames


def observation_slice(manager, group, term):
    names = manager.active_terms[group]
    offset = 0
    for name, shape in zip(names, manager.group_obs_term_dim[group], strict=True):
        width = math.prod(shape)
        if name == term:
            return slice(offset, offset + width)
        offset += width
    raise ValueError(f"Missing {group}/{term} observation")


def sample_reset_frames(count, total_frames, history_frames, settings, generator, device):
    selected = torch.rand(count, device=device, generator=generator) < settings["reference_probability"]
    starts = torch.zeros(count, dtype=torch.long, device=device)
    high = total_frames - settings["warmup_steps"] - 1
    starts[selected] = torch.randint(history_frames, high, (int(selected.sum()),), device=device, generator=generator)
    return selected, starts


class ResetController:
    def __init__(self, state):
        self.state, self.env = state, state.env
        self.motion, self.robot, self.recipe = state.motion, state.robot, state.recipe
        self.actuator = self.robot.actuators[0]
        self.term = self.env.action_manager.get_term("joint_pos")
        self.action_order = [self.motion.joint_names.index(name) for name in self.term.target_names]
        self.history_order = [self.motion.joint_names.index(name) for name in self.actuator.history_target_names]
        self.history_motor_ids = [self.robot.joint_names.index(name) for name in self.actuator.history_target_names]
        self.max_lag = self.actuator.cfg.delay_max_lag
        self.history_frames = validate_history(self.motion, self.max_lag, self.env.cfg.decimation, self.recipe)
        n, device = self.env.num_envs, self.env.device
        # Reset sampling must not change random actor/critic initialization when
        # comparing RSI against an ordinary-reset baseline at the same seed.
        self.sampler_seed = int(self.env.cfg.seed or 0) + self.recipe["reset"]["sampler_seed_offset"]
        self.generator = torch.Generator(device=device).manual_seed(self.sampler_seed)
        self.remaining = torch.zeros(n, dtype=torch.long, device=device)
        self.frames = torch.zeros(n, dtype=torch.long, device=device)
        self.reference = torch.zeros(n, dtype=torch.bool, device=device)
        self.current_warmup = torch.zeros_like(self.reference)
        self.last_reference = self.reference.clone()
        self.awaiting_first = torch.ones_like(self.reference)
        self.first_in_step = torch.zeros_like(self.reference)
        self.action_history = torch.zeros((n, 3, 14), device=device)
        self.applied_history = torch.zeros((n, self.max_lag, 14), device=device)
        self.warmup_targets = torch.zeros((n, 14), device=device)
        self.serial = 0
        self.reset_counts = {"reference": 0, "ordinary": 0}
        self.valid_counts = {"reference": 0, "ordinary": 0}

    def reset(self, env_ids):
        ids = torch.arange(self.env.num_envs, device=self.env.device) if env_ids is None else env_ids
        count = len(ids)
        if count == 0:
            return
        settings = self.recipe["reset"]
        selected, starts = sample_reset_frames(count, len(self.motion.time), self.history_frames,
            settings, self.generator, self.env.device)
        self.reference[ids] = selected
        self.frames[ids] = 0
        self.remaining[ids] = settings["warmup_steps"]
        self.awaiting_first[ids] = True
        self.action_history[ids] = 0
        default = self.robot.data.default_root_state[ids]
        self.state.clock.reset(ids)
        self.state.heading[ids] = yaw(default[:, 3:7]) - yaw(self.motion.arrays["root_quat_wxyz"][0])
        home = self.robot.data.default_joint_pos[ids][:, self.history_motor_ids]
        self.applied_history[ids] = home[:, None, :]
        reference_ids = ids[selected]
        if len(reference_ids):
            start = starts[selected]
            self.frames[reference_ids] = start
            sample = self.motion.sample(self.motion.time[start])
            desired = self.robot.data.default_root_state[reference_ids]
            heading = yaw(desired[:, 3:7]) - yaw(sample["root_quat_wxyz"])
            translation = self.env.scene.env_origins[reference_ids].clone()
            translation[:, :2] += desired[:, :2] - rotate_heading(sample["root_pos"], heading)[:, :2]
            aligned = align_world(sample, heading, translation)
            position, velocity = self.robot.data.joint_pos[reference_ids].clone(), self.robot.data.joint_vel[reference_ids].clone()
            for key, indices in (("motor", self.state.motor_ids), ("passive", self.state.passive_ids)):
                position[:, indices], velocity[:, indices] = sample[f"{key}_qpos"], sample[f"{key}_qvel"]
            self.robot.write_joint_state_to_sim(position, velocity, env_ids=reference_ids)
            self.robot.write_root_state_to_sim(torch.cat((aligned["root_pos"], aligned["root_quat_wxyz"],
                aligned["root_lin_vel_world"], aligned["root_ang_vel_world"]), dim=-1), env_ids=reference_ids)
            self.state.clock.reset(reference_ids, self.motion.time[start])
            self.state.heading[reference_ids] = heading
            previous = start[:, None] - torch.arange(3, device=self.env.device)[None, :]
            self.action_history[reference_ids] = self.motion.arrays["teacher_action"][previous][:, :, self.action_order]
            prior = history_indices(start, self.max_lag, self.env.cfg.decimation)
            self.applied_history[reference_ids] = self.motion.arrays["teacher_applied_targets"][prior][:, :, self.history_order]
        self.reset_counts["reference"] += int(selected.sum())
        self.reset_counts["ordinary"] += count - int(selected.sum())
        self.serial += 1

    def after_reset(self, ids, stream):
        manager = self.env.action_manager
        for index, buffer in enumerate((manager.action, manager.prev_action, manager.prev_prev_action)):
            buffer[ids] = self.action_history[ids, index]
        self.term.raw_action[ids] = manager.action[ids]
        self.actuator.seed_targets(ids, self.applied_history[ids])
        # The pinned manager already computed and cached observations. Patch only
        # action columns, preserving real velocity samples and neighbours' history.
        for group in self.env.observation_manager.active_terms:
            term = self.env.observation_manager.get_term_cfg(group, "actions")
            if term.scale is not None or term.clip is not None or term.delay_max_lag != 0:
                raise ValueError("DeepMimic reset expects unscaled, undelayed last-action observations")
            columns = observation_slice(self.env.observation_manager, group, "actions")
            self.env.obs_buf[group][ids, columns] = manager.action[ids]
        errors = squared_errors(self.state.student(ids), self.motion.sample(self.state.clock.times[ids]), self.state.heading[ids])
        examples = ids[:4]
        stream.write(json.dumps({
            "event": "reset", "serial": self.serial, "step": self.env.common_step_counter,
            "sampler_seed": self.sampler_seed,
            "env_ids": ids.tolist(), "reference": self.reference[ids].tolist(),
            "source_frames": self.frames[ids].tolist(), "warmup_steps": self.recipe["reset"]["warmup_steps"],
            "actuator_delay_physics_steps": [self.actuator.cfg.delay_min_lag, self.max_lag],
            "history_frames": self.history_frames, "example_env_ids": examples.tolist(),
            "example_previous_actions": self.action_history[examples].tolist(),
            "example_applied_targets": self.applied_history[examples].tolist(),
            "residual_squared_error_max": {name: float(value.max()) for name, value in errors.items()},
            "reset_counts": self.reset_counts,
        }, allow_nan=False) + "\n")
        stream.flush()

    def begin_control(self, actions):
        self.current_warmup = self.remaining > 0
        self.last_reference = self.reference.clone()
        self.first_in_step = self.awaiting_first & ~self.current_warmup
        actual = actions.clone()
        ids = self.current_warmup.nonzero(as_tuple=False).squeeze(-1)
        if len(ids):
            self.frames[ids] += 1
            actual[ids] = 0.0
            self.warmup_targets[ids] = self.robot.data.default_joint_pos[ids][:, self.term.target_ids]
            reference_ids = ids[self.reference[ids]]
            if len(reference_ids):
                frames = self.frames[reference_ids]
                actual[reference_ids] = self.motion.arrays["teacher_action"][frames][:, self.action_order]
                self.warmup_targets[reference_ids] = self.motion.arrays["teacher_applied_targets"][frames][:, self.action_order]
        return actual, ids

    def finish_control(self, stream):
        reset = self.env.reset_buf
        warm = self.current_warmup & ~reset
        self.remaining[warm] -= 1
        self.env.extras["training_valid"] = ~self.current_warmup
        self.env.extras["training_next_valid"] = self.remaining == 0
        for label, selected in (("reference", self.last_reference), ("ordinary", ~self.last_reference)):
            self.valid_counts[label] += int((selected & ~self.current_warmup).sum())
        completed = warm & (self.remaining == 0)
        for label, mask in (("warmup_complete", completed), ("first_valid", self.first_in_step)):
            ids = mask.nonzero(as_tuple=False).squeeze(-1)
            if len(ids):
                velocity = observation_slice(self.env.observation_manager, "actor", "joint_vel")
                stream.write(json.dumps({
                    "event": label, "step": self.env.common_step_counter, "env_ids": ids.tolist(),
                    "reference": self.last_reference[ids].tolist(), "reference_time_s": self.state.last_times[ids].tolist(),
                    "squared_error": {name: value[ids].tolist() for name, value in self.state.last_errors.items()},
                    "observed_delayed_velocity": self.env.obs_buf["actor"][ids, velocity].tolist(),
                    "valid_counts": self.valid_counts,
                }, allow_nan=False) + "\n")
        self.awaiting_first[self.first_in_step & ~reset] = False
        stream.flush()


def reference_reset(env, env_ids):
    env._deepmimic_reference.reset_controller.reset(env_ids)


class ReferencePositionAction(JointPositionAction):
    def __init__(self, cfg, env):
        super().__init__(cfg, env)
        self.warm_ids = torch.empty(0, dtype=torch.long, device=self.device)

    def process_actions(self, actions):
        controller = self._env._deepmimic_reference.reset_controller
        actual, self.warm_ids = controller.begin_control(actions)
        super().process_actions(actual)
        self._env.action_manager.action[:] = actual

    def apply_actions(self):
        super().apply_actions()
        state = getattr(self._env, "_deepmimic_reference", None)
        if state is not None:
            state.reset_controller.actuator.physics_pending = True
        if len(self.warm_ids):
            controller = self._env._deepmimic_reference.reset_controller
            self._entity.set_joint_position_target(controller.warmup_targets[self.warm_ids],
                joint_ids=self.target_ids, env_ids=self.warm_ids[:, None])

    def reset(self, env_ids=None):
        super().reset(env_ids)
        # A reset write_data_to_sim happens before the late history restoration.
        # It must not reuse a previous control's warmup row selection.
        self.warm_ids = torch.empty(0, dtype=torch.long, device=self.device)


@dataclass(kw_only=True)
class ReferencePositionActionCfg(JointPositionActionCfg):
    def build(self, env):
        return ReferencePositionAction(self, env)


class ReferenceResetRecorder(RecorderTerm):
    def __init__(self, cfg, env):
        super().__init__(cfg, env)
        self.stream = Path(cfg.params["path"]).open("a", encoding="utf-8")

    def record_post_reset(self, env_ids):
        if len(env_ids):
            self._env._deepmimic_reference.reset_controller.after_reset(env_ids, self.stream)

    def record_post_step(self):
        self._env._deepmimic_reference.reset_controller.finish_control(self.stream)

    def close(self):
        self.stream.close()


def install_reset(cfg, motion, run_dir, recipe):
    from training.body import HistoryBacklashCfg

    robot = cfg.env.scene.entities["robot"]
    original = robot.articulation.actuators[0]
    history_frames = validate_history(motion, original.delay_max_lag, cfg.env.decimation, recipe)
    if len(cfg.env.actions) != 1 or "joint_pos" not in cfg.env.actions:
        raise ValueError("DeepMimic history requires one joint_pos action group")
    actuator = HistoryBacklashCfg(**{field.name: getattr(original, field.name) for field in fields(original) if field.init})
    cfg.env.scene.entities["robot"] = replace(robot, articulation=replace(robot.articulation, actuators=(actuator,)))
    action = cfg.env.actions["joint_pos"]
    cfg.env.actions["joint_pos"] = ReferencePositionActionCfg(**{field.name: getattr(action, field.name) for field in fields(action) if field.init})
    cfg.env.events["initialize_reference"].params["reset_enabled"] = True
    cfg.env.events["reset_reference_clock"] = EventTermCfg(func=reference_reset, mode="reset")
    cfg.env.recorders = {
        "reference_reset": RecorderTermCfg(func=ReferenceResetRecorder,
            params={"path": str(Path(run_dir).resolve() / "reference-reset.jsonl")}),
        **cfg.env.recorders,
    }
    (Path(run_dir) / "reference-reset-configuration.json").write_text(json.dumps({
        "recipe": recipe["reset"], "minimum_history_frames": history_frames,
        "actuator_delay_physics_steps": [original.delay_min_lag, original.delay_max_lag],
        "source_bundle_id": motion.manifest["bundle_id"],
        "source_manifest_sha256": motion.manifest_sha256,
        "sampler_seed_rule": "environment_seed + sampler_seed_offset",
    }, indent=2, allow_nan=False), encoding="utf-8")
