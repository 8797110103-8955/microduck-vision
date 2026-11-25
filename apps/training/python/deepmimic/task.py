"""Product task provider for recorded XL330 groundcontact/backlash motion."""

from copy import deepcopy
import json
from pathlib import Path

from .motion import ReferenceMotion
from .reset import validate_history

from .recipe import default_recipe, resolved_recipe

TASK_ID = "Microduck-DeepMimic-Velstand-Flat-v1"


def make_env_cfg():
    from mjlab.envs import mdp
    from mjlab.managers import EventTermCfg, TerminationTermCfg
    from mjlab_microduck.robot.microduck_constants import MICRODUCK_BACKLASH_ROBOT_CFG
    from mjlab_microduck.tasks import mdp as duck_mdp
    from mjlab_microduck.tasks.backlash import make_backlash_variant
    from mjlab_microduck.tasks.microduck_velocity_env_cfg import make_microduck_velocity_env_cfg

    recipe = default_recipe()
    cfg = deepcopy(make_microduck_velocity_env_cfg())
    # Preserve the contact budget of the recorded groundcontact experiment.
    cfg.sim.nconmax = 200
    cfg.rewards = {
        name: deepcopy(cfg.rewards[name]) for name in recipe["task_rewards"]
    }
    for name, weight in recipe["task_rewards"].items():
        cfg.rewards[name].weight = weight
    cfg.curriculum = {}
    cfg.events = {
        "reset_scene_to_default": EventTermCfg(func=mdp.reset_scene_to_default, mode="reset"),
        "expand_bam_friction_fields": EventTermCfg(func=duck_mdp.expand_bam_friction_fields, mode="startup"),
        "reset_action_history": EventTermCfg(func=duck_mdp.reset_action_history, mode="reset"),
    }
    cfg.terminations = {
        "time_out": TerminationTermCfg(func=mdp.time_out, time_out=True),
        "fallen": TerminationTermCfg(func=fallen, params={
            "minimum_height": recipe["termination"]["minimum_root_height_m"],
            "maximum_tilt": recipe["termination"]["maximum_tilt_rad"],
        }),
        "nan_state": TerminationTermCfg(func=duck_mdp.robot_state_is_nan,
            params={"sensor_names": ("feet_ground_contact",)}),
    }
    cfg = make_backlash_variant(cfg, deepcopy(MICRODUCK_BACKLASH_ROBOT_CFG))
    # The pinned robot's InitialStateCfg leaves root z at zero; its normal
    # walking reset_base event supplies the standing height. We replaced that
    # event, so the ordinary reset must supply an explicit above-ground pose.
    cfg.scene.entities["robot"].init_state.pos = (0.0, 0.0, recipe["reset"]["nominal_root_height_m"])
    twist = cfg.commands["twist"]
    twist.ranges.lin_vel_x = (0.15, 0.15)
    twist.ranges.lin_vel_y = (0.0, 0.0)
    twist.ranges.ang_vel_z = (0.0, 0.0)
    twist.rel_standing_envs = 0.0
    twist.rel_heading_envs = 0.0
    twist.rel_turn_in_place_envs = 0.0
    twist.rel_world_envs = 0.0
    twist.rel_forward_envs = 0.0
    twist.init_velocity_prob = 0.0
    twist.heading_command = False
    twist.ranges.heading = None
    for name in ("head_pose", "body_pose"):
        command = cfg.commands[name]
        command.ranges = tuple((0.0, 0.0) for _ in command.ranges)
    for group in cfg.observations.values():
        group.enable_corruption = False
        for name, term in group.terms.items():
            if term is None:
                continue
            # Keep the deployed velocity observation lag, but remove IMU DR/delay.
            term.delay_min_lag = term.delay_max_lag = int(group is cfg.observations["actor"] and name == "joint_vel")
            term.delay_update_period = 0
            if name == "base_ang_vel":
                term.func, term.params = mdp.base_ang_vel, {}
            if name == "projected_gravity":
                term.func, term.params = mdp.projected_gravity, {}
            if name == "joint_pos":
                term.params["biased"] = False
    cfg.scene.num_envs = recipe["num_envs"]
    cfg.sim.mujoco.timestep = recipe["physics_s"]
    cfg.decimation = 4
    cfg.episode_length_s = recipe["termination"]["maximum_episode_s"]
    cfg.seed = recipe["seed"]
    return cfg


def fallen(env, minimum_height, maximum_tilt):
    from mjlab.envs import mdp

    if getattr(env, "_training_forward_step", -1) != env.common_step_counter:
        env.sim.forward()
    return mdp.root_height_below_minimum(env, minimum_height + env.scene.env_origins[:, 2]) | mdp.bad_orientation(
        env, limit_angle=maximum_tilt,
    )


def make_rl_cfg():
    from mjlab.rl import RslRlPpoAlgorithmCfg
    from mjlab_microduck.tasks.microduck_velstand_env_cfg import MicroduckVelStandRlCfg

    recipe = default_recipe()
    cfg = deepcopy(MicroduckVelStandRlCfg)
    # A plain PPO config contains no frozen experts, BC optimizer or checkpoint BC.
    cfg.algorithm = RslRlPpoAlgorithmCfg(entropy_coef=0.01)
    cfg.resume = False
    cfg.seed = recipe["seed"]
    cfg.max_iterations = recipe["iterations"]
    cfg.save_interval = recipe["iterations"]
    cfg.experiment_name = "deepmimic-velstand"
    cfg.run_name = "recorded-reference"
    cfg.logger = "tensorboard"
    cfg.upload_model = False
    return cfg


def register_tasks():
    from mjlab.tasks.registry import list_tasks, register_mjlab_task
    from mjlab.tasks.velocity.rl import VelocityOnPolicyRunner

    class ReferenceRunner(VelocityOnPolicyRunner):
        def learn(self, num_learning_iterations, init_at_random_ep_len=False):
            return super().learn(num_learning_iterations, init_at_random_ep_len=False)

    if TASK_ID not in list_tasks():
        register_mjlab_task(TASK_ID, make_env_cfg(), make_env_cfg(), make_rl_cfg(), ReferenceRunner)


def validate_config(cfg):
    from mjlab.rl import RslRlPpoAlgorithmCfg

    if (type(cfg.agent.algorithm) is not RslRlPpoAlgorithmCfg or cfg.agent.algorithm.class_name != "PPO"
            or cfg.agent.resume or cfg.wandb_run_path is not None):
        raise ValueError("DeepMimic requires randomly initialized actor/critic and plain PPO without checkpoint BC")
    if cfg.env.sim.mujoco.timestep != 0.005 or cfg.env.decimation != 4:
        raise ValueError("DeepMimic requires 5 ms physics and 20 ms control")
    if cfg.agent.seed < 0:
        raise ValueError("DeepMimic seed must be nonnegative")
    twist = cfg.env.commands["twist"]
    if (twist.ranges.lin_vel_x != (0.15, 0.15) or twist.ranges.lin_vel_y != (0.0, 0.0)
            or twist.ranges.ang_vel_z != (0.0, 0.0) or twist.heading_command
            or any(getattr(twist, name) != 0 for name in (
                "rel_standing_envs", "rel_heading_envs", "rel_turn_in_place_envs",
                "rel_world_envs", "rel_forward_envs", "init_velocity_prob"))):
        raise ValueError("DeepMimic requires a constant body-frame [0.15, 0, 0] twist")


def write_recipe(cfg, profile, run_dir):
    import json
    import os
    import torch

    reference = cfg.env.events.get("initialize_reference")
    result = resolved_recipe(cfg, reference.params["recipe"] if reference is not None else None)
    initialization = run_dir / "initialization.json"
    if initialization.is_file():
        parent = json.loads(initialization.read_bytes())
        result["initialization"] = parent["mode"]
        result["parent_checkpoint_sha256"] = parent["checkpoint_sha256"]
    elif "walking_plan" in result:
        result["initialization"] = "teacher_actor_random_critic"
    result["target_execution_fingerprint"] = profile.execution["execution_fingerprint"]
    result["target_model_kind"] = profile.model_kind
    result["runtime"] = {
        "torch": torch.__version__, "cuda": torch.version.cuda,
        "uv_no_sync": os.environ.get("UV_NO_SYNC") == "1",
    }
    (run_dir / "deepmimic-recipe.json").write_text(json.dumps(result, indent=2, allow_nan=False), encoding="utf-8")


class TrainingBinding:
    def __init__(self, cfg, bundle, description, *, expected_hash=None, baseline=False):
        from training.runtime import describe_value

        self.motion = ReferenceMotion(bundle)
        manifest = self.motion.manifest
        if expected_hash is not None and expected_hash != self.motion.manifest_sha256:
            raise ValueError("DeepMimic bundle manifest changed after selection")
        body = manifest["body"]
        if (body["model_kind"] != "xl330" or body["robot_xml"] != description["robot_xml"]
                or body["robot_sha256"] != description["robot_sha256"]
                or not description["requires_backlash"]
                or manifest["timing"]["control_s"] != description["periods"]["control_s"]
                or manifest["timing"]["physics_s"] != description["periods"]["physics_s"]
                or manifest["joint_names"] != [joint for joint in description["robot_joints"] if joint is not None]):
            raise ValueError("DeepMimic bundle body, periods or joint order does not match the task")
        self.recipe = resolved_recipe(cfg)
        self.method = "ppo-baseline" if baseline else "deepmimic"
        if baseline:
            for options in self.recipe["reference"].values():
                options["weight"] = 0.0
            self.recipe["reset"]["reference_probability"] = 0.0
        actuator = cfg.env.scene.entities["robot"].articulation.actuators[0]
        validate_history(self.motion, actuator.delay_max_lag, cfg.env.decimation, self.recipe)
        self.snapshot = {
            "training_method": self.method,
            "bundle_directory": str(Path(bundle).resolve()),
            "bundle_manifest_sha256": self.motion.manifest_sha256,
            "clip_sha256": manifest["clip"]["sha256"],
            "bundle_manifest": manifest,
            "task": description,
            "recipe": self.recipe,
            "agent": describe_value(cfg.agent),
        }

    def describe(self, run_dir):
        from training.runtime import write_snapshot

        write_snapshot(Path(run_dir) / "deepmimic-described-input.json", self.snapshot)

    def install(self, cfg, profile, run_dir):
        from training.runtime import describe_value
        from .state import install_reference
        
        run_dir = Path(run_dir)
        from training.runtime import verify_snapshot

        verify_snapshot(run_dir / "deepmimic-described-input.json", self.snapshot)
        body = self.motion.manifest["body"]
        if (profile.model_kind != "xl330" or any(
            body[key] != profile.execution["body"][key] for key in ("id", "version", "source_hash")
        )):
            raise ValueError("DeepMimic source and target body identities differ")
        install_reference(cfg, self.snapshot["bundle_directory"], run_dir, recipe=self.recipe)
        write_recipe(cfg, profile, run_dir)
        metadata = {
            **self.snapshot,
            "recipe": json.loads((run_dir / "deepmimic-recipe.json").read_bytes()),
            "target_execution_fingerprint": profile.execution["execution_fingerprint"],
            "target_execution_profile": profile.execution,
            "target_backend": profile.backend,
            "agent": describe_value(cfg.agent),
        }
        (run_dir / "training-metadata.json").write_text(
            json.dumps(metadata, indent=2, allow_nan=False), encoding="utf-8",
        )
