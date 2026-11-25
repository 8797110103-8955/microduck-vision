"""Recorded-motion experiment defaults, saved with every DeepMimic run."""

from copy import deepcopy


def default_recipe():
    # Error scales are standard deviations in exp(-mean(error**2) / scale**2).
    return {
        "version": "microduck-deepmimic-recipe-v1",
        "control_s": 0.02,
        "physics_s": 0.005,
        "twist": [0.15, 0.0, 0.0],
        "head_command": [0.0] * 4,
        "body_command": [0.0] * 6,
        "num_envs": 64,
        "iterations": 50,
        "seed": 0,
        "domain_randomization": False,
        "initialization": "random_actor_and_critic",
        "behavior_cloning": False,
        "random_initial_episode_age": False,
        "reference": {
            "joint_position": {"weight": 1.0, "scale": 0.35, "unit": "rad"},
            "joint_velocity": {"weight": 0.1, "scale": 3.0, "unit": "rad/s"},
            "root_height": {"weight": 0.3, "scale": 0.035, "unit": "m"},
            "root_orientation": {"weight": 0.3, "scale": 0.25, "unit": "rad"},
            "feet": {"weight": 0.5, "scale": 0.025, "unit": "m"},
            "contact": {"weight": 0.0, "available": False},
        },
        "task_rewards": {
            "track_linear_velocity": 0.25,
            "track_angular_velocity": 0.25,
            "action_rate_l2": -0.01,
        },
        "reset": {
            "nominal_root_height_m": 0.125,
            "reference_probability": 0.8,
            "sampler_seed_offset": 1_000_003,
            "warmup_steps": 3,
            "previous_action_source": "teacher_action",
            "actuator_delay_source": "teacher_applied_targets",
            "velocity_history_source": "actual_warmup_observations",
            "warmup_transitions": "excluded_from_ppo",
        },
        "termination": {
            "finite_reference_complete": True,
            "minimum_root_height_m": 0.055,
            "maximum_tilt_rad": 0.9599310885968813,  # 55 degrees.
            "maximum_episode_s": 20.0,
            "nonfinite_state": True,
        },
    }


def resolved_recipe(cfg, recipe=None):
    """Capture overrides and the final manager lists, rather than only defaults."""
    recipe = deepcopy(default_recipe() if recipe is None else recipe)
    recipe.update(
        num_envs=cfg.env.scene.num_envs,
        iterations=cfg.agent.max_iterations,
        seed=cfg.agent.seed,
        control_s=cfg.env.decimation * cfg.env.sim.mujoco.timestep,
        physics_s=cfg.env.sim.mujoco.timestep,
        manager_terms={
            "rewards": {name: term.weight for name, term in cfg.env.rewards.items() if term is not None},
            "events": {name: term.mode for name, term in cfg.env.events.items() if term is not None},
            "terminations": list(cfg.env.terminations),
            "curriculum": list(cfg.env.curriculum),
            "sensors": [sensor.name for sensor in cfg.env.scene.sensors],
        },
        actor_observation_delay={
            name: [term.delay_min_lag, term.delay_max_lag]
            for name, term in cfg.env.observations["actor"].terms.items() if term is not None
        },
        algorithm=cfg.agent.algorithm.class_name,
        resume=cfg.agent.resume,
    )
    return recipe
