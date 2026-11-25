"""List the tasks registered by the pinned project and the application."""

import json
import sys
import argparse
from contextlib import redirect_stdout


def registered_tasks():
    import mjlab_microduck.tasks  # noqa: F401 - registers the pinned tasks
    from deepmimic.task import register_tasks
    from mjlab.tasks.registry import list_tasks

    register_tasks()
    return sorted(list_tasks())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--task")
    args = parser.parse_args()
    # Imports may log to stdout; reserve stdout for the Rust JSON protocol.
    with redirect_stdout(sys.stderr):
        tasks = registered_tasks()
        result = task_configuration(args.task) if args.task else tasks
    print(json.dumps(result, allow_nan=False))


def task_configuration(task_id):
    from mjlab.tasks.registry import load_env_cfg, load_rl_cfg, load_runner_cls
    from training.runtime import describe_value

    def environment(cfg):
        return {
            "numEnvs": cfg.scene.num_envs,
            "controlHz": 1.0 / (cfg.sim.mujoco.timestep * cfg.decimation),
            "episodeLengthS": cfg.episode_length_s,
            "terrain": cfg.scene.terrain.terrain_type if cfg.scene.terrain else None,
            "json": json.dumps(describe_value(cfg), indent=2, allow_nan=False),
        }

    agent = load_rl_cfg(task_id)
    runner = load_runner_cls(task_id)
    return {
        "taskId": task_id,
        "runner": f"{runner.__module__}.{runner.__qualname__}" if runner else "mjlab.rl.MjlabOnPolicyRunner",
        "algorithm": agent.algorithm.class_name,
        "maxIterations": agent.max_iterations,
        "training": environment(load_env_cfg(task_id)),
        "play": environment(load_env_cfg(task_id, play=True)),
        "rlJson": json.dumps(describe_value(agent), indent=2, allow_nan=False),
    }


if __name__ == "__main__":
    main()
