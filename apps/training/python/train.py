"""Compose the selected method with the upstream training runtime."""

import argparse
import json
import os
import sys
from contextlib import redirect_stdout
from pathlib import Path

from training.runtime import task_config, describe_task, install_profile
from training.artifacts import write_artifact


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("operation", choices=("describe", "run", "evaluate-teacher"))
    parser.add_argument("task")
    parser.add_argument("--joint-order", type=json.loads)
    parser.add_argument("--profile", type=Path)
    parser.add_argument("--run-dir", type=Path)
    parser.add_argument("--preview-bodies", type=json.loads)
    parser.add_argument("--num-envs", type=int)
    parser.add_argument("--seed", type=int)
    parser.add_argument("--iterations", type=int)
    parser.add_argument("--deepmimic-bundle", type=Path)
    parser.add_argument("--deepmimic-manifest-sha256")
    parser.add_argument("--deepmimic-baseline", action="store_true")
    parser.add_argument("--deepmimic-plan", type=Path)
    parser.add_argument("--terrain")
    parser.add_argument("--domain-randomization", choices=("on", "off"))
    parser.add_argument("--teacher-spec", type=Path)
    parser.add_argument("--evaluation-steps", type=int, default=100)
    parser.add_argument("--evaluation-seed", type=int, default=0)
    parser.add_argument("--initialization", type=Path)
    args, trailing = parser.parse_known_args(argv)
    if trailing and trailing[0] == "--":
        trailing = trailing[1:]
    if any(flag.split("=", 1)[0] == "--hf-jobs" for flag in trailing):
        parser.error("desktop training runs locally")
    if args.task.startswith("Microduck-DeepMimic-") and any(
        flag.split("=", 1)[0] == "--init-checkpoint" for flag in trailing
    ):
        parser.error("DeepMimic initializes actor and critic randomly; init-checkpoint is not supported")
    from mjlab_microduck.checkpoints import consume_init_flag

    trailing = consume_init_flag(trailing)
    deepmimic = args.task.startswith("Microduck-DeepMimic-")
    if deepmimic and (args.deepmimic_bundle is None or args.operation == "evaluate-teacher"):
        parser.error("DeepMimic training requires a published --deepmimic-bundle")
    if not deepmimic and (args.deepmimic_bundle or args.deepmimic_manifest_sha256 or args.deepmimic_baseline):
        parser.error("DeepMimic options require the dedicated DeepMimic task")
    domain_randomization = (args.domain_randomization or ("off" if deepmimic else "on")) == "on"
    if deepmimic and domain_randomization:
        parser.error("the first DeepMimic recipe requires domain randomization off")

    def configure(joints):
        if deepmimic:
            from deepmimic.task import register_tasks

            register_tasks()
        cfg = task_config(args.task, trailing, num_envs=args.num_envs, terrain=args.terrain,
                          seed=args.seed, iterations=args.iterations)
        description = describe_task(args.task, cfg.env, joints)
        binding = None
        if deepmimic:
            from deepmimic.task import TrainingBinding, validate_config

            validate_config(cfg)

            binding = TrainingBinding(cfg, args.deepmimic_bundle, description,
                                      expected_hash=args.deepmimic_manifest_sha256,
                                      baseline=args.deepmimic_baseline)
        return cfg, description, binding

    if args.operation == "describe":
        if not isinstance(args.joint_order, list) or not args.joint_order:
            parser.error("describe requires --joint-order")
        with redirect_stdout(sys.stderr):
            cfg, result, binding = configure(args.joint_order)
            if binding is not None:
                binding.describe(args.run_dir or Path.cwd())
        print(json.dumps(result, allow_nan=False))
        return
    if args.profile is None or args.run_dir is None:
        parser.error("run requires --profile and --run-dir")
    from training.body import load_profile

    profile = load_profile(args.profile)
    cfg, _, binding = configure(profile.execution["joint_order"])
    walking = None
    if args.deepmimic_plan is not None:
        if binding is None or args.deepmimic_baseline or args.operation != "run":
            parser.error("a walking plan requires a DeepMimic imitation run")
        from deepmimic.walking import WalkingPlan

        walking = WalkingPlan(args.deepmimic_plan, binding.motion)
        walking.configure(cfg, binding)
    if args.operation == "run" and cfg.gpu_ids and os.environ.get("CUDA_VISIBLE_DEVICES") != "":
        import torch

        if not torch.cuda.is_available():
            raise RuntimeError(
                "MJLab requires a CUDA-enabled PyTorch environment for this run. "
                "Check the NVIDIA driver and launch Microduck with the CUDA environment."
            )
    if binding is not None:
        cfg.agent.save_interval = min(50, cfg.agent.save_interval)
    run_dir = args.run_dir.resolve()
    run_dir.mkdir(parents=True, exist_ok=True)
    install_profile(
        args.task, cfg, profile, run_dir,
        domain_randomization=domain_randomization,
    )
    if args.operation == "run":
        from training.rounds import install as install_round

        install_round(args.task, cfg, profile, run_dir, args.initialization,
                      domain_randomization=domain_randomization, walking=walking,
                      reference=None if binding is None else {
                          "method": binding.method, "manifest_sha256": binding.motion.manifest_sha256,
                          "recipe": {key: value for key, value in binding.recipe.items() if key != "iterations"},
                      })
    if binding is not None:
        cfg.agent.algorithm.class_name = "training.masked_ppo:MaskedPPO"
        binding.install(cfg, profile, run_dir)
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
    if not (run_dir / "reviews" / "stopped.json").is_file():
        write_artifact(run_dir, profile, domain_randomization=domain_randomization)



if __name__ == "__main__":
    main()
