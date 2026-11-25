"""Export intermediate policies for headless evaluation and continuation decisions."""

import argparse
import json
import math
from pathlib import Path
import subprocess
import time


def write_json(path, payload):
    pending = path.with_suffix(".tmp")
    pending.write_text(json.dumps(payload, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    pending.replace(path)


class StopAtReview(Exception):
    """The reviewer requested the worker save and finish the current round."""


class Reviews:
    def __init__(self, run_dir, hold, iterations):
        self.run_dir = run_dir
        self.directory = run_dir / "reviews"
        self.directory.mkdir()
        self.run_id = json.loads((run_dir / "round.json").read_bytes())["runId"]
        self.iterations = iterations
        self.hold = hold
        self.milestones = {math.ceil(iterations * percent / 100): percent for percent in (20, 40, 60, 80, 100)}

    def checkpoint(self, runner, percent, completed, stage):
        checkpoint = self.directory / f"review_{percent:03}.pt"
        runner.save(str(checkpoint))
        review_file = self.directory / f"{percent:03}.json"
        payload = {
            "run_id": self.run_id, "checkpoint_id": checkpoint.stem, "stage": stage,
            "progress_percent": percent, "iterations_completed": completed,
            "planned_iterations": self.iterations, "needs_decision": self.hold and percent < 100,
            "review_file": str(review_file),
        }
        write_json(review_file, payload)
        print("MICRODUCK_TRAINING_REVIEW " + json.dumps(payload), flush=True)
        if self.hold and percent < 100:
            decision = self.directory / f"{percent:03}-decision.json"
            deadline = time.monotonic() + 900
            while not decision.is_file() and time.monotonic() < deadline:
                time.sleep(0.25)
            reason = "review timeout"
            if decision.is_file():
                if json.loads(decision.read_bytes())["decision"] == "continue":
                    return
                reason = "reviewer stopped the round"
            write_json(self.directory / "stopped.json", {**payload, "reason": reason})
            raise StopAtReview(reason)

    def attach(self, runner):
        log = runner.logger.log
        offset = runner.current_learning_iteration

        def log_with_review(*args, **kwargs):
            log(*args, **kwargs)
            done = kwargs["it"] - offset + 1
            if done in self.milestones:
                self.checkpoint(runner, self.milestones[done], done, "deepmimic")

        runner.logger.log = log_with_review


def evaluate(review_file, executable, config, bundle, machine):
    from .compare import compare

    review = json.loads(review_file.read_bytes())
    request = review_file.with_name(f"{review_file.stem}-preview-request.json")
    write_json(request, {
        "runId": review["run_id"], "checkpointId": review["checkpoint_id"], "virtualMachineId": machine,
    })
    process = subprocess.run([executable, "--training-preview", config, request],
                             capture_output=True, encoding="utf-8", check=True)
    preview = Path(json.loads(process.stdout)["directory"])
    result = {**review, **compare(bundle, preview), "preview_directory": str(preview)}
    write_json(review_file.with_name(f"{review_file.stem}-score.json"), result)
    return result


def run(executable, config, bundle, request, plan):
    machine = json.loads(request.read_bytes())["virtualMachineId"]
    command = [executable, "--training-run", config, request, plan]
    with subprocess.Popen(command, stdout=subprocess.PIPE, encoding="utf-8") as worker:
        for line in worker.stdout:
            event = json.loads(line)
            if "review_file" in event:
                try:
                    event = evaluate(Path(event["review_file"]), executable, config, bundle, machine)
                except (OSError, KeyError, ValueError, subprocess.CalledProcessError) as error:
                    event["score_error"] = error.stderr.strip() if isinstance(
                        error, subprocess.CalledProcessError) else str(error)
            print(json.dumps(event, allow_nan=False), flush=True)
        return worker.wait()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(required=True, dest="operation")
    score = commands.add_parser("score")
    score.add_argument("review", type=Path)
    score.add_argument("--executable", required=True, type=Path)
    score.add_argument("--config", required=True, type=Path)
    score.add_argument("--bundle", required=True, type=Path)
    score.add_argument("--machine", required=True)
    train = commands.add_parser("run")
    train.add_argument("--executable", required=True, type=Path)
    train.add_argument("--config", required=True, type=Path)
    train.add_argument("--bundle", required=True, type=Path)
    train.add_argument("--request", required=True, type=Path)
    train.add_argument("--plan", required=True, type=Path)
    decide = commands.add_parser("decide")
    decide.add_argument("review", type=Path)
    decide.add_argument("decision", choices=("continue", "stop"))
    args = parser.parse_args()
    try:
        if args.operation == "run":
            raise SystemExit(run(args.executable.resolve(), args.config.resolve(), args.bundle.resolve(),
                                 args.request.resolve(), args.plan.resolve()))
        if args.operation == "score":
            result = evaluate(args.review.resolve(), args.executable.resolve(), args.config.resolve(),
                              args.bundle.resolve(), args.machine)
        else:
            request = json.loads(args.review.read_bytes())
            if not request["needs_decision"]:
                raise ValueError("this review does not require a continuation decision")
            result = {"decision": args.decision, "progress_percent": request["progress_percent"]}
            write_json(args.review.with_name(f"{args.review.stem}-decision.json"), result)
    except subprocess.CalledProcessError as error:
        parser.error(error.stderr.strip() or f"native preview exited with {error.returncode}")
    except (OSError, ValueError, KeyError) as error:
        parser.error(str(error))
    print(json.dumps(result, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
