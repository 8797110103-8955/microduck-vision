"""Compare checkpoint rollouts recorded by the desktop native simulator."""

import argparse
import json
import math
from pathlib import Path


def load_report(path):
    path = path / "report.json" if path.is_dir() else path
    return json.loads(path.read_text(encoding="utf-8"))


def measurements(report):
    forward, lateral = report["displacementM"]
    values = {
        "policySteps": report["policySteps"],
        "forwardM": forward,
        "lateralM": lateral,
        "minHeightM": report["minHeightM"],
        "maxTiltDeg": report["maxTiltDeg"],
        "uprightFraction": report["uprightFraction"],
    }
    if not all(math.isfinite(value) for value in values.values()):
        raise ValueError("preview contains non-finite measurements")
    return values


def compare(before, after):
    for key in ("executionFingerprint", "command", "steps", "seconds"):
        if before[key] != after[key]:
            raise ValueError(f"preview reports disagree on {key}")
    old, new = measurements(before), measurements(after)
    return {
        "before": {"runId": before["runId"], "checkpointId": before["checkpointId"], **old},
        "after": {"runId": after["runId"], "checkpointId": after["checkpointId"], **new},
        "delta": {key: new[key] - old[key] for key in old},
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("before", type=Path, help="preview directory or report.json")
    parser.add_argument("after", type=Path, help="preview directory or report.json")
    args = parser.parse_args()
    try:
        result = compare(load_report(args.before), load_report(args.after))
    except (OSError, KeyError, TypeError, ValueError) as error:
        parser.error(str(error))
    print(json.dumps(result, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
