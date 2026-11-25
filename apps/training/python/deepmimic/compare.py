"""Measure time-aligned imitation of a recorded teacher in a native rollout."""

import argparse
import json
from pathlib import Path

import numpy as np
from scipy.ndimage import gaussian_filter1d
from scipy.spatial.transform import Rotation

from .motion import ReferenceMotion, rotation


STANDARD = {
    "leg_rmse_rad": 0.16,
    "foot_rms_m": 0.018,
    "root_angle_rad": 0.16,
    "root_height_m": 0.020,
    "forward_speed_error_m_s": 0.075,
    "teacher_moving_speed_m_s": 0.035,
    "student_moving_speed_m_s": 0.025,
    "minimum_forward_m": 0.20,
    "minimum_teacher_forward_fraction": 0.50,
    "minimum_upright_fraction": 0.90,
    "minimum_moving_match_fraction": 0.50,
}


def heading_frame(quaternions):
    facing = rotation(quaternions[0]).apply([1.0, 0.0, 0.0])
    return Rotation.from_rotvec([0.0, 0.0, -np.arctan2(facing[1], facing[0])])


def relative_feet(poses, roots, quaternions):
    return np.stack([rotation(quaternions).inv().apply(poses[:, foot] - roots)
                     for foot in range(2)], axis=1)


def similarities(teacher, student, step_s, leg_slots, standard):
    teacher_root, student_root = teacher["root_pos"], student["root_pos"]
    teacher_quat, student_quat = teacher["root_quat_wxyz"], student["root_quat_wxyz"]
    teacher_heading = heading_frame(teacher_quat)
    student_heading = heading_frame(student_quat)
    teacher_x = teacher_heading.apply(teacher_root - teacher_root[0])[:, 0]
    student_x = student_heading.apply(student_root - student_root[0])[:, 0]
    teacher_v = np.gradient(gaussian_filter1d(teacher_x, 2), step_s)
    student_v = np.gradient(gaussian_filter1d(student_x, 2), step_s)

    joint_delta = student["output_qpos"][:, leg_slots] - teacher["output_qpos"][:, leg_slots]
    joint_delta = np.arctan2(np.sin(joint_delta), np.cos(joint_delta))
    feet_delta = relative_feet(student["foot_pos_world"], student_root, student_quat) - relative_feet(
        teacher["foot_pos_world"], teacher_root, teacher_quat)
    angle = ((student_heading * rotation(student_quat)) *
             (teacher_heading * rotation(teacher_quat)).inv()).magnitude()
    errors = {
        "leg_rmse_rad": np.sqrt(np.mean(joint_delta**2, axis=1)),
        "foot_rms_m": np.sqrt(np.mean(np.sum(feet_delta**2, axis=2), axis=1)),
        "root_angle_rad": angle,
        "root_height_m": np.abs(student_root[:, 2] - teacher_root[:, 2]),
        "forward_speed_error_m_s": np.abs(student_v - teacher_v),
    }
    up = rotation(student_quat).apply([0.0, 0.0, 1.0])[:, 2]
    upright = (up >= np.cos(np.deg2rad(45))) & (student_root[:, 2] >= 0.055)
    pose_match = upright & np.logical_and.reduce([errors[key] <= standard[key] for key in errors])
    moving = teacher_v >= standard["teacher_moving_speed_m_s"]
    moving_match = pose_match & moving & (student_v >= standard["student_moving_speed_m_s"])
    return teacher_x, student_x, moving, moving_match, pose_match, upright, errors


def score(teacher, student, step_s, leg_slots, standard=STANDARD):
    teacher_x, student_x, moving, moving_match, pose_match, upright, errors = similarities(
        teacher, student, step_s, leg_slots, standard)
    if not moving.any() or teacher_x[-1] <= 0:
        raise ValueError("teacher clip has no forward walking interval")
    required_distance = max(standard["minimum_forward_m"],
                            float(teacher_x[-1]) * standard["minimum_teacher_forward_fraction"])
    moving_fraction = float(moving_match.sum() / moving.sum())
    upright_fraction = float(upright.mean())
    gates = {
        "forward": bool(student_x[-1] >= required_distance),
        "upright": upright_fraction >= standard["minimum_upright_fraction"],
        "moving_match": moving_fraction >= standard["minimum_moving_match_fraction"],
    }
    return {
        "passed": all(gates.values()),
        "gates": gates,
        "teacher_forward_m": float(teacher_x[-1]),
        "student_forward_m": float(student_x[-1]),
        "required_forward_m": required_distance,
        "teacher_moving_frames": int(moving.sum()),
        "matched_moving_frames": int(moving_match.sum()),
        "moving_match_fraction": moving_fraction,
        "pose_match_fraction": float(pose_match.mean()),
        "upright_fraction": upright_fraction,
        "moving_median_errors": {key: float(np.median(values[moving])) for key, values in errors.items()},
    }


def compare(bundle, preview):
    motion = ReferenceMotion(bundle)
    count = len(motion.time)
    step_s = motion.manifest["timing"]["control_s"]
    preview_report = json.loads((preview / "report.json").read_bytes())
    layout = json.loads((preview / "layout.json").read_bytes())
    with (preview / "samples.jsonl").open(encoding="utf-8") as stream:
        rows = [json.loads(line) for line in stream]
    if len(rows) < count or preview_report["policySteps"] != len(rows):
        raise ValueError(f"comparison needs {count} consecutive policy steps")
    rows = rows[:count]
    if not np.allclose([row["seconds"] for row in rows], step_s * np.arange(1, count + 1), atol=1e-7, rtol=0):
        raise ValueError("teacher and preview control periods differ")
    joint_indices = [layout["jointNames"].index(name) for name in motion.joint_names]
    feet = []
    for foot in motion.manifest["feet"]:
        index = layout["bodyNames"].index(foot["body"])
        poses = np.asarray([row["bodyPoses"][index] for row in rows])
        feet.append(poses[:, :3] + rotation(poses[:, 3:]).apply(foot["local_position"]))
    student = {
        "output_qpos": (np.asarray([row["motorPositions"] for row in rows]) +
                        np.asarray([row["passivePositions"] for row in rows]))[:, joint_indices],
        "root_pos": np.asarray([row["trunkPosition"] for row in rows]),
        "root_quat_wxyz": np.asarray([row["trunkQuaternion"] for row in rows]),
        "foot_pos_world": np.stack(feet, axis=1),
    }
    teacher = {key: motion.arrays[key].numpy() for key in student}
    preview_command = preview_report["command"]
    command = np.r_[preview_command["twist"], preview_command["head"], 0, 0, preview_command["body"], 0]
    if not np.allclose(motion.arrays["teacher_command"].numpy(), command, atol=1e-6, rtol=0):
        raise ValueError("recorded command differs from preview command")
    if not all(np.isfinite(values).all() for values in student.values()):
        raise ValueError("preview contains non-finite pose data")
    leg_slots = [index for index, name in enumerate(motion.joint_names) if name.startswith(
        ("left_hip", "right_hip", "left_knee", "right_knee", "left_ankle", "right_ankle"))]
    result = score(teacher, student, step_s, leg_slots)
    result.update({
        "teacher_manifest_sha256": motion.manifest_sha256,
        "teacher_execution_fingerprint": motion.manifest["source"]["execution_profile"]["execution_fingerprint"],
        "student_execution_fingerprint": preview_report["executionFingerprint"],
        "run_id": preview_report["runId"],
        "checkpoint_id": preview_report["checkpointId"],
        "policy_sha256": preview_report["policySha256"],
        "duration_s": motion.duration_s,
        "compared_frames": count,
        "standard": STANDARD,
    })
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bundle", type=Path)
    parser.add_argument("preview", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    try:
        output = json.dumps(compare(args.bundle, args.preview), indent=2, allow_nan=False)
        if args.output:
            args.output.write_text(output + "\n", encoding="utf-8")
    except (OSError, KeyError, ValueError, TypeError) as error:
        parser.error(str(error))
    print(output)


if __name__ == "__main__":
    main()
