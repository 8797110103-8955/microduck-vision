"""Convert a strict Rust recording export into a verified DeepMimic motion bundle."""

import argparse
import base64
import hashlib
import json
from datetime import datetime, timezone
from pathlib import Path

import mujoco
import numpy as np
from .motion import CYCLIC_THRESHOLDS, cyclic_alignment, rotation

from .schema import SCHEMA_VERSION, array_shapes, validate_array_layout, validate_manifest, valid_bundle_path


PROCESSING = {
    "velocity_method": "central-difference-one-sided-endpoints",
    "angular_velocity_method": "quaternion-log-world",
    "state_timebase": "simulation.time",
    "teacher_semantics": "pre-step-policy-for-transition-to-state",
    "teacher_command_encoding": "constant",
    "tolerances": {"root_position_m": 3e-5, "root_quaternion_rad": 3e-4, "foot_position_m": 3e-5},
    "cyclic_thresholds": CYCLIC_THRESHOLDS,
}




def derivatives(position, time):
    return np.gradient(position, time, axis=0, edge_order=1)


def angular_velocity(quaternions, time):
    poses = rotation(quaternions)
    result = np.empty((len(time), 3), dtype=np.float64)
    result[0] = (poses[1] * poses[0].inv()).as_rotvec() / (time[1] - time[0])
    result[-1] = (poses[-1] * poses[-2].inv()).as_rotvec() / (time[-1] - time[-2])
    result[1:-1] = (poses[2:] * poses[:-2].inv()).as_rotvec() / (time[2:] - time[:-2])[:, None]
    return result


def progress(stage, frames):
    print(json.dumps({"stage": stage, "processed_frames": frames}), flush=True)


def source_model(context):
    export = context["simulation_export"]
    profile = export["profile"]
    expected = {asset["path"]: asset["sha256"] for asset in profile["simulation"]["source_assets"]}
    assets = {}
    for asset in export["assets"]:
        name = asset["path"]
        if not valid_bundle_path(name) or name in assets:
            raise ValueError(f"Invalid or duplicate source resource {name}")
        data = base64.b64decode(asset["base64"], validate=True)
        if hashlib.sha256(data).hexdigest() != expected.get(name):
            raise ValueError(f"Source resource hash mismatch {name}")
        assets[name] = data
    if assets.keys() != expected.keys():
        raise ValueError("Incomplete source model resources")
    scene = profile["simulation"]["scenario"]["scene_xml"]
    return mujoco.MjModel.from_xml_string(assets[scene].decode(), assets=assets)


def extract(context, frames):
    profile = context["simulation_export"]["profile"]
    order = profile["joint_order"]
    slots = [i for i, name in enumerate(order) if name != "mouth"]
    names = [order[i] for i in slots]
    if len(names) != 14 or len(set(names)) != 14 or len(frames) < 3:
        raise ValueError("Expected at least three frames and 14 driven joints")
    snapshots = [frame["status"]["simulation"] for frame in frames]
    time = np.array([snapshot["time"] for snapshot in snapshots], dtype=np.float64)
    if not np.isfinite(time).all() or not np.allclose(np.diff(time), 0.02, atol=1e-8, rtol=0):
        raise ValueError("Discontinuous source simulation time")
    time -= time[0]
    arrays = {"time_s": time}
    for target, field in (("motor_qpos", "motorPositions"), ("passive_qpos", "passivePositions")):
        arrays[target] = np.array([snapshot[field] for snapshot in snapshots], dtype=np.float64)[:, slots]
    arrays["output_qpos"] = arrays["motor_qpos"] + arrays["passive_qpos"]
    for kind in ("motor", "passive", "output"):
        arrays[f"{kind}_qvel"] = derivatives(arrays[f"{kind}_qpos"], time)
    arrays["root_pos"] = np.array([snapshot["trunkPosition"] for snapshot in snapshots], dtype=np.float64)
    quat = np.array([snapshot["trunkQuaternion"] for snapshot in snapshots], dtype=np.float64)
    if not np.isfinite(quat).all() or not np.allclose(np.linalg.norm(quat, axis=1), 1, atol=1e-6, rtol=0):
        raise ValueError("Invalid root quaternion")
    for index in range(1, len(quat)):
        if quat[index] @ quat[index - 1] < 0:
            quat[index] *= -1
    arrays["root_quat_wxyz"] = quat
    arrays["root_lin_vel_world"] = derivatives(arrays["root_pos"], time)
    arrays["root_ang_vel_world"] = angular_velocity(quat, time)
    for target, field in (("teacher_observation", "observation"), ("teacher_action", "action")):
        arrays[target] = np.array([frame["policy"][field] for frame in frames], dtype=np.float64)
    arrays["teacher_command"] = arrays["teacher_observation"][:, 48:].copy()
    available = all(frame["policy"]["appliedTargets"] is not None for frame in frames)
    if available:
        arrays["teacher_applied_targets"] = np.array([frame["policy"]["appliedTargets"] for frame in frames], dtype=np.float64)[:, slots]
    for name, array in arrays.items():
        if not np.isfinite(array).all():
            raise ValueError(f"Non-finite source array {name}")
    return names, arrays, available


def verify_kinematics(context, frames, names, arrays):
    model = source_model(context)
    data = mujoco.MjData(model)
    root = model.joint("trunk_base_freejoint")
    root_qpos = root.qposadr[0]
    joints = [model.joint(name).qposadr[0] for name in names]
    passive = [model.joint(f"passive_{name}_backlash").qposadr[0] for name in names]
    sites = [model.site(name) for name in ("left_foot", "right_foot")]
    feet = [{"site": site.name, "body": model.body(site.bodyid[0]).name,
             "local_position": site.pos.tolist()} for site in sites]
    body_ids = [context["body_names"].index(foot["body"]) for foot in feet]
    foot_positions = []
    maximum = {"root_position_m": 0.0, "root_quaternion_rad": 0.0, "foot_position_m": 0.0}
    for index, frame in enumerate(frames):
        data.qpos[:] = model.qpos0
        data.qpos[root_qpos:root_qpos + 3] = arrays["root_pos"][index]
        data.qpos[root_qpos + 3:root_qpos + 7] = arrays["root_quat_wxyz"][index]
        data.qpos[joints] = arrays["motor_qpos"][index]
        data.qpos[passive] = arrays["passive_qpos"][index]
        mujoco.mj_forward(model, data)
        root_body = model.body("trunk_base").id
        maximum["root_position_m"] = max(maximum["root_position_m"], float(np.linalg.norm(data.xpos[root_body] - arrays["root_pos"][index])))
        angle = (rotation(data.xquat[root_body]) * rotation(arrays["root_quat_wxyz"][index]).inv()).magnitude()
        maximum["root_quaternion_rad"] = max(maximum["root_quaternion_rad"], float(angle))
        positions = []
        body_poses = frame["status"]["simulation"]["frame"]["bodies"]
        for foot, body_index, site in zip(feet, body_ids, sites, strict=True):
            pose = body_poses[body_index]
            if len(pose) != 7 or not np.isfinite(pose).all() or abs(np.linalg.norm(pose[3:]) - 1) > 1e-6:
                raise ValueError(f"Invalid recorded body pose at frame {index}")
            point = rotation(pose[3:]).apply(foot["local_position"]) + pose[:3]
            maximum["foot_position_m"] = max(maximum["foot_position_m"], float(np.linalg.norm(data.site_xpos[site.id] - point)))
            positions.append(point)
        foot_positions.append(positions)
    for key, value in maximum.items():
        if value > PROCESSING["tolerances"][key]:
            raise ValueError(f"Source FK validation failed for {key}: {value}")
    arrays["foot_pos_world"] = np.asarray(foot_positions, dtype=np.float64)
    return feet, maximum



def convert(source, parameters, output):
    progress("reading", 0)
    context = json.loads((source / "source-context.json").read_bytes())
    frames = [json.loads(line) for line in (source / "source-frames.jsonl").read_text(encoding="utf-8").splitlines()]
    if len(frames) != context["frames"]:
        raise ValueError("Source frame count differs from context")
    progress("validating", len(frames))
    names, arrays, targets_available = extract(context, frames)
    feet, fk_errors = verify_kinematics(context, frames, names, arrays)
    progress("converting", len(frames))
    playback = parameters.get("playback", "finite")
    if playback not in ("finite", "cyclic"):
        raise ValueError("Unsupported playback mode")
    seam = cyclic_alignment(arrays) if playback == "cyclic" else {}
    profile = context["simulation_export"]["profile"]
    runtime = context["metadata"]["runtime"]
    warnings = []
    if not targets_available:
        warnings.append("Applied target history unavailable; reference warmup requires a different interval")
    quality = {"excluded_frames": sum(context["excluded_reasons"].values()),
               "target_range_exceeded_frames": sum("body_range_exceeded" in frame["policy"]["saturationReasons"] for frame in frames),
               "contact_available": False, "applied_targets_available": targets_available,
               "fk_max_error": fk_errors, "seam_errors": seam, "warnings": warnings}
    processing = {**PROCESSING, "teacher_command_encoding": runtime["policy_contract"]["command_encoding"] or "constant"}
    robot_xml = profile["simulation"]["scenario"]["robot_xml"]
    robot_hash = next(asset["sha256"] for asset in profile["simulation"]["source_assets"] if asset["path"] == robot_xml)
    output.mkdir(parents=True, exist_ok=True)
    (output / "clips").mkdir(exist_ok=True)
    clip = output / "clips/clip-000.npz"
    np.savez_compressed(clip, **arrays)
    manifest = {"training_method": "deepmimic", "schema_version": SCHEMA_VERSION,
                "bundle_id": parameters["bundle_id"], "name": parameters["name"],
                "created_at": datetime.now(timezone.utc).isoformat(), "converter_version": "1",
                "source": {key: context[key] for key in ("recording_id", "first_frame", "last_frame", "first_simulation_s", "last_simulation_s", "mcap_sha256")},
                "body": {**profile["body"], "model_kind": profile["model_kind"], "robot_xml": robot_xml, "robot_sha256": robot_hash},
                "timing": {"control_s": 0.02, "physics_s": 0.005, "units": {"angle": "rad", "length": "m", "time": "s"},
                           "world_basis": "right-handed-z-up", "quaternion_order": "wxyz"},
                "joint_names": names, "feet": feet,
                "clip": {"file": "clips/clip-000.npz", "sha256": hashlib.sha256(clip.read_bytes()).hexdigest(),
                         "frames": len(frames), "duration_s": float(arrays["time_s"][-1]), "playback": playback,
                         "arrays": {name: {"dtype": "float64", "shape": list(shape)} for name, shape in array_shapes(len(frames), targets_available).items()}},
                "processing": processing, "quality": quality}
    manifest["source"].update(segment_id=context["interval"]["segment_id"], interval_id=context["interval"]["id"],
                              onnx_sha256=runtime["policy_sha256"], execution_profile=profile)
    validate_manifest(manifest)
    validate_array_layout(manifest, arrays)
    progress("saving", len(frames))
    for name, value in (("manifest.json", manifest), ("quality-report.json", {**quality, "raw_contacts": [f["status"]["simulation"]["contacts"] for f in frames]})):
        (output / name).write_text(json.dumps(value, allow_nan=False, indent=2) + "\n", encoding="utf-8")
    return manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--parameters", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    try:
        result = convert(args.source, json.loads(args.parameters.read_bytes()), args.output)
    except (ValueError, KeyError, OSError, TypeError) as error:
        print(json.dumps({"error": str(error)}), flush=True)
        raise SystemExit(1) from error
    print(json.dumps({"bundle_id": result["bundle_id"], "frames": result["clip"]["frames"]}), flush=True)


if __name__ == "__main__":
    main()
