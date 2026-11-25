"""Validate the portable DeepMimic manifest before opening motion arrays."""

import math
from pathlib import PurePosixPath


METHOD = "deepmimic"
SCHEMA_VERSION = "microduck-deepmimic-motion-v1"
UNITS = {"angle": "rad", "length": "m", "time": "s"}


def _require(condition, message):
    if not condition:
        raise ValueError(message)


def _number(value):
    return type(value) in (int, float) and math.isfinite(value)


def valid_id(value):
    return (isinstance(value, str) and value not in ("", ".", "..")
            and all(c.isascii() and (c.isalnum() or c in "-_.") for c in value))


def _digest(value):
    return isinstance(value, str) and len(value) == 64 and all(c in "0123456789abcdef" for c in value)


def valid_bundle_path(value):
    return (isinstance(value, str) and bool(value) and "\\" not in value and ":" not in value
            and all(valid_id(part) for part in value.split("/"))
            and PurePosixPath(value).as_posix() == value)


def array_shapes(frames, applied_targets):
    shapes = {"time_s": (frames,)}
    for name in ("motor_qpos", "passive_qpos", "output_qpos", "motor_qvel", "passive_qvel",
                 "output_qvel", "teacher_action"):
        shapes[name] = (frames, 14)
    for name in ("root_pos", "root_lin_vel_world", "root_ang_vel_world"):
        shapes[name] = (frames, 3)
    shapes.update(root_quat_wxyz=(frames, 4), foot_pos_world=(frames, 2, 3),
                  teacher_observation=(frames, 61), teacher_command=(frames, 13))
    if applied_targets:
        shapes["teacher_applied_targets"] = (frames, 14)
    return shapes


def validate_manifest(manifest):
    """Return the original manifest or raise a field-specific ValueError."""
    try:
        _validate_manifest(manifest)
    except (KeyError, TypeError, AttributeError, OverflowError) as error:
        raise ValueError(f"Invalid DeepMimic manifest field: {error}") from error
    return manifest


def _validate_manifest(m):
    _require(m["training_method"] == METHOD and m["schema_version"] == SCHEMA_VERSION,
             "Unsupported DeepMimic motion method or schema version")
    source, body, timing, clip = (m[key] for key in ("source", "body", "timing", "clip"))
    processing, quality = m["processing"], m["quality"]
    _require(all(valid_id(value) for value in (m["bundle_id"], source["recording_id"], source["interval_id"])),
             "Invalid bundle, recording or interval ID")
    _require(all(isinstance(m[key], str) and m[key].strip() for key in ("name", "created_at", "converter_version")),
             "Missing manifest identity")
    _require(all(_digest(value) for value in (source["mcap_sha256"], source["onnx_sha256"],
                                            clip["sha256"], body["robot_sha256"])), "Invalid SHA-256")
    _require(isinstance(source["execution_profile"], dict), "Missing source execution profile")
    for key in ("segment_id", "first_frame", "last_frame"):
        _require(type(source[key]) is int and 0 <= source[key] <= 2**32 - 1, f"Invalid {key}")
    _require(body["id"] == "official" and type(body["version"]) is int and body["version"] > 0
             and bool(body["source_hash"]) and body["model_kind"] == "xl330"
             and body["robot_xml"] == "robot_groundcontact_backlash.xml", "Unsupported source body")
    _require(type(timing["control_s"]) is float and timing["control_s"] == 0.02
             and type(timing["physics_s"]) is float and timing["physics_s"] == 0.005
             and timing["units"] == UNITS and timing["world_basis"] == "right-handed-z-up"
             and timing["quaternion_order"] == "wxyz", "Unsupported timing, coordinates or units")
    names = m["joint_names"]
    _require(isinstance(names, list) and len(names) == 14
             and all(isinstance(name, str) and name and name != "mouth" for name in names)
             and len(set(names)) == 14, "Invalid joint names")
    profile = source["execution_profile"]
    _require([name for name in profile["joint_order"] if name != "mouth"] == names
             and profile["format_version"] == 1 and profile["execution_kind"] == "simulation"
             and profile["model_kind"] == body["model_kind"]
             and profile["body"] == {"id": body["id"], "version": body["version"], "source_hash": body["source_hash"]}
             and profile["periods"]["control_s"] == timing["control_s"]
             and profile["periods"]["physics_s"] == timing["physics_s"]
             and profile["simulation"]["scenario"]["robot_xml"] == body["robot_xml"]
             and profile["simulation"]["response"]["encoder_through_backlash"] is True
             and _digest(profile["execution_fingerprint"])
             and all(isinstance(profile["simulation"][key], dict) for key in ("motor", "nominal_motor", "response", "scenario"))
             and any(asset["path"] == body["robot_xml"] and asset["sha256"] == body["robot_sha256"]
                     for asset in profile["simulation"]["source_assets"]),
             "Source profile disagrees with motion metadata")
    feet = m["feet"]
    _require(isinstance(feet, list) and len(feet) == 2
             and [foot["site"] for foot in feet] == ["left_foot", "right_foot"], "Invalid foot sites")
    for foot in feet:
        _require(isinstance(foot["body"], str) and bool(foot["body"])
                 and len(foot["local_position"]) == 3 and all(_number(v) for v in foot["local_position"]),
                 "Invalid foot body or local position")
    _require(valid_bundle_path(clip["file"]) and clip["file"].endswith(".npz"), "Invalid clip path")
    frames = clip["frames"]
    _require(type(frames) is int and 3 <= frames <= 2**32 - 1
             and clip["playback"] in ("finite", "cyclic")
             and source["last_frame"] - source["first_frame"] + 1 == frames, "Invalid clip frame range")
    first, last, duration = source["first_simulation_s"], source["last_simulation_s"], clip["duration_s"]
    _require(all(_number(v) for v in (first, last, duration)) and first >= 0 and duration > 0
             and abs(last - first - duration) <= 1e-8
             and abs(duration - (frames - 1) * timing["control_s"]) <= 1e-8, "Invalid clip duration")
    for key in ("excluded_frames", "target_range_exceeded_frames"):
        _require(type(quality[key]) is int and 0 <= quality[key] <= 2**32 - 1, f"Invalid {key}")
    _require(quality["contact_available"] is False and type(quality["applied_targets_available"]) is bool,
             "Invalid availability flags")
    _require(isinstance(quality["warnings"], list) and all(isinstance(w, str) for w in quality["warnings"]),
             "Invalid warnings")
    expected = {name: {"dtype": "float64", "shape": list(shape)}
                for name, shape in array_shapes(frames, quality["applied_targets_available"]).items()}
    _require(clip["arrays"] == expected, "Invalid array layout or dtype")
    _require(all(isinstance(layout["shape"], list) and all(type(size) is int for size in layout["shape"])
                 for layout in clip["arrays"].values()), "Invalid array dimensions")
    _require(processing["state_timebase"] == "simulation.time"
             and processing["velocity_method"] == "central-difference-one-sided-endpoints"
             and processing["angular_velocity_method"] == "quaternion-log-world"
             and processing["teacher_semantics"] == "pre-step-policy-for-transition-to-state"
             and processing["teacher_command_encoding"] in ("constant", "phase", "posture_flag"),
             "Unsupported processing semantics")
    for values in (processing["tolerances"], processing["cyclic_thresholds"],
                   quality["fk_max_error"], quality["seam_errors"]):
        _require(isinstance(values, dict) and all(isinstance(k, str) and _number(v) and v >= 0
                                                for k, v in values.items()), "Invalid quality measurements")


def validate_array_layout(manifest, arrays):
    """Check actual NPZ array names and storage layout, without importing NumPy."""
    validate_manifest(manifest)
    _require(set(arrays) == set(manifest["clip"]["arrays"]), "Motion array names differ from manifest")
    for name, declared in manifest["clip"]["arrays"].items():
        array = arrays[name]
        _require(str(array.dtype) == declared["dtype"] and tuple(array.shape) == tuple(declared["shape"]),
                 f"Invalid array layout for {name}")
