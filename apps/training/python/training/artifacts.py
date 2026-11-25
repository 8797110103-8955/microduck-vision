"""Validate and publish the desktop contract around the official ONNX export."""

import hashlib
import json


def write_artifact(run_dir, profile, *, domain_randomization, model_path=None, destination=None):
    import onnx

    models = [model_path] if model_path is not None else sorted((run_dir / "logs" / "rsl_rl").rglob("*.onnx"))
    if len(models) != 1:
        raise ValueError("training must produce one ONNX artifact for the desktop run")
    path = models[0]
    output = run_dir if destination is None else destination
    model = onnx.load(path, load_external_data=False)
    backlash = profile.simulation["response"]["encoder_through_backlash"]
    velocity_delay = next(term["configuration"]["delay_max_lag"]
                          for term in profile.task["policy_layout"]["observations"] if term["name"] == "joint_vel")
    if backlash:
        from mjlab.utils.spec import get_non_free_joints

        # The pinned velocity exporter includes passive backlash/wheel joints. The
        # desktop policy consumes the 14 motor/output encoder channels instead.
        metadata = {item.key: item.value for item in model.metadata_props}
        names = metadata.get("joint_names", "").split(",")
        home = metadata.get("default_joint_pos", "").split(",")
        motors = profile.task["policy_layout"]["actions"][0]["joints"]
        spec = profile.robot_spec()
        allowed = {joint.name for joint in get_non_free_joints(spec)}
        if (len(names) != len(set(names)) or len(home) != len(names)
                or not set(motors).issubset(names) or not set(names).issubset(allowed)):
            raise ValueError("Backlash ONNX metadata does not describe the expected physical joints")
        metadata["joint_names"] = ",".join(motors)
        metadata["default_joint_pos"] = ",".join(home[names.index(joint)] for joint in motors)
        metadata["microduck_encoder_through_backlash"] = "true"
        metadata["microduck_joint_velocity_delay_ticks"] = str(velocity_delay)
        metadata["microduck_robot_sha256"] = profile.task["robot_sha256"]
        onnx.helper.set_model_props(model, metadata)
        onnx.save(model, path)

    def tensor(value):
        tensor_type = value.type.tensor_type
        return {
            "name": value.name,
            "element_type": onnx.TensorProto.DataType.Name(tensor_type.elem_type),
            "dimensions": [
                length.dim_value if length.HasField("dim_value") else length.dim_param or None
                for length in tensor_type.shape.dim
            ],
        }

    manifest = {
        "format_version": 1,
        "backend": {
            "kind": profile.backend["kind"], "version": profile.backend["version"],
            "execution_fingerprint": profile.backend["execution_fingerprint"],
            "bam_fit_sha256": profile.backend["bam_fit"]["sha256"],
            "configuration_sha256": hashlib.sha256(json.dumps(profile.backend, sort_keys=True, separators=(",", ":")).encode()).hexdigest(),
        },
        "kind": "rl",
        "onnx_path": path.relative_to(output).as_posix(),
        "onnx_sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
        "task_id": profile.task["task_id"],
        "robot_xml": profile.task["robot_xml"],
        "robot_sha256": profile.task["robot_sha256"],
        "joint_order": profile.execution["joint_order"],
        "home_pose_rad": profile.execution["home_pose_rad"],
        "policy_layout": profile.task["policy_layout"],
        "inputs": [tensor(value) for value in model.graph.input],
        "outputs": [tensor(value) for value in model.graph.output],
        "onnx_metadata": {item.key: item.value for item in model.metadata_props},
        "control_s": profile.execution["periods"]["control_s"],
        "body": profile.execution["body"],
        "model_kind": profile.model_kind,
        "domain_randomization": domain_randomization,
        "execution_fingerprint": profile.execution["execution_fingerprint"],
        "encoder_through_backlash": backlash,
        "joint_velocity_delay_ticks": velocity_delay,
    }
    metadata = run_dir / "training-metadata.json"
    if metadata.is_file():
        manifest["training"] = json.loads(metadata.read_bytes())
        manifest["training_method"] = manifest["training"]["training_method"]
    pending = output / "artifact.json.tmp"
    pending.write_text(json.dumps(manifest, allow_nan=False, indent=2), encoding="utf-8")
    pending.replace(output / "artifact.json")

