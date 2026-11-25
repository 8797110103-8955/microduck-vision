"""Validated, device-resident reference motion with finite and cyclic sampling."""

import hashlib
import io
import json
from pathlib import Path

import numpy as np
import torch
from scipy.spatial.transform import Rotation

from .schema import validate_array_layout, validate_manifest


def quat_slerp(start, end, fraction):
    """Shortest-arc interpolation for unit wxyz quaternions."""
    dot = (start * end).sum(-1, keepdim=True)
    end = torch.where(dot < 0, -end, end)
    dot = dot.abs().clamp(0, 1)
    angle = torch.acos(dot)
    denominator = torch.sin(angle).clamp_min(torch.finfo(start.dtype).eps)
    spherical = (torch.sin((1 - fraction) * angle) * start + torch.sin(fraction * angle) * end) / denominator
    linear = start + fraction * (end - start)
    result = torch.where(dot > 0.9995, linear, spherical)
    return torch.nn.functional.normalize(result, dim=-1)


def yaw(quaternion):
    w, x, y, z = quaternion.unbind(-1)
    return torch.atan2(2 * (w * z + x * y), 1 - 2 * (y.square() + z.square()))


def rotate_heading(vectors, angle):
    x, y, z = vectors.unbind(-1)
    x, y, z, angle = torch.broadcast_tensors(x, y, z, angle)
    cosine, sine = torch.cos(angle), torch.sin(angle)
    return torch.stack((cosine * x - sine * y, sine * x + cosine * y, z), dim=-1)


class ReferenceMotion:
    """Load and validate once before constructing the training environment.

    sample(times) returns a dictionary of tensors with the same leading batch shape
    as times, plus a boolean completed tensor. Source teacher arrays retain their
    original encoding and are available for history initialization and diagnostics.
    """

    def __init__(self, directory, *, device="cpu", dtype=torch.float32):
        if dtype not in (torch.float32, torch.float64):
            raise ValueError("Reference motion requires float32 or float64 tensors")
        directory = Path(directory).resolve(strict=True)
        manifest_bytes = (directory / "manifest.json").read_bytes()
        self.manifest = validate_manifest(json.loads(manifest_bytes))
        self.manifest_sha256 = hashlib.sha256(manifest_bytes).hexdigest()
        clip = self.manifest["clip"]
        path = (directory / clip["file"]).resolve(strict=True)
        if not path.is_relative_to(directory) or not path.is_file():
            raise ValueError("Motion file escapes bundle directory")
        # Verify and decode the same bytes, even when another process changes a file.
        content = path.read_bytes()
        if hashlib.sha256(content).hexdigest() != clip["sha256"]:
            raise ValueError("Motion NPZ hash mismatch")
        with np.load(io.BytesIO(content), allow_pickle=False) as archive:
            if len(archive.files) != len(set(archive.files)):
                raise ValueError("Duplicate motion array names")
            arrays = {name: archive[name] for name in archive.files}
        validate_array_layout(self.manifest, arrays)
        for name, array in arrays.items():
            if not np.isfinite(array).all():
                raise ValueError(f"Non-finite motion array {name}")
        time = arrays["time_s"]
        if (abs(time[0]) > 1e-8 or not np.all(np.diff(time) > 0)
                or not np.allclose(np.diff(time), self.manifest["timing"]["control_s"], atol=1e-8, rtol=0)
                or abs(time[-1] - clip["duration_s"]) > 1e-8):
            raise ValueError("Invalid reference timeline")
        quat = arrays["root_quat_wxyz"]
        if not np.allclose(np.linalg.norm(quat, axis=-1), 1, atol=1e-6, rtol=0):
            raise ValueError("Non-unit reference quaternion")
        for suffix in ("qpos", "qvel"):
            if not np.allclose(arrays[f"motor_{suffix}"] + arrays[f"passive_{suffix}"],
                               arrays[f"output_{suffix}"], atol=1e-9, rtol=1e-9):
                raise ValueError(f"Output {suffix} differs from motor plus passive state")
        if clip["playback"] == "cyclic":
            # Recompute seam checks. A changed manifest cannot grant cyclic playback.
            errors = cyclic_alignment({name: array.copy() for name, array in arrays.items()})
            if set(self.manifest["quality"]["seam_errors"]) != set(errors):
                raise ValueError("Missing cyclic seam measurements")
            for name, value in errors.items():
                reported = self.manifest["quality"]["seam_errors"][name]
                if abs(value - reported) > 1e-7:
                    raise ValueError(f"Cyclic seam measurement mismatch: {name}")
        self.device = torch.device(device)
        self.dtype = dtype
        self.duration_s = float(time[-1])
        self.cyclic = clip["playback"] == "cyclic"
        self.time = torch.as_tensor(time.copy(), device=self.device, dtype=torch.float64)
        self.arrays = {name: torch.as_tensor(array.copy(), device=self.device, dtype=dtype)
                       for name, array in arrays.items() if name != "time_s"}
        self.joint_names = tuple(self.manifest["joint_names"])
        self._cycle_yaw = yaw(self.arrays["root_quat_wxyz"][-1]) - yaw(self.arrays["root_quat_wxyz"][0])
        self._cycle_yaw = torch.atan2(torch.sin(self._cycle_yaw), torch.cos(self._cycle_yaw))
        self._cycle_translation = self.arrays["root_pos"][-1] - rotate_heading(self.arrays["root_pos"][0], self._cycle_yaw)
        self._cycle_translation = self._cycle_translation.clone()
        self._cycle_translation[2] = 0

    def sample(self, times):
        requested = torch.as_tensor(times, dtype=torch.float64, device=self.device)
        if not torch.isfinite(requested).all() or (requested < 0).any():
            raise ValueError("Reference sample times must be finite and non-negative")
        cycles = torch.floor(requested / self.duration_s)
        times = requested - cycles * self.duration_s if self.cyclic else requested.clamp_max(self.duration_s)
        upper = torch.searchsorted(self.time, times.contiguous(), right=True).clamp(1, len(self.time) - 1)
        lower = upper - 1
        fraction = ((times - self.time[lower]) / (self.time[upper] - self.time[lower])).to(self.dtype)
        result = {}
        for name, values in self.arrays.items():
            weight = fraction.reshape(*fraction.shape, *([1] * (values.ndim - 1)))
            if name == "root_quat_wxyz":
                result[name] = quat_slerp(values[lower], values[upper], weight)
            else:
                result[name] = torch.lerp(values[lower], values[upper], weight)
        if self.cyclic:
            self._accumulate_cycle(result, cycles.to(self.dtype))
        result["time_s"] = times.to(self.dtype)
        result["completed"] = torch.zeros_like(requested, dtype=torch.bool) if self.cyclic else requested >= self.duration_s
        return result

    def _accumulate_cycle(self, result, cycles):
        angle = cycles * self._cycle_yaw
        # Sum rotated translations over k cycles, including the straight-line limit.
        amplitude = cycles * torch.sinc(angle / (2 * torch.pi)) / torch.sinc(self._cycle_yaw / (2 * torch.pi))
        middle_angle = (cycles - 1) * self._cycle_yaw / 2
        translation = rotate_heading(self._cycle_translation, middle_angle) * amplitude[..., None]
        for name in ("root_pos", "foot_pos_world"):
            extra = result[name].ndim - cycles.ndim - 1
            heading = angle.reshape(angle.shape + (1,) * extra)
            offset = translation.reshape(*translation.shape[:-1], *([1] * extra), 3)
            result[name] = rotate_heading(result[name], heading) + offset
        for name in ("root_lin_vel_world", "root_ang_vel_world"):
            result[name] = rotate_heading(result[name], angle)
        zeros = torch.zeros_like(angle)
        turn = torch.stack((torch.cos(angle / 2), zeros, zeros, torch.sin(angle / 2)), dim=-1)
        from mjlab.utils.lab_api.math import quat_mul

        result["root_quat_wxyz"] = quat_mul(turn, result["root_quat_wxyz"])



CYCLIC_THRESHOLDS = {"pose_rad": 0.05, "velocity_rad_s": 0.5, "foot_position_m": 0.01,
                     "root_height_m": 0.01, "root_orientation_rad": 0.1}


def rotation(quaternion):
    return Rotation.from_quat(np.asarray(quaternion)[..., [1, 2, 3, 0]])


def wxyz(value):
    return value.as_quat()[..., [3, 0, 1, 2]]


def cyclic_alignment(arrays):
    root = arrays["root_pos"]
    quat = arrays["root_quat_wxyz"]
    forward = rotation(quat[0]).apply([1, 0, 0])
    heading = np.arctan2(forward[1], forward[0])
    transform = Rotation.from_rotvec([0, 0, -heading])
    origin = np.array([*root[0, :2], 0])
    arrays["root_pos"] = transform.apply(root - origin)
    arrays["root_quat_wxyz"] = wxyz(transform * rotation(quat))
    feet = arrays["foot_pos_world"]
    arrays["foot_pos_world"] = transform.apply((feet - origin).reshape(-1, 3)).reshape(feet.shape)
    for name in ("root_lin_vel_world", "root_ang_vel_world"):
        arrays[name] = transform.apply(arrays[name])
    orientation = rotation(arrays["root_quat_wxyz"])
    local_feet = np.stack([orientation[i].inv().apply(arrays["foot_pos_world"][i] - arrays["root_pos"][i]) for i in range(len(root))])
    errors = {"pose_rad": float(np.max(np.abs(arrays["output_qpos"][-1] - arrays["output_qpos"][0]))),
              "velocity_rad_s": float(np.max(np.abs(arrays["output_qvel"][-1] - arrays["output_qvel"][0]))),
              "foot_position_m": float(np.max(np.linalg.norm(local_feet[-1] - local_feet[0], axis=1))),
              "root_height_m": float(abs(arrays["root_pos"][-1, 2] - arrays["root_pos"][0, 2]))}
    up = orientation.apply(np.tile([0, 0, 1], (len(root), 1)))
    errors["root_orientation_rad"] = float(np.arccos(np.clip(up[0] @ up[-1], -1, 1)))
    failed = {key: value for key, value in errors.items() if value > CYCLIC_THRESHOLDS[key]}
    if failed:
        raise ValueError(f"Cyclic seam exceeds thresholds: {failed}")
    return errors

