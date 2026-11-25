"""Validate the Rust/Python boundary independently of product model rules."""

import copy
import hashlib
import json
from pathlib import Path
import tempfile
import unittest

from training.body import load_profile


class TrainingBodyContractTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory(prefix="body-contract-")
        self.addCleanup(self.directory.cleanup)
        self.path = Path(self.directory.name) / "simulation-profile.json"
        xml = b"<mujoco/>"
        import base64

        digest = hashlib.sha256(xml).hexdigest()
        fit = json.dumps({"model": "m6", "actuator": "xl330"})
        periods = {"physics_s": 0.005, "physics_substeps": 4, "control_s": 0.02}
        root = [0, 0, 0.125, 1, 0, 0, 0]
        self.document = {
            "format_version": 2,
            "units": {"angle": "rad", "angular_velocity": "rad/s", "length": "m", "time": "s", "voltage": "V"},
            "profile": {"format_version": 1, "execution_kind": "simulation", "gain_mode": "fixed_model",
                "physical_gains": None, "execution_fingerprint": "a" * 64, "model_kind": "fixture",
                "joint_order": ["joint"], "home_pose_rad": [0], "periods": periods,
                "simulation": {"root_home_pose": root, "source_assets": [{"path": "robot.xml", "sha256": digest}],
                    "scenario": {"robot_xml": "robot.xml", "scene_xml": None, "task_id": "fixture"}}},
            "task": {"task_id": "fixture", "robot_xml": "robot.xml", "robot_sha256": digest,
                "joint_order": ["joint"], "home_pose_rad": [0], "periods": periods,
                "root_home_pose": root, "requires_backlash": False},
            "assets": [{"path": "robot.xml", "base64": base64.b64encode(xml).decode()}],
            "backend": {"kind": "mjlab-bam", "version": 1, "execution_fingerprint": "a" * 64,
                "encoder_through_backlash": False, "name_servo_collision_geoms": False,
                "actuator": {"vin": 5, "kp_fw": 32, "error_gain": 1, "max_pwm": 1, "max_current": None,
                    "delay_min_lag": 0, "delay_max_lag": 0, "vin_min": 4, "voltage_drop_gain": 0.1},
                "bam_fit": {"file": "bam-fit.json", "content": fit, "sha256": hashlib.sha256(fit.encode()).hexdigest()}},
        }
        (self.path.parent / "bam-fit.json").write_text(fit, encoding="utf-8")

    def load(self, document=None):
        self.path.write_text(json.dumps(self.document if document is None else document), encoding="utf-8")
        return load_profile(self.path)

    def test_model_identity_is_metadata_not_a_python_dispatch_table(self):
        for name in ("xl330", "hd1910", "hd1910-compatible", "new-rust-resolved-model"):
            self.document["profile"]["model_kind"] = name
            self.assertEqual(self.load().model_kind, name)

    def test_invalid_protocol_backend_and_fingerprint_fail(self):
        for route, value in [
            (("format_version",), 1), (("backend", "kind"), "unknown"),
            (("backend", "version"), 2), (("backend", "execution_fingerprint"), "b" * 64),
            (("backend", "encoder_through_backlash"), 0),
            (("backend", "actuator", "max_pwm"), float("nan")),
            (("backend", "actuator", "delay_min_lag"), True),
            (("backend", "actuator", "delay_max_lag"), -1),
            (("task", "home_pose_rad"), [1]),
        ]:
            with self.subTest(route=route):
                changed = copy.deepcopy(self.document)
                parent = changed
                for key in route[:-1]:
                    parent = parent[key]
                parent[route[-1]] = value
                with self.assertRaises(ValueError):
                    self.load(changed)
        changed = copy.deepcopy(self.document)
        del changed["backend"]["actuator"]["kp_fw"]
        with self.assertRaises(ValueError):
            self.load(changed)

    def test_resource_and_fit_integrity_and_paths_fail_closed(self):
        for route, value in [
            (("assets", 0, "base64"), "AAAA"), (("assets", 0, "path"), "../robot.xml"),
            (("backend", "bam_fit", "content"), "{}"),
            (("backend", "bam_fit", "file"), "../bam-fit.json"),
            (("backend", "bam_fit", "file"), "C:/bam-fit.json"),
        ]:
            changed = copy.deepcopy(self.document)
            parent = changed
            for key in route[:-1]:
                parent = parent[key]
            parent[route[-1]] = value
            with self.subTest(route=route), self.assertRaises(ValueError):
                self.load(changed)
        fit_path = self.path.parent / "bam-fit.json"
        fit_path.write_text("{}")
        with self.assertRaisesRegex(ValueError, "file digest"):
            self.load()
        fit_path.unlink()
        with self.assertRaises(ValueError):
            self.load()


if __name__ == "__main__":
    unittest.main()
