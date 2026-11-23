import hashlib
import json
from pathlib import Path
import subprocess
import sys
from tempfile import TemporaryDirectory
import unittest

import onnxruntime as ort
import numpy as np


class TeacherArtifactTest(unittest.TestCase):
    def test_export_identifies_model_and_recorded_policy_inputs(self):
        contract = {
            "id": "teacher-fixture", "body_id": "official-xl330-m6", "body_version": 1,
            "obs_len": 61, "action_len": 14, "control_hz": 50, "recurrent": False,
        }
        segments = {
            "first": {"backend_contract": "recorded-execution", "policy_contract": contract},
            "outside-window": {"backend_contract": "other-execution"},
        }
        samples = [
            {"segment_id": "first", "observation": [value] * 61, "teacher_action": [value] * 14}
            for value in (0.125, -0.25)
        ]
        script = Path(__file__).resolve().parents[1] / "teacher_train.py"
        with TemporaryDirectory() as work:
            root = Path(work)
            dataset = root / "dataset"
            dataset.mkdir()
            (dataset / "dataset.json").write_text(json.dumps({
                "kind": "policy", "policy_contract": contract, "segments": segments,
            }), encoding="utf-8")
            samples_path = dataset / "samples.jsonl"
            samples_path.write_text("\n".join(map(json.dumps, samples)), encoding="utf-8")
            output = root / "result"
            subprocess.run([
                sys.executable, str(script), "--dataset", str(dataset), "--output", str(output),
                "--epochs", "1",
            ], check=True, capture_output=True, text=True)

            artifact = json.loads((output / "artifact.json").read_text(encoding="utf-8"))
            self.assertEqual(artifact["kind"], "teacher")
            self.assertEqual(artifact["policy_contract"], contract)
            self.assertEqual(artifact["source_segments"], {"first": segments["first"]})
            self.assertEqual(artifact["samples_sha256"], hashlib.sha256(samples_path.read_bytes()).hexdigest())
            model = output / artifact["onnx_path"]
            self.assertEqual(artifact["onnx_sha256"], hashlib.sha256(model.read_bytes()).hexdigest())

            session = ort.InferenceSession(str(model), providers=["CPUExecutionProvider"])
            for graph, manifest in ((session.get_inputs(), artifact["inputs"]), (session.get_outputs(), artifact["outputs"])):
                self.assertEqual([item.name for item in graph], [item["name"] for item in manifest])
                self.assertEqual([item.shape for item in graph], [item["dimensions"] for item in manifest])
            observations = np.asarray([sample["observation"] for sample in samples], dtype=np.float32)
            (actions,) = session.run(None, {"observation": observations})
            self.assertEqual(actions.shape, (2, 14))
            self.assertTrue(np.isfinite(actions).all())
