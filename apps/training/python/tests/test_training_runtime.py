import json
from dataclasses import dataclass
from enum import Enum
from functools import partial
from pathlib import Path
import tempfile
import unittest

import numpy as np

from training.runtime import describe_value, task_config, verify_snapshot, write_snapshot


class Mode(Enum):
    TRAIN = "train"


@dataclass
class Settings:
    seed: int
    mode: Mode


class TrainingRuntimeTests(unittest.TestCase):
    def test_description_records_effective_values_and_callable_identity(self):
        value = describe_value({"settings": Settings(7, Mode.TRAIN), "path": Path("logs/run"),
            "array": np.array([1, 2]), "slice": slice(0, 14), "function": partial(pow, 2),
            "nonfinite": float("inf")})
        self.assertEqual(value["settings"]["seed"], 7)
        self.assertEqual(value["settings"]["mode"], "train")
        self.assertEqual(value["path"], "logs/run")
        self.assertEqual(value["array"], [1, 2])
        self.assertEqual(value["slice"], {"slice": [0, 14, None]})
        self.assertEqual(value["function"], {"function": "builtins.pow", "args": [2], "kwargs": {}})
        self.assertEqual(value["nonfinite"], "inf")
        json.dumps(value, allow_nan=False)
        with self.assertRaisesRegex(ValueError, "unsupported"):
            describe_value(object())

    def test_snapshot_rejects_changed_inputs_instead_of_rewriting_them(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "snapshot.json"
            write_snapshot(path, {"seed": 0, "hash": "a" * 64})
            original = path.read_bytes()
            verify_snapshot(path, {"seed": 0, "hash": "a" * 64})
            with self.assertRaisesRegex(ValueError, "between describe and run"):
                verify_snapshot(path, {"seed": 1, "hash": "a" * 64})
            self.assertEqual(path.read_bytes(), original)

    def test_upstream_task_overrides_keep_environment_and_agent_seeds_aligned(self):
        task = "Mjlab-Velocity-Flat-MicroDuck"
        cfg = task_config(task, [], num_envs=3, seed=17, iterations=5)
        self.assertEqual(cfg.env.scene.num_envs, 3)
        self.assertEqual((cfg.env.seed, cfg.agent.seed), (17, 17))
        self.assertEqual((cfg.agent.max_iterations, cfg.agent.save_interval), (5, 5))
        for options in ({"num_envs": 0}, {"seed": -1}, {"iterations": 0}):
            with self.subTest(options=options), self.assertRaises(ValueError):
                task_config(task, [], **options)
