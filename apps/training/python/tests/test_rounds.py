from copy import deepcopy
import unittest

from training.rounds import validate_parent


class RoundCompatibilityTests(unittest.TestCase):
    def setUp(self):
        self.parent = {"task_id": "task", "layout": {"observations": ["angle"], "action_len": 14},
                       "joint_order": ["leg"], "control_s": 0.02, "execution_fingerprint": "body-a",
                       "environment": {"friction": 1}, "agent": {"learning_rate": 0.001}, "reference": None}

    def test_resume_rejects_changed_dynamics_and_adaptation_accepts_them(self):
        current = deepcopy(self.parent)
        current["execution_fingerprint"] = "body-b"
        current["environment"]["friction"] = 0.5
        with self.assertRaisesRegex(ValueError, "choose adaptation"):
            validate_parent(current, self.parent, "resume")
        validate_parent(current, self.parent, "adapt")
        validate_parent(self.parent, self.parent, "resume")

    def test_adaptation_still_requires_the_same_observation_and_action_meanings(self):
        for key, value in (("task_id", "another"), ("layout", {}), ("joint_order", ["arm"]), ("control_s", 0.01)):
            with self.subTest(key=key):
                current = {**self.parent, key: value}
                with self.assertRaisesRegex(ValueError, key):
                    validate_parent(current, self.parent, "adapt")

    def test_resume_rejects_a_different_reference_motion(self):
        with self.assertRaises(ValueError):
            validate_parent({**self.parent, "reference": "new-motion"}, self.parent, "resume")


class CheckpointLoadingTests(unittest.TestCase):
    def test_adaptation_loads_only_actor_and_can_reset_its_normalizer(self):
        import tempfile
        from pathlib import Path
        from types import SimpleNamespace
        import torch
        from training.rounds import digest, load_parent

        with tempfile.TemporaryDirectory() as directory:
            checkpoint = Path(directory) / "model.pt"
            torch.save({"actor_state_dict": {"mlp.weight": torch.tensor([3.0]), "obs_normalizer.mean": torch.tensor([7.0])},
                        "critic_state_dict": {"weight": torch.tensor([4.0])}, "optimizer_state_dict": {"old": True}}, checkpoint)
            loaded = []
            actor = SimpleNamespace(state_dict=lambda: {"mlp.weight": torch.tensor([0.0]), "obs_normalizer.mean": torch.tensor([0.0])})
            runner = SimpleNamespace(device="cpu", alg=SimpleNamespace(actor=actor, load=lambda state, cfg, strict: loaded.append((state, cfg, strict))),
                                     load=lambda path: loaded.append(path))
            parent = {"mode": "adapt", "checkpoint_path": str(checkpoint), "checkpoint_sha256": digest(checkpoint)}
            load_parent(runner, parent)
            self.assertEqual(loaded[-1][1], {"actor": True})
            self.assertEqual(loaded[-1][0]["actor_state_dict"]["obs_normalizer.mean"].item(), 7)
            load_parent(runner, {**parent, "reset_normalization": True})
            self.assertEqual(loaded[-1][0]["actor_state_dict"]["obs_normalizer.mean"].item(), 0)
            self.assertEqual(loaded[-1][0]["actor_state_dict"]["mlp.weight"].item(), 3)
            load_parent(runner, {**parent, "mode": "resume"})
            self.assertEqual(loaded[-1], str(checkpoint))
            checkpoint.write_bytes(b"changed")
            with self.assertRaisesRegex(ValueError, "digest changed"):
                load_parent(runner, parent)
