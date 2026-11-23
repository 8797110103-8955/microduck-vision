"""Exercise the frozen MJLab environment with actual Rust profile exports."""

import json
import os
from pathlib import Path
import sys
import tempfile
import unittest


PYTHON = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(PYTHON / "body_adapters"), str(PYTHON)]

from simulation_profile import load_profile
from train import install_profile, task_config


@unittest.skipUnless(os.environ.get("MICRODUCK_PROFILE_FIXTURES"), "supply Rust task exports")
class MjlabProfiles(unittest.TestCase):
    @property
    def fixtures(self):
        return Path(os.environ["MICRODUCK_PROFILE_FIXTURES"])

    def test_all_three_bam_models_preserve_the_task_and_apply_effective_parameters_once(self):
        for case in (
            "xl330-walk", "hd1910-walk", "hd1910-compatible-walk",
            "xl330-allcollisions", "xl330-backlash",
        ):
            with self.subTest(case=case), tempfile.TemporaryDirectory() as directory:
                profile = load_profile(self.fixtures / f"{case}.json")
                task_id = profile.task["task_id"]
                cfg = task_config(task_id, [], num_envs=1)
                terrain = cfg.env.scene.terrain
                sensors = cfg.env.scene.sensors
                events = tuple(cfg.env.events)
                install_profile(task_id, cfg, profile, Path(directory), domain_randomization=True)
                self.assertIs(cfg.env.scene.terrain, terrain)
                self.assertIs(cfg.env.scene.sensors, sensors)
                self.assertEqual(tuple(cfg.env.events), events)
                robot = cfg.env.scene.entities["robot"].build()
                model = robot.spec.compile()
                self.assertEqual(len(robot.actuators[0].target_names), 14)
                self.assertEqual(model.nu, 14)
                self.assertNotIn("floor", robot.geom_names)
                if case.endswith("allcollisions"):
                    self.assertTrue(any(name.endswith("_servo_collision") for name in robot.geom_names))
                actuator = robot.actuators[0]
                motor = profile.simulation["motor"]
                response = profile.simulation["response"]
                self.assertEqual(actuator.cfg.kp_fw, motor["kp"] * response["position_gain_scale"])
                self.assertEqual(actuator._bam_model.actuator.error_gain, motor["error_gain"])
                self.assertEqual(actuator._bam_model.actuator.max_pwm, motor["max_pwm"])
                self.assertEqual(actuator._bam_model.kt.value, motor["kt"])
                self.assertEqual(actuator._bam_model.friction_base.value, motor["friction_base"])
                self.assertEqual(
                    [actuator.cfg.delay_min_lag, actuator.cfg.delay_max_lag],
                    response["delay_steps"],
                )
                self.assertEqual(len(list(Path(directory).glob("*.xml"))), 0)

    def test_actor_observation_delay_is_independent_of_the_hd_target_delay(self):
        profile = load_profile(self.fixtures / "hd1910-walk.json")
        task_id = profile.task["task_id"]
        cfg = task_config(task_id, [], num_envs=1)
        with tempfile.TemporaryDirectory() as directory:
            install_profile(task_id, cfg, profile, Path(directory), domain_randomization=False)
        actuator = cfg.env.scene.entities["robot"].articulation.actuators[0]
        self.assertEqual((actuator.delay_min_lag, actuator.delay_max_lag), (0, 0))
        velocity = cfg.env.observations["actor"].terms["joint_vel"]
        self.assertEqual((velocity.delay_min_lag, velocity.delay_max_lag), (1, 1))
        self.assertEqual(actuator.vin_drop_gain_range, (0.1, 0.1))

    def test_changed_task_or_tampered_resource_is_rejected(self):
        path = self.fixtures / "xl330-walk.json"
        profile = load_profile(path)
        task_id = profile.task["task_id"]
        cfg = task_config(task_id, [], num_envs=2)
        with tempfile.TemporaryDirectory() as directory:
            path_out = Path(directory)
            with self.assertRaisesRegex(ValueError, "task description or source changed"):
                install_profile(task_id, cfg, profile, path_out, domain_randomization=True)
            export = json.loads(path.read_bytes())
            export["assets"][0]["base64"] = "AAAA"
            changed = path_out / "modified.json"
            changed.write_text(json.dumps(export), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "resource digest mismatch"):
                load_profile(changed)


if __name__ == "__main__":
    unittest.main()
