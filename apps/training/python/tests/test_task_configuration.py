import json
import unittest

from training.tasks import registered_tasks, task_configuration


class TaskConfigurationTests(unittest.TestCase):
    def test_all_registered_tasks_expose_serializable_training_play_and_rl_defaults(self):
        tasks = registered_tasks()
        self.assertIn("Mjlab-Velocity-Flat-MicroDuck", tasks)
        self.assertIn("Microduck-DeepMimic-Velstand-Flat-v1", tasks)
        for task_id in tasks:
            with self.subTest(task=task_id):
                result = task_configuration(task_id)
                self.assertEqual(result["taskId"], task_id)
                self.assertTrue(result["runner"])
                self.assertGreater(result["maxIterations"], 0)
                for mode in ("training", "play"):
                    self.assertGreater(result[mode]["controlHz"], 0)
                    config = json.loads(result[mode]["json"])
                    self.assertIn("terminations", config)
                    self.assertIn("rewards", config)
                    self.assertEqual(config["scene"]["num_envs"], result[mode]["numEnvs"])
                self.assertEqual(json.loads(result["rlJson"])["algorithm"]["class_name"], result["algorithm"])

    def test_unknown_task_is_rejected(self):
        registered_tasks()
        with self.assertRaises(KeyError):
            task_configuration("missing-task")


if __name__ == "__main__":
    unittest.main()
