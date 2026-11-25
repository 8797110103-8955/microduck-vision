"""Exercise import boundaries in fresh interpreters, including vendor hooks."""

import ast
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


PYTHON = Path(__file__).resolve().parents[1]


class TrainingBoundaryTests(unittest.TestCase):
    def run_isolated(self, code):
        with tempfile.TemporaryDirectory() as directory:
            result = subprocess.run([sys.executable, "-B", "-c", code], cwd=directory,
                env={**os.environ, "PYTHONPATH": str(PYTHON)}, capture_output=True, text=True, timeout=90)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_shared_modules_do_not_import_methods_or_cli(self):
        for path in (PYTHON / "training").glob("*.py"):
            # Discovery composes all application tasks; execution helpers stay independent.
            if path.name == "tasks.py":
                continue
            for node in ast.walk(ast.parse(path.read_text(encoding="utf-8"))):
                imports = [alias.name for alias in node.names] if isinstance(node, ast.Import) else (
                    [node.module or ""] if isinstance(node, ast.ImportFrom) else [])
                for name in imports:
                    self.assertFalse(name == "train" or name.startswith("deepmimic"), (path, name))
                if path.name == "body.py" and isinstance(node, (ast.If, ast.Match)):
                    expression = node.test if isinstance(node, ast.If) else node.subject
                    self.assertNotIn("model_kind", ast.unparse(expression))

    def test_converter_does_not_load_mjlab_registry_or_initialize_cuda(self):
        self.run_isolated('''
import importlib.abc
import sys
attempted = []
class Guard(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if fullname == "mjlab" or fullname.startswith("mjlab_microduck"):
            attempted.append(fullname)
            raise RuntimeError("conversion must not load training tasks")
sys.meta_path.insert(0, Guard())
import torch
def forbidden(*args, **kwargs):
    attempted.append("cuda.init")
    raise RuntimeError("conversion must not initialize CUDA")
torch.cuda.init = torch.cuda._lazy_init = forbidden
import deepmimic.convert
assert not attempted, attempted
assert "mjlab_microduck.tasks" not in sys.modules
assert not torch.cuda.is_initialized()
''')

    def test_official_task_loads_without_deepmimic(self):
        self.run_isolated('''
import importlib.abc
import sys
attempted = []
class Guard(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if fullname.startswith("deepmimic"):
            attempted.append(fullname)
            raise RuntimeError("ordinary training must not load DeepMimic")
sys.meta_path.insert(0, Guard())
from training.runtime import task_config
cfg = task_config("Mjlab-Velocity-Flat-MicroDuck", [], num_envs=1, iterations=1)
assert cfg.agent.algorithm.class_name != "training.masked_ppo:MaskedPPO"
assert not attempted, attempted
assert not any(name.startswith("deepmimic") for name in sys.modules)
''')
