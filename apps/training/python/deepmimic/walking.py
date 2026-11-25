"""Teacher-initialized DeepMimic for preserving and improving forward motion."""

from copy import deepcopy
import hashlib
import json
from pathlib import Path


class WalkingPlan:
    def __init__(self, path, motion):
        document = json.loads(path.read_bytes())
        self.teacher = (path.parent / document["teacher_onnx"]).read_bytes()
        self.teacher_sha256 = hashlib.sha256(self.teacher).hexdigest()
        if self.teacher_sha256 != motion.manifest["source"]["onnx_sha256"]:
            raise ValueError("initializing ONNX must be the teacher recorded in the action bundle")
        self.learning_rate = float(document.get("learning_rate", 1e-5))
        self.exploration_std = float(document.get("exploration_std", 0.08))
        self.forward_weight = float(document.get("forward_weight", 0.8))
        self.hold_reviews = document.get("hold_reviews", False)
        if not (0 < self.learning_rate <= 0.001 and 0 < self.exploration_std <= 1
                and 0 < self.forward_weight <= 5 and isinstance(self.hold_reviews, bool)):
            raise ValueError("invalid DeepMimic walking plan")
        self.observations = motion.arrays["teacher_observation"].numpy()
        self.actions = motion.arrays["teacher_action"].numpy()

    def configure(self, cfg, binding):
        # The describe snapshot records the registered task. Save the actual recipe
        # separately after applying the experiment settings.
        binding.recipe = deepcopy(binding.recipe)
        binding.recipe["walking_plan"] = {
            "teacher_onnx_sha256": self.teacher_sha256,
            "learning_rate": self.learning_rate,
            "exploration_std": self.exploration_std,
            "forward_weight": self.forward_weight,
        }
        binding.recipe["reference"]["forward_velocity"] = {
            "weight": self.forward_weight, "scale": 0.14, "unit": "m/s",
        }
        cfg.env.rewards["track_linear_velocity"].weight = self.forward_weight
        cfg.agent.actor.distribution_cfg = {
            **cfg.agent.actor.distribution_cfg, "init_std": self.exploration_std, "std_type": "log",
        }
        cfg.agent.algorithm.learning_rate = self.learning_rate
        cfg.agent.algorithm.schedule = "fixed"
        cfg.agent.algorithm.entropy_coef = 0.001

    def initialize(self, actor, *, from_teacher):
        import torch

        normalizer = actor.obs_normalizer
        # The input scaling was learned by the recorded teacher. Keep it fixed
        # through PPO even as the reference-state reset mixture changes.
        normalizer.until = 0
        if not from_teacher:
            return None
        import numpy as np
        import onnx
        import onnxruntime as ort

        model = onnx.load_model_from_string(self.teacher)
        nodes = list(model.graph.node)
        if [node.op_type for node in nodes] != ["Sub", "Div", "Gemm", "Elu", "Gemm", "Elu", "Gemm", "Elu", "Gemm"]:
            raise ValueError("unsupported teacher network for walking initialization")
        weights = {item.name: onnx.numpy_helper.to_array(item).copy() for item in model.graph.initializer}
        mean, divide = (weights[node.input[1]] for node in nodes[:2])
        layers = [layer for layer in actor.mlp if isinstance(layer, torch.nn.Linear)]
        parameters = [weights[name] for node in nodes if node.op_type == "Gemm" for name in node.input[1:]]
        targets = [value for layer in layers for value in (layer.weight, layer.bias)]
        with torch.no_grad():
            for target, parameter in zip(targets, parameters, strict=True):
                target.copy_(torch.as_tensor(parameter, device=target.device))
            normalizer._mean.copy_(torch.as_tensor(mean, device=normalizer._mean.device))
            normalizer._std.copy_(torch.as_tensor(divide, device=normalizer._std.device) - normalizer.eps)
            normalizer._var.copy_(normalizer._std.square())
            # Deployment evaluates the exported graph on CPU. CUDA training may
            # use lower-precision matrix math and is not the transfer oracle.
            obs = torch.from_numpy(self.observations)
            transferred = deepcopy(actor.mlp).cpu()(deepcopy(normalizer).cpu()(obs)).numpy()
        options = ort.SessionOptions()
        options.intra_op_num_threads = 1
        session = ort.InferenceSession(self.teacher, sess_options=options, providers=["CPUExecutionProvider"])
        name = session.get_inputs()[0].name
        expected = np.concatenate([session.run(None, {name: row[None, :]})[0] for row in self.observations])
        transfer_error = float(np.max(np.abs(transferred - expected)))
        if not np.allclose(transferred, expected, atol=1e-5, rtol=1e-5):
            raise ValueError(f"transferred teacher actor does not reproduce the source graph: {transfer_error:g}")
        if not np.allclose(expected, self.actions, atol=1e-4, rtol=1e-4):
            raise ValueError("teacher ONNX does not reproduce actions from the recorded observations")
        return {
            "teacher_sha256": self.teacher_sha256,
            "validated_observations": len(expected),
            "transfer_max_action_error": transfer_error,
            "recording_max_action_error": float(np.max(np.abs(expected - self.actions))),
        }

    def learn(self, runner, train, run_dir, iterations, *, from_teacher):
        from .review import Reviews, StopAtReview, write_json

        reviews = Reviews(run_dir, self.hold_reviews, iterations)
        transfer = self.initialize(runner.alg.actor, from_teacher=from_teacher)
        if transfer is not None:
            write_json(reviews.directory / "teacher-transfer.json", transfer)
        reviews.attach(runner)
        try:
            if transfer is not None:
                reviews.checkpoint(runner, 0, 0, "teacher_initialization")
            return train()
        except StopAtReview:
            if runner.logger.writer is not None:
                runner.logger.stop_logging_writer()
