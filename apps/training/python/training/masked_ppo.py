"""Pinned PPO with warmup transitions excluded from targets and minibatches.

The return recursion follows RSL-RL's PPO (BSD-3-Clause); its update is reused
unchanged. Only validity-aware storage, GAE and normalizer admission differ.
"""

import torch
from rsl_rl.algorithms import PPO
from rsl_rl.storage import RolloutStorage


class MaskedRolloutStorage(RolloutStorage):
    def __init__(self, original):
        super().__init__("rl", original.num_envs, original.num_transitions_per_env,
                         original.observations[0], original.actions_shape, original.device)
        self.valid = torch.zeros((self.num_transitions_per_env, self.num_envs), dtype=torch.bool, device=self.device)

    def clear(self):
        super().clear()
        self.valid.zero_()

    def mini_batch_generator(self, num_mini_batches, num_epochs=8):
        admitted = self.valid.flatten().nonzero(as_tuple=False).squeeze(-1)
        if len(admitted) < num_mini_batches:
            raise ValueError("PPO minibatch count exceeds valid reference transitions")
        flattened = {
            "observations": self.observations.flatten(0, 1),
            "actions": self.actions.flatten(0, 1), "values": self.values.flatten(0, 1),
            "advantages": self.advantages.flatten(0, 1), "returns": self.returns.flatten(0, 1),
            "old_actions_log_prob": self.actions_log_prob.flatten(0, 1),
        }
        distributions = tuple(value.flatten(0, 1) for value in self.distribution_params)
        for _ in range(num_epochs):
            order = admitted[torch.randperm(len(admitted), device=self.device)]
            for indices in torch.tensor_split(order, num_mini_batches):
                yield RolloutStorage.Batch(
                    **{name: value[indices] for name, value in flattened.items()},
                    old_distribution_params=tuple(value[indices] for value in distributions),
                )


class MaskedPPO(PPO):
    def __init__(self, actor, critic, storage, **kwargs):
        super().__init__(actor, critic, MaskedRolloutStorage(storage), **kwargs)
        if actor.is_recurrent or critic.is_recurrent or self.rnd or self.symmetry or self.is_multi_gpu:
            raise ValueError("Masked PPO requires feedforward PPO on one device")
        self.next_valid = torch.ones(self.storage.num_envs, dtype=torch.bool, device=self.device)

    def _mask(self, extras, name):
        value = extras.get(name)
        if not isinstance(value, torch.Tensor) or value.dtype != torch.bool or value.shape != (self.storage.num_envs,):
            raise ValueError(f"{name} must be a boolean [num_envs] tensor")
        return value.to(device=self.device)

    def process_env_step(self, obs, rewards, dones, extras):
        valid = self._mask(extras, "training_valid")
        self.next_valid = self._mask(extras, "training_next_valid").clone()
        admitted = valid & self.next_valid & ~dones.bool()
        if admitted.any():
            self.actor.update_normalization(obs[admitted])
            self.critic.update_normalization(obs[admitted])
        self.storage.valid[self.storage.step] = valid
        self.transition.rewards = rewards.clone()
        self.transition.dones = dones
        if "time_outs" in extras:
            self.transition.rewards += self.gamma * torch.squeeze(
                self.transition.values * extras["time_outs"].to(self.device).unsqueeze(1), 1)
        self.storage.add_transition(self.transition)
        self.transition.clear()
        self.actor.reset(dones)
        self.critic.reset(dones)

    def compute_returns(self, obs):
        storage = self.storage
        last_values = self.critic(obs).detach()
        advantage = torch.zeros_like(last_values)
        for step in reversed(range(storage.num_transitions_per_env)):
            valid = storage.valid[step, :, None]
            next_valid = self.next_valid[:, None] if step == storage.num_transitions_per_env - 1 else storage.valid[step + 1, :, None]
            next_values = last_values if step == storage.num_transitions_per_env - 1 else storage.values[step + 1]
            continuation = next_valid & ~storage.dones[step].bool()
            bootstrapped = torch.where(continuation, next_values, 0.0)
            values = torch.where(valid, storage.values[step], 0.0)
            delta = torch.where(valid, storage.rewards[step], 0.0) + self.gamma * bootstrapped - values
            advantage = torch.where(valid, delta + self.gamma * self.lam * torch.where(continuation, advantage, 0.0), 0.0)
            storage.returns[step] = advantage + values
        storage.advantages = storage.returns - torch.where(storage.valid[:, :, None], storage.values, 0.0)
        if not self.normalize_advantage_per_mini_batch:
            admitted = storage.advantages[storage.valid]
            if len(admitted):
                mean, std = admitted.mean(), admitted.std(unbiased=False)
                storage.advantages = torch.where(storage.valid[:, :, None], (storage.advantages - mean) / (std + 1e-8), 0.0)

    def update(self):
        count = int(self.storage.valid.sum())
        total = self.storage.num_envs * self.storage.num_transitions_per_env
        if count == 0:
            self.storage.clear()
            return {"value": 0.0, "surrogate": 0.0, "entropy": 0.0,
                    "valid_transitions": 0, "warmup_transitions": total}
        requested = self.num_mini_batches
        self.num_mini_batches = min(requested, count)
        try:
            losses = super().update()
        finally:
            self.num_mini_batches = requested
        return {**losses, "valid_transitions": count, "warmup_transitions": total - count}
