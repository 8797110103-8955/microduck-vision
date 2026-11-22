"""Train an observation-to-action student from an exported MicroDuck policy dataset."""

import argparse
import hashlib
import json
from pathlib import Path

import torch


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--epochs", type=int, default=100)
    args = parser.parse_args()
    manifest = json.loads((args.dataset / "dataset.json").read_text(encoding="utf-8"))
    if manifest["kind"] != "policy":
        raise ValueError("The student trainer requires a policy dataset")
    rows = [json.loads(line) for line in (args.dataset / "samples.jsonl").read_text(encoding="utf-8").splitlines()]
    if len(rows) < 2:
        raise ValueError("At least two valid samples are required")
    contract = manifest["policy_contract"]
    observations = torch.tensor([row["observation"] for row in rows], dtype=torch.float32)
    actions = torch.tensor([row["teacher_action"] for row in rows], dtype=torch.float32)
    if observations.shape[1] != contract["obs_len"] or actions.shape[1] != contract["action_len"]:
        raise ValueError("Dataset dimensions disagree with the policy contract")
    if not torch.isfinite(observations).all() or not torch.isfinite(actions).all():
        raise ValueError("Dataset contains non-finite values")
    if contract.get("recurrent"):
        raise ValueError("This trainer supports feed-forward policies")
    torch.manual_seed(0)
    student = torch.nn.Sequential(
        torch.nn.Linear(observations.shape[1], 256), torch.nn.ELU(),
        torch.nn.Linear(256, 256), torch.nn.ELU(),
        torch.nn.Linear(256, actions.shape[1]),
    )
    optimizer = torch.optim.Adam(student.parameters(), lr=3e-4)
    for epoch in range(args.epochs):
        total = 0.0
        order = torch.randperm(len(rows))
        for batch in order.split(256):
            loss = torch.nn.functional.mse_loss(student(observations[batch]), actions[batch])
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()
            total += loss.item() * len(batch)
        print(json.dumps({"step": epoch + 1, "total_steps": args.epochs, "loss": total / len(rows)}), flush=True)
    args.output.mkdir(parents=True, exist_ok=True)
    torch.save({"state_dict": student.state_dict(), "policy_contract": contract}, args.output / "student.pt")
    student.eval()
    torch.onnx.export(
        student, observations[:1], args.output / "student.onnx",
        input_names=["observation"], output_names=["action"],
        dynamic_axes={"observation": {0: "batch"}, "action": {0: "batch"}},
        opset_version=17, dynamo=False,
    )
    metadata = {
        "architecture": [observations.shape[1], 256, 256, actions.shape[1]],
        "activation": "ELU", "policy_contract": contract,
        "source_dataset": str(args.dataset.resolve()),
        "source_sha256": hashlib.sha256((args.dataset / "samples.jsonl").read_bytes()).hexdigest(),
        "samples": len(rows), "epochs": args.epochs,
        "input": "recorded_policy_input", "output": "raw_teacher_action",
    }
    (args.output / "student.json").write_text(json.dumps(metadata, indent=2), encoding="utf-8")


if __name__ == "__main__":
    main()
