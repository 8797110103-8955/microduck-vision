"""Exercise the production MJLab actuator on the same complete VFS scene as Rust."""

from dataclasses import replace
import json
import os
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest

import mujoco
import numpy as np
import torch


PYTHON = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(PYTHON / "body_adapters"), str(PYTHON)]
import mjlab  # initialize the pinned task registry before importing its adapter
from mjlab_adapter import adapter
from simulation_profile import load_profile
from train import describe_task, task_config


FIXTURE_ENV = "MICRODUCK_NATIVE_RESPONSE_FIXTURES"
TOLERANCES = {
    "trunkPosition": 3e-5,
    "trunkQuaternion": 3e-5,
    "motorPositions": 3e-5,
    "passivePositions": 3e-5,
    "samplePositions": 3e-5,
    "sampleVelocities": 3e-4,
    "motorTorques": 3e-4,
    "frictionBudgets": 3e-4,
    "effectiveVoltage": 1e-5,
}


def build_case(profile, directory):
    backlash = profile.simulation["response"]["encoder_through_backlash"]
    task_id = "Mjlab-Velocity-Flat-Backlash-MicroDuck" if backlash else "Mjlab-Velocity-Flat-MicroDuck"
    cfg = task_config(task_id, [], num_envs=1)
    task = describe_task(task_id, cfg.env, profile.execution["joint_order"])
    # Use the task's actuator configuration. The complete physical scene below comes
    # from the native fixture so the default training terrain does not enter this test.
    adapter(profile.model_kind)(cfg.env, replace(profile, task=task), directory, domain_randomization=False)
    act_cfg = cfg.env.scene.entities["robot"].articulation.actuators[0]
    act_cfg.delay_hold_prob = 1.0

    source = profile.simulation["scenario"]["scene_xml"]
    spec = mujoco.MjSpec.from_string(
        profile.assets[source].decode(),
        include={key: value for key, value in profile.assets.items() if key.endswith(".xml") and key != source},
        assets={key: value for key, value in profile.assets.items() if not key.endswith(".xml")},
    )
    names = tuple(joint.name for joint in spec.joints if joint.type != mujoco.mjtJoint.mjJNT_FREE)
    slots = [slot for slot, name in enumerate(profile.execution["joint_order"]) if name != "mouth"]
    driven = [profile.execution["joint_order"][slot] for slot in slots]
    entity = SimpleNamespace(joint_names=names)
    actuator = act_cfg.build(entity, [names.index(name) for name in driven], driven)
    actuator.edit_spec(spec, driven)
    model = spec.compile()
    model.opt.timestep = profile.execution["periods"]["physics_s"]
    entity.indexing = SimpleNamespace(
        joint_ids=torch.tensor([model.joint(name).id for name in names], dtype=torch.long),
        ctrl_ids=torch.arange(model.nu),
    )
    qadr = [model.joint(name).qposadr[0] for name in names]
    vadr = [model.joint(name).dofadr[0] for name in names]
    root = model.joint("trunk_base_freejoint").qposadr[0]
    data = mujoco.MjData(model)
    data.qpos[root:root + 7] = profile.simulation["root_home_pose"]
    for slot, name in zip(slots, driven, strict=True):
        data.qpos[model.joint(name).qposadr[0]] = profile.execution["home_pose_rad"][slot]
    mujoco.mj_forward(model, data)
    return spec, model, data, actuator, slots, driven, names, qadr, vadr


class CpuBridge:
    """Expose MuJoCo C arrays to the same Torch actuator, without changing its law."""

    def __init__(self, model, data):
        self.model = model
        self.data = data
        self.model_fields = SimpleNamespace(
            dof_frictionloss=torch.from_numpy(model.dof_frictionloss[None]),
            dof_damping=torch.from_numpy(model.dof_damping[None]),
        )
        self.data_fields = SimpleNamespace(nworld=1, efc=SimpleNamespace())
        self.refresh()

    def refresh(self):
        self.data_fields.qfrc_bias = self.data.qfrc_bias[None]
        self.data_fields.qfrc_constraint = self.data.qfrc_constraint[None]
        self.data_fields.qfrc_actuator = self.data.qfrc_actuator[None]
        self.data_fields.nefc = np.array([self.data.nefc])
        self.data_fields.efc.type = self.data.efc_type[None]
        self.data_fields.efc.id = self.data.efc_id[None]
        self.data_fields.efc.force = self.data.efc_force[None]

    def tensor(self, name):
        return torch.from_numpy(getattr(self.data, name)[None])

    def step(self, actuator, torque):
        self.data.ctrl[actuator.global_ctrl_ids.numpy()] = torque[0].detach().numpy()
        mujoco.mj_step(self.model, self.data)
        self.refresh()

    def snapshot(self):
        return self.data


class LegacyIndexBridge(CpuBridge):
    """Diagnostic: feed BAM the historical native constraint row selection."""

    def __init__(self, model, data):
        driven = model.actuator_trnid[:, 0]
        self.dof_by_joint = dict(zip(driven, model.jnt_dofadr[driven], strict=True))
        super().__init__(model, data)

    def refresh(self):
        super().refresh()
        types, ids, forces = self.data.efc_type, self.data.efc_id, self.data.efc_force
        selected = types == mujoco.mjtConstraint.mjCNSTR_FRICTION_DOF
        mapped = np.array([self.dof_by_joint.get(value, -1) for value in ids])
        self.data_fields.efc.id = np.where(selected & (mapped >= 0), mapped, 0)[None]
        self.data_fields.efc.force = np.where(selected & (mapped < 0), 0.0, forces)[None]


class WarpBridge:
    """Use the pinned MJLab solver/array bridge with the exact same compiled scene."""

    def __init__(self, model, data):
        import mujoco_warp
        import warp
        from mjlab.sim.sim_data import WarpBridge as TorchArrayBridge

        self.model = model
        self.data = data
        self.engine = mujoco_warp
        self.device = warp.ScopedDevice("cpu")
        with self.device:
            self.warp_model = mujoco_warp.put_model(model)
            self.warp_data = mujoco_warp.put_data(model, data, nworld=1, nconmax=128, njmax=512)
            self.engine.forward(self.warp_model, self.warp_data)
        self.model_fields = TorchArrayBridge(self.warp_model, nworld=1)
        self.data_fields = TorchArrayBridge(self.warp_data)

    def tensor(self, name):
        return getattr(self.data_fields, name)[...]

    def step(self, actuator, torque):
        self.data_fields.ctrl[:, actuator.global_ctrl_ids] = torque
        with self.device:
            self.engine.step(self.warp_model, self.warp_data)

    def snapshot(self):
        self.engine.get_data_into(self.data, self.model, self.warp_data)
        return self.data


def rollout(profile, native, directory, bridge_factory=CpuBridge):
    # MJLab keeps MjsActuator handles until initialize resolves the compiled control IDs.
    _spec, model, data, act, slots, driven, names, qadr, vadr = build_case(profile, directory)
    bridge = bridge_factory(model, data)
    act.initialize(model, bridge.model_fields, bridge.data_fields, "cpu")
    q_target = torch.zeros((1, len(names)), dtype=bridge.tensor("qpos").dtype)
    raw = SimpleNamespace(joint_pos_target=q_target, joint_vel_target=torch.zeros_like(q_target), joint_effort_target=torch.zeros_like(q_target))
    render = mujoco.MjData(model)
    has_backlash = profile.simulation["response"]["encoder_through_backlash"]
    passive = [names.index(f"passive_{name}_backlash") for name in driven] if has_backlash else []
    active = [names.index(name) for name in driven]
    previous = []
    result = []
    substeps = []
    num_slots = len(profile.execution["joint_order"])

    def project(values):
        canonical = [0.0] * num_slots
        for slot, value in zip(slots, values, strict=True):
            canonical[slot] = float(value)
        return canonical

    initial = bridge.tensor("qpos")[0, np.asarray(qadr)[active]].numpy()
    np.testing.assert_allclose(initial, np.asarray(native["initial"]["motorPositions"])[slots], rtol=0, atol=1e-7)
    for tick, block in enumerate(native["commands"], start=1):
        for lag in block["lag_physics_steps"]:
            raw.joint_pos = bridge.tensor("qpos")[:, qadr]
            raw.joint_vel = bridge.tensor("qvel")[:, vadr]
            q_target[0, active] = torch.tensor(np.asarray(block["targets_rad"])[slots], dtype=q_target.dtype)
            previous.append(q_target[0, active].clone())
            act.set_lags(torch.tensor([lag]))
            command = act.apply_delay(act.get_command(raw))
            torch.testing.assert_close(command.position_target[0], previous[max(0, len(previous) - 1 - lag)], rtol=0, atol=0)
            pre = raw.joint_pos[0, active].tolist()
            motor_torque = act.compute(command)
            voltage = float(act._bam_model.actuator.vin[0, 0])
            friction = act._as_tensor(bridge.model_fields.dof_frictionloss)[0, act._dof_ids].tolist()
            substeps.append({
                "physics_step": len(substeps) + 1, "lag": lag,
                "target_rad": command.position_target[0].tolist(),
                "motor_torque_nm": motor_torque[0].tolist(),
                "friction_nm": friction,
            })
            bridge.step(act, motor_torque)
        after = bridge.snapshot()
        np.testing.assert_allclose(after.time, tick * profile.execution["periods"]["control_s"], rtol=0, atol=5e-7)
        render.qpos[:] = after.qpos
        mujoco.mj_kinematics(model, render)
        q = after.qpos[np.asarray(qadr)[active]]
        velocities = after.qvel[np.asarray(vadr)[active]]
        passive_q = after.qpos[np.asarray(qadr)[passive]] if has_backlash else np.zeros(len(driven))
        passive_dq = after.qvel[np.asarray(vadr)[passive]] if has_backlash else np.zeros(len(driven))
        result.append({
            "trunkPosition": render.body("trunk_base").xpos.tolist(),
            "trunkQuaternion": render.body("trunk_base").xquat.tolist(),
            "preMotorPositions": project(pre),
            "motorPositions": project(q), "passivePositions": project(passive_q),
            "samplePositions": project(q + passive_q), "sampleVelocities": project(velocities + passive_dq),
            "motorTorques": project(motor_torque[0].tolist()), "frictionBudgets": project(friction),
            "effectiveVoltage": voltage, "contacts": after.contact.geom.copy().tolist(),
        })
    return result, substeps


def compare(native, result):
    differences = []
    for reference, measured in zip(native["commands"], result, strict=True):
        snapshot = dict(reference["snapshot"], samplePositions=reference["sample_positions_rad"], sampleVelocities=reference["sample_velocities_rad_s"])
        differences.append({
            key: float(np.max(np.abs(np.array(snapshot[key]) - np.array(measured[key]))))
            for key in TOLERANCES
        })
    return differences


@unittest.skipUnless(os.environ.get(FIXTURE_ENV), "export the fixed-response native Rust fixtures")
class NativeAndMjlabResponses(unittest.TestCase):
    def test_measure_fixed_response_on_the_same_scene_and_delay_sequence(self):
        fixtures = Path(os.environ[FIXTURE_ENV])
        for label in ("xl330", "hd1910", "hd1910-compatible"):
            with self.subTest(model=label):
                profile = load_profile(fixtures / f"{label}-profile.json")
                native = json.loads((fixtures / f"{label}-trace.json").read_bytes())
                self.assertEqual(native["profile_fingerprint"], profile.execution["execution_fingerprint"])
                self.assertEqual(native["joint_order"], profile.execution["joint_order"])
                self.assertEqual(len(native["commands"]), 8)
                self.assertTrue(all(len(command["lag_physics_steps"]) == 4 for command in native["commands"]))
                self.run_profile(fixtures, label, profile, native)

    def run_profile(self, fixtures, label, profile, native):
        for mode, bridge in (("cpu", CpuBridge), ("legacy-id", LegacyIndexBridge), ("warp", WarpBridge)):
            with self.subTest(solver=mode), tempfile.TemporaryDirectory() as directory:
                result, steps = rollout(profile, native, Path(directory), bridge)
                differences = compare(native, result)
                self.assertEqual(len(steps), 32)
                # Preserve measurements from the two default solver paths and the
                # separate diagnostic. Differences can grow only after the first tick.
                out = fixtures / f"python-{mode}-{label}"
                out.mkdir(exist_ok=True)
                for name, data in (("trace", result), ("substeps", steps), ("differences", differences)):
                    (out / f"{name}.json").write_text(json.dumps(data, indent=2, allow_nan=False), encoding="utf-8")
                maxima = {field: max(tick[field] for tick in differences) for field in TOLERANCES}
                print(f"{label} {mode}: maxima {maxima}", flush=True)
                for field, tolerance in TOLERANCES.items():
                    self.assertLessEqual(differences[0][field], tolerance, field)
                    if mode == "legacy-id":
                        self.assertLessEqual(maxima[field], tolerance, field)


if __name__ == "__main__":
    unittest.main()
