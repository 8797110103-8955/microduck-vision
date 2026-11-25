"""Pure reference errors and manager terms evaluated before any episode reset."""

import torch

from mjlab.utils.lab_api.math import quat_apply, quat_conjugate, quat_error_magnitude, quat_mul

from .motion import rotate_heading


def squared_errors(student, reference, heading):
    """Return per-environment errors, preserving the left/right foot ordering.

    Joint values are output-side motor+passive state. Root orientation uses a
    common reset heading alignment; feet are expressed in each root's full frame.
    World root translations therefore cannot change the foot or pose objective.
    """
    zeros = torch.zeros_like(heading)
    turn = torch.stack((torch.cos(heading / 2), zeros, zeros, torch.sin(heading / 2)), dim=-1)
    target_quat = quat_mul(*torch.broadcast_tensors(turn, reference["root_quat_wxyz"]))
    angle = quat_error_magnitude(student["root_quat_wxyz"], target_quat)
    student_feet = quat_apply(
        quat_conjugate(student["root_quat_wxyz"])[:, None, :].expand(-1, 2, -1),
        student["foot_pos_world"] - student["root_pos"][:, None, :],
    )
    target_feet = quat_apply(
        quat_conjugate(reference["root_quat_wxyz"])[:, None, :].expand(-1, 2, -1),
        reference["foot_pos_world"] - reference["root_pos"][:, None, :],
    )
    return {
        "joint_position": (student["output_qpos"] - reference["output_qpos"]).square().mean(-1),
        "joint_velocity": (student["output_qvel"] - reference["output_qvel"]).square().mean(-1),
        "root_height": (student["root_pos"][:, 2] - reference["root_pos"][:, 2]).square(),
        "root_orientation": angle.square(),
        "feet": (student_feet - target_feet).square().mean(dim=(-1, -2)),
    }


def exponentials(errors, recipe):
    return {name: torch.exp(-error / recipe["reference"][name]["scale"] ** 2)
            for name, error in errors.items()}


def reference_reward(env, component):
    state = env._deepmimic_reference
    state.prepare()
    return state.last_rewards[component]


def reference_error(env, component):
    state = env._deepmimic_reference
    state.prepare()
    return torch.nan_to_num(state.last_errors[component], nan=1e6, posinf=1e6, neginf=1e6)


def reference_complete(env):
    state = env._deepmimic_reference
    state.prepare()
    return state.last_sample["completed"]


def align_world(reference, heading, translation):
    """Same SE(2) reset alignment for root, feet, orientation and velocities."""
    aligned = dict(reference)
    zeros = torch.zeros_like(heading)
    turn = torch.stack((torch.cos(heading / 2), zeros, zeros, torch.sin(heading / 2)), dim=-1)
    aligned["root_quat_wxyz"] = quat_mul(*torch.broadcast_tensors(turn, reference["root_quat_wxyz"]))
    aligned["root_pos"] = rotate_heading(reference["root_pos"], heading) + translation
    aligned["foot_pos_world"] = rotate_heading(reference["foot_pos_world"], heading[:, None]) + translation[:, None, :]
    for name in ("root_lin_vel_world", "root_ang_vel_world"):
        aligned[name] = rotate_heading(reference[name], heading)
    return aligned
