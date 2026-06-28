#!/usr/bin/env python3

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[1]


ROOT = _repo_root()
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from src.forward_kinematics import Arm7DOFDH
from src.geometric_jacobian import analyze_position_jacobian, geometric_jacobian, position_jacobian
from src.inverse_differential_kinematics import (
    adaptive_damping,
    analyze_self_collision,
    analyze_task_jacobian,
    default_link_collision_radius,
    inverse_differential_kinematics,
    saturate_norm,
    self_collision_avoidance_command,
    task_jacobian,
    tool_axis_misalignment,
)
from src.robot_arm_qt_dashboard import (
    CartesianTrackingSession,
    SIMULATION_LINK_SEGMENT_LABELS,
    cartesian_target_should_advance,
    selection_highlight_state,
)
from src.joint_space_controller import JointSpacePDController, PDGains, simulate_joint_step_response
from src.joint_space_dynamics import SimpleDynamics7DOF


def _ensure(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def main() -> None:
    arm = Arm7DOFDH()
    q = np.deg2rad([30.0, 20.0, -15.0, 10.0, -12.0, 8.0, 5.0])
    q_sing = np.deg2rad([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 25.0])
    q_safe = np.zeros(7, dtype=float)
    q_collision = np.array(
        [
            1.16579467,
            0.94536351,
            1.18404573,
            -0.69792728,
            -2.29275628,
            1.39165228,
            0.15930591,
        ],
        dtype=float,
    )
    collision_radius = default_link_collision_radius(arm)

    state = arm.frame_state(q)
    _ensure(state.joint_7_transform.shape == (4, 4), "FK joint-7 transform has wrong shape.")
    _ensure(state.tool_transform.shape == (4, 4), "FK tool transform has wrong shape.")
    _ensure(len(state.joint_transforms) == 7, "FK joint transform count is wrong.")
    _ensure(state.joint_points.shape == (8, 3), "FK joint points have wrong shape.")
    _ensure(state.tool_point.shape == (3,), "FK tool point has wrong shape.")
    _ensure(np.all(np.isfinite(state.tool_point)), "FK tool point contains non-finite values.")
    _ensure(
        not np.allclose(state.tool_point, state.joint_points[-1], atol=1e-9, rtol=1e-9),
        "Default tool transform should create a visible tool extension.",
    )

    home_state = arm.frame_state(np.zeros(7, dtype=float))
    expected_home_points = np.array(
        [
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.34],
            [0.0, 0.0, 0.34],
            [-0.4, 0.0, 0.34],
            [-0.4, 0.0, 0.34],
            [-0.4, 0.4, 0.34],
            [-0.4, 0.4, 0.34],
            [-0.274, 0.4, 0.34],
        ],
        dtype=float,
    )
    expected_segment_lengths = np.array([0.34, 0.0, 0.40, 0.0, 0.40, 0.0, 0.126], dtype=float)
    _ensure(
        np.allclose([link.a for link in arm.links], np.zeros(7), atol=1e-9, rtol=1e-9),
        "The refreshed DH table should use zero lateral offsets.",
    )
    _ensure(
        np.allclose([link.d for link in arm.links], [0.34, 0.0, 0.40, 0.0, 0.40, 0.0, 0.126], atol=1e-9, rtol=1e-9),
        "The refreshed DH table should use the new iiwa-style axial spans.",
    )
    _ensure(
        np.allclose(home_state.joint_points, expected_home_points, atol=1e-9, rtol=1e-9),
        "Default home pose should match the refreshed iiwa-style spacing.",
    )
    _ensure(
        np.allclose(arm.segment_lengths, expected_segment_lengths, atol=1e-9, rtol=1e-9),
        "Nominal segment lengths should be derived from the refreshed home pose.",
    )
    _ensure(
        np.isclose(arm.reach, 0.6402, atol=1e-9, rtol=1e-9),
        "Reach proxy should be recomputed from the refreshed home pose.",
    )

    q_roll = q.copy()
    q_roll[6] += 0.5
    rolled_state = arm.frame_state(q_roll)
    _ensure(
        np.allclose(state.tool_point, rolled_state.tool_point, atol=1e-9, rtol=1e-9),
        "Tool point should stay fixed when only q7 changes.",
    )

    J = geometric_jacobian(arm, q)
    J_v = position_jacobian(arm, q)
    analysis = analyze_position_jacobian(arm, q)
    J_task = task_jacobian(arm, q)
    task_analysis = analyze_task_jacobian(arm, q)
    J_task_sing = task_jacobian(arm, q_sing)
    _ensure(J.shape == (6, 7), "Geometric Jacobian has wrong shape.")
    _ensure(J_v.shape == (3, 7), "Position Jacobian has wrong shape.")
    _ensure(analysis.singular_values.shape == (3,), "Jacobian analysis has wrong singular-value shape.")
    _ensure(J_task.shape == (5, 7), "Task Jacobian has wrong shape.")
    _ensure(task_analysis.singular_values.shape == (5,), "Task Jacobian analysis has wrong singular-value shape.")
    _ensure(np.all(np.isfinite(J)), "Geometric Jacobian contains non-finite values.")
    _ensure(np.all(np.isfinite(J_v)), "Position Jacobian contains non-finite values.")
    _ensure(np.allclose(J_v[:, 6], 0.0, atol=1e-9, rtol=1e-9), "Default geometry should make the q7 position column zero.")
    _ensure(np.allclose(J_task[:, 6], 0.0, atol=1e-9, rtol=1e-9), "Default geometry should leave q7 out of the 5D task Jacobian.")

    highlight = selection_highlight_state(3, 8)
    _ensure(
        highlight.joint_marker_index == 2
        and highlight.joint_segment_index == 2
        and highlight.link_marker_index == 7
        and highlight.link_segment_index == 7,
        "Simulation selection mapping should highlight the selected joint and link segment.",
    )
    _ensure(
        SIMULATION_LINK_SEGMENT_LABELS
        == ("B-J1", "J1-J2", "J2-J3", "J3-J4", "J4-J5", "J5-J6", "J6-J7", "J7-tool"),
        "Link labels should match the rendered FK chain.",
    )

    v_raw = np.array([0.8, 0.8, 0.0], dtype=float)
    v_desired = saturate_norm(v_raw, 0.5)
    _ensure(np.isclose(np.linalg.norm(v_desired), 0.5, atol=1e-9, rtol=1e-9), "Cartesian speed saturation should limit the vector norm.")

    lam_near = adaptive_damping(analyze_task_jacobian(arm, q_sing).min_singular_value)
    lam_far = adaptive_damping(task_analysis.min_singular_value)
    _ensure(lam_near > lam_far, "Adaptive damping should increase as the smallest singular value decreases.")

    q_dot_cmd = inverse_differential_kinematics(arm, q, v_desired, collision_radius=collision_radius, collision_gain=0.0)
    _ensure(q_dot_cmd.shape == (7,), "Inverse kinematics returned the wrong shape.")
    _ensure(np.all(np.isfinite(q_dot_cmd)), "Inverse kinematics returned non-finite values.")

    axis_before = tool_axis_misalignment(arm, q)
    q_axis_cmd = inverse_differential_kinematics(
        arm,
        q,
        np.zeros(3, dtype=float),
        nullspace_gain=0.0,
        collision_radius=collision_radius,
        collision_gain=0.0,
    )
    q_axis_next = q + 0.01 * q_axis_cmd
    axis_after = tool_axis_misalignment(arm, q_axis_next)
    _ensure(axis_after < axis_before, "Down-axis control should reduce the tool-axis misalignment.")
    _ensure(
        not cartesian_target_should_advance(
            position_error_norm=0.0,
            axis_misalignment=axis_before,
            position_tolerance=0.01,
            axis_tolerance=0.10,
            target_steps=1,
            max_steps=490,
        ),
        "The Cartesian demo should not advance a target until both tolerances are met.",
    )
    _ensure(
        cartesian_target_should_advance(
            position_error_norm=0.0,
            axis_misalignment=0.05,
            position_tolerance=0.01,
            axis_tolerance=0.10,
            target_steps=1,
            max_steps=490,
        ),
        "The Cartesian demo should advance once both tolerances are met.",
    )
    _ensure(
        cartesian_target_should_advance(
            position_error_norm=0.5,
            axis_misalignment=0.5,
            position_tolerance=0.01,
            axis_tolerance=0.10,
            target_steps=490,
            max_steps=490,
        ),
        "The Cartesian demo should keep the max-step fallback.",
    )

    safe_collision = analyze_self_collision(arm, q_safe, collision_radius=collision_radius)
    _ensure(not safe_collision.has_active_pair, "The safe regression pose should not trigger the self-collision band.")
    _ensure(np.isclose(safe_collision.total_penalty, 0.0, atol=1e-9, rtol=1e-9), "The safe regression pose should have zero self-collision penalty.")
    qdot_safe, safe_command_analysis = self_collision_avoidance_command(
        arm,
        q_safe,
        collision_radius=collision_radius,
        collision_gain=0.3,
    )
    _ensure(not safe_command_analysis.has_active_pair, "Safe collision command should report no active pair.")
    _ensure(np.allclose(qdot_safe, 0.0, atol=1e-9, rtol=1e-9), "Safe collision pose should produce a negligible repulsion command.")

    session = CartesianTrackingSession()
    session_snapshot = session.snapshot()
    for key in (
        "min_self_collision_distance",
        "self_collision_safe_distance",
        "self_collision_penalty",
        "self_collision_active",
        "self_collision_active_pair_count",
    ):
        _ensure(key in session_snapshot.telemetry, f"Cartesian telemetry is missing {key}.")

    collision_analysis = analyze_self_collision(arm, q_collision, collision_radius=collision_radius)
    _ensure(collision_analysis.has_active_pair, "The folded regression pose should enter the collision band.")
    _ensure(collision_analysis.total_penalty > 0.0, "The folded regression pose should produce a positive collision penalty.")

    qdot_collision_only, collision_analysis_again = self_collision_avoidance_command(
        arm,
        q_collision,
        collision_radius=collision_radius,
        collision_gain=0.3,
    )
    _ensure(collision_analysis_again.has_active_pair, "Collision command should report the active collision pair.")
    _ensure(np.all(np.isfinite(qdot_collision_only)), "Collision avoidance returned non-finite values.")
    _ensure(np.linalg.norm(qdot_collision_only) > 0.0, "Collision avoidance should generate a correction in the folded pose.")

    collision_next = analyze_self_collision(arm, q_collision + 0.02 * qdot_collision_only, collision_radius=collision_radius)
    _ensure(
        collision_next.min_distance > collision_analysis.min_distance,
        "Collision repulsion should increase the minimum capsule distance.",
    )

    qdot_primary = inverse_differential_kinematics(
        arm,
        q_sing,
        np.array([0.05, 0.0, 0.0], dtype=float),
        nullspace_gain=0.0,
        collision_radius=collision_radius,
        collision_gain=0.0,
    )
    qdot_secondary = inverse_differential_kinematics(
        arm,
        q_sing,
        np.array([0.05, 0.0, 0.0], dtype=float),
        nullspace_gain=0.1,
        collision_radius=collision_radius,
        collision_gain=0.0,
    )
    _ensure(np.all(np.isfinite(qdot_primary)), "Primary IK command near the singular pose contains non-finite values.")
    _ensure(np.all(np.isfinite(qdot_secondary)), "Nullspace IK command near the singular pose contains non-finite values.")
    _ensure(abs(qdot_primary[6]) < 1e-8, "q7 should remain free in the primary 5D task.")
    _ensure(abs(qdot_secondary[6]) > abs(qdot_primary[6]) + 1e-5, "The nullspace objective should use the redundant joint for the secondary task.")
    task_delta = np.linalg.norm(J_task_sing @ (qdot_secondary - qdot_primary))
    task_motion = np.linalg.norm(J_task_sing @ qdot_primary)
    _ensure(
        task_delta <= 0.35 * max(task_motion, 1e-9) + 1e-6,
        "Nullspace motion should stay mostly in the task nullspace.",
    )

    dyn = SimpleDynamics7DOF(arm)
    controller = JointSpacePDController(
        gains=[
            PDGains(kp=5.0, kd=1.0),
            PDGains(kp=4.5, kd=0.9),
            PDGains(kp=4.0, kd=0.8),
            PDGains(kp=3.5, kd=0.7),
            PDGains(kp=3.0, kd=0.6),
            PDGains(kp=2.5, kd=0.5),
            PDGains(kp=2.0, kd=0.4),
        ],
        dyn=dyn,
    )

    tau_gravity = controller.compute_torque(
        q=q,
        q_dot=np.zeros(7),
        q_ref=q,
        q_dot_ref=np.zeros(7),
    )
    _ensure(
        np.allclose(tau_gravity, dyn.gravity_torque(q), atol=1e-9, rtol=1e-9),
        "Controller should add gravity compensation directly.",
    )

    tau = controller.compute_torque(
        q=q,
        q_dot=np.zeros(7),
        q_ref=q + np.deg2rad([20.0, -10.0, 15.0, 5.0, -8.0, 6.0, 4.0]),
        q_dot_ref=np.zeros(7),
    )
    _ensure(tau.shape == (7,), "Controller returned the wrong torque shape.")
    _ensure(np.all(np.isfinite(tau)), "Controller returned non-finite torques.")
    _ensure(
        np.max(np.abs(tau)) <= controller.tau_limit + 1e-9,
        "Controller torque saturation is not enforcing the limit.",
    )

    q_next, q_dot_next = dyn.step(q, np.zeros(7), tau, dt=0.01)
    _ensure(q_next.shape == (7,), "Dynamics returned the wrong q shape.")
    _ensure(q_dot_next.shape == (7,), "Dynamics returned the wrong q_dot shape.")
    _ensure(np.all(np.isfinite(q_next)), "Dynamics returned non-finite q values.")
    _ensure(np.all(np.isfinite(q_dot_next)), "Dynamics returned non-finite q_dot values.")

    ts, qs, qdots = simulate_joint_step_response(
        dyn=dyn,
        controller=controller,
        q_init=np.zeros(7),
        qdot_init=np.zeros(7),
        q_ref=np.deg2rad([10.0, -5.0, 15.0, 0.0, -8.0, 6.0, 4.0]),
        dt=0.01,
        t_final=0.05,
    )
    _ensure(ts.ndim == 1 and qs.shape[1] == 7 and qdots.shape[1] == 7, "Step-response simulation returned inconsistent shapes.")
    _ensure(np.all(np.isfinite(qs)), "Step-response simulation produced non-finite joint positions.")
    _ensure(np.all(np.isfinite(qdots)), "Step-response simulation produced non-finite joint velocities.")

    print("Smoke check passed.")


if __name__ == "__main__":
    main()
