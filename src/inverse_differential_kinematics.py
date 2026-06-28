import numpy as np
from dataclasses import dataclass
from typing import Sequence

try:
    # When used as part of the package
    from .forward_kinematics import Arm7DOFDH
    from .geometric_jacobian import geometric_jacobian, position_jacobian
except ImportError:
    # When run directly from the src directory
    from forward_kinematics import Arm7DOFDH
    from geometric_jacobian import geometric_jacobian, position_jacobian


WORLD_X_AXIS = np.array([1.0, 0.0, 0.0], dtype=float)
WORLD_Y_AXIS = np.array([0.0, 1.0, 0.0], dtype=float)
WORLD_DOWN_AXIS = np.array([0.0, 0.0, -1.0], dtype=float)

DEFAULT_LINK_VISUAL_RADIUS_MIN = 0.018
DEFAULT_LINK_VISUAL_RADIUS_SCALE = 0.022
DEFAULT_COLLISION_RADIUS_SCALE = 0.75
SELF_COLLISION_CLEARANCE = 0.01
SELF_COLLISION_GAIN_DEFAULT = 0.25
SELF_COLLISION_COMMAND_LIMIT = 0.35
SELF_COLLISION_FD_STEP = 1e-4

TASK_AXIS_GAIN = 1.5
NULLSPACE_GAIN_DEFAULT = 0.06
NULLSPACE_FD_STEP = 1e-4
NULLSPACE_SINGULARITY_WEIGHT = 1.0
NULLSPACE_HEMISPHERE_WEIGHT = 0.35
NULLSPACE_POSTURE_WEIGHT = 0.02
NULLSPACE_SINGULARITY_SCALE = 0.12
NULLSPACE_COMMAND_LIMIT = 0.35
NULLSPACE_HEMISPHERE_MARGIN = 0.08


@dataclass(frozen=True)
class TaskJacobianAnalysis:
    """
    SVD-based analysis of the 5D task Jacobian.

    The task Jacobian stacks tool-position tracking and tool-axis alignment
    in the base frame.
    """

    singular_values: np.ndarray
    rank: int
    min_singular_value: float
    condition_number: float
    manipulability: float


@dataclass(frozen=True)
class SelfCollisionAnalysis:
    """
    Summary of the capsule-based self-collision state.

    Attributes
    ----------
    min_distance : float
        Smallest segment-to-segment distance across all checked capsule pairs.
    safe_distance : float
        Minimum distance required before the repulsion turns off.
    total_penalty : float
        Scalar potential energy accumulated across all active collision pairs.
    active_pair_count : int
        Number of capsule pairs currently inside the safety band.
    has_active_pair : bool
        Convenience flag for whether the collision potential is active.
    closest_pair : tuple[int, int] | None
        Segment indices for the closest non-adjacent pair.
    """

    min_distance: float
    safe_distance: float
    total_penalty: float
    active_pair_count: int
    has_active_pair: bool
    closest_pair: tuple[int, int] | None


def _skew(vector: Sequence[float]) -> np.ndarray:
    v = np.asarray(vector, dtype=float)
    if v.shape != (3,):
        raise ValueError(f"Expected a 3-vector, got shape {v.shape}.")
    return np.array(
        [
            [0.0, -v[2], v[1]],
            [v[2], 0.0, -v[0]],
            [-v[1], v[0], 0.0],
        ],
        dtype=float,
    )


def damped_pseudo_inverse(J: np.ndarray, lam: float = 1e-3) -> np.ndarray:
    """
    Compute the damped least-squares pseudo-inverse of a Jacobian.

    Parameters
    ----------
    J : ndarray, shape (m, n)
        Jacobian matrix.
    lam : float
        Damping factor (lambda). Larger values give more robustness near
        singularities but less accuracy.

    Returns
    -------
    J_pinv : ndarray, shape (n, m)
        Damped pseudo-inverse of J.
    """
    m, n = J.shape
    if lam <= 0.0:
        raise ValueError("lam must be positive.")
    if m >= n:
        lhs = J.T @ J + (lam**2) * np.eye(n)
        return np.linalg.solve(lhs, J.T)
    lhs = J @ J.T + (lam**2) * np.eye(m)
    return J.T @ np.linalg.solve(lhs, np.eye(m))


def saturate_norm(vector: Sequence[float], max_norm: float) -> np.ndarray:
    """
    Scale a vector so its Euclidean norm does not exceed ``max_norm``.
    """
    if max_norm <= 0.0:
        raise ValueError("max_norm must be positive.")

    v = np.asarray(vector, dtype=float)
    norm = float(np.linalg.norm(v))
    if norm <= max_norm or norm == 0.0:
        return v
    return v * (max_norm / norm)


def adaptive_damping(
    sigma_min_value: float,
    lambda_0: float = 1e-2,
    k_lambda: float = 3e-3,
    epsilon: float = 1e-3,
) -> float:
    """
    Compute the singularity-aware damping term used by the resolved-rate loop.
    """
    if lambda_0 <= 0.0:
        raise ValueError("lambda_0 must be positive.")
    if k_lambda < 0.0:
        raise ValueError("k_lambda must be non-negative.")
    if epsilon <= 0.0:
        raise ValueError("epsilon must be positive.")

    return float(lambda_0 + k_lambda / (sigma_min_value + epsilon))


def default_link_visual_radius(arm: Arm7DOFDH) -> float:
    """
    Return the link radius used by the cylindrical 3D visualization.
    """
    return float(max(DEFAULT_LINK_VISUAL_RADIUS_MIN, arm.reach * DEFAULT_LINK_VISUAL_RADIUS_SCALE))


def default_link_collision_radius(arm: Arm7DOFDH) -> float:
    """
    Return the conservative capsule radius used for self-collision checks.
    """
    return float(DEFAULT_COLLISION_RADIUS_SCALE * default_link_visual_radius(arm))


def tool_axis_direction(arm: Arm7DOFDH, q: Sequence[float]) -> np.ndarray:
    """
    Return the tool z-axis expressed in the base frame.
    """
    state = arm.frame_state(q)
    return np.asarray(state.tool_transform[:3, 2], dtype=float)


def tool_axis_misalignment(arm: Arm7DOFDH, q: Sequence[float]) -> float:
    """
    Return the angular deviation between the tool z-axis and world -Z.
    """
    axis = tool_axis_direction(arm, q)
    down_alignment = float(np.clip(np.dot(axis, WORLD_DOWN_AXIS), -1.0, 1.0))
    return float(np.arccos(down_alignment))


def tool_axis_alignment_error(arm: Arm7DOFDH, q: Sequence[float]) -> np.ndarray:
    """
    Return the 2D tool-axis alignment residual used by the task solver.

    The task tracks the horizontal components of the tool z-axis in the world
    x/y basis. The separate hemisphere safeguard keeps the solver on the
    down-pointing branch.
    """
    axis = tool_axis_direction(arm, q)
    return np.array(
        [
            float(np.dot(axis, WORLD_X_AXIS)),
            float(np.dot(axis, WORLD_Y_AXIS)),
        ],
        dtype=float,
    )


def tool_axis_alignment_jacobian(arm: Arm7DOFDH, q: Sequence[float]) -> np.ndarray:
    """
    Return the 2x7 Jacobian that tracks the horizontal components of the tool z-axis.
    """
    axis = tool_axis_direction(arm, q)
    J_omega = geometric_jacobian(arm, q)[3:6, :]
    axis_dot = -_skew(axis) @ J_omega
    return np.vstack((WORLD_X_AXIS, WORLD_Y_AXIS)) @ axis_dot


def self_collision_segment_pairs(segment_count: int = 8) -> tuple[tuple[int, int], ...]:
    """
    Return the non-adjacent capsule pairs used for self-collision checks.
    """
    if segment_count < 2:
        return ()
    return tuple((i, j) for i in range(segment_count) for j in range(i + 2, segment_count))


def _point_segment_distance(
    point: np.ndarray,
    seg_a: np.ndarray,
    seg_b: np.ndarray,
) -> tuple[float, np.ndarray]:
    point = np.asarray(point, dtype=float)
    seg_a = np.asarray(seg_a, dtype=float)
    seg_b = np.asarray(seg_b, dtype=float)
    delta = seg_b - seg_a
    denom = float(np.dot(delta, delta))
    if denom <= 1e-12:
        closest = seg_a
        return float(np.linalg.norm(point - closest)), closest

    t = float(np.dot(point - seg_a, delta) / denom)
    t = float(np.clip(t, 0.0, 1.0))
    closest = seg_a + t * delta
    return float(np.linalg.norm(point - closest)), closest


def segment_segment_distance(
    seg_a0: Sequence[float],
    seg_a1: Sequence[float],
    seg_b0: Sequence[float],
    seg_b1: Sequence[float],
) -> tuple[float, np.ndarray, np.ndarray]:
    """
    Return the shortest distance between two 3D line segments.
    """
    p0 = np.asarray(seg_a0, dtype=float)
    p1 = np.asarray(seg_a1, dtype=float)
    q0 = np.asarray(seg_b0, dtype=float)
    q1 = np.asarray(seg_b1, dtype=float)

    u = p1 - p0
    v = q1 - q0
    w = p0 - q0
    a = float(np.dot(u, u))
    b = float(np.dot(u, v))
    c = float(np.dot(v, v))
    d = float(np.dot(u, w))
    e = float(np.dot(v, w))
    small_num = 1e-12

    if a <= small_num and c <= small_num:
        cp_p = p0
        cp_q = q0
        return float(np.linalg.norm(cp_p - cp_q)), cp_p, cp_q
    if a <= small_num:
        dist, cp_q = _point_segment_distance(p0, q0, q1)
        return dist, p0, cp_q
    if c <= small_num:
        dist, cp_p = _point_segment_distance(q0, p0, p1)
        return dist, cp_p, q0

    d_nom = a * c - b * b
    s_n = d_nom
    s_d = d_nom
    t_n = d_nom
    t_d = d_nom

    if d_nom < small_num:
        s_n = 0.0
        s_d = 1.0
        t_n = e
        t_d = c
    else:
        s_n = b * e - c * d
        t_n = a * e - b * d
        if s_n < 0.0:
            s_n = 0.0
            t_n = e
            t_d = c
        elif s_n > s_d:
            s_n = s_d
            t_n = e + b
            t_d = c

    if t_n < 0.0:
        t_n = 0.0
        if -d < 0.0:
            s_n = 0.0
            s_d = 1.0
        elif -d > a:
            s_n = s_d
        else:
            s_n = -d
            s_d = a
    elif t_n > t_d:
        t_n = t_d
        if (-d + b) < 0.0:
            s_n = 0.0
            s_d = 1.0
        elif (-d + b) > a:
            s_n = s_d
        else:
            s_n = -d + b
            s_d = a

    sc = 0.0 if abs(s_n) <= small_num else s_n / s_d
    tc = 0.0 if abs(t_n) <= small_num else t_n / t_d
    cp_p = p0 + sc * u
    cp_q = q0 + tc * v
    return float(np.linalg.norm(cp_p - cp_q)), cp_p, cp_q


def _segments_touch_at_endpoint(
    seg_a0: Sequence[float],
    seg_a1: Sequence[float],
    seg_b0: Sequence[float],
    seg_b1: Sequence[float],
    atol: float = 1e-9,
) -> bool:
    """
    Return True when two FK segments meet at a shared endpoint.

    The zero-offset iiwa-style table used by the sim can place some non-adjacent
    FK points at the same location in the home pose. Those touching pairs are
    not treated as self-collisions.
    """
    endpoints_a = (np.asarray(seg_a0, dtype=float), np.asarray(seg_a1, dtype=float))
    endpoints_b = (np.asarray(seg_b0, dtype=float), np.asarray(seg_b1, dtype=float))
    for endpoint_a in endpoints_a:
        for endpoint_b in endpoints_b:
            if float(np.linalg.norm(endpoint_a - endpoint_b)) <= atol:
                return True
    return False


def _collision_points(arm: Arm7DOFDH, q: Sequence[float]) -> np.ndarray:
    state = arm.frame_state(q)
    return np.vstack((state.joint_points, state.tool_point[None, :]))


def analyze_self_collision(
    arm: Arm7DOFDH,
    q: Sequence[float],
    collision_radius: float | None = None,
    clearance: float = SELF_COLLISION_CLEARANCE,
) -> SelfCollisionAnalysis:
    """
    Evaluate the capsule self-collision state for the current configuration.

    The arm is modeled as capsules over the FK segments
    base->J1, J1->J2, ..., J7->tool.
    """
    if collision_radius is None:
        collision_radius = default_link_collision_radius(arm)
    if collision_radius <= 0.0:
        raise ValueError("collision_radius must be positive.")
    if clearance < 0.0:
        raise ValueError("clearance must be non-negative.")

    points = _collision_points(arm, q)
    safe_distance = float(2.0 * collision_radius + clearance)
    total_penalty = 0.0
    min_distance = float("inf")
    closest_pair: tuple[int, int] | None = None
    active_pair_count = 0

    for i, j in self_collision_segment_pairs(points.shape[0] - 1):
        if _segments_touch_at_endpoint(points[i], points[i + 1], points[j], points[j + 1]):
            continue

        distance, _, _ = segment_segment_distance(points[i], points[i + 1], points[j], points[j + 1])
        if distance < min_distance:
            min_distance = distance
            closest_pair = (i, j)
        gap = safe_distance - distance
        if gap > 0.0:
            total_penalty += 0.5 * gap * gap
            active_pair_count += 1

    if not np.isfinite(min_distance):
        min_distance = 0.0

    return SelfCollisionAnalysis(
        min_distance=float(min_distance),
        safe_distance=safe_distance,
        total_penalty=float(total_penalty),
        active_pair_count=active_pair_count,
        has_active_pair=active_pair_count > 0,
        closest_pair=closest_pair,
    )


def self_collision_potential(
    arm: Arm7DOFDH,
    q: Sequence[float],
    collision_radius: float | None = None,
    clearance: float = SELF_COLLISION_CLEARANCE,
) -> float:
    """
    Return the scalar soft-collision penalty used by the repulsion term.
    """
    return analyze_self_collision(arm, q, collision_radius=collision_radius, clearance=clearance).total_penalty


def self_collision_avoidance_command(
    arm: Arm7DOFDH,
    q: Sequence[float],
    collision_radius: float | None = None,
    collision_gain: float = SELF_COLLISION_GAIN_DEFAULT,
    command_limit: float = SELF_COLLISION_COMMAND_LIMIT,
    clearance: float = SELF_COLLISION_CLEARANCE,
) -> tuple[np.ndarray, SelfCollisionAnalysis]:
    """
    Compute the collision-avoidance joint-velocity correction.

    The returned command is intended to stay below the primary 5D task and
    above the weaker singularity and posture biases.
    """
    q_arr = np.asarray(q, dtype=float)
    if q_arr.shape != (arm.dof,):
        raise ValueError(f"Expected {arm.dof} joint angles, got shape {q_arr.shape}.")
    if collision_gain < 0.0:
        raise ValueError("collision_gain must be non-negative.")
    if command_limit <= 0.0:
        raise ValueError("command_limit must be positive.")

    analysis = analyze_self_collision(arm, q_arr, collision_radius=collision_radius, clearance=clearance)
    if not analysis.has_active_pair or collision_gain == 0.0:
        return np.zeros(arm.dof, dtype=float), analysis

    def potential(qq: np.ndarray) -> float:
        return self_collision_potential(arm, qq, collision_radius=collision_radius, clearance=clearance)

    gradient = _finite_difference_gradient(potential, q_arr, SELF_COLLISION_FD_STEP)
    qdot = saturate_norm(-collision_gain * gradient, command_limit)
    return qdot, analysis


def task_jacobian(arm: Arm7DOFDH, q: Sequence[float]) -> np.ndarray:
    """
    Return the 5x7 task Jacobian for position + down-axis tracking.
    """
    J_v = position_jacobian(arm, q)
    J_axis = tool_axis_alignment_jacobian(arm, q)
    return np.vstack((J_v, J_axis))


def analyze_task_jacobian(
    arm: Arm7DOFDH,
    q: Sequence[float],
    tol: float = 1e-9,
) -> TaskJacobianAnalysis:
    """
    Compute SVD-derived rank, conditioning, and manipulability for the task Jacobian.
    """
    if tol <= 0.0:
        raise ValueError("tol must be positive.")

    J_task = task_jacobian(arm, q)
    singular_values = np.linalg.svd(J_task, compute_uv=False)
    rank = int(np.sum(singular_values > tol))
    min_singular_value = float(singular_values[-1])
    if min_singular_value > tol:
        condition_number = float(singular_values[0] / min_singular_value)
    else:
        condition_number = float("inf")
    manipulability = float(np.prod(singular_values))

    return TaskJacobianAnalysis(
        singular_values=singular_values,
        rank=rank,
        min_singular_value=min_singular_value,
        condition_number=condition_number,
        manipulability=manipulability,
    )


def _finite_difference_gradient(cost_fn, q: np.ndarray, step: float) -> np.ndarray:
    if step <= 0.0:
        raise ValueError("step must be positive.")

    gradient = np.zeros_like(q, dtype=float)
    for index in range(q.size):
        delta = np.zeros_like(q, dtype=float)
        delta[index] = step
        gradient[index] = (cost_fn(q + delta) - cost_fn(q - delta)) / (2.0 * step)
    return gradient


def _nullspace_objective(
    arm: Arm7DOFDH,
    q: np.ndarray,
    q_center: np.ndarray,
) -> float:
    task_analysis = analyze_task_jacobian(arm, q)
    singularity_cost = float(np.exp(-task_analysis.min_singular_value / NULLSPACE_SINGULARITY_SCALE))

    axis = tool_axis_direction(arm, q)
    down_alignment = float(np.dot(axis, WORLD_DOWN_AXIS))
    hemisphere_gap = max(0.0, NULLSPACE_HEMISPHERE_MARGIN - down_alignment)
    hemisphere_cost = hemisphere_gap * hemisphere_gap

    posture_error = q - q_center
    posture_cost = 0.5 * float(np.dot(posture_error, posture_error))

    return (
        NULLSPACE_SINGULARITY_WEIGHT * singularity_cost
        + NULLSPACE_HEMISPHERE_WEIGHT * hemisphere_cost
        + NULLSPACE_POSTURE_WEIGHT * posture_cost
    )


def inverse_differential_kinematics(
    arm: Arm7DOFDH,
    q: Sequence[float],
    v_desired: np.ndarray,
    lam: float | None = None,
    nullspace_gain: float = NULLSPACE_GAIN_DEFAULT,
    collision_radius: float | None = None,
    collision_gain: float = SELF_COLLISION_GAIN_DEFAULT,
) -> np.ndarray:
    """
    Compute joint velocities for the 5D position + down-axis task.

    The primary task uses the desired tool-origin linear velocity together with
    the horizontal components of the tool z-axis. The remaining redundancy is
    reserved for a soft self-collision repulsion term, a smaller singularity
    repulsion term, a hemisphere safeguard, and a tiny posture bias.
    """
    q_arr = np.asarray(q, dtype=float)
    if q_arr.shape != (arm.dof,):
        raise ValueError(f"Expected {arm.dof} joint angles, got shape {q_arr.shape}.")

    v_desired = np.asarray(v_desired, dtype=float)
    if v_desired.shape != (3,):
        raise ValueError(f"Expected a 3-vector for v_desired, got shape {v_desired.shape}.")
    if nullspace_gain < 0.0:
        raise ValueError("nullspace_gain must be non-negative.")
    if collision_gain < 0.0:
        raise ValueError("collision_gain must be non-negative.")

    J_task = task_jacobian(arm, q_arr)
    if lam is None:
        task_analysis = analyze_task_jacobian(arm, q_arr)
        lam_eff = adaptive_damping(task_analysis.min_singular_value)
    else:
        lam_eff = float(lam)

    axis_error = tool_axis_alignment_error(arm, q_arr)
    task_velocity = np.concatenate(
        (
            v_desired,
            -TASK_AXIS_GAIN * axis_error,
        )
    )

    J_pinv = damped_pseudo_inverse(J_task, lam=lam_eff)
    qdot_primary = J_pinv @ task_velocity

    qdot_collision, _ = self_collision_avoidance_command(
        arm,
        q_arr,
        collision_radius=collision_radius,
        collision_gain=collision_gain,
    )

    if nullspace_gain <= 0.0 and np.allclose(qdot_collision, 0.0):
        return qdot_primary

    q_center = np.zeros(arm.dof, dtype=float)
    nullspace_gradient = _finite_difference_gradient(
        lambda qq: _nullspace_objective(arm, qq, q_center),
        q_arr,
        NULLSPACE_FD_STEP,
    )
    qdot_singularity = saturate_norm(-nullspace_gain * nullspace_gradient, NULLSPACE_COMMAND_LIMIT)
    qdot_null = qdot_collision + qdot_singularity
    nullspace_projector = np.eye(arm.dof, dtype=float) - J_pinv @ J_task
    return qdot_primary + nullspace_projector @ qdot_null


def main() -> None:
    """
    Small demo: print a joint-velocity command for a sample Cartesian command.
    """
    arm = Arm7DOFDH()
    q = np.deg2rad([30.0, 20.0, -15.0, 40.0, -25.0, 15.0, 10.0])
    v_desired = np.array([0.1, 0.0, 0.0], dtype=float)
    qdot = inverse_differential_kinematics(arm, q, v_desired)
    np.set_printoptions(precision=3, suppress=True)
    print("qdot:")
    print(qdot)


if __name__ == "__main__":
    main()
