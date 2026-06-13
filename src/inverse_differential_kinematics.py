import numpy as np
from typing import Sequence

try:
    # When used as part of the package
    from .forward_kinematics import Arm4DOFDH
    from .geometric_jacobian import geometric_jacobian
except ImportError:
    # When run directly from the src directory
    from forward_kinematics import Arm4DOFDH
    from geometric_jacobian import geometric_jacobian


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
    if m >= n:
        return np.linalg.inv(J.T @ J + (lam ** 2) * np.eye(n)) @ J.T
    return J.T @ np.linalg.inv(J @ J.T + (lam ** 2) * np.eye(m))


def inverse_differential_kinematics(
    arm: Arm4DOFDH,
    q: Sequence[float],
    xdot_desired: np.ndarray,
    lam: float = 1e-3,
) -> np.ndarray:
    """
    Compute joint velocities from a desired end-effector spatial velocity.

    Parameters
    ----------
    arm : Arm4DOFDH
        Arm model.
    q : array_like, shape (4,)
        Current joint configuration.
    xdot_desired : ndarray, shape (6,)
        Desired spatial velocity [v; w] of the end-effector.
    lam : float
        Damping factor for the pseudo-inverse.

    Returns
    -------
    qdot : ndarray, shape (4,)
        Joint velocities realizing the desired spatial velocity in the
        least-squares sense.
    """
    J = geometric_jacobian(arm, q)
    J_pinv = damped_pseudo_inverse(J, lam=lam)
    return J_pinv @ xdot_desired


def main() -> None:
    """
    Small demo: print a joint-velocity command for a sample spatial command.
    """
    arm = Arm4DOFDH()
    q = np.deg2rad([30.0, 20.0, -15.0, 40.0])
    xdot_desired = np.array([0.1, 0.0, 0.0, 0.0, 0.0, 0.0], dtype=float)
    qdot = inverse_differential_kinematics(arm, q, xdot_desired)
    np.set_printoptions(precision=3, suppress=True)
    print("qdot:")
    print(qdot)


if __name__ == "__main__":
    main()
