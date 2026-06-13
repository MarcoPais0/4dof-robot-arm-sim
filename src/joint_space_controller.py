import numpy as np
from dataclasses import dataclass
from typing import Sequence, Tuple

try:
    # When used as part of the package
    from .joint_space_dynamics import SimpleDynamics4DOF
except ImportError:
    # When run directly from the src directory
    from joint_space_dynamics import SimpleDynamics4DOF


@dataclass
class PIDGains:
    kp: float
    ki: float = 0.0
    kd: float = 0.0


class JointSpacePIDController:
    """
    Per-joint PID controller with optional feed-forward velocity.
    """

    def __init__(
        self,
        gains: Sequence[PIDGains],
        dyn: SimpleDynamics4DOF,
    ) -> None:
        if len(gains) != dyn.arm.dof:
            raise ValueError("Number of gain sets must equal arm DOF.")
        self.gains = gains
        self.dyn = dyn
        self.integral_error = np.zeros(dyn.arm.dof, dtype=float)

    def reset(self) -> None:
        self.integral_error[:] = 0.0

    def compute_torque(
        self,
        q: Sequence[float],
        q_dot: Sequence[float],
        q_ref: Sequence[float],
        q_dot_ref: Sequence[float] | None = None,
        dt: float = 0.01,
    ) -> np.ndarray:
        """
        Compute joint torques for tracking reference trajectories.
        """
        q = np.asarray(q, dtype=float)
        q_dot = np.asarray(q_dot, dtype=float)
        q_ref = np.asarray(q_ref, dtype=float)
        if q_dot_ref is None:
            q_dot_ref = np.zeros_like(q)
        else:
            q_dot_ref = np.asarray(q_dot_ref, dtype=float)

        error = q_ref - q
        d_error = q_dot_ref - q_dot
        self.integral_error += error * dt

        tau = np.zeros_like(q)
        for i, g in enumerate(self.gains):
            tau[i] = (
                g.kp * error[i]
                + g.ki * self.integral_error[i]
                + g.kd * d_error[i]
            )
        return tau


def simulate_joint_step_response(
    dyn: SimpleDynamics4DOF,
    controller: JointSpacePIDController,
    q_init: Sequence[float],
    qdot_init: Sequence[float],
    q_ref: Sequence[float],
    dt: float = 0.01,
    t_final: float = 5.0,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Simulate joint-space response to a step in desired joint positions.

    Returns
    -------
    ts : ndarray
        Time stamps.
    qs : ndarray, shape (N, dof)
        Joint positions over time.
    qdots : ndarray, shape (N, dof)
        Joint velocities over time.
    """
    n = dyn.arm.dof
    q = np.asarray(q_init, dtype=float).copy()
    q_dot = np.asarray(qdot_init, dtype=float).copy()
    q_ref_arr = np.asarray(q_ref, dtype=float)

    steps = int(t_final / dt)
    ts = np.linspace(0.0, t_final, steps + 1)
    qs = np.zeros((steps + 1, n), dtype=float)
    qdots = np.zeros((steps + 1, n), dtype=float)

    qs[0] = q
    qdots[0] = q_dot

    controller.reset()

    for k in range(steps):
        tau = controller.compute_torque(
            q=q,
            q_dot=q_dot,
            q_ref=q_ref_arr,
            q_dot_ref=np.zeros_like(q),
            dt=dt,
        )
        q, q_dot = dyn.step(q, q_dot, tau, dt)
        qs[k + 1] = q
        qdots[k + 1] = q_dot

    return ts, qs, qdots


def main() -> None:
    """
    Quick smoke test of the controller and dynamics on a simple step.
    """
    try:
        from .forward_kinematics import Arm4DOFDH
    except ImportError:
        from forward_kinematics import Arm4DOFDH

    arm = Arm4DOFDH()
    dyn = SimpleDynamics4DOF(arm)
    gains = [
        PIDGains(kp=5.0, ki=0.0, kd=1.0),
        PIDGains(kp=4.0, ki=0.0, kd=0.8),
        PIDGains(kp=3.0, ki=0.0, kd=0.6),
        PIDGains(kp=2.0, ki=0.0, kd=0.5),
    ]
    controller = JointSpacePIDController(gains, dyn)

    q_init = np.zeros(4)
    qdot_init = np.zeros(4)
    q_ref = np.deg2rad([30.0, 20.0, -15.0, 10.0])

    ts, qs, _ = simulate_joint_step_response(
        dyn, controller, q_init, qdot_init, q_ref, dt=0.01, t_final=3.0
    )

    print("Final q:", qs[-1])


if __name__ == "__main__":
    main()
