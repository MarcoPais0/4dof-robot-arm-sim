import matplotlib

matplotlib.use("TkAgg")

import matplotlib.pyplot as plt
import numpy as np
from dataclasses import dataclass


@dataclass
class SecondOrderPlant:
    """
    Simple second-order LTI plant:
        q_ddot + 2*zeta*wn*q_dot + wn^2*q = u
    """

    wn: float = 1.0
    zeta: float = 0.5

    def step(self, q: float, q_dot: float, u: float, dt: float) -> tuple[float, float]:
        q_ddot = u - 2.0 * self.zeta * self.wn * q_dot - (self.wn**2) * q
        q_dot_new = q_dot + dt * q_ddot
        q_new = q + dt * q_dot_new
        return q_new, q_dot_new


@dataclass
class PID:
    kp: float
    ki: float
    kd: float

    def __post_init__(self) -> None:
        self.integral_error: float = 0.0
        self.prev_error: float = 0.0

    def reset(self) -> None:
        self.integral_error = 0.0
        self.prev_error = 0.0

    def control(self, ref: float, q: float, q_dot: float, dt: float) -> float:
        error = ref - q
        self.integral_error += error * dt
        d_error = (error - self.prev_error) / dt
        self.prev_error = error
        return self.kp * error + self.ki * self.integral_error + self.kd * d_error


def simulate_pid_on_second_order(
    plant: SecondOrderPlant,
    pid: PID,
    q0: float,
    qdot0: float,
    q_ref: float,
    t_final: float = 5.0,
    dt: float = 0.01,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    steps = int(t_final / dt)
    ts = np.linspace(0.0, t_final, steps + 1)
    qs = np.zeros_like(ts)
    qdots = np.zeros_like(ts)

    q = q0
    q_dot = qdot0
    pid.reset()

    qs[0] = q
    qdots[0] = q_dot

    for k in range(steps):
        u = pid.control(q_ref, q, q_dot, dt)
        q, q_dot = plant.step(q, q_dot, u, dt)
        qs[k + 1] = q
        qdots[k + 1] = q_dot

    return ts, qs, qdots


def main() -> None:
    """
    PID tuning sandbox on a configurable second-order plant.
    """
    plant = SecondOrderPlant(wn=1.5, zeta=0.3)

    # Example: manually chosen PID gains (can be replaced with ZN rules later)
    pid = PID(kp=4.0, ki=0.5, kd=1.0)

    ts, qs, _ = simulate_pid_on_second_order(
        plant=plant,
        pid=pid,
        q0=0.0,
        qdot0=0.0,
        q_ref=1.0,
        t_final=8.0,
        dt=0.01,
    )

    plt.figure()
    plt.plot(ts, qs, label="q(t)")
    plt.axhline(1.0, color="red", linestyle="--", label="reference")
    plt.xlabel("Time [s]")
    plt.ylabel("Output")
    plt.title("PID Control of Second-Order Plant")
    plt.legend()
    plt.grid(True)
    plt.show()


if __name__ == "__main__":
    main()

