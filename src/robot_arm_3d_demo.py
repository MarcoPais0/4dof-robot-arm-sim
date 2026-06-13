import argparse
import time
from dataclasses import dataclass, field
from typing import Any

import numpy as np
import pyvista as pv

try:
    # When run as part of the package: python -m 4DOF_3D_Robotic_Arm_DH_Simulation.src.robot_arm_3d_demo
    from .forward_kinematics import Arm4DOFDH
    from .geometric_jacobian import analyze_position_jacobian
    from .inverse_differential_kinematics import inverse_differential_kinematics
    from .joint_space_controller import JointSpacePDController, PDGains
    from .joint_space_dynamics import SimpleDynamics4DOF
except ImportError:
    # When run directly from the src directory as a script
    from forward_kinematics import Arm4DOFDH
    from geometric_jacobian import analyze_position_jacobian
    from inverse_differential_kinematics import inverse_differential_kinematics
    from joint_space_controller import JointSpacePDController, PDGains
    from joint_space_dynamics import SimpleDynamics4DOF


@dataclass
class LiveMetricPlot:
    plotter: pv.Plotter
    row: int
    col: int
    title: str
    color: str
    history_seconds: float = 10.0
    num_points: int = 240
    line_mesh: pv.PolyData = field(init=False)
    time_axis: np.ndarray = field(init=False)
    values: np.ndarray = field(init=False)

    def __post_init__(self) -> None:
        self.time_axis = np.linspace(-self.history_seconds, 0.0, self.num_points)
        self.values = np.zeros(self.num_points, dtype=float)

        self.plotter.subplot(self.row, self.col)
        self.plotter.set_background("white")
        self.plotter.add_text(self.title, font_size=10)
        self.plotter.show_grid(color="lightgray")
        self.plotter.view_xy()
        self.plotter.camera.parallel_projection = True

        points = self._points()
        self.line_mesh = pv.lines_from_points(points)
        self.plotter.add_mesh(self.line_mesh, color=self.color, line_width=3)

    def _points(self) -> np.ndarray:
        return np.column_stack((self.time_axis, self.values, np.zeros_like(self.time_axis)))

    def update(self, value: float) -> None:
        self.values = np.roll(self.values, -1)
        self.values[-1] = float(value)
        self.line_mesh.points = self._points()


@dataclass
class JointStepMetrics:
    q_initial: float
    q_target: float
    times: list[float] = field(default_factory=list)
    values: list[float] = field(default_factory=list)

    @property
    def delta(self) -> float:
        return self.q_target - self.q_initial

    @property
    def tolerance_band(self) -> float:
        return 0.02 * abs(self.delta)

    def append(self, t: float, value: float) -> None:
        self.times.append(float(t))
        self.values.append(float(value))

    @staticmethod
    def _first_crossing_time(times: np.ndarray, values: np.ndarray, threshold: float, direction: float) -> float | None:
        for i in range(1, len(values)):
            prev = values[i - 1]
            curr = values[i]
            if direction >= 0.0:
                crossed = prev < threshold <= curr
            else:
                crossed = prev > threshold >= curr
            if crossed:
                if curr == prev:
                    return float(times[i])
                alpha = (threshold - prev) / (curr - prev)
                return float(times[i - 1] + alpha * (times[i] - times[i - 1]))
        return None

    def rise_time(self) -> float | None:
        delta = self.delta
        if abs(delta) < 1e-12 or len(self.times) < 2:
            return None

        times = np.asarray(self.times, dtype=float)
        values = np.asarray(self.values, dtype=float)
        direction = 1.0 if delta >= 0.0 else -1.0
        t10 = self._first_crossing_time(times, values, self.q_initial + 0.1 * delta, direction)
        if t10 is None:
            return None
        t90 = self._first_crossing_time(times, values, self.q_initial + 0.9 * delta, direction)
        if t90 is None:
            return None
        return max(0.0, t90 - t10)

    def settling_time(self) -> float | None:
        delta = self.delta
        if abs(delta) < 1e-12 or len(self.times) < 2:
            return None

        times = np.asarray(self.times, dtype=float)
        values = np.asarray(self.values, dtype=float)
        band = self.tolerance_band
        if band <= 0.0:
            return None

        outside = np.where(np.abs(values - self.q_target) > band)[0]
        if len(outside) == 0:
            return float(times[0])

        last_outside = int(outside[-1])
        if last_outside >= len(values) - 1:
            return None

        prev_value = values[last_outside]
        next_value = values[last_outside + 1]
        prev_time = times[last_outside]
        next_time = times[last_outside + 1]
        upper = self.q_target + band
        lower = self.q_target - band

        if prev_value > upper:
            boundary = upper
        elif prev_value < lower:
            boundary = lower
        else:
            return float(next_time)

        if next_value == prev_value:
            return float(next_time)

        alpha = (boundary - prev_value) / (next_value - prev_value)
        alpha = float(np.clip(alpha, 0.0, 1.0))
        return float(prev_time + alpha * (next_time - prev_time))

    def overshoot_percent(self) -> float:
        delta = self.delta
        if abs(delta) < 1e-12 or len(self.values) == 0:
            return 0.0

        values = np.asarray(self.values, dtype=float)
        signed_excursion = np.maximum(0.0, np.sign(delta) * (values - self.q_target))
        return float(np.max(signed_excursion) / abs(delta) * 100.0)

    def summary_text(self, current_error: float) -> str:
        rise = self.rise_time()
        settle = self.settling_time()
        rise_text = "pending" if rise is None else f"{rise:.2f} s"
        settle_text = "pending" if settle is None else f"{settle:.2f} s"
        return (
            f"Rise time (10-90%): {rise_text}\n"
            f"Settling time (2% band): {settle_text}\n"
            f"Overshoot: {self.overshoot_percent():.1f}%\n"
            f"Joint 1 error: {current_error:.3f} rad"
        )


@dataclass
class LiveTextPanel:
    plotter: pv.Plotter
    row: int
    col: int
    title: str
    actor: Any = field(init=False)

    def __post_init__(self) -> None:
        self.plotter.subplot(self.row, self.col)
        self.plotter.set_background("white")
        self.plotter.add_text(self.title, font_size=10)
        self.actor = self.plotter.add_text("", position="lower_left", font_size=10)

    def update(self, text: str) -> None:
        self.actor.SetInput(text)


class Arm4DOF3DSimulation:
    """
    3D visualization and demo scripts for the spatial 4DOF DH arm.
    """

    def __init__(self) -> None:
        self.arm = Arm4DOFDH()
        self.dyn = SimpleDynamics4DOF(self.arm)
        self.controller = JointSpacePDController(
            gains=[
                PDGains(kp=5.0, kd=1.0),
                PDGains(kp=4.0, kd=0.8),
                PDGains(kp=3.0, kd=0.6),
                PDGains(kp=2.0, kd=0.5),
            ],
            dyn=self.dyn,
            tau_limit=50.0,
        )
        self.cartesian_position_gain = 1.35
        self.cartesian_velocity_limit = 0.9
        self.cartesian_lambda_0 = 1e-2
        self.cartesian_lambda_gain = 3e-3
        self.cartesian_lambda_epsilon = 1e-3
        self.cartesian_target_tolerance = 0.10
        self.cartesian_target_max_steps = 240
        self.cartesian_target_count = 15
        self.cartesian_target_seed = 7

    def _generate_targets(self, count: int) -> np.ndarray:
        rng = np.random.default_rng(self.cartesian_target_seed)
        targets: list[np.ndarray] = []
        lower = np.array([0.5, -2.5, 1.0], dtype=float)
        upper = np.array([6.0, 2.5, 7.0], dtype=float)
        min_radius = 1.75
        max_radius = self.arm.l1 + self.arm.l2 + self.arm.l3 + self.arm.l_tool

        while len(targets) < count:
            candidate = rng.uniform(lower, upper)
            radius = np.linalg.norm(candidate)
            if min_radius <= radius <= max_radius:
                targets.append(candidate)

        return np.asarray(targets, dtype=float)

    def _update_arm_mesh(self, arm_mesh: pv.PolyData, q: np.ndarray) -> None:
        state = self.arm.frame_state(q)
        arm_mesh.points = np.vstack((state.joint_points, state.tool_point))

    def _update_trace_mesh(self, trace_mesh: pv.PolyData, trace_points: np.ndarray, point: np.ndarray) -> None:
        trace_points[:] = np.roll(trace_points, -1, axis=0)
        trace_points[-1] = point
        trace_mesh.points = trace_points

    def _move_polydata(self, mesh: pv.PolyData, center: np.ndarray) -> None:
        offset = np.asarray(center, dtype=float) - np.asarray(mesh.center, dtype=float)
        mesh.points = mesh.points + offset

    def _make_scene(self, title: str, target_count: int | None = None) -> tuple[pv.Plotter, pv.PolyData, pv.PolyData, pv.PolyData, pv.PolyData | None, pv.PolyData | None]:
        plotter = pv.Plotter(shape=(2, 3), window_size=(1800, 1000))
        plotter.set_background("white")

        q0 = np.zeros(4, dtype=float)
        state = self.arm.frame_state(q0)
        initial_points = np.vstack((state.joint_points, state.tool_point))
        arm_mesh = pv.lines_from_points(initial_points)
        trace_points = np.repeat(state.tool_point[None, :], 180, axis=0)
        trace_mesh = pv.lines_from_points(trace_points)

        plotter.subplot(0, 0)
        plotter.add_text(title, font_size=12)
        plotter.add_axes()
        plotter.add_mesh(arm_mesh, color="black", line_width=4)
        plotter.add_mesh(trace_mesh, color="darkorange", line_width=3)

        target_points_mesh = None
        active_target_sphere = None
        if target_count is not None:
            targets = self._generate_targets(target_count)
            target_points_mesh = pv.PolyData(targets)
            plotter.add_mesh(
                target_points_mesh,
                color="royalblue",
                point_size=14,
                render_points_as_spheres=True,
            )
            active_target_sphere = pv.Sphere(radius=0.18, center=targets[0])
            plotter.add_mesh(
                active_target_sphere,
                color="red",
            )

        plotter.subplot(0, 0)
        plotter.set_focus((0.0, 0.0, 0.0))
        plotter.set_position((20.0, 20.0, 15.0))
        plotter.set_viewup((0.0, 0.0, 1.0))

        return plotter, arm_mesh, trace_mesh, trace_points, target_points_mesh, active_target_sphere

    def run_joint_step_demo(self) -> None:
        """
        Joint-space step response demo used for validation.
        Runs with an interactive window until the user closes it.
        """
        plotter, arm_mesh, trace_mesh, trace_points, _, _ = self._make_scene(
            "Joint-space validation",
        )

        error_plot = LiveMetricPlot(plotter, 0, 1, "Joint error norm", "crimson")
        torque_plot = LiveMetricPlot(plotter, 0, 2, "Torque norm", "slateblue")
        max_error_plot = LiveMetricPlot(plotter, 1, 0, "Max joint error", "seagreen")
        primary_error_plot = LiveMetricPlot(plotter, 1, 1, "Primary joint error", "darkorange")
        status_panel = LiveTextPanel(plotter, 1, 2, "Live summary")

        q_ref = np.deg2rad([30.0, 20.0, -15.0, 10.0])
        q = np.zeros(4, dtype=float)
        q_dot = np.zeros(4, dtype=float)
        dt = 0.02
        metrics = JointStepMetrics(q_initial=q[0], q_target=q_ref[0])
        t = 0.0

        plotter.show(auto_close=False, interactive_update=True)

        while plotter.is_active:
            tau = self.controller.compute_torque(
                q=q,
                q_dot=q_dot,
                q_ref=q_ref,
                q_dot_ref=np.zeros_like(q),
            )
            q_next, q_dot_next = self.dyn.step(q, q_dot, tau, dt)
            q = q_next
            q_dot = q_dot_next
            t += dt

            state = self.arm.frame_state(q)
            self._update_arm_mesh(arm_mesh, q)
            self._update_trace_mesh(trace_mesh, trace_points, state.tool_point)

            metrics.append(t, q[0])

            error = q_ref - q
            error_norm = float(np.linalg.norm(error))
            torque_norm = float(np.linalg.norm(tau))
            max_abs_error = float(np.max(np.abs(error)))
            primary_error = float(error[0])

            error_plot.update(error_norm)
            torque_plot.update(torque_norm)
            max_error_plot.update(max_abs_error)
            primary_error_plot.update(abs(primary_error))
            status_panel.update(metrics.summary_text(abs(primary_error)))

            plotter.render()
            time.sleep(dt)

    def run_cartesian_demo(self) -> None:
        """
        Differential-kinematics-based Cartesian tracking demo in 3D.

        A lifted path of randomly scattered targets is tracked using
        differential IK on the tool position. This is the main project demo.
        The window remains interactive until the user closes it.
        """
        plotter, arm_mesh, trace_mesh, trace_points, target_points_mesh, active_target_sphere = self._make_scene(
            "Cartesian tracking",
            target_count=self.cartesian_target_count,
        )

        error_plot = LiveMetricPlot(plotter, 0, 1, "Position error norm", "crimson")
        vcmd_plot = LiveMetricPlot(plotter, 0, 2, "Commanded Cartesian speed", "slateblue")
        qdot_plot = LiveMetricPlot(plotter, 1, 0, "Joint-speed norm", "seagreen")
        sigma_plot = LiveMetricPlot(plotter, 1, 1, "Min singular value", "darkorange")
        status_panel = LiveTextPanel(plotter, 1, 2, "Live status")

        targets = np.asarray(target_points_mesh.points, dtype=float) if target_points_mesh is not None else self._generate_targets(self.cartesian_target_count)
        active_index = 0
        current_target = targets[active_index]
        target_steps = 0
        dt = 0.02
        q = np.zeros(4, dtype=float)
        q_dot = np.zeros(4, dtype=float)
        plotter.show(auto_close=False, interactive_update=True)

        while plotter.is_active:
            state = self.arm.frame_state(q)
            x = state.tool_transform[:3, 3]
            error = current_target - x
            error_norm = float(np.linalg.norm(error))

            v_des_raw = self.cartesian_position_gain * error
            v_des = saturate_norm(v_des_raw, self.cartesian_velocity_limit)

            analysis = analyze_position_jacobian(self.arm, q)
            lam = adaptive_damping(
                analysis.min_singular_value,
                lambda_0=self.cartesian_lambda_0,
                k_lambda=self.cartesian_lambda_gain,
                epsilon=self.cartesian_lambda_epsilon,
            )
            q_dot = inverse_differential_kinematics(
                arm=self.arm,
                q=q,
                v_desired=v_des,
                lam=lam,
            )
            q_ref_next = q + dt * q_dot
            q = q_ref_next

            state = self.arm.frame_state(q)
            self._update_arm_mesh(arm_mesh, q)
            self._update_trace_mesh(trace_mesh, trace_points, state.tool_point)

            if active_target_sphere is not None:
                self._move_polydata(active_target_sphere, current_target)

            error_plot.update(error_norm)
            vcmd_plot.update(float(np.linalg.norm(v_des)))
            qdot_plot.update(float(np.linalg.norm(q_dot)))
            sigma_plot.update(float(analysis.min_singular_value))
            status_panel.update(
                f"Target: {active_index + 1}/{len(targets)}\n"
                f"Error norm: {error_norm:.3f} m\n"
                f"Sigma min: {analysis.min_singular_value:.3f}\n"
                f"Lambda: {lam:.3f}"
            )

            target_steps += 1
            if error_norm <= self.cartesian_target_tolerance or target_steps >= self.cartesian_target_max_steps:
                active_index = (active_index + 1) % len(targets)
                current_target = targets[active_index]
                target_steps = 0

            plotter.render()
            time.sleep(dt)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="4DOF robotic arm demo runner")
    parser.add_argument(
        "--mode",
        choices=("cartesian", "joint"),
        default="cartesian",
        help="Choose the main Cartesian demo or the joint-space validation demo.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    sim = Arm4DOF3DSimulation()
    if args.mode == "joint":
        sim.run_joint_step_demo()
    else:
        sim.run_cartesian_demo()


if __name__ == "__main__":
    main()
