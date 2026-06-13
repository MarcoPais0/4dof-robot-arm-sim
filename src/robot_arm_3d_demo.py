import time
import numpy as np
import pyvista as pv

try:
    # When run as part of the package: python -m 4DOF_3D_Robotic_Arm_DH_Simulation.src.robot_arm_3d_demo
    from .forward_kinematics import Arm4DOFDH
    from .joint_space_controller import (
        JointSpacePIDController,
        PIDGains,
        simulate_joint_step_response,
    )
    from .joint_space_dynamics import SimpleDynamics4DOF
    from .inverse_differential_kinematics import inverse_differential_kinematics
except ImportError:
    # When run directly from the src directory as a script
    from forward_kinematics import Arm4DOFDH
    from joint_space_controller import (
        JointSpacePIDController,
        PIDGains,
        simulate_joint_step_response,
    )
    from joint_space_dynamics import SimpleDynamics4DOF
    from inverse_differential_kinematics import inverse_differential_kinematics


class Arm4DOF3DSimulation:
    """
    3D visualization and demo scripts for the spatial 4DOF DH arm.
    """

    def __init__(self) -> None:
        self.arm = Arm4DOFDH()
        self.dyn = SimpleDynamics4DOF(self.arm)
        self.controller = JointSpacePIDController(
            gains=[
                PIDGains(kp=5.0, kd=1.0),
                PIDGains(kp=4.0, kd=0.8),
                PIDGains(kp=3.0, kd=0.6),
                PIDGains(kp=2.0, kd=0.5),
            ],
            dyn=self.dyn,
        )

        # PyVista plotter setup
        self.plotter = pv.Plotter()
        self.plotter.add_axes()
        self.plotter.set_background("white")
        self.plotter.add_text("Spatial 4DOF DH Arm - 3D Simulation", font_size=12)

        # Initial configuration
        self.q = np.zeros(4, dtype=float)
        state = self.arm.frame_state(self.q)
        arm_points = np.vstack((state.joint_points, state.tool_point))
        self.arm_mesh = pv.lines_from_points(arm_points)
        self.arm_actor = self.plotter.add_mesh(
            self.arm_mesh,
            color="black",
            line_width=4,
        )

        # End-effector marker
        self.ee_sphere = pv.Sphere(radius=0.2, center=state.tool_point)
        self.ee_actor = self.plotter.add_mesh(self.ee_sphere, color="blue")

        # Adjust camera roughly similar to previous view box
        self.plotter.set_focus((0.0, 0.0, 0.0))
        self.plotter.set_position((20.0, 20.0, 15.0))
        self.plotter.set_viewup((0.0, 0.0, 1.0))

    def _update_arm_geometry(self, q: np.ndarray) -> None:
        state = self.arm.frame_state(q)
        self.arm_mesh.points = np.vstack((state.joint_points, state.tool_point))

        # Update end-effector sphere position
        self.ee_sphere.center = state.tool_point

    def run_joint_step_demo(self) -> None:
        """
        Joint-space step response demo using the PID controller and dynamics.
        Runs with an interactive window until the user closes it.
        """
        # Start interactive window (allows rotate/zoom/close)
        self.plotter.show(auto_close=False, interactive_update=True)

        q_init = np.zeros(4)
        qdot_init = np.zeros(4)
        q_ref = np.deg2rad([30.0, 20.0, -15.0, 10.0])

        ts, qs, _ = simulate_joint_step_response(
            dyn=self.dyn,
            controller=self.controller,
            q_init=q_init,
            qdot_init=qdot_init,
            q_ref=q_ref,
            dt=0.02,
            t_final=5.0,
        )

        idx = 0
        n = len(ts)
        # Manual animation loop; window stays interactive because of interactive_update=True
        while self.plotter.is_active:
            self._update_arm_geometry(qs[idx])
            idx = (idx + 1) % n
            self.plotter.render()
            time.sleep(0.01)

    def run_cartesian_demo(self) -> None:
        """
        Differential-kinematics-based Cartesian tracking demo in 3D.

        A lifted circular path in a horizontal plane is tracked by using
        differential IK on the tool position. The window remains interactive
        until the user closes it.
        """
        # Start interactive window (allows rotate/zoom/close)
        self.plotter.show(auto_close=False, interactive_update=True)

        q = np.zeros(4, dtype=float)
        q_dot = np.zeros(4, dtype=float)
        dt = 0.02

        center = np.array([self.arm.l2 + self.arm.l3, 0.0, self.arm.l1])
        radius = 0.5 * min(self.arm.l2, self.arm.l3)
        t_final = 8.0

        # Precompute and show the lifted reference path.
        steps = int(t_final / dt)
        ts = np.linspace(0.0, t_final, steps)
        path_points = []
        for t in ts:
            angle = 2.0 * np.pi * (t / t_final)
            xd = center + radius * np.array([np.cos(angle), np.sin(angle), 0.0])
            path_points.append(xd)
        path_points = np.array(path_points)
        path_cloud = pv.PolyData(path_points)
        self.plotter.add_mesh(
            path_cloud,
            color="red",
            point_size=8,
            render_points_as_spheres=True,
        )

        t = 0.0

        # Infinite animation loop (until window is closed)
        while self.plotter.is_active:
            angle = 2.0 * np.pi * (t / t_final)
            xd = center + radius * np.array([np.cos(angle), np.sin(angle), 0.0])

            state = self.arm.frame_state(q)
            x = state.tool_transform[:3, 3]

            v_des = (xd - x) / dt
            xdot_desired = np.zeros(6, dtype=float)
            xdot_desired[:3] = v_des

            q_dot = inverse_differential_kinematics(
                arm=self.arm,
                q=q,
                xdot_desired=xdot_desired,
                lam=1e-2,
            )
            q = q + dt * q_dot

            self._update_arm_geometry(q)

            t += dt
            if t >= t_final:
                t -= t_final

            self.plotter.render()
            time.sleep(0.01)


def main() -> None:
    sim = Arm4DOF3DSimulation()
    # Uncomment one of the demos to run:
    # sim.run_joint_step_demo()
    sim.run_cartesian_demo()


if __name__ == "__main__":
    main()
