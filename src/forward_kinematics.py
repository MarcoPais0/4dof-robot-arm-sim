import numpy as np
from dataclasses import dataclass
from typing import Sequence


@dataclass
class DHLink:
    """
    Standard Denavit–Hartenberg (DH) parameters for a single revolute joint.

    All joints are assumed revolute. The parameters (a, alpha, d, theta_offset)
    are defined using the standard DH convention for T_(i-1)^i. The actual
    joint angle is theta_i = q_i + theta_offset_i.
    """

    a: float
    alpha: float
    d: float
    theta_offset: float = 0.0


@dataclass(frozen=True)
class ArmFrameState:
    """
    Frame-wise forward-kinematics result for the 4R chain.

    Attributes
    ----------
    joint_transforms : tuple of ndarray
        Ordered base-to-joint transforms for frames {1}, {2}, {3}, and {4}.
    tool_transform : (4, 4) ndarray
        Homogeneous transform from the base frame to the tool frame.
    joint_points : (5, 3) ndarray
        Frame origins ordered as [base, 1, 2, 3, 4].
    tool_point : (3,) ndarray
        Origin of the tool frame expressed in the base frame.
    """

    joint_transforms: tuple[np.ndarray, ...]
    tool_transform: np.ndarray
    joint_points: np.ndarray
    tool_point: np.ndarray

    @property
    def joint_4_transform(self) -> np.ndarray:
        """
        Homogeneous transform T_0^4 for the final joint frame.
        """
        return self.joint_transforms[-1]


def dh_transform(a: float, alpha: float, d: float, theta: float) -> np.ndarray:
    """
    Compute the standard DH homogeneous transform T_(i-1)^i.

    Parameters
    ----------
    a : float
        DH link length a_i.
    alpha : float
        DH link twist alpha_i.
    d : float
        DH link offset d_i.
    theta : float
        DH joint angle theta_i.

    Returns
    -------
    T : (4, 4) ndarray
        Homogeneous transformation matrix from frame {i-1} to frame {i}.
    """
    ct = np.cos(theta)
    st = np.sin(theta)
    ca = np.cos(alpha)
    sa = np.sin(alpha)

    return np.array(
        [
            [ct, -st * ca, st * sa, a * ct],
            [st, ct * ca, -ct * sa, a * st],
            [0.0, sa, ca, d],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=float,
    )


class Arm4DOFDH:
    """
    Spatial 4DOF revolute manipulator defined by the Topic 1/2 model.

    This is the authoritative geometry definition for:
    - frames {0}, {1}, {2}, {3}, {4}, and {tool},
    - four revolute joints (R-R-R-R),
    - the standard DH links T_(i-1)^i,
    - the explicit tool transform T_4^tool.

    The arm configuration is:
        q = [q1, q2, q3, q4]^T

    The default Topic 2 DH table (standard convention) uses:
        a_i     = [0, L2, L3, 0]
        alpha_i = [pi/2, 0, pi/2, 0]
        d_i     = [L1, 0, 0, 0]
        theta_i = q_i + theta_offset_i
        T_4^tool = Trans_z(L_tool), with L_tool > 0

    Aligning the tool offset with z4 makes joint 4 a roll joint for the tool:
    changing q4 changes the tool orientation but does not move the tool origin
    when q1, q2, and q3 are fixed. This is a partial decoupling because the
    first three joints still set the position and direction of z4.
    """

    BASE_FRAME = "0"
    JOINT_FRAMES = ("1", "2", "3", "4")
    TOOL_FRAME = "tool"

    def __init__(
        self,
        l1: float = 4.0,
        l2: float = 3.0,
        l3: float = 2.0,
        l_tool: float = 1.0,
        theta_offsets: Sequence[float] | None = None,
        T_4_tool: np.ndarray | None = None,
    ) -> None:
        self.l1 = float(l1)
        self.l2 = float(l2)
        self.l3 = float(l3)
        self.l_tool = float(l_tool)
        if min(self.l1, self.l2, self.l3, self.l_tool) <= 0.0:
            raise ValueError("All geometry lengths must be strictly positive.")

        if theta_offsets is None:
            theta_offsets = (0.0, 0.0, 0.0, 0.0)
        if len(theta_offsets) != 4:
            raise ValueError("theta_offsets must contain exactly four values.")

        self.theta_offsets = tuple(float(offset) for offset in theta_offsets)
        self.links = [
            DHLink(a=0.0, alpha=np.pi / 2.0, d=self.l1, theta_offset=self.theta_offsets[0]),
            DHLink(a=self.l2, alpha=0.0, d=0.0, theta_offset=self.theta_offsets[1]),
            DHLink(a=self.l3, alpha=np.pi / 2.0, d=0.0, theta_offset=self.theta_offsets[2]),
            DHLink(a=0.0, alpha=0.0, d=0.0, theta_offset=self.theta_offsets[3]),
        ]
        if T_4_tool is None:
            T_4_tool = np.eye(4, dtype=float)
            T_4_tool[2, 3] = self.l_tool

        self.T_4_tool = np.asarray(T_4_tool, dtype=float).copy()
        if self.T_4_tool.shape != (4, 4):
            raise ValueError("T_4_tool must have shape (4, 4).")
        if not np.all(np.isfinite(self.T_4_tool)):
            raise ValueError("T_4_tool must contain only finite values.")
        if not np.allclose(self.T_4_tool[3], np.array([0.0, 0.0, 0.0, 1.0])):
            raise ValueError("T_4_tool must be a homogeneous transform.")
        R_4_tool = self.T_4_tool[:3, :3]
        if not np.allclose(R_4_tool.T @ R_4_tool, np.eye(3)):
            raise ValueError("T_4_tool rotation must be orthonormal.")
        if not np.isclose(np.linalg.det(R_4_tool), 1.0):
            raise ValueError("T_4_tool rotation must be right-handed.")
        if not np.allclose(R_4_tool[:, 2], np.array([0.0, 0.0, 1.0])):
            raise ValueError("T_4_tool must keep the tool z-axis aligned with z4.")
        tool_offset = self.T_4_tool[:3, 3]
        if not np.allclose(tool_offset[:2], np.zeros(2)) or tool_offset[2] <= 0.0:
            raise ValueError("T_4_tool translation must be positive along z4.")

    @property
    def dof(self) -> int:
        return len(self.links)

    def frame_state(self, q: Sequence[float]) -> ArmFrameState:
        """
        Evaluate the base, joint, and tool frames for a joint configuration.

        Parameters
        ----------
        q : array_like, shape (4,)
            Joint angles [q1, q2, q3, q4].

        Returns
        -------
        ArmFrameState
            Ordered joint-frame transforms, joint frame origins, and the tool
            transform/origin with respect to the base frame.
        """
        q_arr = np.asarray(q, dtype=float)
        if q_arr.shape != (self.dof,):
            raise ValueError(f"Expected {self.dof} joint angles, got shape {q_arr.shape}.")

        T = np.eye(4)
        joint_transforms = []
        points = [T[:3, 3].copy()]
        for qi, link in zip(q_arr, self.links):
            theta = qi + link.theta_offset
            T = T @ dh_transform(link.a, link.alpha, link.d, theta)
            joint_transforms.append(T.copy())
            points.append(T[:3, 3].copy())

        tool_transform = joint_transforms[-1] @ self.T_4_tool
        joint_points = np.stack(points, axis=0)
        tool_point = tool_transform[:3, 3].copy()

        return ArmFrameState(
            joint_transforms=tuple(joint_transforms),
            tool_transform=tool_transform,
            joint_points=joint_points,
            tool_point=tool_point,
        )


def main() -> None:
    """
    Small sanity check: print the joint-4 and tool transforms for a sample q.
    """
    arm = Arm4DOFDH()
    q = np.deg2rad([30.0, 20.0, -15.0, 40.0])
    state = arm.frame_state(q)
    np.set_printoptions(precision=3, suppress=True)
    print("T_0_4(q):")
    print(state.joint_4_transform)
    print("T_0_tool(q):")
    print(state.tool_transform)


if __name__ == "__main__":
    main()
