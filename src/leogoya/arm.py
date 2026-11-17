"""SO-101 brush kinematics: canvas-space brush positions -> joint angles (degrees).

World frame is the URDF base frame: origin at the base, +x straight out in front of
the arm, +z up. The table the base sits on is z = 0. Units are metres and degrees,
which match what `SO101Follower.send_action()` expects with `use_degrees=True`.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np
from lerobot.model.kinematics import RobotKinematics

URDF = Path(__file__).resolve().parents[2] / "hardware" / "so101_urdf" / "so101_new_calib.urdf"
ARM_JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]

# A pose whose tool z-axis points straight down; used for the brush orientation.
_BRUSH_DOWN_REFERENCE = np.array([0.0, 0.0, 90.0, 0.0, 0.0])
# IK starting guess with the brush pointing down in front of the base.
HOME = np.array([0.0, 56.0, -63.0, 95.0, 0.0])


@dataclass
class Solve:
    joints: np.ndarray  # degrees, ARM_JOINTS order
    pos_error_mm: float
    tilt_deg: float  # brush angle away from vertical


class BrushArm:
    def __init__(self, brush_length: float = 0.0, urdf: Path = URDF):
        """brush_length: metres from `gripper_frame_link` (between the jaw tips) to the bristle tip."""
        self.kin = RobotKinematics(str(urdf), "gripper_frame_link", ARM_JOINTS)
        self.kin.solver.enable_joint_limits(True)
        self.brush_length = brush_length
        self._r_down = self.kin.forward_kinematics(_BRUSH_DOWN_REFERENCE)[:3, :3].copy()
        self.limits_deg = np.array(
            [np.degrees(self.kin.robot.get_joint_limits(j)) for j in ARM_JOINTS]
        )  # (5, 2) lower/upper

    def brush_tip(self, joints: np.ndarray) -> tuple[np.ndarray, float]:
        """Forward kinematics: bristle-tip position (m) and tilt from vertical (deg)."""
        t = self.kin.forward_kinematics(joints)
        tool_z = t[:3, 2]
        tip = t[:3, 3] + self.brush_length * tool_z
        tilt = float(np.degrees(np.arccos(np.clip(-tool_z[2], -1.0, 1.0))))
        return tip, tilt

    def target(self, tip: np.ndarray) -> np.ndarray:
        """4x4 gripper-frame pose that puts the bristle tip at `tip`, brush vertical.

        Yaw follows the shoulder pan so the wrist roll stays near zero.
        """
        yaw = np.arctan2(tip[1], tip[0])
        c, s = np.cos(yaw), np.sin(yaw)
        pose = np.eye(4)
        pose[:3, :3] = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]]) @ self._r_down
        pose[:3, 3] = tip + np.array([0.0, 0.0, self.brush_length])  # gripper frame sits above the tip
        return pose

    def solve(self, tip: np.ndarray, seed: np.ndarray, iters: int = 30) -> Solve:
        joints = self.kin.inverse_kinematics(
            seed, self.target(tip), position_weight=1.0, orientation_weight=0.5, max_iters=iters
        )
        reached, tilt = self.brush_tip(joints)
        return Solve(joints, float(np.linalg.norm(reached - tip) * 1000), tilt)

    def solve_path(self, tips: np.ndarray, seed: np.ndarray = HOME) -> list[Solve]:
        """Solve each point, warm-starting from the previous one so the arm moves smoothly."""
        out: list[Solve] = []
        first = self.solve(tips[0], seed, iters=200)
        out.append(first)
        for tip in tips[1:]:
            out.append(self.solve(tip, out[-1].joints))
        return out
