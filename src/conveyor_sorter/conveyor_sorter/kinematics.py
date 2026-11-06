"""Pure-python kinematics + belt math (no ROS imports, so it is unit-testable).

Arm model (matches urdf/robot_arm.urdf.xacro exactly):
  base (world x,y, floor) -> pedestal up to base_z
  joint1: yaw about +Z at height base_z
  link1 : vertical column, shoulder_h tall
  joint2: pitch about +Y (positive pitch tilts the arm DOWN)
  link2 : upper arm, l2 long, along +X
  joint3: pitch about +Y (positive = forearm folds DOWN relative to upper arm)
  link3 : forearm + gripper, l3 long (tool tip at the very end)
"""

import math
from dataclasses import dataclass
from typing import List, Optional, Tuple

JOINT_LIMIT = 3.14


@dataclass(frozen=True)
class ArmGeometry:
    base_x: float
    base_y: float
    base_z: float = 0.5        # top of pedestal (joint1 height)
    shoulder_h: float = 0.25   # joint1 -> joint2 column height
    l2: float = 0.45
    l3: float = 0.42

    @property
    def shoulder_z(self) -> float:
        return self.base_z + self.shoulder_h

    @property
    def max_reach(self) -> float:
        return self.l2 + self.l3


def fk(q: List[float], g: ArmGeometry) -> Tuple[float, float, float]:
    """Tool-tip position in world frame for joint angles q=[q1,q2,q3]."""
    q1, q2, q3 = q
    phi2 = -q2              # elevation of upper arm
    phi3 = -(q2 + q3)       # elevation of forearm
    r = g.l2 * math.cos(phi2) + g.l3 * math.cos(phi3)
    h = g.l2 * math.sin(phi2) + g.l3 * math.sin(phi3)
    return (g.base_x + r * math.cos(q1),
            g.base_y + r * math.sin(q1),
            g.shoulder_z + h)


def elbow_xyz(q: List[float], g: ArmGeometry) -> Tuple[float, float, float]:
    q1, q2, _ = q
    phi2 = -q2
    r = g.l2 * math.cos(phi2)
    return (g.base_x + r * math.cos(q1),
            g.base_y + r * math.sin(q1),
            g.shoulder_z + g.l2 * math.sin(phi2))


def ik(x: float, y: float, z: float, g: ArmGeometry) -> Optional[List[float]]:
    """Analytic elbow-up IK. Returns [q1,q2,q3] or None if unreachable."""
    dx, dy = x - g.base_x, y - g.base_y
    r = math.hypot(dx, dy)
    dz = z - g.shoulder_z
    d2 = r * r + dz * dz
    d = math.sqrt(d2)
    if d > g.l2 + g.l3 - 1e-6 or d < abs(g.l2 - g.l3) + 1e-6:
        return None
    c = (d2 - g.l2 ** 2 - g.l3 ** 2) / (2.0 * g.l2 * g.l3)
    c = max(-1.0, min(1.0, c))
    rel = -math.acos(c)  # forearm bends down relative to upper arm (elbow up)
    phi2 = math.atan2(dz, r) - math.atan2(g.l3 * math.sin(rel), g.l2 + g.l3 * math.cos(rel))
    q1 = math.atan2(dy, dx) if r > 1e-6 else 0.0
    q2 = -phi2
    q3 = -rel
    q = [q1, q2, q3]
    if any(abs(v) > JOINT_LIMIT for v in q):
        return None
    return q


def belt_x(x0: float, t0: float, speed: float, t: float) -> float:
    """Feed-forward belt position: the belt never stops, so x is analytic."""
    return x0 + speed * (t - t0)


def smoothstep(s: float) -> float:
    s = max(0.0, min(1.0, s))
    return s * s * (3.0 - 2.0 * s)
