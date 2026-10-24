"""Local controller interface (Nav2's nav2_core::Controller, simplified).

A controller follows a Path: every control cycle it gets the current pose and
returns a velocity command. It does not plan; replanning is the caller's job.
"""
from __future__ import annotations

import math
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Optional

from navigation.path_planner.planner import Path
from common.data_types import Pose


@dataclass(frozen=True)
class VelocityCmd:
    v: float  # m/s, forward
    w: float  # rad/s, counter-clockwise


class Controller(ABC):
    def __init__(self, xy_goal_tolerance: float = 0.02, yaw_goal_tolerance: float = 0.03):
        """Goal tolerances (m, rad). Keep them well inside the 5 cm / 5 deg the simulator accepts."""
        self.xy_goal_tolerance = xy_goal_tolerance
        self.yaw_goal_tolerance = yaw_goal_tolerance
        self.path: Optional[Path] = None
        self.speed_limit = 1.0  # fraction of the controller's max speeds

    def set_plan(self, path: Path) -> None:
        """Start following a new path (replaces the current one)."""
        if len(path) == 0:
            raise ValueError("empty path")
        self.path = path
        self.reset()

    @abstractmethod
    def compute_velocity_cmd(self, pose: Pose, dt: float) -> VelocityCmd:
        """Velocity to send now. dt: seconds since the previous call (for acceleration limits)."""

    @abstractmethod
    def is_goal_reached(self) -> bool:
        """True once the robot is at the end of the path within tolerance (and heading, if required)."""

    def get_progress_index(self) -> int:
        """Index of the path pose the robot has reached; the part before it is behind the robot."""
        return 0

    def reset(self) -> None:
        """Clear internal state (progress along the path, last command). Called by set_plan()."""

    def set_speed_limit(self, fraction: float) -> None:
        """Scale the max speeds, e.g. 0.5 near obstacles. Clamped to (0, 1]."""
        self.speed_limit = min(1.0, max(1e-3, fraction))

    # ----------------------------------------------------------------- helpers

    @staticmethod
    def normalize_angle(a: float) -> float:
        """Wrap to (-pi, pi]."""
        a = math.fmod(a + math.pi, 2.0 * math.pi)
        if a <= 0.0:
            a += 2.0 * math.pi
        return a - math.pi
