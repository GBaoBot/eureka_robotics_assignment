"""Global planner interface (Nav2's nav2_core::GlobalPlanner, simplified).

A planner turns a start pose and a goal into a collision-free Path on a Costmap2D.
Implementations override compute_plan(); everything else is shared.
"""
from __future__ import annotations

import math
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Optional

from navigation.costmap import INSCRIBED, Costmap2D
from drivers.data_types import Pose


class PlanningError(RuntimeError):
    """No valid path (start/goal in collision, goal unreachable, no map)."""


@dataclass
class Path:
    poses: list[Pose]  # start -> goal, roughly uniform spacing; yaw = direction of travel
    goal_yaw: Optional[float] = None  # required final heading, None = any

    def __len__(self) -> int:
        return len(self.poses)

    def length(self) -> float:
        return sum(math.hypot(b.x - a.x, b.y - a.y) for a, b in zip(self.poses, self.poses[1:]))


class Planner(ABC):
    def __init__(self, path_spacing: float = 0.05):
        """path_spacing: distance (m) between consecutive poses of the returned path."""
        self.path_spacing = path_spacing
        self.costmap: Optional[Costmap2D] = None

    def set_map(self, costmap: Costmap2D) -> None:
        """Give the planner the costmap to plan on. The planner keeps a reference, so
        obstacles added to the costmap later are seen by the next compute_plan()."""
        self.costmap = costmap

    @abstractmethod
    def compute_plan(self, start: Pose, goal_xy: tuple[float, float],
                     goal_yaw: Optional[float] = None) -> Path:
        """Plan from start to goal. Raises PlanningError if no path exists."""

    def validate_path(self, path: Path, start_index: int = 0) -> bool:
        """True if the path from start_index to the end is still collision-free on the current costmap.

        Use it to decide whether to replan after the costmap has changed.
        """
        costmap = self._require_costmap()
        poses = path.poses[start_index:]
        for a, b in zip(poses, poses[1:]):
            if costmap.max_cost_on_segment(a.x, a.y, b.x, b.y) >= INSCRIBED:
                return False
        return len(poses) > 0 and not costmap.in_collision(poses[-1].x, poses[-1].y)

    # ----------------------------------------------------------------- helpers for implementations

    def _require_costmap(self) -> Costmap2D:
        if self.costmap is None:
            raise PlanningError("no map: call set_map() first")
        return self.costmap

    def _make_path(self, points: list[tuple[float, float]], goal_yaw: Optional[float]) -> Path:
        """Resample a polyline to path_spacing and attach headings."""
        dense: list[tuple[float, float]] = [points[0]]
        for (x0, y0), (x1, y1) in zip(points, points[1:]):
            seg = math.hypot(x1 - x0, y1 - y0)
            n = max(1, int(math.ceil(seg / self.path_spacing)))
            dense.extend((x0 + (x1 - x0) * i / n, y0 + (y1 - y0) * i / n) for i in range(1, n + 1))

        poses = []
        for i, (x, y) in enumerate(dense):
            if i + 1 < len(dense):
                nx, ny = dense[i + 1]
                yaw = math.atan2(ny - y, nx - x)
            elif goal_yaw is not None:
                yaw = goal_yaw
            else:
                yaw = poses[-1].yaw if poses else 0.0
            poses.append(Pose(x, y, yaw))
        return Path(poses, goal_yaw)
