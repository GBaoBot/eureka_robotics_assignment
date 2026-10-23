"""ComputePathToPose: plan from the current pose to the goal."""
from __future__ import annotations

import logging
import time
from typing import Optional

from navigation.path_planner.planner import Path, Planner, PlanningError
from drivers.data_types import RobotState

log = logging.getLogger(__name__)


class ComputePathToPose:
    def __init__(self, planner: Planner):
        self.planner = planner

    def run(self, state: RobotState, goal_xy: tuple[float, float], goal_yaw: Optional[float]) -> Optional[Path]:
        """The new path, or None if the planner finds no path."""
        t0 = time.monotonic()
        try:
            path = self.planner.compute_plan(state.pose, goal_xy, goal_yaw)
        except PlanningError as e:
            log.warning("planning failed: %s", e)
            return None
        log.info("planned %.2f m (%d poses) in %.0f ms", path.length(), len(path), 1000 * (time.monotonic() - t0))
        return path
