"""IsPathValid: can the current path still be followed?

Not valid when there is no path, the costmap changed and the rest of the path now
goes through an obstacle, or the robot has drifted more than max_deviation from it.
"""
from __future__ import annotations

import logging
import math
from typing import Optional

from vacuum.navigation.controller.controller import Controller
from vacuum.navigation.path_planner.planner import Path, Planner
from vacuum.drivers.data_types import RobotState

log = logging.getLogger(__name__)


class IsPathValid:
    def __init__(self, planner: Planner, controller: Controller, max_deviation: float = 0.3):
        self.planner = planner
        self.controller = controller
        self.max_deviation = max_deviation

    def run(self, path: Optional[Path], state: RobotState, map_changed: bool) -> bool:
        if path is None:
            return False

        # Only the part ahead of the robot matters; until FollowPath has taken the path, that's all of it.
        index = self.controller.get_progress_index() if self.controller.path is path else 0
        if map_changed and not self.planner.validate_path(path, start_index=index):
            log.info("path blocked")
            return False

        p = path.poses[index]
        if math.hypot(p.x - state.pose.x, p.y - state.pose.y) > self.max_deviation:
            log.info("robot off path")
            return False
        return True
