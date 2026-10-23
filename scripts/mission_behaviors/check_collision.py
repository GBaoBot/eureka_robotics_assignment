"""CheckCollision: has the robot hit something?"""
from __future__ import annotations

import logging

from drivers.data_types import RobotState

log = logging.getLogger(__name__)


class CheckCollision:
    def run(self, state: RobotState) -> bool:
        """True while the bumper is pressed."""
        if state.bumper:
            log.warning("collision at (%.2f, %.2f)", state.pose.x, state.pose.y)
        return state.bumper
