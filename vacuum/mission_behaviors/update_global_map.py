"""UpdateGlobalMap: add obstacles discovered at runtime to the global costmap.

Bumper: the robot only collides while translating (turning in place is always
possible), so the contact is ahead of it when driving forward, behind when reversing.
Hook for later: obstacles from the depth-camera local costmap go here too.
"""
from __future__ import annotations

import logging
import math

from vacuum.navigation.controller.controller import VelocityCmd
from vacuum.navigation.costmap import Costmap2D
from vacuum.drivers.data_types import RobotState

log = logging.getLogger(__name__)


class UpdateGlobalMap:
    def __init__(self, costmap: Costmap2D, robot_radius: float = 0.17, obstacle_radius: float = 0.04):
        self.costmap = costmap
        self.robot_radius = robot_radius
        self.obstacle_radius = obstacle_radius
        self._was_bumped = False

    def run(self, state: RobotState, last_cmd: VelocityCmd) -> bool:
        """Returns True if the costmap changed this call."""
        changed = False
        if state.bumper and not self._was_bumped:  # mark once per contact, not every tick it stays pressed
            pose = state.pose
            direction = 1.0 if last_cmd.v >= 0.0 else -1.0
            d = direction * (self.robot_radius + self.obstacle_radius)
            x, y = pose.x + d * math.cos(pose.yaw), pose.y + d * math.sin(pose.yaw)
            self.costmap.add_obstacle(x, y, self.obstacle_radius)
            log.info("bumper: marked obstacle at (%.2f, %.2f)", x, y)
            changed = True
        self._was_bumped = state.bumper
        return changed
