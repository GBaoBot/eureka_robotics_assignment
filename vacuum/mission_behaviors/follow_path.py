"""FollowPath: drive along a path with the controller, one control step per run().

When the controller reports done, the robot stops and the pose is checked against
the simulator's goal criteria (5 cm, 5 deg): ARRIVED if it would count, MISSED otherwise.
A different path object (replan) is handed to the controller automatically.
"""
from __future__ import annotations

import enum
import logging
import math
import time
from typing import Optional

from vacuum.navigation.controller.controller import Controller, VelocityCmd
from vacuum.navigation.path_planner.planner import Path
from vacuum.common.data_types import Pose
from vacuum.drivers.robot_driver import RobotDriver
from vacuum.drivers.data_types import RobotState

log = logging.getLogger(__name__)

# The simulator counts a goal as reached within 5 cm and 5 deg (assignment brief).
GOAL_XY_TOLERANCE = 0.05
GOAL_YAW_TOLERANCE = math.radians(5.0)


class FollowResult(enum.Enum):
    RUNNING = "running"
    ARRIVED = "arrived"
    MISSED = "missed"  # controller finished but outside the goal tolerance


def is_at_goal(pose: Pose, goal_xy: tuple[float, float], goal_yaw: Optional[float]) -> bool:
    if math.hypot(goal_xy[0] - pose.x, goal_xy[1] - pose.y) > GOAL_XY_TOLERANCE:
        return False
    if goal_yaw is None:
        return True
    err = math.atan2(math.sin(goal_yaw - pose.yaw), math.cos(goal_yaw - pose.yaw))
    return abs(err) <= GOAL_YAW_TOLERANCE


class FollowPath:
    def __init__(self, robot: RobotDriver, controller: Controller):
        self.robot = robot
        self.controller = controller
        self.last_cmd = VelocityCmd(0.0, 0.0)
        self._last_tick: Optional[float] = None

    def run(self, path: Path, state: RobotState) -> FollowResult:
        if self.controller.path is not path:  # new or replanned path
            self.controller.set_plan(path)

        now = time.monotonic()
        dt = 0.0 if self._last_tick is None else now - self._last_tick
        self._last_tick = now

        self.last_cmd = self.controller.compute_velocity_cmd(state.pose, dt)
        self.robot.set_velocity(self.last_cmd.v, self.last_cmd.w)
        if not self.controller.is_goal_reached():
            return FollowResult.RUNNING

        self.stop()
        goal = path.poses[-1]
        if is_at_goal(state.pose, (goal.x, goal.y), path.goal_yaw):
            return FollowResult.ARRIVED
        log.warning("controller finished but pose is outside the goal tolerance")
        return FollowResult.MISSED

    def stop(self) -> None:
        self.robot.stop()
        self.last_cmd = VelocityCmd(0.0, 0.0)
        self._last_tick = None
