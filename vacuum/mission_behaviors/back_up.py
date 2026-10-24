"""BackUp: after a collision, drive away from the contact so the robot can turn and replan.

start() once, then run() every tick until it returns True.
"""
from __future__ import annotations

import time
from typing import Optional

from vacuum.navigation.controller.controller import VelocityCmd
from vacuum.drivers.robot_driver import RobotDriver


class BackUp:
    def __init__(self, robot: RobotDriver, distance: float = 0.08, speed: float = 0.08):
        self.robot = robot
        self.distance = distance
        self.speed = speed
        self.last_cmd = VelocityCmd(0.0, 0.0)
        self._end: Optional[float] = None

    def start(self, bumped_cmd: VelocityCmd) -> None:
        """bumped_cmd: the command that was active at the collision; we drive the opposite way."""
        v = -self.speed if bumped_cmd.v >= 0.0 else self.speed
        self.last_cmd = VelocityCmd(v, 0.0)
        self._end = time.monotonic() + self.distance / self.speed

    def run(self) -> bool:
        """Keep backing up; True when done (robot stopped)."""
        if self._end is None or time.monotonic() >= self._end:
            self.stop()
            return True
        self.robot.set_velocity(self.last_cmd.v, self.last_cmd.w)  # resent every tick for the dead-man
        return False

    def stop(self) -> None:
        self.robot.stop()
        self.last_cmd = VelocityCmd(0.0, 0.0)
        self._end = None
