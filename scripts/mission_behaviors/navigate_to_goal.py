"""NavigateToGoal: drive the robot to one named scene goal and mark it.

Runs the behaviours in mission_behaviors/ once per robot state message (50 Hz),
like a simplified Nav2 NavigateToPose behaviour tree:

    UpdateGlobalMap -> CheckCollision ? BackUp (then replan)
                    -> IsPathValid    ? else ComputePathToPose
                    -> FollowPath     -> ARRIVED: MarkGoal / MISSED: replan

Poses come from Localization (one tick per localized state); commands go to the driver.
The costmap lives as long as this object, so obstacles found while driving to one
goal are still avoided on the way to the next (see NavigateToGoals).
Progress is published to self.telemetry for viewers.
"""
from __future__ import annotations

import enum
import logging
import threading
import time
from dataclasses import dataclass
from typing import Optional

from drivers.robot_driver import RobotDriver
from drivers.rpc_client import RpcClient
from localization.localization import Localization
from mission_behaviors.back_up import BackUp
from mission_behaviors.check_collision import CheckCollision
from mission_behaviors.compute_path_to_pose import ComputePathToPose
from mission_behaviors.follow_path import FollowPath, FollowResult
from mission_behaviors.get_pose_from_goal import GetPoseFromGoal
from mission_behaviors.is_path_valid import IsPathValid
from mission_behaviors.mark_goal import MarkGoal
from mission_behaviors.telemetry import Telemetry
from mission_behaviors.update_global_map import UpdateGlobalMap
from navigation.controller.pure_pursuit_controller import PurePursuitController
from navigation.costmap import Costmap2D
from navigation.path_planner.astar import AStarPlanner
from navigation.path_planner.planner import Path

log = logging.getLogger("navigate_to_goal")


class GoalResult(enum.Enum):
    SUCCEEDED = "succeeded"
    FAILED = "failed"
    CANCELLED = "cancelled"


@dataclass
class MissionLimits:
    goal_timeout: float = 180.0  # s per goal
    max_recoveries: int = 10  # back-ups after collisions, per goal
    max_planning_failures: int = 5  # consecutive, per goal
    max_missed_arrivals: int = 3  # controller done but outside the goal tolerance
    replan_backoff: float = 0.5  # s to wait after a failed plan before trying again
    state_timeout: float = 1.0  # s without a state message before the robot is stopped
    link_timeout: float = 5.0  # s without a state message before the mission aborts


class NavigateToGoal:
    def __init__(self, rpc: RpcClient, robot: RobotDriver, localization: Localization,
                 limits: Optional[MissionLimits] = None):
        """robot: where commands go. localization: where poses come from."""
        self.rpc = rpc
        self.robot = robot
        self.localization = localization
        self.limits = limits or MissionLimits()

        self.costmap = Costmap2D(rpc.get_map())
        self.planner = AStarPlanner()
        self.planner.set_map(self.costmap)  # keeps a reference: bumper obstacles are seen on replan
        self.controller = PurePursuitController()

        self.get_pose_from_goal = GetPoseFromGoal(rpc)
        self.update_global_map = UpdateGlobalMap(self.costmap)
        self.check_collision = CheckCollision()
        self.back_up = BackUp(robot)
        self.is_path_valid = IsPathValid(self.planner, self.controller)
        self.compute_path_to_pose = ComputePathToPose(self.planner)
        self.follow_path = FollowPath(robot, self.controller)
        self.mark_goal = MarkGoal(rpc)

        self.telemetry = Telemetry(self.costmap)
        self.path: Optional[Path] = None  # current plan

    def run(self, goal_name: str, cancel: Optional[threading.Event] = None) -> GoalResult:
        """Drive to the goal and mark it; returns CANCELLED soon after `cancel` is set.

        Raises ConnectionError if the robot state stream is lost.
        """
        t = self.telemetry
        t.goal_name, t.goal, t.path, t.reason = goal_name, None, None, ""
        try:
            result = self._navigate(goal_name, cancel or threading.Event())
        finally:
            self.back_up.stop()
            self.follow_path.stop()
            t.cmd = self.follow_path.last_cmd
        t.status = result.value
        t.results = t.results + ((goal_name, result.value),)
        return result

    def _navigate(self, goal_name: str, cancel: threading.Event) -> GoalResult:
        goal = self.get_pose_from_goal.run(goal_name)
        if goal is None:
            return self._fail(f"unknown goal {goal_name!r}")
        goal_xy, goal_yaw = goal
        t = self.telemetry
        t.goal = (*goal_xy, goal_yaw)
        self._set_status("planning", "new goal")

        lim = self.limits
        deadline = time.monotonic() + lim.goal_timeout
        self._set_path(None)
        backing_up = False
        recoveries = planning_failures = missed = 0
        next_plan_time = 0.0
        replan_reason = "new goal"  # why self.path is None, shown when replanning
        seq = -1

        while True:
            if cancel.is_set():
                self._set_status("cancelled", "stopped by user")
                return GoalResult.CANCELLED
            if time.monotonic() > deadline:
                return self._fail(f"timed out after {lim.goal_timeout:.0f} s")

            state = self._next_state(seq)
            if state is None:
                continue
            seq = state.seq
            last_cmd = self.back_up.last_cmd if backing_up else self.follow_path.last_cmd
            t.cmd = last_cmd

            map_changed = self.update_global_map.run(state, last_cmd)
            if map_changed:
                t.map_version += 1

            # Collision: back away from the contact, then plan again around the new obstacle.
            if not backing_up and self.check_collision.run(state):
                recoveries += 1
                if recoveries > lim.max_recoveries:
                    return self._fail(f"too many collisions ({recoveries - 1})")
                t.collisions = t.collisions + ((state.pose.x, state.pose.y),)
                self.follow_path.stop()
                self.back_up.start(last_cmd)
                self._set_path(None)
                replan_reason = f"collision at ({state.pose.x:.2f}, {state.pose.y:.2f})"
                self._set_status("backing up", replan_reason)
                backing_up = True
            if backing_up:
                backing_up = not self.back_up.run()
                continue

            # Invalid Path
            if not self.is_path_valid.run(self.path, state, map_changed):
                if self.path is not None:  # IsPathValid only rejects an existing path for these two
                    replan_reason = "path blocked by a new obstacle" if map_changed else "robot off path"
                self.robot.stop()
                self._set_path(None)
                if time.monotonic() < next_plan_time:
                    continue
                self._set_status("planning", replan_reason)
                self._set_path(self.compute_path_to_pose.run(state, goal_xy, goal_yaw))
                if self.path is None:
                    planning_failures += 1
                    if planning_failures >= lim.max_planning_failures:
                        return self._fail(f"no path after {planning_failures} attempts")
                    replan_reason = f"no path found (attempt {planning_failures})"
                    next_plan_time = time.monotonic() + lim.replan_backoff
                    continue
                planning_failures = 0
                self._set_status("following", replan_reason)

            # Follow Path
            result = self.follow_path.run(self.path, state)
            if result is FollowResult.ARRIVED:
                # Pose was checked against the 5 cm / 5 deg criteria, so this mark should count.
                if self.mark_goal.run(goal_name):
                    self.telemetry.reason = "arrived and marked"
                    return GoalResult.SUCCEEDED
                return self._fail("mark_goal failed")
            if result is FollowResult.MISSED:
                missed += 1
                if missed >= lim.max_missed_arrivals:
                    return self._fail(f"stopped outside the goal tolerance {missed} times")
                replan_reason = "stopped outside the goal tolerance"
                self._set_path(None)  # replan from where the robot stopped

    def _set_path(self, path: Optional[Path]) -> None:
        self.path = self.telemetry.path = path

    def _set_status(self, status: str, reason: str) -> None:
        self.telemetry.status, self.telemetry.reason = status, reason

    def _fail(self, reason: str) -> GoalResult:
        log.error("goal %s: %s", self.telemetry.goal_name, reason)
        self.telemetry.reason = reason
        return GoalResult.FAILED

    def _next_state(self, seq: int):
        """Next robot state with the localized pose, or None after state_timeout (robot stopped).

        Raises ConnectionError on link_timeout.
        """
        try:
            return self.localization.wait_for(timeout=self.limits.state_timeout, newer_than=seq).state
        except TimeoutError:
            self.robot.stop()
            if self.localization.state_age() > self.limits.link_timeout:
                raise ConnectionError(f"no robot state for {self.limits.link_timeout:.0f} s")
            log.warning("robot state stream stalled, robot stopped")
            return None
