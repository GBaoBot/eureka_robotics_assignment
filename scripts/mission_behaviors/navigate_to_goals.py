"""NavigateToGoals: drive the robot to named scene goals in order.

Each goal is a NavigateToGoal run (navigate_to_goal.py); a failed goal is skipped.
Front ends don't use this directly: they go through MissionExecutor.
"""
from __future__ import annotations

import logging
import threading
from typing import Optional

from mission_behaviors.navigate_to_goal import GoalResult, NavigateToGoal

log = logging.getLogger("navigate_to_goals")


class NavigateToGoals:
    """Visit goals in order with one NavigateToGoal, so the map it learns carries over."""

    def __init__(self, navigate_to_goal: NavigateToGoal):
        self.navigate_to_goal = navigate_to_goal

    def run(self, goal_names: list[str], cancel: Optional[threading.Event] = None) -> dict[str, GoalResult]:
        """Result per attempted goal. A failed goal doesn't stop the mission; `cancel` and a
        lost link (ConnectionError) do."""
        results = {}
        for name in goal_names:
            log.info("---- goal %s", name)
            results[name] = self.navigate_to_goal.run(name, cancel)
            log.info("---- goal %s: %s", name, results[name].value)
            if results[name] is GoalResult.CANCELLED:
                break
        return results
