"""GetPoseFromGoal: look up a scene goal by name."""
from __future__ import annotations

import logging
from typing import Optional

from vacuum.drivers.data_types import Goal
from vacuum.drivers.rpc_client import RpcClient, RpcError

log = logging.getLogger(__name__)


class GetPoseFromGoal:
    def __init__(self, rpc: RpcClient):
        self.rpc = rpc
        self._goals: Optional[list[Goal]] = None  # cached: goals are fixed per scene

    def run(self, goal_name: str) -> Optional[tuple[tuple[float, float], Optional[float]]]:
        """((x, y), yaw-or-None) for the goal, or None if it can't be found."""
        try:
            if self._goals is None:
                self._goals = self.rpc.get_goals()
        except RpcError as e:
            log.warning("get_goals failed: %s", e)
            return None

        for g in self._goals:
            if g.name == goal_name:
                log.info("goal %s: (%.2f, %.2f) yaw %s", goal_name, g.x, g.y, g.yaw)
                return (g.x, g.y), g.yaw

        log.error("unknown goal %r; scene goals: %s", goal_name, [g.name for g in self._goals])
        return None
