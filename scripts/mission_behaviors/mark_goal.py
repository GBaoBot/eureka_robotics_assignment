"""MarkGoal: tell the simulator the robot has arrived at a goal.

Only the first mark per goal after a load/reset counts, so call it only after
FollowPath has confirmed the pose is within the goal tolerance.
"""
from __future__ import annotations

import logging

from drivers.rpc_client import RpcClient, RpcError

log = logging.getLogger(__name__)


class MarkGoal:
    def __init__(self, rpc: RpcClient):
        self.rpc = rpc

    def run(self, goal_name: str) -> bool:
        try:
            t = self.rpc.mark_goal(goal_name)
        except RpcError as e:
            log.error("mark_goal %s failed: %s", goal_name, e)
            return False
        log.info("marked goal %s at t=%.2f", goal_name, t)
        return True
