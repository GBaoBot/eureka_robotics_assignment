"""Telemetry: the latest mission state, for viewers (visualiser, logs, a remote client later).

Written only by the mission thread, read by anyone. Values are replaced, never mutated in
place (e.g. collisions = collisions + (xy,)), so a reader in another thread never sees a
half-updated value. Fields read one after another may come from different ticks, which is
fine for display; anything that acts on several fields together needs its own snapshot.
"""
from __future__ import annotations

from typing import Optional

from vacuum.navigation.controller.controller import VelocityCmd
from vacuum.navigation.costmap import Costmap2D
from vacuum.navigation.path_planner.planner import Path


class Telemetry:
    def __init__(self, costmap: Optional[Costmap2D] = None):
        self.costmap = costmap  # costmap.costs is replaced on every change, see map_version
        self.map_version = 0  # incremented whenever the costmap changes

        self.goal_name: Optional[str] = None
        self.goal: Optional[tuple[float, float, Optional[float]]] = None  # x, y, yaw-or-None
        self.path: Optional[Path] = None  # current plan; a new object on every replan
        self.cmd = VelocityCmd(0.0, 0.0)  # last velocity command sent
        self.status = "idle"  # idle / planning / following / backing up / succeeded / failed / cancelled
        self.reason = ""  # why the mission is in this status, for humans
        self.collisions: tuple[tuple[float, float], ...] = ()  # robot positions at bumper contact
        self.results: tuple[tuple[str, str], ...] = ()  # (goal name, result) per finished goal
