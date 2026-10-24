"""GetRobotState: the next robot state with the localized pose; the first node of every tick.

Only reads Localization (which the driver feeds continuously); it never commands the robot,
so what to do on a stall (stop) is the caller's decision.
"""
from __future__ import annotations

import logging
from typing import Optional

from vacuum.drivers.data_types import RobotState
from vacuum.localization.localization import Localization

log = logging.getLogger(__name__)


class GetRobotState:
    def __init__(self, localization: Localization, state_timeout: float = 1.0, link_timeout: float = 5.0):
        """state_timeout: s to wait for a new state before giving up this tick.
        link_timeout: s without any state before the link counts as lost."""
        self.localization = localization
        self.state_timeout = state_timeout
        self.link_timeout = link_timeout

    def run(self, newer_than: int = -1) -> Optional[RobotState]:
        """The next state with seq > newer_than, or None if none came within state_timeout.

        Raises ConnectionError after link_timeout without any state.
        """
        try:
            return self.localization.wait_for(timeout=self.state_timeout, newer_than=newer_than).state
        except TimeoutError:
            if self.localization.state_age() > self.link_timeout:
                raise ConnectionError(f"no robot state for {self.link_timeout:.0f} s")
            log.warning("robot state stream stalled")
            return None
