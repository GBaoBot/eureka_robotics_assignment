"""Localization: turns raw robot states into pose estimates, in its own thread (like a ROS 2 node).

    RobotDriver --RobotState (raw)--> Localization --LocalizedState--> navigation, GUI

It takes each state the driver receives, in order, runs the estimator (EkfLocalizer, or
PassThrough for the raw pose) and publishes the latest estimate. Consumers read it with
latest() or wait_for(newer_than), like subscribing to a topic; nothing else feeds it.
Commands don't go through here: they go to the driver.
"""
from __future__ import annotations

import dataclasses
import logging
import math
import threading
import time
from dataclasses import dataclass
from typing import Optional, Protocol

from drivers.data_types import Pose, RobotState
from drivers.robot_driver import RobotDriver

log = logging.getLogger(__name__)

SCENE_RESTART = 0.5  # s: sim time going back by more than this means load_scene / reset


class Estimator(Protocol):
    sigma: Optional[tuple[float, float]]

    def reset(self, pose: Optional[Pose] = None) -> None: ...
    def step(self, left_ticks: int, right_ticks: int, measured: Pose) -> Pose: ...


class PassThrough:
    """No filtering: the estimate is the simulator's pose."""
    sigma = None

    def reset(self, pose: Optional[Pose] = None) -> None:
        pass

    def step(self, left_ticks: int, right_ticks: int, measured: Pose) -> Pose:
        return measured


@dataclass(frozen=True)
class LocalizedState:
    state: RobotState  # the state as received, with its pose replaced by the estimate
    raw_pose: Pose  # the simulator's pose, for comparison
    sigma: Optional[tuple[float, float]]  # (position m, yaw rad) uncertainty; None without a filter


class Localization:
    def __init__(self, driver: RobotDriver, estimator: Optional[Estimator] = None,
                 initial_pose: Optional[Pose] = None):
        """estimator: None = PassThrough. initial_pose: a known start (e.g. load_scene's exact
        start pose); None = start from the first measurement."""
        self.driver = driver
        self.estimator = estimator or PassThrough()
        self.estimator.reset(initial_pose)
        self._cv = threading.Condition()
        self._latest: Optional[LocalizedState] = None
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._loop, name="localization", daemon=True)
        self._thread.start()

    # ----------------------------------------------------------------- lifecycle

    def __enter__(self) -> "Localization":
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    def close(self) -> None:
        """Stop the thread. Call before closing the driver."""
        self._stop.set()
        self._thread.join(timeout=1.0)

    def reset(self, pose: Optional[Pose] = None) -> None:
        """Restart the estimate, from a known pose or from the next measurement."""
        with self._cv:
            self.estimator.reset(pose)
            self._latest = None

    # ----------------------------------------------------------------- output

    def latest(self) -> Optional[LocalizedState]:
        with self._cv:
            return self._latest

    def wait_for(self, timeout: float = 2.0, newer_than: int = -1) -> LocalizedState:
        """Block until an estimate for a state with seq > newer_than is published."""
        deadline = time.monotonic() + timeout
        with self._cv:
            while self._latest is None or self._latest.state.seq <= newer_than:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError("no localized state")
                self._cv.wait(remaining)
            return self._latest

    def state_age(self) -> float:
        """Seconds since the state behind the latest estimate arrived (inf if none): link health."""
        latest = self.latest()
        return math.inf if latest is None else time.monotonic() - latest.state.received

    # ----------------------------------------------------------------- thread

    def _loop(self) -> None:
        seq = -1
        while not self._stop.is_set():
            try:
                raw = self.driver.wait_for_state(timeout=0.2, newer_than=seq)
            except TimeoutError:
                current = self.driver.state()
                if current is None or current.seq >= seq:
                    continue
                raw = current  # seq went back: the scene was reloaded
            try:
                self._publish(raw)
            except Exception:  # a bad state must not kill localization for the rest of the run
                log.exception("localization step failed for state %d", raw.seq)
            seq = raw.seq

    def _publish(self, raw: RobotState) -> None:
        with self._cv:
            last = self._latest
            if last is not None and raw.t < last.state.t - SCENE_RESTART:
                log.info("scene restarted: localization starts over from the measured pose")
                self.estimator.reset()
            pose = self.estimator.step(raw.left_ticks, raw.right_ticks, raw.pose)
            self._latest = LocalizedState(dataclasses.replace(raw, pose=pose), raw.pose, self.estimator.sigma)
            self._cv.notify_all()
