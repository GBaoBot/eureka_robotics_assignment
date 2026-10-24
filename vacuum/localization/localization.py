"""Localization: turns measurements into pose estimates, in its own thread (like a ROS 2 node).

    any source --Measurement (raw)--> submit() -> Localization --LocalizedState--> navigation, GUI

It knows nothing about where measurements come from: whoever owns the source calls submit()
with each one, in order (MissionExecutor wires RobotDriver.on_state to it). The thread runs
the estimator (EkfLocalizer, or PassThrough for the raw pose) on every measurement and
publishes the latest estimate. Consumers read it with latest() or wait_for(newer_than),
like subscribing to a topic. Commands don't go through here: they go to the driver.
"""
from __future__ import annotations

import dataclasses
import logging
import math
import queue
import threading
import time
from dataclasses import dataclass
from typing import Optional, Protocol

from vacuum.common.data_types import Pose

log = logging.getLogger(__name__)

SCENE_RESTART = 0.5  # s: sensor time going back by more than this means load_scene / reset


class Measurement(Protocol):
    """What Localization needs from each input, e.g. a RobotState.

    Must be a dataclass: the output is a copy of it with the pose replaced by the estimate.
    """
    @property
    def t(self) -> float: ...  # sensor time, s
    @property
    def seq(self) -> int: ...
    @property
    def pose(self) -> Pose: ...  # measured pose
    @property
    def left_ticks(self) -> int: ...
    @property
    def right_ticks(self) -> int: ...
    @property
    def received(self) -> float: ...  # local time.monotonic() at reception


class Estimator(Protocol):
    sigma: Optional[tuple[float, float]]

    def reset(self, pose: Optional[Pose] = None) -> None: ...
    def step(self, left_ticks: int, right_ticks: int, measured: Pose) -> Pose: ...


class PassThrough:
    """No filtering: the estimate is the measured pose."""
    sigma = None

    def reset(self, pose: Optional[Pose] = None) -> None:
        pass

    def step(self, left_ticks: int, right_ticks: int, measured: Pose) -> Pose:
        return measured


@dataclass(frozen=True)
class LocalizedState:
    state: Measurement  # the measurement as submitted, with its pose replaced by the estimate
    raw_pose: Pose  # the measured pose, for comparison
    sigma: Optional[tuple[float, float]]  # (position m, yaw rad) uncertainty; None without a filter


class Localization:
    def __init__(self, estimator: Optional[Estimator] = None, initial_pose: Optional[Pose] = None):
        """estimator: None = PassThrough. initial_pose: a known start (e.g. load_scene's exact
        start pose); None = start from the first measurement."""
        self.estimator = estimator or PassThrough()
        self.estimator.reset(initial_pose)
        self._cv = threading.Condition()
        self._latest: Optional[LocalizedState] = None
        self._inbox: queue.SimpleQueue[Measurement] = queue.SimpleQueue()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._loop, name="localization", daemon=True)
        self._thread.start()

    # ----------------------------------------------------------------- lifecycle

    def __enter__(self) -> "Localization":
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    def close(self) -> None:
        """Stop the thread; measurements submitted after this are ignored."""
        self._stop.set()
        self._thread.join(timeout=1.0)

    def reset(self, pose: Optional[Pose] = None) -> None:
        """Restart the estimate, from a known pose or from the next measurement."""
        with self._cv:
            self.estimator.reset(pose)
            self._latest = None

    # ----------------------------------------------------------------- input

    def submit(self, measurement: Measurement) -> None:
        """Queue a measurement; returns at once, so it is safe to call from a driver's receive thread."""
        self._inbox.put(measurement)

    # ----------------------------------------------------------------- output

    def latest(self) -> Optional[LocalizedState]:
        with self._cv:
            return self._latest

    def wait_for(self, timeout: float = 2.0, newer_than: int = -1) -> LocalizedState:
        """Block until an estimate for a measurement with seq > newer_than is published."""
        deadline = time.monotonic() + timeout
        with self._cv:
            while self._latest is None or self._latest.state.seq <= newer_than:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError("no localized state")
                self._cv.wait(remaining)
            return self._latest

    def state_age(self) -> float:
        """Seconds since the measurement behind the latest estimate arrived (inf if none): link health."""
        latest = self.latest()
        return math.inf if latest is None else time.monotonic() - latest.state.received

    # ----------------------------------------------------------------- thread

    def _loop(self) -> None:
        while not self._stop.is_set():
            try:
                raw = self._inbox.get(timeout=0.2)
            except queue.Empty:
                continue
            try:
                self._publish(raw)
            except Exception:  # a bad measurement must not kill localization for the rest of the run
                log.exception("localization step failed for measurement %s", getattr(raw, "seq", repr(raw)))

    def _publish(self, raw: Measurement) -> None:
        with self._cv:
            last = self._latest
            if last is not None and raw.t < last.state.t - SCENE_RESTART:
                log.info("scene restarted: localization starts over from the measured pose")
                self.estimator.reset()
            pose = self.estimator.step(raw.left_ticks, raw.right_ticks, raw.pose)
            self._latest = LocalizedState(dataclasses.replace(raw, pose=pose), raw.pose, self.estimator.sigma)
            self._cv.notify_all()
