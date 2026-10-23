"""MissionExecutor: the entry point to the robot system for any front end (CLI, Qt GUI, tests).

It connects the drivers, loads the scene, starts localization, builds the mission and the
camera recorder, and runs missions in a background thread. Data flows
driver -> localization -> navigation; the executor only wires it. Front ends only use this small interface:

    with MissionExecutor.connect(ExecutorConfig(scene="01")) as executor:
        executor.goals                      # [Goal(name, x, y, yaw)]
        executor.start(["kitchen", "sofa"])  # returns immediately; cancel() to stop
        executor.running / executor.error / executor.results
        executor.telemetry                  # mission state: path, status, reason, costmap, ...
        executor.robot_state()              # latest RobotState, EKF-filtered pose; state_age() for link health
        executor.raw_robot_state()          # the same with the simulator's noisy pose
        executor.localization_sigma()       # (position m, yaw rad) uncertainty of the filtered pose
        executor.recording_status()         # camera recording of the current / last mission

Each mission is recorded (into assets/videos by default), from start until the robot has
stopped. connect() builds the real system; the constructor takes ready-made parts, so tests
can pass fakes.
"""
from __future__ import annotations

import argparse
import logging
import os
import re
import threading
from contextlib import ExitStack
from dataclasses import dataclass
from typing import Callable, Iterable, NamedTuple, Optional

from drivers.camera_driver import CameraDriver
from drivers.data_types import Pose, RobotState
from drivers.robot_driver import RobotDriver
from drivers.rpc_client import RpcClient, RpcError
from mission_behaviors.navigate_to_goal import GoalResult, MissionLimits, NavigateToGoal
from mission_behaviors.navigate_to_goals import NavigateToGoals
from mission_behaviors.record_camera import RecordCamera
from mission_behaviors.telemetry import Telemetry
from localization.ekf_localizer import EkfLocalizer
from localization.localization import Localization

log = logging.getLogger("mission_executor")

REPO_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
DEFAULT_OUTPUT = os.path.join(REPO_DIR, "assets", "videos")


class Goal(NamedTuple):
    name: str
    x: float
    y: float
    yaw: Optional[float]  # None: any heading


@dataclass(frozen=True)
class RecordingStatus:
    path: str
    frames: int
    lost: int
    active: bool  # still recording
    error: Optional[str]  # why it stopped early, None if it didn't


@dataclass
class ExecutorConfig:
    host: str = "localhost"
    scene: Optional[str] = None  # None: keep the scene the simulator has loaded
    seed: Optional[int] = None
    record: bool = True
    localization: bool = True  # EKF-filter the pose (odometry + simulator pose)
    output_dir: str = DEFAULT_OUTPUT
    goal_timeout: float = MissionLimits.goal_timeout

    @staticmethod
    def add_arguments(parser: argparse.ArgumentParser) -> None:
        """The options every front end shares."""
        parser.add_argument("--scene", help="scene id to load, e.g. 01 (default: keep the current scene)")
        parser.add_argument("--seed", type=int, help="start-pose seed for --scene")
        parser.add_argument("--host", default="localhost", help="simulator host")
        parser.add_argument("--output", default=DEFAULT_OUTPUT, help="camera recordings directory (default: %(default)s)")
        parser.add_argument("--no-record", action="store_true", help="don't record the camera")
        parser.add_argument("--no-ekf", action="store_true", help="use the simulator's raw pose, no EKF")
        parser.add_argument("--goal-timeout", type=float, default=MissionLimits.goal_timeout, help="seconds per goal")

    @classmethod
    def from_args(cls, args: argparse.Namespace) -> "ExecutorConfig":
        return cls(host=args.host, scene=args.scene, seed=args.seed, record=not args.no_record, localization=not args.no_ekf,
                   output_dir=args.output, goal_timeout=args.goal_timeout)


class MissionExecutor:
    def __init__(self, mission: NavigateToGoals, robot: RobotDriver, localization: Localization,
                 goals: list[Goal], recorder: Optional[RecordCamera] = None, scene: Optional[str] = None,
                 on_close: Optional[Callable[[], None]] = None):
        """Ready-made parts; use connect() for the real system.

        robot: only state() is used (the raw pose). on_close: releases the parts (localization, drivers).
        """
        self.mission = mission
        self.goals = goals
        self.scene = scene
        self.results: dict[str, GoalResult] = {}  # of the last finished mission
        self.error: Optional[str] = None  # why the last mission aborted, None if it didn't
        self._robot = robot
        self._localization = localization
        self._recorder = recorder
        self._on_close = on_close
        self._cancel = threading.Event()
        # Set by the mission thread itself when it has really finished. Not Thread.join() / is_alive():
        # in CPython a Ctrl-C during Thread.join() marks the thread finished while it still runs.
        self._done = threading.Event()
        self._done.set()

    @classmethod
    def connect(cls, config: ExecutorConfig) -> "MissionExecutor":
        """Connect to the simulator and build the system. Raises ConnectionError if it isn't reachable."""
        stack = ExitStack()  # closes in reverse order: drivers before the RpcClient that owns their context
        try:
            rpc = stack.enter_context(RpcClient(config.host))
            driver = stack.enter_context(RobotDriver(rpc, host=config.host))
            camera = stack.enter_context(CameraDriver(rpc, host=config.host)) if config.record else None
            scene, start = config.scene, None
            if scene is not None:
                start = rpc.load_scene(scene, seed=config.seed)["start"]
                log.info("loaded scene %s, start (%.2f, %.2f, %.2f)", scene, start["x"], start["y"], start["yaw"])
            goals = [Goal(g["name"], float(g["pose"][0]), float(g["pose"][1]),
                          float(g["pose"][2]) if len(g["pose"]) > 2 else None) for g in rpc.get_goals()]
            state = driver.wait_for_state(timeout=5.0)  # the robot must be streaming before we drive it
            scene = scene or state.scene or None
            estimator = EkfLocalizer.from_robot_info(driver.robot_info) if config.localization else None
            # load_scene's start pose is exact; without it, start from the first measurement.
            # Entered after the driver, so the stack stops it before the driver closes.
            localization = stack.enter_context(Localization(
                driver, estimator, Pose(start["x"], start["y"], start["yaw"]) if start else None))
            navigate_to_goal = NavigateToGoal(rpc, driver, localization,
                                              MissionLimits(goal_timeout=config.goal_timeout))
        except (RpcError, TimeoutError) as e:
            stack.close()
            raise ConnectionError(f"simulator not reachable at {config.host}: {e}") from e
        except BaseException:
            stack.close()
            raise
        recorder = RecordCamera(camera, config.output_dir) if camera is not None else None
        return cls(NavigateToGoals(navigate_to_goal), driver, localization, goals, recorder, scene,
                   on_close=stack.close)

    # ----------------------------------------------------------------- lifecycle

    def __enter__(self) -> "MissionExecutor":
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    def close(self) -> None:
        """Stop any mission (and its recording), then release the drivers."""
        self.cancel()
        if not self.join(timeout=15.0):  # long enough for the recorder to finalise its files
            log.error("mission thread did not stop")
        if self._on_close is not None:
            on_close, self._on_close = self._on_close, None
            on_close()

    # ----------------------------------------------------------------- commands

    @property
    def running(self) -> bool:
        return not self._done.is_set()

    def start(self, goal_names: Iterable[str]) -> bool:
        """Start visiting the goals in order; False if a mission is already running.

        Returns immediately; the mission (and its recording) runs in a background thread.
        """
        if self.running:
            log.warning("a mission is already running")
            return False
        # A fresh event per mission: a cancel() meant for the previous one can't leak into this one.
        self._cancel = threading.Event()
        self.results, self.error = {}, None
        self._done.clear()
        threading.Thread(target=self._run, args=(list(goal_names), self._cancel), name="mission", daemon=True).start()
        return True

    def cancel(self) -> None:
        """Stop the running mission; the robot stops within one control tick."""
        self._cancel.set()

    def join(self, timeout: Optional[float] = None) -> bool:
        """Wait for the mission (and its recording) to finish; True if it has."""
        return self._done.wait(timeout)

    # ----------------------------------------------------------------- status

    @property
    def telemetry(self) -> Telemetry:
        return self.mission.navigate_to_goal.telemetry

    def robot_state(self) -> Optional[RobotState]:
        """Latest state with the localized pose."""
        latest = self._localization.latest()
        return None if latest is None else latest.state

    def raw_robot_state(self) -> Optional[RobotState]:
        """Latest state with the simulator's unfiltered pose."""
        return self._robot.state()

    def localization_sigma(self) -> Optional[tuple[float, float]]:
        """(position m, yaw rad) standard deviation of the localized pose; None without the EKF."""
        latest = self._localization.latest()
        return None if latest is None else latest.sigma

    def state_age(self) -> float:
        """Seconds since the latest localized state's data arrived: link (and localization) health."""
        return self._localization.state_age()

    def recording_status(self) -> Optional[RecordingStatus]:
        """The current / last mission's recording; None if recording is off or nothing was recorded yet."""
        r = self._recorder
        if r is None or r.path is None:
            return None
        return RecordingStatus(r.path, r.frames_written, r.frames_lost, r.recording, r.error)

    # ----------------------------------------------------------------- mission thread

    def _run(self, goal_names: list[str], cancel: threading.Event) -> None:
        try:
            # Recording starts here, not in start(): it waits for the first camera frame (up to 2 s).
            if self._recorder is not None and not self._recorder.start(self._recording_name(goal_names)):
                # Don't drive a mission the user expects to be recorded without it.
                self.error = "camera recording could not start (see log)"
                log.error("mission not started: %s", self.error)
                return
            try:
                self.results = self.mission.run(goal_names, cancel)
            except Exception as e:  # ConnectionError, RpcError, or a bug: report it, don't kill the app silently
                log.exception("mission aborted")
                self.error = str(e)
            finally:
                if self._recorder is not None:
                    self._recorder.stop()  # after the mission, so the recording covers the robot stopping
        finally:
            self._done.set()

    def _recording_name(self, goal_names: list[str]) -> str:
        name = "-".join(goal_names)
        if self.scene:
            name = f"scene{self.scene}_{name}"
        return re.sub(r"[^A-Za-z0-9_.-]+", "_", name)[:80]
