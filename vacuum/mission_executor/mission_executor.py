"""MissionExecutor: the entry point to the robot system for any front end (CLI, Qt GUI, tests).

It connects the drivers, loads the scene, starts localization, builds the mission and the
camera recorder, and runs missions in a background thread. Data flows
driver -> localization -> navigation; the executor only wires it. Front ends only use this small interface:

    with MissionExecutor.connect(ExecutorConfig(scene="01")) as executor:
        executor.goals                      # [Goal(name, x, y, yaw)] of the current scene
        executor.scenes / executor.scene    # [Scene(id, name, difficulty)] / current scene id
        executor.load_scene("02", seed=7)    # returns immediately, like start(); new map, goals, start pose
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
from typing import Callable, Iterable, Optional

from vacuum.drivers.camera_driver import CameraDriver
from vacuum.drivers.robot_driver import RobotDriver
from vacuum.drivers.data_types import Goal, RobotState, Scene
from vacuum.drivers.rpc_client import RpcClient, RpcError
from vacuum.mission_behaviors.navigate_to_goal import GoalResult, MissionLimits, NavigateToGoal
from vacuum.mission_behaviors.navigate_to_goals import NavigateToGoals
from vacuum.mission_behaviors.record_camera import RecordCamera
from vacuum.mission_behaviors.telemetry import Telemetry
from vacuum.localization.ekf_localizer import EkfLocalizer
from vacuum.localization.localization import Localization
from vacuum.navigation.controller.pure_pursuit_controller import PurePursuitController
from vacuum.navigation.path_planner.astar import AStarPlanner

log = logging.getLogger("mission_executor")

REPO_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
DEFAULT_OUTPUT = os.path.join(REPO_DIR, "assets", "videos")


# (scene id, seed) -> (mission for the new map, its goals); loads the scene and restarts localization.
SceneLoader = Callable[[str, Optional[int]], tuple[NavigateToGoals, list[Goal]]]


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
                 on_close: Optional[Callable[[], None]] = None, scenes: Optional[list[Scene]] = None,
                 scene_loader: Optional[SceneLoader] = None):
        """Ready-made parts; use connect() for the real system.

        robot: only state() is used (the raw pose). on_close: releases the parts (localization, drivers).
        scene_loader: enables load_scene(); None = the scene can't be changed.
        """
        self.mission = mission
        self.goals = goals  # replaced (not mutated) on a scene change: viewers can compare identity
        self.scene = scene
        self.scenes = scenes or []
        self._scene_loader = scene_loader
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
                start = rpc.load_scene(scene, seed=config.seed)
                log.info("loaded scene %s, start (%.2f, %.2f, %.2f)", scene, start.x, start.y, start.yaw)
            goals = rpc.get_goals()
            scenes = rpc.list_scenes()
            state = driver.wait_for_state(timeout=5.0)  # the robot must be streaming before we drive it
            scene = scene or state.scene or None
            estimator = EkfLocalizer.from_robot_info(driver.robot_info) if config.localization else None
            # load_scene's start pose is exact; without it, start from the first measurement.
            # Entered after the driver, so the stack stops it before the driver closes.
            localization = stack.enter_context(Localization(driver, estimator, start))
            limits = MissionLimits(goal_timeout=config.goal_timeout)

            def build_navigate_to_goal() -> NavigateToGoal:
                # The algorithms are chosen here; NavigateToGoal only knows Planner / Controller.
                return NavigateToGoal(rpc, driver, localization, AStarPlanner(), PurePursuitController(), limits)

            navigate_to_goal = build_navigate_to_goal()
        except (RpcError, TimeoutError) as e:
            stack.close()
            raise ConnectionError(f"simulator not reachable at {config.host}: {e}") from e
        except BaseException:
            stack.close()
            raise
        recorder = RecordCamera(camera, config.output_dir) if camera is not None else None

        def load_scene(scene_id: str, seed: Optional[int]) -> tuple[NavigateToGoals, list[Goal]]:
            start = rpc.load_scene(scene_id, seed=seed)  # the drivers stop the robot and drop old states
            log.info("loaded scene %s, start (%.2f, %.2f, %.2f)", scene_id, start.x, start.y, start.yaw)
            localization.reset(start)  # exact start, before the robot moves
            # A new NavigateToGoal fetches the new map: fresh costmap, planner state and goal cache.
            return NavigateToGoals(build_navigate_to_goal()), rpc.get_goals()

        return cls(NavigateToGoals(navigate_to_goal), driver, localization, goals, recorder, scene,
                   on_close=stack.close, scenes=scenes, scene_loader=load_scene)

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

    def load_scene(self, scene_id: str, seed: Optional[int] = None) -> bool:
        """Load another scene (new map, goals and start pose); False if busy or not supported.

        Returns immediately; loading (about a second, plus building the costmap) runs in the
        background thread, so `running` is True until it's done and `error` says if it failed.
        """
        if self.running:
            log.warning("can't change the scene while a mission is running")
            return False
        if self._scene_loader is None:
            log.error("this executor can't change the scene")
            return False
        self.error = None
        self._done.clear()
        threading.Thread(target=self._load_scene, args=(scene_id, seed), name="scene-load", daemon=True).start()
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

    def _load_scene(self, scene_id: str, seed: Optional[int]) -> None:
        try:
            mission, goals = self._scene_loader(scene_id, seed)
            # Mission first: a viewer that sees the new goals then reads the new mission's telemetry.
            self.mission, self.scene, self.results = mission, scene_id, {}
            self.goals = goals
        except Exception as e:  # RpcError, or a bug: keep the app alive and say what happened
            log.exception("loading scene %s failed", scene_id)
            self.error = f"loading scene {scene_id} failed: {e}"
        finally:
            self._done.set()

    def _recording_name(self, goal_names: list[str]) -> str:
        name = "-".join(goal_names)
        if self.scene:
            name = f"scene{self.scene}_{name}"
        return re.sub(r"[^A-Za-z0-9_.-]+", "_", name)[:80]
