"""Robot driver: wheel / goto commands (port 5556) and the robot state stream (port 5557).

Shares the RpcClient's zmq.Context, so close the driver before the RpcClient.

    with RpcClient("localhost") as rpc, RobotDriver(rpc) as robot:
        rpc.load_scene("01", seed=42)
        robot.set_velocity(0.2, 0.0)   # m/s, rad/s; resent at 20 Hz by the driver
        print(robot.wait_for_state().pose)
"""

import json
import logging
import math
import threading
import time
from typing import Optional

import zmq

from drivers.data_types import RobotState
from drivers.rpc_client import RpcClient

log = logging.getLogger(__name__)


class RobotDriver:
    """Thread-safe interface to the robot's command and state sockets.

    Threads:
      - receiver: polls the state stream and keeps the latest state.
      - commander: resends the current wheel command at `command_rate` Hz so the
        simulator's 250 ms watchdog never fires while a velocity is active. If the
        caller stops updating the velocity for `command_timeout` s, it sends zeros.
    """

    def __init__(
        self,
        rpc_client: RpcClient,
        host: str = "localhost",
        cmd_port: int = 5556,
        state_port: int = 5557,
        command_rate: float = 20.0,
        command_timeout: float = 0.5,
    ):
        self._rpc = rpc_client
        self._host = host
        self._command_period = 1.0 / command_rate
        self._command_timeout = command_timeout

        # Robot command.
        self._cmd_lock = threading.Lock()
        self._cmd_socket = self._rpc.context.socket(zmq.PUSH)
        self._cmd_socket.setsockopt(zmq.LINGER, 0)
        self._cmd_socket.setsockopt(zmq.SNDHWM, 10)
        self._cmd_socket.setsockopt(zmq.IMMEDIATE, 1)
        self._cmd_socket.connect(f"tcp://{self._host}:{cmd_port}")

        # Robot state
        self._state_socket = self._rpc.context.socket(zmq.SUB)
        self._state_socket.setsockopt(zmq.RCVHWM, 10)
        self._state_socket.connect(f"tcp://{self._host}:{state_port}")
        self._state_socket.setsockopt(zmq.SUBSCRIBE, b"")

        self._data_lock = threading.Lock()
        self._state_cv = threading.Condition(self._data_lock)
        self._state: Optional[RobotState] = None
        self._robot_info: Optional[dict] = None
        # Wheel command (rad/s) and when the caller last set it; None = not in velocity mode.
        self._wheel_cmd: Optional[tuple[float, float]] = None
        self._wheel_cmd_time = 0.0
        self._next_goto_id = int(time.time())  # ids must be new across driver restarts too
        self._active_goto_id: Optional[int] = None

        self._rpc.on_scene_change(self._on_scene_change)

        self._stop = threading.Event()
        self._threads = [
            threading.Thread(target=self._receive_loop, name="robot-rx", daemon=True),
            threading.Thread(target=self._command_loop, name="robot-cmd", daemon=True),
        ]
        for t in self._threads:
            t.start()

    # ----------------------------------------------------------------- lifecycle

    def __enter__(self) -> "RobotDriver":
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    def close(self) -> None:
        """Stop the robot, then the threads. Call before RpcClient.close()."""
        if self._stop.is_set():
            return
        try:
            with self._data_lock:
                goto_id = self._active_goto_id
                self._stop_locked()
                if goto_id is not None:  # zero wheels already pre-empt it; cancel in case they're lost
                    self._send({"type": "cancel", "id": goto_id})
        except zmq.ZMQError:
            pass
        self._stop.set()
        for t in self._threads:
            t.join(timeout=1.0)
        # Give the final stop / cancel a moment to reach the simulator.
        self._cmd_socket.close(linger=200)
        self._state_socket.close(linger=0)

    def _on_scene_change(self) -> None:
        # Called by RpcClient before load_scene / reset.
        self.stop()
        with self._data_lock:
            self._state = None  # old-scene state must not leak into the new scene
            self._active_goto_id = None

    # ----------------------------------------------------------------- commands

    @property
    def robot_info(self) -> dict:
        with self._data_lock:
            info = self._robot_info
        if info is None:
            # Fetched outside the lock: a slow RPC must not stall the command thread.
            info = self._rpc.get_robot_info()
            with self._data_lock:
                self._robot_info = info
        return info

    def _send(self, msg: dict) -> None:
        with self._cmd_lock:
            try:
                self._cmd_socket.send_json(msg, flags=zmq.NOBLOCK)
            except zmq.Again:
                log.warning("command queue full, dropped %s", msg.get("type"))

    def set_wheels(self, left_wheel: float, right_wheel: float) -> None:
        """Wheel speeds in rad/s. Kept alive by the driver until changed, stop() or timeout."""
        left_wheel, right_wheel = float(left_wheel), float(right_wheel)
        with self._data_lock:
            self._wheel_cmd = (left_wheel, right_wheel)
            self._wheel_cmd_time = time.monotonic()
            self._active_goto_id = None  # a wheels command pre-empts any goto
            self._send({"type": "wheels", "left": left_wheel, "right": right_wheel})

    def set_velocity(self, v: float, w: float) -> None:
        """Body velocity: v in m/s (forward), w in rad/s (CCW).

        If a wheel would exceed its limit, both are scaled down together so the
        curvature (and therefore the path shape) is preserved.
        """
        info = self.robot_info
        radius, base = info["wheel_radius"], info["wheel_base"]
        left_wheel_speed = (v - w * base * 0.5) / radius
        right_wheel_speed = (v + w * base * 0.5) / radius
        max_wheel = info["max_wheel_speed"]
        scale = max(abs(left_wheel_speed), abs(right_wheel_speed)) / max_wheel
        if scale > 1.0:
            left_wheel_speed, right_wheel_speed = left_wheel_speed / scale, right_wheel_speed / scale
        self.set_wheels(left_wheel=left_wheel_speed, right_wheel=right_wheel_speed)

    def goto(self, x: float, y: float, yaw: Optional[float] = None) -> int:
        """Simulator 'go to pose': turn, straight line, optional final turn. No obstacle avoidance.

        Returns the goto id; follow progress with state().goto_id / goto_status.
        """
        msg = {"type": "goto", "x": float(x), "y": float(y)}
        if yaw is not None:
            msg["yaw"] = float(yaw)
        # Under _data_lock so the command thread can't resend wheels after the goto.
        with self._data_lock:
            self._wheel_cmd = None
            self._next_goto_id += 1
            goto_id = msg["id"] = self._next_goto_id
            self._active_goto_id = goto_id
            self._send(msg)
        return goto_id

    def cancel_goto(self, goto_id: int) -> None:
        self._send({"type": "cancel", "id": goto_id})

    def stop(self) -> None:
        """Stop immediately: leave velocity mode, then zero wheels (also pre-empts any running goto)."""
        with self._data_lock:
            self._stop_locked()

    def _stop_locked(self) -> None:
        self._wheel_cmd = None
        self._active_goto_id = None
        self._send({"type": "wheels", "left": 0.0, "right": 0.0})

    def _command_loop(self) -> None:
        next_tick = time.monotonic()
        while not self._stop.is_set():
            next_tick += self._command_period
            try:
                with self._data_lock:
                    cmd = self._wheel_cmd
                    if cmd is not None and time.monotonic() - self._wheel_cmd_time > self._command_timeout:
                        log.warning("no velocity update for %.2f s, stopping robot", self._command_timeout)
                        self._wheel_cmd = cmd = (0.0, 0.0)
                        self._wheel_cmd_time = float("inf")
                    if cmd is not None:
                        self._send({"type": "wheels", "left": cmd[0], "right": cmd[1]})
            except zmq.ZMQError:
                log.exception("command thread stopped")
                return
            delay = next_tick - time.monotonic()
            if delay > 0:
                self._stop.wait(delay)
            else:
                next_tick = time.monotonic()

    # ----------------------------------------------------------------- state

    def state(self) -> Optional[RobotState]:
        """Latest robot state, or None if none received since start / last scene change."""
        with self._data_lock:
            return self._state

    def wait_for_state(self, timeout: float = 2.0, newer_than: int = -1) -> RobotState:
        """Block until a state with seq > newer_than arrives."""
        deadline = time.monotonic() + timeout
        with self._state_cv:
            while self._state is None or self._state.seq <= newer_than:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError("no robot state received")
                self._state_cv.wait(remaining)
            return self._state

    def state_age(self) -> float:
        """Seconds since the last state message (inf if none) — use to detect a dead link."""
        s = self.state()
        return math.inf if s is None else time.monotonic() - s.received

    def _receive_loop(self) -> None:
        while not self._stop.is_set():
            try:
                if self._state_socket.poll(100):
                    self._drain_state()
            except zmq.ZMQError:
                if not self._stop.is_set():
                    log.exception("state receiver stopped")
                return

    def _drain_state(self) -> None:
        # Only the newest state matters; skip anything queued behind it.
        # ZMQ keeps messages in order on one connection, so no reordering check is needed.
        raw = None
        while True:
            try:
                raw = self._state_socket.recv(zmq.NOBLOCK)
            except zmq.Again:
                break
        if raw is None:
            return
        try:
            st = RobotState.from_json(json.loads(raw), time.monotonic())
        except (ValueError, KeyError, TypeError) as e:
            log.warning("bad state message: %s", e)
            return
        with self._state_cv:
            self._state = st
            self._state_cv.notify_all()
