"""RPC client: the one REQ socket to the simulator's REP port (5555).

Shared by RobotDriver and CameraDriver (inject the same instance into both).
It also owns the zmq.Context those drivers create their own sockets from, so
close it last.

    rpc = RpcClient("localhost")
    robot = RobotDriver(rpc)  # registers its own scene-change listener
    rpc.load_scene("01", seed=42)
    grid = rpc.get_map()
"""
from __future__ import annotations

import io
import json
import logging
import threading
from typing import Callable, Optional

import numpy as np
import zmq
from PIL import Image

from common.data_types import OccupancyMap

log = logging.getLogger(__name__)


class RpcError(RuntimeError):
    """The simulator answered {"ok": false} or did not answer in time."""


class RpcClient:
    """Thread-safe: one request in flight at a time, guarded by a lock."""

    def __init__(
        self,
        host: str = "localhost",
        port: int = 5555,
        rpc_timeout: float = 5.0,
    ):
        self._url = f"tcp://{host}:{port}"
        self._rpc_timeout_ms = int(rpc_timeout * 1000)

        self._ctx = zmq.Context()
        self._rpc_lock = threading.Lock()
        self._rpc = self._make_rpc_socket()
        self._scene_listeners: list[Callable[[], None]] = []
        self._closed = False

    # ----------------------------------------------------------------- lifecycle

    @property
    def context(self) -> zmq.Context:
        """Shared context for the drivers' own sockets."""
        return self._ctx

    def __enter__(self) -> "RpcClient":
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    def close(self) -> None:
        """Destroys the shared context: close the drivers first."""
        if self._closed:
            return
        self._closed = True
        self._ctx.destroy(linger=0)

    # ----------------------------------------------------------------- transport

    def _make_rpc_socket(self) -> zmq.Socket:
        s = self._ctx.socket(zmq.REQ)
        s.setsockopt(zmq.LINGER, 0)
        s.setsockopt(zmq.RCVTIMEO, self._rpc_timeout_ms)
        s.setsockopt(zmq.SNDTIMEO, self._rpc_timeout_ms)
        s.connect(self._url)
        return s

    def call(self, method: str, **params) -> tuple[object, list[bytes]]:
        """Send one RPC; returns (result, extra binary frames).

        A REQ socket is stuck after a lost reply, so on timeout it is recreated
        ("lazy pirate") and RpcError is raised; the caller decides whether to retry.
        """
        with self._rpc_lock:
            try:
                self._rpc.send_json({"method": method, "params": params})
                frames = self._rpc.recv_multipart()
            except zmq.Again:
                self._rpc.close()
                self._rpc = self._make_rpc_socket()
                raise RpcError(f"{method}: no reply within {self._rpc_timeout_ms} ms")
        rep = json.loads(frames[0])
        if not rep.get("ok"):
            raise RpcError(f"{method}: {rep.get('error')}")
        return rep.get("result"), frames[1:]

    # ----------------------------------------------------------------- scene

    def on_scene_change(self, callback: Callable[[], None]) -> None:
        """Called before load_scene / reset, e.g. to stop the robot and drop stale data."""
        self._scene_listeners.append(callback)

    def _call_scene_change(self, method: str, params: dict) -> dict:
        for cb in list(self._scene_listeners):
            try:
                cb()
            except Exception:  # a broken listener must not block the scene change
                log.exception("scene-change listener %r failed", cb)
        result, _ = self.call(method, **params)
        return result

    def ping(self) -> float:
        return float(self.call("ping")[0]["t"])

    def list_scenes(self) -> list[dict]:
        return self.call("list_scenes")[0]

    def load_scene(self, scene_id: str, seed: Optional[int] = None) -> dict:
        """Load a scene. Returns {"id", "start": {...}, "goals": [...]}; start is exact."""
        params = {"id": scene_id} if seed is None else {"id": scene_id, "seed": seed}
        return self._call_scene_change("load_scene", params)

    def reset(self, seed: Optional[int] = None) -> dict:
        """Reload the current scene with a new start pose; same result as load_scene."""
        return self._call_scene_change("reset", {} if seed is None else {"seed": seed})

    def get_goals(self) -> list[dict]:
        """[{"name": str, "pose": [x, y] or [x, y, yaw]}]"""
        return self.call("get_goals")[0]

    def mark_goal(self, name: str) -> float:
        """Record arrival at a goal. Only the first mark per goal after load/reset counts."""
        return float(self.call("mark_goal", name=name)[0]["t"])

    # ----------------------------------------------------------------- map / info

    def get_map(self) -> OccupancyMap:
        info, frames = self.call("get_map")
        if not frames:
            raise RpcError("get_map: reply has no PNG frame")
        img = np.array(Image.open(io.BytesIO(frames[0])).convert("L"))
        if img.shape != (info["height"], info["width"]):
            raise RpcError(f"get_map: PNG {img.shape} != header {info['height']}x{info['width']}")
        return OccupancyMap(grid=img > 127, resolution=float(info["resolution"]),
                            origin=(float(info["origin"][0]), float(info["origin"][1])))

    def get_robot_info(self) -> dict:
        return self.call("get_robot_info")[0]

    def get_camera_info(self) -> dict:
        return self.call("get_camera_info")[0]
