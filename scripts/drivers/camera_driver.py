"""Camera driver: the RGBD stream (port 5558, 15 Hz).

Shares the RpcClient's zmq.Context, so close the driver before the RpcClient.

    with RpcClient("localhost") as rpc, CameraDriver(rpc) as camera:
        rpc.load_scene("01", seed=42)
        frame = camera.wait_for_frame()
        rgb, depth = frame.rgb(), frame.depth(camera.camera_info["depth_scale"])
"""

import logging
import math
import threading
import time
from typing import Optional

import zmq

from drivers.data_types import CameraFrame
from drivers.rpc_client import RpcClient

log = logging.getLogger(__name__)


class CameraDriver:
    """Thread-safe interface to the camera stream.

    A receiver thread keeps the latest frame. To see every frame (e.g. when
    recording), loop on wait_for_frame(newer_than=last.seq) and check seq for gaps.
    """

    def __init__(
        self,
        rpc_client: RpcClient,
        host: str = "localhost",
        port: int = 5558,
    ):
        self._rpc = rpc_client
        self._host = host

        self._cam_socket = self._rpc.context.socket(zmq.SUB)
        self._cam_socket.setsockopt(zmq.RCVHWM, 10)
        self._cam_socket.connect(f"tcp://{self._host}:{port}")
        self._cam_socket.setsockopt(zmq.SUBSCRIBE, b"")

        self._data_lock = threading.Lock()
        self._frame_cv = threading.Condition(self._data_lock)
        self._frame: Optional[CameraFrame] = None
        self._camera_info: Optional[dict] = None

        self._rpc.on_scene_change(self._on_scene_change)

        self._stop = threading.Event()
        self._receive_thread = threading.Thread(target=self._receive_loop, name="camera-rx", daemon=True)
        self._receive_thread.start()

    # ----------------------------------------------------------------- lifecycle

    def __enter__(self) -> "CameraDriver":
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    def close(self) -> None:
        """Stop the receiver thread and close the socket. Call before RpcClient.close()."""
        if self._stop.is_set():
            return
        self._stop.set()
        self._receive_thread.join(timeout=1.0)
        self._cam_socket.close(linger=0)

    def _on_scene_change(self) -> None:
        # Called by RpcClient before load_scene / reset.
        with self._data_lock:
            self._frame = None  # old-scene frame must not leak into the new scene

    # ----------------------------------------------------------------- info

    @property
    def camera_info(self) -> dict:
        """Intrinsics, depth_scale, max_depth, rate, T_base_camera (fetched once)."""
        with self._data_lock:
            info = self._camera_info
        if info is None:
            # Fetched outside the lock: a slow RPC must not stall the receiver thread.
            info = self._rpc.get_camera_info()
            with self._data_lock:
                self._camera_info = info
        return info

    # ----------------------------------------------------------------- frames

    def frame(self) -> Optional[CameraFrame]:
        """Latest frame, or None if none received since start / last scene change."""
        with self._data_lock:
            return self._frame

    def wait_for_frame(self, timeout: float = 2.0, newer_than: int = -1) -> CameraFrame:
        """Block until a frame with seq > newer_than arrives."""
        deadline = time.monotonic() + timeout
        with self._frame_cv:
            while self._frame is None or self._frame.seq <= newer_than:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError("no camera frame received")
                self._frame_cv.wait(remaining)
            return self._frame

    def frame_age(self) -> float:
        """Seconds since the last frame (inf if none) — use to detect a dead link."""
        f = self.frame()
        return math.inf if f is None else time.monotonic() - f.received

    def _receive_loop(self) -> None:
        while not self._stop.is_set():
            try:
                if self._cam_socket.poll(100):
                    self._drain_frames()
            except zmq.ZMQError:
                if not self._stop.is_set():
                    log.exception("camera receiver stopped")
                return

    def _drain_frames(self) -> None:
        # Only the newest frame matters; skip anything queued behind it.
        raw = None
        while True:
            try:
                raw = self._cam_socket.recv_multipart(zmq.NOBLOCK)
            except zmq.Again:
                break
        if raw is None:
            return
        try:
            frame = CameraFrame.from_frames(raw, time.monotonic())
        except (ValueError, KeyError, TypeError) as e:
            log.warning("bad camera message: %s", e)
            return
        with self._frame_cv:
            self._frame = frame
            self._frame_cv.notify_all()
