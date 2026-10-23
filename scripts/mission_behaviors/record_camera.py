"""RecordCamera: record the RGBD stream to video for offline processing.

start() once, stop() at the end; check `error` (or run()) for a failure in between.
A background thread takes every camera frame (15 Hz, independent of the mission tick)
and pipes it to two ffmpeg processes. Each recording is a directory:

    rgb.mkv        RGB, the simulator's JPEGs stream-copied as MJPEG (no re-encoding)
    depth.mkv      depth, raw uint16 in FFV1 gray16le: lossless, metres = value * depth_scale, 0 = invalid
    frames.csv     one row per video frame: frame, seq, t (sim s), received (unix s), robot pose at capture
    metadata.json  camera info (intrinsics, depth_scale, T_base_camera), scene, frame counts, ffmpeg settings

Both videos are written at the nominal camera rate; use frames.csv for the real timing.
Frame i of rgb.mkv, frame i of depth.mkv and row i of frames.csv are the same capture.
Gaps in seq are frames lost before they reached the recorder.

Reading depth back, e.g.:
    ffmpeg -i depth.mkv -f rawvideo -pix_fmt gray16le -  ->  np.frombuffer(..., "<u2").reshape(-1, H, W)
"""
from __future__ import annotations

import csv
import json
import logging
import os
import shutil
import subprocess
import threading
import time
from datetime import datetime
from typing import Optional

from drivers.camera_driver import CameraDriver
from drivers.data_types import CameraFrame

log = logging.getLogger(__name__)


class RecordCamera:
    def __init__(self, camera: CameraDriver, output_dir: str, frame_timeout: float = 2.0):
        """
        output_dir: each recording gets its own timestamped directory inside it.
        frame_timeout: seconds without a frame before a stall is logged (recording continues).
        """
        self.camera = camera
        self.output_dir = output_dir
        self.frame_timeout = frame_timeout
        self.path: Optional[str] = None  # directory of the current / last recording
        self.frames_written = 0
        self.frames_lost = 0

        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._error: Optional[str] = None
        self._rgb: Optional[subprocess.Popen] = None
        self._depth: Optional[subprocess.Popen] = None
        self._ffmpeg_log = None
        self._csv_file = None
        self._meta: dict = {}

    @property
    def recording(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    @property
    def error(self) -> Optional[str]:
        """Why the current / last recording stopped early, None if it didn't."""
        return self._error

    def start(self, name: str = "") -> bool:
        """Start a new recording; False (and nothing recorded) if it can't start."""
        if self.recording:
            log.warning("already recording to %s", self.path)
            return True
        if shutil.which("ffmpeg") is None:
            log.error("ffmpeg not found: camera recording disabled")
            return False
        try:
            info = self.camera.camera_info
            first = self.camera.wait_for_frame(timeout=self.frame_timeout)
        except Exception as e:  # RpcError, TimeoutError: no camera, nothing to record
            log.error("camera not available, not recording: %s", e)
            return False

        stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
        self.path = os.path.join(self.output_dir, f"{stamp}_{name}" if name else stamp)
        os.makedirs(self.path, exist_ok=True)
        w, h, rate = first.width, first.height, float(info.get("rate", 15.0))

        self._ffmpeg_log = open(os.path.join(self.path, "ffmpeg.log"), "wb")
        self._rgb = self._ffmpeg(["-f", "mjpeg", "-framerate", f"{rate:g}", "-i", "pipe:0",
                                  "-c:v", "copy", os.path.join(self.path, "rgb.mkv")])
        self._depth = self._ffmpeg(["-f", "rawvideo", "-pix_fmt", "gray16le", "-video_size", f"{w}x{h}",
                                    "-framerate", f"{rate:g}", "-i", "pipe:0",
                                    "-c:v", "ffv1", "-level", "3", os.path.join(self.path, "depth.mkv")])
        self._csv_file = open(os.path.join(self.path, "frames.csv"), "w", newline="")
        self._meta = {
            "name": name,
            "started": datetime.now().astimezone().isoformat(),
            "width": w,
            "height": h,
            "nominal_rate": rate,
            "camera_info": info,
            "rgb": {"file": "rgb.mkv", "codec": "mjpeg (stream copy of the simulator's JPEGs)"},
            "depth": {"file": "depth.mkv", "codec": "ffv1", "pix_fmt": "gray16le",
                      "units": "value * depth_scale = z-depth in metres; 0 = invalid / out of range"},
            "frames": {"file": "frames.csv",
                       "note": "row i = frame i of both videos; t = simulation time (s) at capture, "
                               "received = local unix time (s); pose = robot pose at capture"},
        }
        self.frames_written = self.frames_lost = 0
        self._error = None
        self._stop.clear()
        self._thread = threading.Thread(target=self._record_loop, args=(first,), name="camera-rec", daemon=True)
        self._thread.start()
        log.info("recording camera to %s", self.path)
        return True

    def run(self) -> bool:
        """True while the recording is healthy (always True when not recording)."""
        return self._error is None

    def stop(self) -> None:
        """Finish the recording: flush the videos and write metadata.json."""
        if self._thread is None:
            return
        self._stop.set()
        self._thread.join(timeout=5.0)
        self._thread = None
        for proc in (self._rgb, self._depth):
            try:
                proc.stdin.close()  # EOF: ffmpeg finalises the file
            except OSError:
                pass
        for proc in (self._rgb, self._depth):
            try:
                proc.wait(timeout=10.0)
            except subprocess.TimeoutExpired:
                proc.kill()
                log.error("ffmpeg did not finish, killed (see %s)", os.path.join(self.path, "ffmpeg.log"))
        self._csv_file.close()
        self._ffmpeg_log.close()

        self._meta.update(stopped=datetime.now().astimezone().isoformat(),
                          frames_written=self.frames_written, frames_lost=self.frames_lost)
        with open(os.path.join(self.path, "metadata.json"), "w") as f:
            json.dump(self._meta, f, indent=2)
        log.info("camera recording saved: %s (%d frames, %d lost)", self.path, self.frames_written, self.frames_lost)

    # ----------------------------------------------------------------- recorder thread

    def _ffmpeg(self, args: list[str]) -> subprocess.Popen:
        # Own session: a terminal Ctrl-C goes to our process group and would make ffmpeg quit
        # before stop() has written the last frames. ffmpeg finishes on stdin EOF instead,
        # which also happens if this process dies.
        return subprocess.Popen(["ffmpeg", "-hide_banner", "-loglevel", "warning", "-y"] + args,
                                stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=self._ffmpeg_log,
                                start_new_session=True)

    def _record_loop(self, frame: CameraFrame) -> None:
        writer = csv.writer(self._csv_file)
        writer.writerow(["frame", "seq", "t", "received", "x", "y", "yaw"])
        w, h = frame.width, frame.height
        last_seq = frame.seq - 1
        while True:
            if frame.width != w or frame.height != h:
                log.warning("camera frame %d is %dx%d, expected %dx%d: skipped", frame.seq, frame.width, frame.height, w, h)
            else:
                if frame.seq > last_seq + 1:
                    self.frames_lost += frame.seq - last_seq - 1
                try:
                    self._rgb.stdin.write(frame.jpeg)
                    self._depth.stdin.write(frame.depth_raw.astype("<u2", copy=False).tobytes())
                except (BrokenPipeError, OSError) as e:
                    self._error = f"ffmpeg stopped accepting frames ({e}); see {self.path}/ffmpeg.log"
                    log.error("camera recording failed: %s", self._error)  # the files so far stay valid
                    return
                # CameraFrame.received is monotonic; store wall-clock time, which means something offline.
                received = time.time() - (time.monotonic() - frame.received)
                p = frame.pose
                writer.writerow([self.frames_written, frame.seq, f"{frame.t:.4f}", f"{received:.4f}",
                                 f"{p.x:.4f}", f"{p.y:.4f}", f"{p.yaw:.4f}"])
                self.frames_written += 1
            last_seq = frame.seq

            frame = None
            waiting_since = time.monotonic()
            while frame is None:
                if self._stop.is_set():
                    return
                try:
                    # Short timeout so stop() doesn't wait for a frame.
                    frame = self.camera.wait_for_frame(timeout=0.2, newer_than=last_seq)
                except TimeoutError:
                    latest = self.camera.frame()
                    if latest is not None and latest.seq < last_seq:  # seq restarted (scene load / reset)
                        last_seq, frame = latest.seq - 1, latest
                    elif time.monotonic() - waiting_since > self.frame_timeout:
                        log.warning("no camera frame for %.1f s", self.frame_timeout)
                        waiting_since = time.monotonic()
