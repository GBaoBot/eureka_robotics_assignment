import io
import json
import math

import numpy as np
from PIL import Image

from typing import Optional

from dataclasses import dataclass

@dataclass
class OccupancyMap:
    """Global map from get_map. grid[r, c]: True = occupied. Row 0 is the top (largest y)."""

    grid: np.ndarray
    resolution: float
    origin: tuple[float, float]  # world position of the bottom-left corner of (row H-1, col 0)

    @property
    def height(self) -> int:
        return self.grid.shape[0]

    @property
    def width(self) -> int:
        return self.grid.shape[1]

    def world_to_cell(self, x: float, y: float) -> tuple[int, int]:
        c = int(math.floor((x - self.origin[0]) / self.resolution))
        r = self.height - 1 - int(math.floor((y - self.origin[1]) / self.resolution))
        return r, c

    def cell_to_world(self, r: int, c: int) -> tuple[float, float]:
        x = self.origin[0] + (c + 0.5) * self.resolution
        y = self.origin[1] + (self.height - 1 - r + 0.5) * self.resolution
        return x, y

    def in_bounds(self, r: int, c: int) -> bool:
        return 0 <= r < self.height and 0 <= c < self.width

@dataclass(frozen=True)
class Pose:
    x: float
    y: float
    yaw: float

    @classmethod
    def from_json(cls, d: dict) -> "Pose":
        return cls(float(d["x"]), float(d["y"]), float(d["yaw"]))

@dataclass(frozen=True)
class RobotState:
    t: float  # simulation time
    seq: int
    scene: str
    pose: Pose
    left_ticks: int
    right_ticks: int
    bumper: bool
    goto_id: Optional[int]
    goto_status: Optional[str]  # running | reached | bumped | cancelled | preempted
    received: float  # local time.monotonic() at reception

    @classmethod
    def from_json(cls, d: dict, received: float) -> "RobotState":
        goto = d.get("goto") or {}
        return cls(
            t=float(d["t"]),
            seq=int(d["seq"]),
            scene=str(d.get("scene", "")),
            pose=Pose.from_json(d["pose"]),
            left_ticks=int(d["wheels"]["left_ticks"]),
            right_ticks=int(d["wheels"]["right_ticks"]),
            bumper=bool(d["bumper"]),
            goto_id=goto.get("id"),
            goto_status=goto.get("status"),
            received=received,
        )
@dataclass(frozen=True)
class CameraFrame:
    t: float  # simulation time at capture
    seq: int
    pose: Pose  # robot pose at capture time
    jpeg: bytes  # RGB as received; kept encoded so recorders don't re-compress
    depth_raw: np.ndarray  # (H, W) uint16, 0 = invalid / out of range
    received: float  # local time.monotonic() at reception

    @property
    def width(self) -> int:
        return self.depth_raw.shape[1]

    @property
    def height(self) -> int:
        return self.depth_raw.shape[0]

    def rgb(self) -> np.ndarray:
        """Decode the JPEG to an (H, W, 3) uint8 array."""
        return np.array(Image.open(io.BytesIO(self.jpeg)).convert("RGB"))

    def depth(self, depth_scale: float) -> np.ndarray:
        """Z-depth in metres (float32); invalid pixels are NaN."""
        d = self.depth_raw.astype(np.float32) * depth_scale
        d[self.depth_raw == 0] = np.nan
        return d

    @classmethod
    def from_frames(cls, frames: list[bytes], received: float) -> "CameraFrame":
        if len(frames) != 3:
            raise ValueError(f"expected 3 frames, got {len(frames)}")
        header = json.loads(frames[0])
        w, h = int(header["width"]), int(header["height"])
        if len(frames[2]) != w * h * 2:
            raise ValueError(f"depth is {len(frames[2])} bytes, expected {w}x{h} uint16")
        depth = np.frombuffer(frames[2], dtype="<u2").reshape(h, w)
        return cls(
            t=float(header["t"]),
            seq=int(header["seq"]),
            pose=Pose.from_json(header["pose"]),
            jpeg=bytes(frames[1]),
            depth_raw=depth,
            received=received,
        )
