import math

import numpy as np

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
