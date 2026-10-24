"""2D costmap shared by planners and controllers (Nav2's Costmap2D + inflation layer, simplified).

Costs follow the Nav2 convention (uint8):
    0          free, far from obstacles
    1..252     near an obstacle; decays exponentially with distance
    253        INSCRIBED: the robot centre here means the disc touches an obstacle
    254        LETHAL: the cell itself is an obstacle
"""
from __future__ import annotations

import math

import numpy as np

from vacuum.common.data_types import OccupancyMap

FREE = 0
MAX_NON_OBSTACLE = 252
INSCRIBED = 253
LETHAL = 254


class Costmap2D:
    def __init__(
        self,
        occ_map: OccupancyMap,
        robot_radius: float = 0.17,
        footprint_padding: float = 0.02,
        inflation_radius: float = 0.45,
        cost_scaling_factor: float = 8.0,
    ):
        """
        robot_radius + footprint_padding is the inscribed radius: closer than this is a collision.
        inflation_radius: beyond this distance from any obstacle the cost is 0.
        cost_scaling_factor: how fast the cost decays outside the inscribed radius (1/m).
        """
        self.resolution = occ_map.resolution
        self.origin = occ_map.origin
        self.height, self.width = occ_map.grid.shape
        self.inscribed_radius = robot_radius + footprint_padding
        self.inflation_radius = max(inflation_radius, self.inscribed_radius)
        self.cost_scaling_factor = cost_scaling_factor

        self._static = occ_map.grid.astype(bool)
        self._dynamic = np.zeros_like(self._static)  # obstacles added at runtime (local costmap)
        self.costs = np.zeros(self._static.shape, dtype=np.uint8)
        self._update_costs()

    # ----------------------------------------------------------------- coordinates

    def world_to_cell(self, x: float, y: float) -> tuple[int, int]:
        c = int(math.floor((x - self.origin[0]) / self.resolution))
        r = self.height - 1 - int(math.floor((y - self.origin[1]) / self.resolution))
        return r, c

    def cell_to_world(self, r: int, c: int) -> tuple[float, float]:
        x = self.origin[0] + (c + 0.5) * self.resolution
        y = self.origin[1] + (self.height - 1 - r + 0.5) * self.resolution
        return x, y

    def is_in_bounds(self, r: int, c: int) -> bool:
        return 0 <= r < self.height and 0 <= c < self.width

    # ----------------------------------------------------------------- queries

    def cost(self, r: int, c: int) -> int:
        """Cost of a cell; outside the map counts as lethal."""
        if not self.is_in_bounds(r, c):
            return LETHAL
        return int(self.costs[r, c])

    def cost_at(self, x: float, y: float) -> int:
        """Cost of a cell at world coordinate (x, y)"""
        return self.cost(*self.world_to_cell(x, y))

    def max_cost_on_segment(self, x0: float, y0: float, x1: float, y1: float) -> int:
        """Highest cell cost along a straight segment, sampled every half cell."""
        n = max(1, int(math.ceil(math.hypot(x1 - x0, y1 - y0) / (0.5 * self.resolution))))
        worst = 0
        for i in range(n + 1):
            s = i / n
            worst = max(worst, self.cost_at(x0 + s * (x1 - x0), y0 + s * (y1 - y0)))
            if worst >= LETHAL:
                break
        return worst

    def is_in_collision(self, x: float, y: float) -> bool:
        """True if the robot centred at (x, y) would touch an obstacle."""
        return self.cost_at(x, y) >= INSCRIBED

    # ----------------------------------------------------------------- updates

    def set_dynamic_obstacles(self, mask: np.ndarray) -> None:
        """Replace the runtime obstacle layer (same shape as the map) and re-inflate."""
        if mask.shape != self._static.shape:
            raise ValueError(f"mask shape {mask.shape} != map shape {self._static.shape}")
        self._dynamic = mask.astype(bool)
        self._update_costs()

    def add_obstacle(self, x: float, y: float, radius: float = 0.0) -> None:
        """Mark a disc of cells as occupied (e.g. where the bumper fired) and re-inflate."""
        r0, c0 = self.world_to_cell(x, y)
        k = int(math.ceil(radius / self.resolution))
        for dr in range(-k, k + 1):
            for dc in range(-k, k + 1):
                if dr * dr + dc * dc <= k * k and self.is_in_bounds(r0 + dr, c0 + dc):
                    self._dynamic[r0 + dr, c0 + dc] = True
        self._update_costs()

    def _update_costs(self) -> None:
        obstacles = self._static | self._dynamic
        dist = self._distance_to_obstacles(obstacles)  # metres from cell centre to nearest obstacle edge

        costs = np.zeros(obstacles.shape, dtype=np.float64)
        decay = (dist > self.inscribed_radius) & (dist <= self.inflation_radius)
        costs[decay] = MAX_NON_OBSTACLE * np.exp(
            -self.cost_scaling_factor * (dist[decay] - self.inscribed_radius))
        costs[dist <= self.inscribed_radius] = INSCRIBED
        costs[obstacles] = LETHAL
        self.costs = costs.astype(np.uint8)

    def _distance_to_obstacles(self, obstacles: np.ndarray) -> np.ndarray:
        """Euclidean distance (m) to the nearest obstacle, capped at inflation_radius + 1 cell.

        Brute force over all offsets within the inflation radius, nearest first: every
        offset is one vectorised numpy pass, so this needs no scipy/OpenCV.
        """
        res = self.resolution
        k = int(math.ceil(self.inflation_radius / res)) + 1
        h, w = obstacles.shape
        padded = np.zeros((h + 2 * k, w + 2 * k), dtype=bool)
        padded[k:k + h, k:k + w] = obstacles

        offsets = sorted(
            ((dr * dr + dc * dc, dr, dc) for dr in range(-k, k + 1) for dc in range(-k, k + 1)
             if dr * dr + dc * dc <= k * k))
        dist = np.full(obstacles.shape, np.inf)
        for d2, dr, dc in offsets:
            hit = padded[k + dr:k + dr + h, k + dc:k + dc + w] & np.isinf(dist)
            # Distance to the obstacle cell's edge, not its centre (half a cell closer).
            dist[hit] = max(0.0, math.sqrt(d2) * res - 0.5 * res) if d2 else 0.0
        return dist
