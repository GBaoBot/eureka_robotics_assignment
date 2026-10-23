"""A* global planner on an 8-connected grid (cost-aware, like Nav2's NavFn/Smac 2D).

Edge cost = step length * (1 + cost_weight * cell_cost / 252), so the path keeps
away from walls where there is room but still squeezes through narrow gaps.
The raw grid path is then shortened with line-of-sight shortcuts and resampled.
"""
from __future__ import annotations

import heapq
import math
from collections import deque
from typing import Optional

from navigation.costmap import INSCRIBED, MAX_NON_OBSTACLE
from navigation.path_planner.planner import Path, Planner, PlanningError
from drivers.data_types import Pose

SQRT2 = math.sqrt(2.0)
NEIGHBOURS = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
              (-1, -1, SQRT2), (-1, 1, SQRT2), (1, -1, SQRT2), (1, 1, SQRT2)]


class AStarPlanner(Planner):
    def __init__(
        self,
        cost_weight: float = 3.0,
        start_escape_radius: float = 0.3,
        smooth: bool = True,
        path_spacing: float = 0.05,
    ):
        """
        cost_weight: how strongly to avoid high-cost cells (0 = shortest path, hugs walls).
        start_escape_radius: if the start is in collision (pose noise, map mismatch),
            search this far for the nearest free cell and plan from there.
        smooth: shortcut the grid path where a straight line is no more costly.
        """
        super().__init__(path_spacing)
        self.cost_weight = cost_weight
        self.start_escape_radius = start_escape_radius
        self.smooth = smooth

    def compute_plan(self, start: Pose, goal_xy: tuple[float, float],
                     goal_yaw: Optional[float] = None) -> Path:
        cm = self._require_costmap()
        start_cell = cm.world_to_cell(start.x, start.y)
        goal_cell = cm.world_to_cell(*goal_xy)

        if cm.cost(*goal_cell) >= INSCRIBED:
            raise PlanningError(f"goal {goal_xy} is in collision")
        if cm.cost(*start_cell) >= INSCRIBED:
            escaped = self._nearest_free_cell(start_cell)
            if escaped is None:
                raise PlanningError(f"start ({start.x:.2f}, {start.y:.2f}) is in collision")
            start_cell = escaped

        cells = self._search(start_cell, goal_cell)
        if self.smooth:
            cells = self._shortcut(cells)

        # Exact start and goal positions instead of the cell centres at the ends.
        points = [(start.x, start.y)] + [cm.cell_to_world(r, c) for r, c in cells[1:-1]] + [goal_xy]
        return self._make_path(points, goal_yaw)

    # ----------------------------------------------------------------- search

    def _search(self, start: tuple[int, int], goal: tuple[int, int]) -> list[tuple[int, int]]:
        cm = self.costmap
        h, w = cm.height, cm.width
        costs = cm.costs.ravel().tolist()  # plain list: much faster to index than numpy in a loop
        weight = self.cost_weight / MAX_NON_OBSTACLE
        s, g = start[0] * w + start[1], goal[0] * w + goal[1]
        gr, gc = goal

        def heuristic(r: int, c: int) -> float:  # octile distance: admissible for 8-connected moves
            dr, dc = abs(r - gr), abs(c - gc)
            return (dr + dc) + (SQRT2 - 2.0) * min(dr, dc)

        g_cost = {s: 0.0}
        parent = {s: -1}
        closed = set()
        open_heap = [(heuristic(*start), 0.0, s)]

        while open_heap:
            _, g_here, idx = heapq.heappop(open_heap)
            if idx in closed:
                continue  # stale heap entry
            if idx == g:
                return self._reconstruct(parent, g, w)
            closed.add(idx)
            r, c = divmod(idx, w)
            for dr, dc, step in NEIGHBOURS:
                nr, nc = r + dr, c + dc
                if not (0 <= nr < h and 0 <= nc < w):
                    continue
                n = nr * w + nc
                cell_cost = costs[n]
                if cell_cost >= INSCRIBED or n in closed:
                    continue
                # No diagonal moves that cut a blocked corner.
                if dr and dc and (costs[r * w + nc] >= INSCRIBED or costs[nr * w + c] >= INSCRIBED):
                    continue
                g_new = g_here + step * (1.0 + weight * cell_cost)
                if g_new < g_cost.get(n, math.inf):
                    g_cost[n] = g_new
                    parent[n] = idx
                    heapq.heappush(open_heap, (g_new + heuristic(nr, nc), g_new, n))

        raise PlanningError("goal unreachable on the current map")

    @staticmethod
    def _reconstruct(parent: dict[int, int], goal: int, width: int) -> list[tuple[int, int]]:
        cells = []
        idx = goal
        while idx != -1:
            cells.append(divmod(idx, width))
            idx = parent[idx]
        cells.reverse()
        return cells

    def _nearest_free_cell(self, cell: tuple[int, int]) -> Optional[tuple[int, int]]:
        """Breadth-first search outwards for the closest cell not in collision."""
        cm = self.costmap
        max_steps = int(math.ceil(self.start_escape_radius / cm.resolution))
        seen = {cell}
        queue = deque([(cell, 0)])
        while queue:
            (r, c), d = queue.popleft()
            if cm.cost(r, c) < INSCRIBED:
                return r, c
            if d >= max_steps:
                continue
            for dr, dc, _ in NEIGHBOURS:
                n = (r + dr, c + dc)
                if n not in seen and cm.in_bounds(*n):
                    seen.add(n)
                    queue.append((n, d + 1))
        return None

    # ----------------------------------------------------------------- post-processing

    def _shortcut(self, cells: list[tuple[int, int]]) -> list[tuple[int, int]]:
        """Greedy line-of-sight shortcutting.

        From each kept cell, jump to the farthest later cell whose straight segment is
        no more costly than the grid path it replaces, so smoothing never pulls the
        path closer to obstacles than A* chose to go.
        """
        if len(cells) <= 2:
            return cells
        cm = self.costmap
        out = [cells[0]]
        i = 0
        while i < len(cells) - 1:
            best = i + 1
            worst_on_path = cm.cost(*cells[i + 1])
            for j in range(i + 2, len(cells)):
                worst_on_path = max(worst_on_path, cm.cost(*cells[j]))
                (x0, y0), (x1, y1) = cm.cell_to_world(*cells[i]), cm.cell_to_world(*cells[j])
                if cm.max_cost_on_segment(x0, y0, x1, y1) <= worst_on_path:
                    best = j
            out.append(cells[best])
            i = best
        return out
