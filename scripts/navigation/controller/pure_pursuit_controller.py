"""Pure pursuit path-following controller with rotate-to-heading (like a stripped-down Nav2 RPP).

Behaviour per cycle:
  1. FOLLOW: track a lookahead point on the path. If it is far off to the side,
     turn in place first; otherwise drive a pure-pursuit arc. Slow down in tight
     turns and when approaching the end of the path.
  2. ALIGN: once within xy tolerance of the goal, stop and turn in place to the
     goal yaw (if the path has one).
  3. DONE: command zero.
"""
from __future__ import annotations

import math

from navigation.controller.controller import Controller, VelocityCmd
from drivers.data_types import Pose

FOLLOW, ALIGN, DONE = "follow", "align", "done"


class PurePursuitController(Controller):
    def __init__(
        self,
        max_v: float = 0.30,
        max_w: float = 1.5,
        max_accel: float = 0.8,
        max_ang_accel: float = 4.0,
        lookahead: float = 0.25,
        rotate_to_heading_threshold: float = 0.6,
        rotate_gain: float = 2.5,
        min_turn_radius_full_speed: float = 0.5,
        min_approach_v: float = 0.03,
        xy_goal_tolerance: float = 0.02,
        yaw_goal_tolerance: float = 0.03,
    ):
        """
        max_v, max_w, max_accel, max_ang_accel: limits (m/s, rad/s, m/s^2, rad/s^2), below the robot's own.
        lookahead: pure-pursuit lookahead distance (m). Larger = smoother, cuts corners more.
        rotate_to_heading_threshold: heading error (rad) above which the robot turns in place.
        rotate_gain: P gain for in-place rotation (rad/s per rad).
        min_turn_radius_full_speed: arcs tighter than this (m) are driven proportionally slower.
        min_approach_v: lowest forward speed while approaching the goal, so it never stalls short.
        """
        super().__init__(xy_goal_tolerance, yaw_goal_tolerance)
        self.max_v = max_v
        self.max_w = max_w
        self.max_accel = max_accel
        self.max_ang_accel = max_ang_accel
        self.lookahead = lookahead
        self.rotate_to_heading_threshold = rotate_to_heading_threshold
        self.rotate_gain = rotate_gain
        self.min_turn_radius_full_speed = min_turn_radius_full_speed
        self.min_approach_v = min_approach_v
        self.reset()

    def reset(self) -> None:
        self._mode = FOLLOW
        self._index = 0  # closest path pose so far; only moves forward
        self._last = VelocityCmd(0.0, 0.0)
        self._arc = self._cumulative_lengths() if self.path else [0.0]

    def is_goal_reached(self) -> bool:
        return self._mode == DONE

    def get_progress_index(self) -> int:
        return self._index

    def compute_velocity_cmd(self, pose: Pose, dt: float) -> VelocityCmd:
        if self.path is None:
            return VelocityCmd(0.0, 0.0)
        dt = min(max(dt, 0.0), 0.2)  # a stalled loop must not allow a huge velocity jump

        goal = self.path.poses[-1]
        if self._mode == FOLLOW and math.hypot(goal.x - pose.x, goal.y - pose.y) <= self.xy_goal_tolerance:
            self._mode = ALIGN if self.path.goal_yaw is not None else DONE

        if self._mode == FOLLOW:
            v, w = self._follow(pose)
        elif self._mode == ALIGN:
            err = self.normalize_angle(self.path.goal_yaw - pose.yaw)
            if abs(err) <= self.yaw_goal_tolerance:
                self._mode = DONE
                v, w = 0.0, 0.0
            else:
                v, w = 0.0, self._rotate(err)
        else:
            v, w = 0.0, 0.0

        cmd = self._limit_acceleration(v, w, dt)
        if self._mode == DONE:
            cmd = VelocityCmd(0.0, 0.0)  # stop now; the robot's own accel limit handles the rest
        self._last = cmd
        return cmd

    # ----------------------------------------------------------------- follow

    def _follow(self, pose: Pose) -> tuple[float, float]:
        self._update_progress(pose)
        target = self._lookahead_point(pose)

        # Target in the robot frame.
        dx, dy = target.x - pose.x, target.y - pose.y
        cos_y, sin_y = math.cos(pose.yaw), math.sin(pose.yaw)
        x_r = cos_y * dx + sin_y * dy
        y_r = -sin_y * dx + cos_y * dy
        heading_err = math.atan2(y_r, x_r)

        if abs(heading_err) > self.rotate_to_heading_threshold:
            return 0.0, self._rotate(heading_err)

        max_v = self.max_v * self.speed_limit
        dist2 = x_r * x_r + y_r * y_r
        curvature = 2.0 * y_r / dist2 if dist2 > 1e-9 else 0.0  # pure pursuit

        # Slow down in tight arcs.
        v = max_v
        if abs(curvature) > 1e-6:
            v *= min(1.0, (1.0 / abs(curvature)) / self.min_turn_radius_full_speed)

        # Slow down so we can stop at the end of the path: v <= sqrt(2 a d).
        remaining = self._remaining_length(pose)
        v = min(v, math.sqrt(2.0 * self.max_accel * 0.5 * remaining))  # use half accel: gentle stop
        v = max(v, min(self.min_approach_v, max_v))

        w = v * curvature
        max_w = self.max_w * self.speed_limit
        if abs(w) > max_w:  # keep the arc, slow down instead of cutting the turn
            v *= max_w / abs(w)
            w = math.copysign(max_w, w)
        return v, w

    def _update_progress(self, pose: Pose) -> None:
        """Advance to the closest path pose, searching only a window ahead so the
        robot can't jump to a later part of the path that passes nearby."""
        poses = self.path.poses
        window_end = min(len(poses), self._index + int(2.0 * self.lookahead / self._spacing()) + 2)
        best, best_d = self._index, math.inf
        for i in range(self._index, window_end):
            d = (poses[i].x - pose.x) ** 2 + (poses[i].y - pose.y) ** 2
            if d < best_d:
                best, best_d = i, d
        self._index = best

    def _lookahead_point(self, pose: Pose) -> Pose:
        """First path pose at least `lookahead` from the robot; the goal near the end."""
        poses = self.path.poses
        for i in range(self._index, len(poses)):
            if math.hypot(poses[i].x - pose.x, poses[i].y - pose.y) >= self.lookahead:
                return poses[i]
        return poses[-1]

    def _remaining_length(self, pose: Pose) -> float:
        p = self.path.poses[self._index]
        return self._arc[-1] - self._arc[self._index] + math.hypot(p.x - pose.x, p.y - pose.y)

    def _cumulative_lengths(self) -> list[float]:
        arc = [0.0]
        for a, b in zip(self.path.poses, self.path.poses[1:]):
            arc.append(arc[-1] + math.hypot(b.x - a.x, b.y - a.y))
        return arc

    def _spacing(self) -> float:
        n = len(self.path.poses)
        return max(1e-3, self._arc[-1] / (n - 1)) if n > 1 else 1.0

    # ----------------------------------------------------------------- shared

    def _rotate(self, err: float) -> float:
        """In-place rotation towards err, decelerating so it doesn't overshoot."""
        max_w = self.max_w * self.speed_limit
        w = min(abs(self.rotate_gain * err), max_w, math.sqrt(2.0 * self.max_ang_accel * abs(err)))
        return math.copysign(w, err)

    def _limit_acceleration(self, v: float, w: float, dt: float) -> VelocityCmd:
        dv, dw = self.max_accel * dt, self.max_ang_accel * dt
        v = min(max(v, self._last.v - dv), self._last.v + dv)
        w = min(max(w, self._last.w - dw), self._last.w + dw)
        return VelocityCmd(v, w)
