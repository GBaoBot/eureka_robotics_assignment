"""EKF localisation: wheel odometry (predict) fused with the simulator's noisy pose (update).

Pure math, no threads: call step() with every robot state, in order.
"""
from __future__ import annotations

import logging
import math
from typing import Optional

import numpy as np

from drivers.data_types import Pose

log = logging.getLogger(__name__)

CHI2_3DOF_999 = 16.27  # 99.9 % gate for a 3-D innovation


def _wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


class EkfLocalizer:
    def __init__(
        self,
        wheel_radius: float,
        wheel_base: float,
        ticks_per_rev: int,
        pose_sigma_xy: float = 0.01,
        pose_sigma_yaw: float = math.radians(0.5),
        wheel_noise: float = 0.005,
        floor_sigma_xy: float = 0.0005,
        floor_sigma_yaw: float = 0.0002,
        gate: float = CHI2_3DOF_999,
        max_rejections: int = 25,
    ):
        """
        pose_sigma_xy / pose_sigma_yaw: noise of the reported pose (m, rad): the measurement noise R.
        wheel_noise: odometry noise k (m / sqrt(m)); each wheel's distance has variance k^2 * |distance|.
        floor_sigma_xy / floor_sigma_yaw: process noise added every step, so the filter keeps listening
            to the pose even when the wheels don't move (m, rad per step).
        gate: Mahalanobis distance^2 above which a pose measurement is rejected as an outlier.
        max_rejections: consecutive rejections after which the filter re-initialises from the pose
            (the robot was moved, or the odometry is wrong): better a noisy pose than a wrong one.
        """
        self.m_per_tick = 2.0 * math.pi * wheel_radius / ticks_per_rev
        self.wheel_base = wheel_base
        self.R = np.diag([pose_sigma_xy ** 2, pose_sigma_xy ** 2, pose_sigma_yaw ** 2])
        self.wheel_noise = wheel_noise
        self.Q_floor = np.diag([floor_sigma_xy ** 2, floor_sigma_xy ** 2, floor_sigma_yaw ** 2])
        self.gate = gate
        self.max_rejections = max_rejections

        self.x: Optional[np.ndarray] = None  # [x, y, yaw]; None until initialised
        self.P: Optional[np.ndarray] = None
        self._ticks: Optional[tuple[int, int]] = None  # last (left, right) ticks seen
        self._rejections = 0
        self.last_nis = 0.0  # normalised innovation squared of the last update, ~3 on average if tuned

    @classmethod
    def from_robot_info(cls, info: dict, **kwargs) -> "EkfLocalizer":
        return cls(info["wheel_radius"], info["wheel_base"], info["ticks_per_rev"], **kwargs)

    @property
    def pose(self) -> Optional[Pose]:
        return None if self.x is None else Pose(float(self.x[0]), float(self.x[1]), float(self.x[2]))

    @property
    def sigma(self) -> Optional[tuple[float, float]]:
        """(position, yaw) standard deviation of the estimate: sqrt of the largest x/y eigenvalue, and of P_yaw."""
        if self.P is None:
            return None
        return math.sqrt(max(np.linalg.eigvalsh(self.P[:2, :2]))), math.sqrt(self.P[2, 2])

    def reset(self, pose: Optional[Pose] = None, sigma_xy: float = 0.001, sigma_yaw: float = 0.001) -> None:
        """Start from a known pose (e.g. load_scene's exact start), or from the next measurement if None."""
        self._ticks = None
        self._rejections = 0
        if pose is None:
            self.x = self.P = None
        else:
            self.x = np.array([pose.x, pose.y, pose.yaw])
            self.P = np.diag([sigma_xy ** 2, sigma_xy ** 2, sigma_yaw ** 2])

    def step(self, left_ticks: int, right_ticks: int, measured: Pose) -> Pose:
        """One robot state: predict with the ticks since the last call, update with the measured pose."""
        if self.x is None:
            self._init_from(measured)
        elif self._ticks is not None:
            self.predict(left_ticks - self._ticks[0], right_ticks - self._ticks[1])
            self.update(measured)
        else:  # known start pose, first ticks: nothing to predict yet
            self.update(measured)
        self._ticks = (left_ticks, right_ticks)
        return self.pose

    # ----------------------------------------------------------------- filter

    def predict(self, d_left_ticks: int, d_right_ticks: int) -> None:
        dl, dr = d_left_ticks * self.m_per_tick, d_right_ticks * self.m_per_tick
        ds, dth = 0.5 * (dl + dr), (dr - dl) / self.wheel_base
        th = self.x[2] + 0.5 * dth  # midpoint heading: exact for a constant-curvature step
        c, s = math.cos(th), math.sin(th)
        self.x = self.x + np.array([ds * c, ds * s, dth])
        self.x[2] = _wrap(self.x[2])

        F = np.array([[1.0, 0.0, -ds * s],
                      [0.0, 1.0, ds * c],
                      [0.0, 0.0, 1.0]])
        # Jacobian w.r.t. the wheel distances (dl, dr): wheel noise grows with distance driven.
        k = 0.5 * ds / self.wheel_base
        G = np.array([[0.5 * c + k * s, 0.5 * c - k * s],
                      [0.5 * s - k * c, 0.5 * s + k * c],
                      [-1.0 / self.wheel_base, 1.0 / self.wheel_base]])
        Q_wheels = np.diag([self.wheel_noise ** 2 * abs(dl), self.wheel_noise ** 2 * abs(dr)])
        self.P = F @ self.P @ F.T + G @ Q_wheels @ G.T + self.Q_floor

    def update(self, measured: Pose) -> bool:
        """Correct with a pose measurement (H = I). False if it was rejected as an outlier."""
        y = np.array([measured.x, measured.y, measured.yaw]) - self.x
        y[2] = _wrap(y[2])
        S = self.P + self.R
        S_inv = np.linalg.inv(S)
        self.last_nis = float(y @ S_inv @ y)
        if self.last_nis > self.gate:
            self._rejections += 1
            if self._rejections >= self.max_rejections:
                log.warning("localisation: %d pose measurements in a row disagree with odometry, "
                            "re-initialising from the pose", self._rejections)
                self._init_from(measured)
            return False
        self._rejections = 0

        K = self.P @ S_inv
        self.x = self.x + K @ y
        self.x[2] = _wrap(self.x[2])
        I_K = np.eye(3) - K
        self.P = I_K @ self.P @ I_K.T + K @ self.R @ K.T  # Joseph form: stays symmetric positive definite
        return True

    def _init_from(self, measured: Pose) -> None:
        self.x = np.array([measured.x, measured.y, measured.yaw])
        self.P = self.R.copy()
        self._rejections = 0
