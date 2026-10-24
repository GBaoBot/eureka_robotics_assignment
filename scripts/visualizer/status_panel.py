"""StatusPanel: what the robot is doing and why, as text.

refresh() reads the pose, localization, telemetry and recording from MissionExecutor;
the show_*() methods below it render plain values.
"""
from __future__ import annotations

import math
from typing import TYPE_CHECKING, Optional, Sequence

from PyQt6.QtWidgets import QFormLayout, QLabel, QWidget

from drivers.data_types import Pose

if TYPE_CHECKING:  # type hint only: the panel depends on the executor's interface, not its imports
    from mission_executor.mission_executor import MissionExecutor

STATUS_COLORS = {"succeeded": "#1a8f3c", "failed": "#d01e1e", "cancelled": "#b07800",
                 "backing up": "#d01e1e", "planning": "#b07800", "following": "#1e64dc"}
LINK_STALE = 0.5  # s without a robot state before the link is shown as stale


class StatusPanel(QWidget):
    def __init__(self, executor: MissionExecutor, parent=None):
        super().__init__(parent)
        self._executor = executor
        layout = QFormLayout(self)
        self._labels = {}
        for key in ("goal", "status", "reason", "pose", "localization", "command", "collisions", "link",
                    "recording", "results"):
            label = QLabel("–")
            label.setWordWrap(True)
            layout.addRow(f"{key.capitalize()}:", label)
            self._labels[key] = label

    def refresh(self) -> None:
        ex = self._executor
        state = ex.robot_state()
        self.show_robot(state.pose if state is not None else None, ex.state_age())
        self.show_localization(ex.localization_sigma())
        t = ex.telemetry
        self.show_mission(t.goal_name, t.status, t.reason, t.cmd.v, t.cmd.w, len(t.collisions), t.results)
        self.show_recording(ex.recording_status())

    def show_mission(self, goal: Optional[str], status: str, reason: str,
                     v: float, w: float, collisions: int, results: Sequence[tuple[str, str]]) -> None:
        self._labels["goal"].setText(goal or "–")
        color = STATUS_COLORS.get(status, "#505050")
        self._labels["status"].setText(f'<b style="color:{color}">{status}</b>')
        self._labels["reason"].setText(reason or "–")
        self._labels["command"].setText(f"v {v:+.2f} m/s   w {w:+.2f} rad/s")
        self._labels["collisions"].setText(str(collisions))
        self._labels["results"].setText(
            "<br>".join(f'{name}: <span style="color:{STATUS_COLORS.get(r, "#505050")}">{r}</span>'
                        for name, r in results) or "–")

    def show_recording(self, rec) -> None:
        """rec: a RecordingStatus (path, frames, lost, active, error), or None when there is none."""
        label = self._labels["recording"]
        if rec is None:
            label.setText("–")
        elif rec.error:
            label.setText(f'<b style="color:#d01e1e">failed</b> after {rec.frames} frames: {rec.error}')
        else:
            state = '<span style="color:#d01e1e">● REC</span>' if rec.active else "saved"
            label.setText(f"{state} {rec.frames} frames, {rec.lost} lost<br><small>{rec.path}</small>")

    def show_localization(self, sigma: Optional[tuple[float, float]]) -> None:
        """sigma: (position m, yaw rad) of the EKF estimate, or None when the raw pose is used."""
        if sigma is None:
            self._labels["localization"].setText("raw pose (no EKF)")
        else:
            self._labels["localization"].setText(f"EKF  σ {1000 * sigma[0]:.1f} mm, {math.degrees(sigma[1]):.2f}°")

    def show_robot(self, pose: Optional[Pose], state_age: float) -> None:
        if pose is None:
            self._labels["pose"].setText("–")
        else:
            self._labels["pose"].setText(f"x {pose.x:+.2f}  y {pose.y:+.2f}  yaw {math.degrees(pose.yaw):+.0f}°")
        if math.isinf(state_age):
            self._labels["link"].setText('<b style="color:#d01e1e">no robot state</b>')
        elif state_age > LINK_STALE:
            self._labels["link"].setText(f'<b style="color:#d01e1e">stale ({state_age:.1f} s)</b>')
        else:
            self._labels["link"].setText(f'<span style="color:#1a8f3c">ok ({1000 * state_age:.0f} ms)</span>')
