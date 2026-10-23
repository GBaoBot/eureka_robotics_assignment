"""MainWindow: goal picker, Start/Stop, and the views.

The only GUI piece that reads data, and only through MissionExecutor's interface:
goals, start() / cancel() / running / error, telemetry, robot_state() / raw_robot_state() /
state_age() / localization_sigma() and recording_status(). A QTimer polls the latest values and hands them to the views,
so the views stay pure renderers and the mission thread never touches Qt.
"""
from __future__ import annotations

from typing import TYPE_CHECKING

from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtWidgets import (QAbstractItemView, QDockWidget, QHBoxLayout, QListWidget, QListWidgetItem,
                             QMainWindow, QPushButton, QVBoxLayout, QWidget)

from visualizer.log_panel import LogPanel
from visualizer.map_view import MapView
from visualizer.status_panel import StatusPanel

if TYPE_CHECKING:  # type hint only: the window depends on the executor's interface, not its imports
    from mission_executor.mission_executor import MissionExecutor


class MainWindow(QMainWindow):
    def __init__(self, executor: MissionExecutor, refresh_hz: float = 30.0):
        super().__init__()
        self.setWindowTitle("Robot vacuum")
        self.resize(1280, 800)
        self._executor = executor
        goals = executor.goals
        self._map_version = -1
        self._path = None
        self._active_goal = None

        self.map_view = MapView()
        self.map_view.set_goals(goals)
        self.setCentralWidget(self.map_view)

        # Right: goals, buttons, status.
        self._goal_list = QListWidget()
        self._goal_list.setSelectionMode(QAbstractItemView.SelectionMode.ExtendedSelection)
        for name, x, y, yaw in goals:
            item = QListWidgetItem(f"{name}   ({x:.2f}, {y:.2f}{'' if yaw is None else f', {yaw:.2f} rad'})")
            item.setData(Qt.ItemDataRole.UserRole, name)
            self._goal_list.addItem(item)
        self._goal_list.itemDoubleClicked.connect(lambda item: self._start([item.data(Qt.ItemDataRole.UserRole)]))

        self._start_button = QPushButton("Go to selected")
        self._start_button.clicked.connect(self._start_selected)
        self._all_button = QPushButton("Go to all")
        self._all_button.clicked.connect(lambda: self._start([g[0] for g in goals]))
        self._stop_button = QPushButton("Stop")
        self._stop_button.clicked.connect(self._executor.cancel)
        fit_button = QPushButton("Fit map")
        fit_button.clicked.connect(self.map_view.fit_map)
        buttons = QHBoxLayout()
        for b in (self._start_button, self._all_button, self._stop_button, fit_button):
            buttons.addWidget(b)

        self.status_panel = StatusPanel()
        side = QWidget()
        side_layout = QVBoxLayout(side)
        side_layout.addWidget(self._goal_list)
        side_layout.addLayout(buttons)
        side_layout.addWidget(self.status_panel)
        side_layout.addStretch()
        self._add_dock("Mission", side, Qt.DockWidgetArea.RightDockWidgetArea)

        # Bottom: log.
        self.log_panel = LogPanel()
        self._add_dock("Log", self.log_panel, Qt.DockWidgetArea.BottomDockWidgetArea)

        self._timer = QTimer(self)
        self._timer.timeout.connect(self._refresh)
        self._timer.start(int(1000 / refresh_hz))
        self._refresh()

    def _add_dock(self, title: str, widget: QWidget, area: Qt.DockWidgetArea) -> None:
        dock = QDockWidget(title, self)
        dock.setWidget(widget)
        dock.setFeatures(QDockWidget.DockWidgetFeature.DockWidgetMovable
                         | QDockWidget.DockWidgetFeature.DockWidgetFloatable)
        self.addDockWidget(area, dock)

    # ----------------------------------------------------------------- commands

    def _start_selected(self) -> None:
        rows = sorted(self._goal_list.row(item) for item in self._goal_list.selectedItems())
        self._start([self._goal_list.item(r).data(Qt.ItemDataRole.UserRole) for r in rows])

    def _start(self, goal_names: list[str]) -> None:
        if goal_names and self._executor.start(goal_names):
            self.statusBar().showMessage(f"mission: {', '.join(goal_names)}")

    # ----------------------------------------------------------------- refresh

    def _refresh(self) -> None:
        ex = self._executor
        state = ex.robot_state()
        pose = state.pose if state is not None else None
        self.map_view.set_robot_pose(pose)
        raw = ex.raw_robot_state()
        self.map_view.set_raw_pose(raw.pose if raw is not None else None)
        self.status_panel.show_robot(pose, ex.state_age())
        self.status_panel.show_localization(ex.localization_sigma())

        t = ex.telemetry
        # map_version is bumped after costmap.costs is replaced, so a new version means new costs.
        if t.costmap is not None and t.map_version != self._map_version:
            self._map_version = t.map_version
            self.map_view.set_costmap(t.costmap.costs, t.costmap.resolution, t.costmap.origin)
        path = t.path
        if path is not self._path:  # a replan always makes a new Path object
            self._path = path
            self.map_view.set_path(None if path is None else [(p.x, p.y) for p in path.poses])
        if t.goal_name != self._active_goal:
            self._active_goal = t.goal_name
            self.map_view.set_active_goal(t.goal_name)
        self.map_view.set_collisions(t.collisions)
        cmd = t.cmd
        self.status_panel.show_mission(t.goal_name, t.status, t.reason, cmd.v, cmd.w, len(t.collisions), t.results)

        self.status_panel.show_recording(ex.recording_status())

        running = ex.running
        self._start_button.setEnabled(not running)
        self._all_button.setEnabled(not running)
        self._stop_button.setEnabled(running)
        if not running and ex.error:
            self.statusBar().showMessage(f"mission aborted: {ex.error}")

    def closeEvent(self, event) -> None:
        # Stop the robot now; app.py's `with executor` then finalises the recording and closes the drivers.
        self._timer.stop()
        self._executor.cancel()
        super().closeEvent(event)
