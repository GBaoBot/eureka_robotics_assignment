"""MainWindow: scene picker, goal picker, Start/Stop, and the views.

The only GUI piece that reads data, and only through MissionExecutor's interface:
scenes / scene / load_scene(), goals, start() / cancel() / running / error, telemetry,
robot_state() / raw_robot_state() / state_age() / localization_sigma() and recording_status().
A QTimer polls the latest values and hands them to the views, so the views stay pure
renderers and the mission thread never touches Qt.
"""
from __future__ import annotations

from typing import TYPE_CHECKING

from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui import QIntValidator
from PyQt6.QtWidgets import (QAbstractItemView, QComboBox, QDockWidget, QHBoxLayout, QLabel, QLineEdit, QListWidget,
                             QListWidgetItem, QMainWindow, QPushButton, QVBoxLayout, QWidget)

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
        self._goals = None  # the executor's goal list we show; a new list object means a new scene
        self._map_version = -1
        self._path = None
        self._active_goal = None
        self._loading = False

        self.map_view = MapView()
        self.setCentralWidget(self.map_view)

        # Right: scene, goals, buttons, status.
        self._scene_box = QComboBox()
        for sc in executor.scenes:
            self._scene_box.addItem(f"{sc.id}  {sc.name}  ({sc.difficulty})", sc.id)
        self._seed_edit = QLineEdit()
        self._seed_edit.setPlaceholderText("random")
        self._seed_edit.setValidator(QIntValidator(0, 2 ** 31 - 1))
        self._seed_edit.setMaximumWidth(90)
        self._load_button = QPushButton("Load scene")
        self._load_button.clicked.connect(self._load_scene)
        scene_row = QHBoxLayout()
        scene_row.addWidget(self._scene_box, 1)
        scene_row.addWidget(QLabel("Seed:"))
        scene_row.addWidget(self._seed_edit)
        scene_row.addWidget(self._load_button)

        self._goal_list = QListWidget()
        self._goal_list.setSelectionMode(QAbstractItemView.SelectionMode.ExtendedSelection)
        self._goal_list.itemDoubleClicked.connect(lambda item: self._start([item.data(Qt.ItemDataRole.UserRole)]))

        self._start_button = QPushButton("Go to selected")
        self._start_button.clicked.connect(self._start_selected)
        self._all_button = QPushButton("Go to all")
        self._all_button.clicked.connect(lambda: self._start([g.name for g in self._goals]))
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
        if executor.scenes:  # an executor that can't list scenes can't switch them either
            side_layout.addLayout(scene_row)
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

    def _load_scene(self) -> None:
        scene_id = self._scene_box.currentData()
        seed = int(self._seed_edit.text()) if self._seed_edit.text() else None
        if scene_id is not None and self._executor.load_scene(scene_id, seed):
            self._loading = True
            self.statusBar().showMessage(f"loading scene {scene_id}" + (f" (seed {seed})" if seed is not None else "") + " ...")

    def _show_scene(self, goals) -> None:
        """A new scene was loaded: new goal list, and nothing from the old map on screen."""
        self._goals = goals
        self._goal_list.clear()
        for name, x, y, yaw in goals:
            item = QListWidgetItem(f"{name}   ({x:.2f}, {y:.2f}{'' if yaw is None else f', {yaw:.2f} rad'})")
            item.setData(Qt.ItemDataRole.UserRole, name)
            self._goal_list.addItem(item)
        self.map_view.clear_scene()
        self.map_view.set_goals(goals)
        self._map_version, self._path, self._active_goal = -1, None, None  # redraw from the new telemetry
        index = self._scene_box.findData(self._executor.scene)
        if index >= 0:
            self._scene_box.setCurrentIndex(index)

    # ----------------------------------------------------------------- refresh

    def _refresh(self) -> None:
        ex = self._executor
        if ex.goals is not self._goals:  # first refresh, or a scene change finished
            self._show_scene(ex.goals)
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
        if self._loading and not running:
            self._loading = False
            if not ex.error:
                self.statusBar().showMessage(f"scene {ex.scene} loaded")
        for widget in (self._start_button, self._all_button, self._load_button, self._scene_box, self._seed_edit):
            widget.setEnabled(not running)
        self._stop_button.setEnabled(running and not self._loading)  # a scene load can't be cancelled
        if not running and ex.error:
            self.statusBar().showMessage(f"error: {ex.error}")

    def closeEvent(self, event) -> None:
        # Stop the robot now; app.py's `with executor` then finalises the recording and closes the drivers.
        self._timer.stop()
        self._executor.cancel()
        super().closeEvent(event)
