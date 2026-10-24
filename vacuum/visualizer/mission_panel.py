"""MissionPanel: scene picker, goal list and Start/Stop buttons.

Owns the mission commands: its buttons call the executor directly (load_scene(), start(),
cancel()), and refresh() reads scenes / scene / goals / running / error to keep the list
and buttons current. Messages for the window's status bar leave as a signal.
"""
from __future__ import annotations

from typing import TYPE_CHECKING

from PyQt6.QtCore import Qt, pyqtSignal
from PyQt6.QtGui import QIntValidator
from PyQt6.QtWidgets import (QAbstractItemView, QComboBox, QHBoxLayout, QLabel, QLineEdit, QListWidget,
                             QListWidgetItem, QPushButton, QVBoxLayout, QWidget)

if TYPE_CHECKING:  # type hint only: the panel depends on the executor's interface, not its imports
    from vacuum.mission_executor.mission_executor import MissionExecutor


class MissionPanel(QWidget):
    status_message = pyqtSignal(str)  # what the mission is doing, for the window's status bar
    fit_map_requested = pyqtSignal()

    def __init__(self, executor: MissionExecutor, parent=None):
        super().__init__(parent)
        self._executor = executor
        self._goals = None  # the executor's goal list we show; a new list object means a new scene
        self._loading = False
        self._message = ""

        self._scene_box = QComboBox()
        for scene_id, name, difficulty in executor.scenes:
            self._scene_box.addItem(f"{scene_id}  {name}  ({difficulty})", scene_id)
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
        self._all_button.clicked.connect(lambda: self._start([g.name for g in self._goals or ()]))
        self._stop_button = QPushButton("Stop")
        self._stop_button.clicked.connect(self._executor.cancel)
        fit_button = QPushButton("Fit map")
        fit_button.clicked.connect(self.fit_map_requested)
        buttons = QHBoxLayout()
        for b in (self._start_button, self._all_button, self._stop_button, fit_button):
            buttons.addWidget(b)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        if executor.scenes:  # an executor that can't list scenes can't switch them either
            layout.addLayout(scene_row)
        layout.addWidget(self._goal_list)
        layout.addLayout(buttons)

    # ----------------------------------------------------------------- commands

    def _start_selected(self) -> None:
        rows = sorted(self._goal_list.row(item) for item in self._goal_list.selectedItems())
        self._start([self._goal_list.item(r).data(Qt.ItemDataRole.UserRole) for r in rows])

    def _start(self, goal_names: list[str]) -> None:
        if goal_names and self._executor.start(goal_names):
            self._show_message(f"mission: {', '.join(goal_names)}")

    def _load_scene(self) -> None:
        scene_id = self._scene_box.currentData()
        seed = int(self._seed_edit.text()) if self._seed_edit.text() else None
        if scene_id is not None and self._executor.load_scene(scene_id, seed):
            self._loading = True
            self._show_message(f"loading scene {scene_id}" + (f" (seed {seed})" if seed is not None else "") + " ...")

    def _show_message(self, message: str) -> None:
        if message != self._message:  # refresh() runs at 30 Hz: only pass on changes
            self._message = message
            self.status_message.emit(message)

    # ----------------------------------------------------------------- refresh

    def refresh(self) -> None:
        ex = self._executor
        if ex.goals is not self._goals:  # first refresh, or a scene change finished
            self._show_scene(ex.goals)

        running = ex.running
        if self._loading and not running:
            self._loading = False
            if not ex.error:
                self._show_message(f"scene {ex.scene} loaded")
        for widget in (self._start_button, self._all_button, self._load_button, self._scene_box, self._seed_edit):
            widget.setEnabled(not running)
        self._stop_button.setEnabled(running and not self._loading)  # a scene load can't be cancelled
        if not running and ex.error:
            self._show_message(f"error: {ex.error}")

    def _show_scene(self, goals) -> None:
        self._goals = goals
        self._goal_list.clear()
        for name, x, y, yaw in goals:
            item = QListWidgetItem(f"{name}   ({x:.2f}, {y:.2f}{'' if yaw is None else f', {yaw:.2f} rad'})")
            item.setData(Qt.ItemDataRole.UserRole, name)
            self._goal_list.addItem(item)
        index = self._scene_box.findData(self._executor.scene)
        if index >= 0:
            self._scene_box.setCurrentIndex(index)
