"""MainWindow: assembles the panels and drives their refresh.

Each panel reads what it shows from MissionExecutor itself and owns its own commands:
MapView (centre), MissionPanel and StatusPanel (right), LogPanel (bottom). The window
only creates them, places them, connects the few signals that cross panels, and runs
one QTimer that calls every panel's refresh(), so the mission thread never touches Qt.
"""
from __future__ import annotations

from typing import TYPE_CHECKING

from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtWidgets import QDockWidget, QMainWindow, QVBoxLayout, QWidget

from visualizer.log_panel import LogPanel
from visualizer.map_view import MapView
from visualizer.mission_panel import MissionPanel
from visualizer.status_panel import StatusPanel

if TYPE_CHECKING:  # type hint only: the window depends on the executor's interface, not its imports
    from mission_executor.mission_executor import MissionExecutor


class MainWindow(QMainWindow):
    def __init__(self, executor: MissionExecutor, refresh_hz: float = 30.0):
        super().__init__()
        self.setWindowTitle("Robot vacuum")
        self.resize(1280, 800)
        self._executor = executor

        # Centre: Map View
        self.map_view = MapView(executor)
        self.setCentralWidget(self.map_view)

        # Top Right: Mission Panel
        self.mission_panel = MissionPanel(executor)
        self.mission_panel.status_message.connect(self.statusBar().showMessage)
        self.mission_panel.fit_map_requested.connect(self.map_view.fit_map)

        # Bottom Right: Status Panel
        self.status_panel = StatusPanel(executor)
        side = QWidget()
        side_layout = QVBoxLayout(side)
        side_layout.addWidget(self.mission_panel)
        side_layout.addWidget(self.status_panel)
        side_layout.addStretch()
        self._add_dock("Mission", side, Qt.DockWidgetArea.RightDockWidgetArea)

        # Bottom: Log
        self.log_panel = LogPanel()
        self._add_dock("Log", self.log_panel, Qt.DockWidgetArea.BottomDockWidgetArea)

        # Main Window settings
        self._refreshed = (self.map_view, self.mission_panel, self.status_panel)
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

    def _refresh(self) -> None:
        for panel in self._refreshed:
            panel.refresh()

    def closeEvent(self, event) -> None:
        # Stop the robot now; app.py's `with executor` then finalises the recording and closes the drivers.
        self._timer.stop()
        self._executor.cancel()
        super().closeEvent(event)
