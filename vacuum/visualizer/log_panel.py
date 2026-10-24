"""LogPanel: the application log in the window.

The mission already logs why it does things ("path blocked", "bumper: marked obstacle
at ..."), so showing the log needs no coupling to the mission at all: attach
panel.handler() to the root logger. Records can come from any thread; a queued Qt
signal carries them to the GUI thread.
"""
from __future__ import annotations

import logging

from PyQt6.QtCore import QObject, pyqtSignal
from PyQt6.QtWidgets import QPlainTextEdit

LEVEL_COLORS = {logging.WARNING: "#b07800", logging.ERROR: "#d01e1e", logging.CRITICAL: "#d01e1e"}


class _Bridge(QObject):
    record = pyqtSignal(str, int)  # message, level


class _QtLogHandler(logging.Handler):
    def __init__(self, bridge: _Bridge, level: int):
        super().__init__(level)
        self._bridge = bridge
        self.setFormatter(logging.Formatter("%(asctime)s %(name)s: %(message)s", datefmt="%H:%M:%S"))

    def emit(self, record: logging.LogRecord) -> None:
        try:
            self._bridge.record.emit(self.format(record), record.levelno)
        except RuntimeError:  # widget already destroyed while the app shuts down
            pass


class LogPanel(QPlainTextEdit):
    def __init__(self, max_lines: int = 1000, parent=None):
        super().__init__(parent)
        self.setReadOnly(True)
        self.setMaximumBlockCount(max_lines)
        self._bridge = _Bridge(self)
        self._bridge.record.connect(self._append)

    def handler(self, level: int = logging.INFO) -> logging.Handler:
        return _QtLogHandler(self._bridge, level)

    def _append(self, message: str, level: int) -> None:
        color = LEVEL_COLORS.get(level)
        text = message.replace("&", "&amp;").replace("<", "&lt;")
        self.appendHtml(f'<span style="color:{color}">{text}</span>' if color else text)
