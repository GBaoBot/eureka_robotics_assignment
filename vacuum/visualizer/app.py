"""Qt front end: visualise and drive the robot.

Knows only MissionExecutor, the entry point to the robot system.

    python3 -m vacuum.visualizer.app --scene 01 --seed 42
    python3 -m vacuum.visualizer.app --scene 01 --no-record
    python3 -m vacuum.visualizer.app               # keep the scene the simulator has loaded
"""
from __future__ import annotations

import argparse
import logging
import signal
import sys
from typing import Optional

from PyQt6.QtCore import QTimer
from PyQt6.QtWidgets import QApplication

from vacuum.mission_executor.mission_executor import ExecutorConfig, MissionExecutor
from vacuum.visualizer.main_window import MainWindow

log = logging.getLogger("visualizer")


def parse_args(argv: Optional[list[str]] = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Visualise and drive the simulated robot.")
    ExecutorConfig.add_arguments(p)
    p.add_argument("-v", "--verbose", action="store_true", help="debug logging")
    return p.parse_args(argv)


def main(argv: Optional[list[str]] = None) -> int:
    args = parse_args(argv)
    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO,
                        format="%(asctime)s %(levelname)-7s %(name)s: %(message)s", datefmt="%H:%M:%S")
    try:
        executor = MissionExecutor.connect(ExecutorConfig.from_args(args))
    except ConnectionError as e:
        log.error("%s", e)
        return 2

    with executor:  # on exit: stops the mission and its recording, closes the drivers
        app = QApplication(sys.argv[:1])
        window = MainWindow(executor)
        handler = window.log_panel.handler()
        logging.getLogger().addHandler(handler)

        # Ctrl-C in the terminal closes the window like the close button. Qt's event loop
        # doesn't run Python signal handlers by itself, so wake Python up regularly.
        signal.signal(signal.SIGINT, lambda *_: window.close())
        wakeup = QTimer()
        wakeup.timeout.connect(lambda: None)
        wakeup.start(200)

        window.show()
        try:
            return app.exec()
        finally:
            logging.getLogger().removeHandler(handler)


if __name__ == "__main__":
    sys.exit(main())
