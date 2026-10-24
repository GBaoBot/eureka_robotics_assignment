"""MapView: top-down 2D view of the costmap, robot, trail, raw pose, goals, path and collisions.

refresh() reads goals, poses and telemetry from MissionExecutor and redraws only what
changed; the set_*() methods below it draw plain data (numpy costs, (x, y) tuples, poses).
Everything is drawn in world metres; the view flips y so +y is up and +x is right, as in
the world frame. Scroll to zoom, drag to pan.
"""
from __future__ import annotations

import math
from collections import deque
from typing import TYPE_CHECKING, Optional, Sequence

import numpy as np
from PyQt6.QtCore import Qt
from PyQt6.QtGui import QBrush, QColor, QImage, QPainter, QPainterPath, QPen, QPixmap, QTransform
from PyQt6.QtWidgets import (QGraphicsEllipseItem, QGraphicsItem, QGraphicsLineItem, QGraphicsPathItem,
                             QGraphicsPixmapItem, QGraphicsScene, QGraphicsSimpleTextItem, QGraphicsView)

from vacuum.common.data_types import Pose

if TYPE_CHECKING:  # type hint only: the view depends on the executor's interface, not its imports
    from vacuum.mission_executor.mission_executor import MissionExecutor

# Costmap colours (Nav2 cost convention, see navigation/costmap.py).
FREE_COLOR = QColor(250, 250, 250)
INFLATION_NEAR, INFLATION_FAR = QColor(150, 170, 230), QColor(235, 240, 252)  # cost 252 .. 1
INSCRIBED_COLOR = QColor(200, 200, 215)  # robot centre here would touch an obstacle
LETHAL_COLOR = QColor(40, 40, 40)

PATH_COLOR = QColor(0, 150, 60)
TRAIL_COLOR = QColor(255, 140, 0)
RAW_POSE_COLOR = QColor(110, 110, 110)  # the simulator's unfiltered pose
ROBOT_COLOR = QColor(30, 100, 220)
GOAL_COLOR, ACTIVE_GOAL_COLOR = QColor(120, 120, 120), QColor(220, 30, 30)
COLLISION_COLOR = QColor(220, 30, 30)


def _color_table() -> list[int]:
    table = [FREE_COLOR.rgb()]
    for cost in range(1, 253):  # inflation: fade from near (252) to far (1)
        s = cost / 252.0
        table.append(QColor(*(round(f + (n - f) * s) for n, f in
                              zip(INFLATION_NEAR.getRgb()[:3], INFLATION_FAR.getRgb()[:3]))).rgb())
    table += [INSCRIBED_COLOR.rgb(), LETHAL_COLOR.rgb(), LETHAL_COLOR.rgb()]
    return table


def _pen(color: QColor, width: float = 2.0, style=Qt.PenStyle.SolidLine) -> QPen:
    pen = QPen(color, width, style)
    pen.setCosmetic(True)  # width in screen pixels, whatever the zoom
    return pen


class MapView(QGraphicsView):
    def __init__(self, executor: MissionExecutor, robot_radius: float = 0.17, trail_length: int = 10000,
                 parent=None):
        super().__init__(parent)
        self._executor = executor
        self._goals = None  # the executor's goal list we show; a new list object means a new scene
        self._map_version = -1
        self._shown_path = None
        self._active_goal = None
        self.setScene(QGraphicsScene(self))
        self.setRenderHint(QPainter.RenderHint.Antialiasing)
        self.setDragMode(QGraphicsView.DragMode.ScrollHandDrag)
        self.setTransformationAnchor(QGraphicsView.ViewportAnchor.AnchorUnderMouse)
        self.setBackgroundBrush(QBrush(QColor(225, 225, 225)))
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)  # drag to pan instead
        self.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.scale(1.0, -1.0)  # world y up
        self._color_table = _color_table()
        self._fitted = False

        scene = self.scene()
        self._map_item = QGraphicsPixmapItem()
        self._map_item.setZValue(0)
        self._path_item = QGraphicsPathItem()
        self._path_item.setPen(_pen(PATH_COLOR, 2.5))
        self._path_item.setZValue(2)
        self._trail: list[tuple[float, float]] = []  # one point per 2 cm driven
        self._trail_length = trail_length
        self._trail_path = QPainterPath()
        self._trail_item = QGraphicsPathItem()
        self._trail_item.setPen(_pen(TRAIL_COLOR, 1.5))
        self._trail_item.setZValue(1)
        self._raw = deque(maxlen=50)  # last second of raw poses at 50 Hz
        self._raw_item = QGraphicsPathItem()
        self._raw_item.setPen(_pen(RAW_POSE_COLOR, 1.0))
        self._raw_item.setZValue(3)
        for item in (self._map_item, self._path_item, self._trail_item, self._raw_item):
            scene.addItem(item)

        # Robot: disc + heading line, positioned and rotated as one item.
        r = robot_radius
        self._robot = QGraphicsEllipseItem(-r, -r, 2 * r, 2 * r)
        self._robot.setPen(_pen(ROBOT_COLOR, 2.0))
        self._robot.setBrush(QBrush(QColor(ROBOT_COLOR.red(), ROBOT_COLOR.green(), ROBOT_COLOR.blue(), 60)))
        heading = QGraphicsLineItem(0.0, 0.0, r, 0.0, self._robot)
        heading.setPen(_pen(ROBOT_COLOR.darker(160), 3.0))
        self._robot.setZValue(5)
        self._robot.setVisible(False)
        scene.addItem(self._robot)

        self._goal_items: dict[str, QGraphicsEllipseItem] = {}
        self._collision_items: list[QGraphicsItem] = []

    # ----------------------------------------------------------------- refresh

    def refresh(self) -> None:
        ex = self._executor
        if ex.goals is not self._goals:  # first refresh, or a scene change finished
            self._goals = ex.goals
            self.clear_scene()
            self.set_goals(ex.goals)
            self._map_version, self._shown_path, self._active_goal = -1, None, None  # redraw from the new telemetry
        state = ex.robot_state()
        self.set_robot_pose(state.pose if state is not None else None)
        raw = ex.raw_robot_state()
        self.set_raw_pose(raw.pose if raw is not None else None)

        t = ex.telemetry
        # map_version is bumped after costmap.costs is replaced, so a new version means new costs.
        if t.costmap is not None and t.map_version != self._map_version:
            self._map_version = t.map_version
            self.set_costmap(t.costmap.costs, t.costmap.resolution, t.costmap.origin)
        path = t.path
        if path is not self._shown_path:  # a replan always makes a new Path object
            self._shown_path = path
            self.set_path(None if path is None else [(p.x, p.y) for p in path.poses])
        if t.goal_name != self._active_goal:
            self._active_goal = t.goal_name
            self.set_active_goal(t.goal_name)
        self.set_collisions(t.collisions)

    # ----------------------------------------------------------------- data in

    def set_costmap(self, costs: np.ndarray, resolution: float, origin: tuple[float, float]) -> None:
        """costs: (H, W) uint8, row 0 = largest y (Costmap2D layout)."""
        h, w = costs.shape
        data = np.ascontiguousarray(costs, dtype=np.uint8)
        image = QImage(data.data, w, h, w, QImage.Format.Format_Indexed8)
        image.setColorTable(self._color_table)
        self._map_item.setPixmap(QPixmap.fromImage(image))  # copies, so `data` may go
        # Pixel (row r, col c) covers x = ox + c*res, y = oy + (H-1-r)*res: image top-left is (ox, oy + H*res).
        self._map_item.setTransform(QTransform.fromScale(resolution, -resolution))
        self._map_item.setPos(origin[0], origin[1] + h * resolution)
        if not self._fitted:
            self._fitted = True
            self.scene().setSceneRect(self._map_item.sceneBoundingRect().adjusted(-1, -1, 1, 1))
            self.fit_map()

    def set_goals(self, goals: Sequence[tuple[str, float, float, Optional[float]]]) -> None:
        """(name, x, y, yaw-or-None) for every scene goal."""
        for item in self._goal_items.values():
            self.scene().removeItem(item)
        self._goal_items.clear()
        for name, x, y, yaw in goals:
            item = QGraphicsEllipseItem(-0.05, -0.05, 0.1, 0.1)  # 5 cm: the arrival tolerance
            item.setPos(x, y)
            item.setZValue(4)
            if yaw is not None:
                arrow = QGraphicsLineItem(0.0, 0.0, 0.25 * math.cos(yaw), 0.25 * math.sin(yaw), item)
                arrow.setPen(_pen(GOAL_COLOR, 2.0))
            label = QGraphicsSimpleTextItem(name, item)
            label.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIgnoresTransformations)  # upright, fixed size
            label.setPos(0.07, 0.0)
            self.scene().addItem(item)
            self._goal_items[name] = item
        self.set_active_goal(None)

    def set_active_goal(self, name: Optional[str]) -> None:
        for goal_name, item in self._goal_items.items():
            color = ACTIVE_GOAL_COLOR if goal_name == name else GOAL_COLOR
            item.setPen(_pen(color, 2.0))
            item.setBrush(QBrush(color))
            for child in item.childItems():
                if isinstance(child, QGraphicsLineItem):
                    child.setPen(_pen(color, 2.0))
                else:
                    child.setBrush(QBrush(color))

    def set_path(self, points: Optional[Sequence[tuple[float, float]]]) -> None:
        self._path_item.setPath(self._polyline(points or ()))

    def set_robot_pose(self, pose: Optional[Pose]) -> None:
        """Move the robot and extend the trail; None hides the robot (no state yet)."""
        if pose is None:
            self._robot.setVisible(False)
            return
        self._robot.setVisible(True)
        self._robot.setPos(pose.x, pose.y)
        self._robot.setRotation(math.degrees(pose.yaw))  # CCW on screen, since the view flips y
        if self._trail and math.hypot(pose.x - self._trail[-1][0], pose.y - self._trail[-1][1]) < 0.02:
            return
        self._trail.append((pose.x, pose.y))
        if len(self._trail) > self._trail_length:  # drop the oldest half at once, not one point per tick
            del self._trail[:self._trail_length // 2]
            self._trail_path = self._polyline(self._trail)
        elif len(self._trail) == 1:
            self._trail_path.moveTo(pose.x, pose.y)
        else:
            self._trail_path.lineTo(pose.x, pose.y)
        self._trail_item.setPath(self._trail_path)

    def set_raw_pose(self, pose: Optional[Pose]) -> None:
        """The unfiltered pose, drawn as a thin line through its last second: shows the noise the EKF removes."""
        if pose is not None and (not self._raw or self._raw[-1] != (pose.x, pose.y)):
            self._raw.append((pose.x, pose.y))
        self._raw_item.setPath(self._polyline(self._raw))

    def set_collisions(self, points: Sequence[tuple[float, float]]) -> None:
        """Bumper contact positions; only new ones are added."""
        for x, y in points[len(self._collision_items):]:
            item = QGraphicsPathItem(self._cross(x, y, 0.06))
            item.setPen(_pen(COLLISION_COLOR, 2.5))
            item.setZValue(6)
            self.scene().addItem(item)
            self._collision_items.append(item)

    def clear_trail(self) -> None:
        self._trail.clear()
        self._trail_path = QPainterPath()
        self._trail_item.setPath(self._trail_path)

    def clear_scene(self) -> None:
        """Forget everything from the previous scene; the next costmap is fitted to the view."""
        self.clear_trail()
        self._raw.clear()
        self._raw_item.setPath(QPainterPath())
        self.set_path(None)
        for item in self._collision_items:
            self.scene().removeItem(item)
        self._collision_items.clear()
        self._robot.setVisible(False)
        self._fitted = False

    # ----------------------------------------------------------------- interaction

    def fit_map(self) -> None:
        if not self._map_item.pixmap().isNull():
            self.fitInView(self._map_item, Qt.AspectRatioMode.KeepAspectRatio)

    def wheelEvent(self, event) -> None:
        factor = 1.15 if event.angleDelta().y() > 0 else 1.0 / 1.15
        self.scale(factor, factor)

    def mouseDoubleClickEvent(self, event) -> None:
        self.fit_map()

    # ----------------------------------------------------------------- helpers

    @staticmethod
    def _polyline(points) -> QPainterPath:
        path = QPainterPath()
        for i, (x, y) in enumerate(points):
            if i == 0:
                path.moveTo(x, y)
            else:
                path.lineTo(x, y)
        return path

    @staticmethod
    def _cross(x: float, y: float, s: float) -> QPainterPath:
        path = QPainterPath()
        path.moveTo(x - s, y - s)
        path.lineTo(x + s, y + s)
        path.moveTo(x - s, y + s)
        path.lineTo(x + s, y - s)
        return path
