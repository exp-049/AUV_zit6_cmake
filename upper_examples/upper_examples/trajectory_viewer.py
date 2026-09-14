#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""RViz-like 3D trajectory and pose monitor for the ZIT6 console.

The firmware already publishes the complete world-frame pose on
``/zit6/state/pos`` as ``[x, y, z, roll, pitch, yaw]``.  This module keeps the
visualisation dependency-free: the 3D scene is projected and drawn with Qt,
so the console does not require pyqtgraph or Python OpenGL bindings.
"""

import math

from std_msgs.msg import Float32MultiArray
from zit6_interfaces.msg import ZitStatus

try:
    from PyQt5.QtCore import Qt, pyqtSignal, QPoint
    from PyQt5.QtGui import QColor, QFont, QPainter, QPen, QPolygon
    from PyQt5.QtWidgets import (
        QCheckBox, QFormLayout, QGridLayout, QGroupBox, QHBoxLayout,
        QLabel, QPushButton, QSpinBox, QVBoxLayout, QWidget,
    )
except ImportError:
    from PySide6.QtCore import Qt, Signal as pyqtSignal, QPoint
    from PySide6.QtGui import QColor, QFont, QPainter, QPen, QPolygon
    from PySide6.QtWidgets import (
        QCheckBox, QFormLayout, QGridLayout, QGroupBox, QHBoxLayout,
        QLabel, QPushButton, QSpinBox, QVBoxLayout, QWidget,
    )


def _finite_pose(values):
    """Return a finite six-element pose, or ``None`` for malformed input."""
    if len(values) < 6:
        return None
    pose = [float(value) for value in values[:6]]
    return pose if all(math.isfinite(value) for value in pose) else None


class Pose3DCanvas(QWidget):
    """Small interactive 3D canvas with an RViz-style path and body triad."""

    # NED-friendly oblique view: +N is up, +E is right, +D is down.
    DEFAULT_AZIMUTH = -90.0
    DEFAULT_ELEVATION = -28.0

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(520, 420)
        self.setMouseTracking(True)
        self.setFocusPolicy(Qt.StrongFocus)
        self.points = []
        self.pose = None
        self.max_points = 3000
        self.azimuth = self.DEFAULT_AZIMUTH
        self.elevation = self.DEFAULT_ELEVATION
        self.zoom = 1.0
        self.follow = True
        self.show_grid = True
        self._drag_pos = None

    def set_pose(self, pose):
        self.pose = list(pose)
        self.points.append(tuple(pose[:3]))
        if len(self.points) > self.max_points:
            del self.points[:len(self.points) - self.max_points]
        self.update()

    def clear(self):
        self.points.clear()
        self.pose = None
        self.update()

    def set_max_points(self, value):
        self.max_points = max(10, int(value))
        if len(self.points) > self.max_points:
            del self.points[:len(self.points) - self.max_points]
        self.update()

    def set_follow(self, checked):
        self.follow = bool(checked)
        self.update()

    def set_grid(self, checked):
        self.show_grid = bool(checked)
        self.update()

    def reset_view(self):
        """Restore the NED-friendly view and the default zoom."""
        self.azimuth = self.DEFAULT_AZIMUTH
        self.elevation = self.DEFAULT_ELEVATION
        self.zoom = 1.0
        self.update()

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            self._drag_pos = event.pos()
            self.setCursor(Qt.ClosedHandCursor)
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event):
        if self._drag_pos is not None:
            delta = event.pos() - self._drag_pos
            self.azimuth += delta.x() * 0.6
            self.elevation = max(-85.0, min(85.0, self.elevation + delta.y() * 0.6))
            self._drag_pos = event.pos()
            self.update()
        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.LeftButton:
            self._drag_pos = None
            self.setCursor(Qt.ArrowCursor)
        super().mouseReleaseEvent(event)

    def wheelEvent(self, event):
        delta = event.angleDelta().y()
        self.zoom *= 1.15 if delta > 0 else 1.0 / 1.15
        self.zoom = max(0.15, min(12.0, self.zoom))
        self.update()
        super().wheelEvent(event)

    def _center(self):
        if not self.follow or not self.points:
            return (0.0, 0.0, 0.0)
        # Follow the vehicle, while preserving a little room around it.
        return self.points[-1]

    def _project(self, point, center, scale):
        x, y, z = (point[i] - center[i] for i in range(3))
        az = math.radians(self.azimuth)
        el = math.radians(self.elevation)
        horizontal = math.cos(az) * x - math.sin(az) * y
        depth = math.sin(az) * x + math.cos(az) * y
        # Z is NED (positive down), hence positive Z is drawn downward.
        screen_x = self.width() * 0.5 + horizontal * scale
        screen_y = self.height() * 0.5 + (math.cos(el) * z - math.sin(el) * depth) * scale
        return QPoint(round(screen_x), round(screen_y))

    def _scene_scale(self):
        if not self.points:
            return 45.0 * self.zoom
        center = self._center()
        radius = max(
            math.sqrt(sum((p[i] - center[i]) ** 2 for i in range(3)))
            for p in self.points
        )
        if self.pose:
            radius = max(radius, 1.0)
        return min(self.width(), self.height()) * 0.34 / max(radius, 1.0) * self.zoom

    def _draw_line_3d(self, painter, start, end, center, scale, color, width=1):
        painter.setPen(QPen(color, width))
        painter.drawLine(self._project(start, center, scale), self._project(end, center, scale))

    def _draw_axis_3d(self, painter, start, end, center, scale, color, label,
                      style=Qt.SolidLine, width=2):
        """Draw a labeled 3D axis with an arrowhead in screen space."""
        start_px = self._project(start, center, scale)
        end_px = self._project(end, center, scale)
        pen = QPen(color, width)
        pen.setStyle(style)
        painter.setPen(pen)
        painter.setBrush(Qt.NoBrush)
        painter.drawLine(start_px, end_px)

        dx = end_px.x() - start_px.x()
        dy = end_px.y() - start_px.y()
        length = math.hypot(dx, dy)
        if length > 1.0:
            ux, uy = dx / length, dy / length
            px, py = -uy, ux
            base_x = end_px.x() - ux * 10.0
            base_y = end_px.y() - uy * 10.0
            arrow = QPolygon([
                end_px,
                QPoint(round(base_x + px * 4.0), round(base_y + py * 4.0)),
                QPoint(round(base_x - px * 4.0), round(base_y - py * 4.0)),
            ])
            painter.setPen(QPen(color, 1))
            painter.setBrush(color)
            painter.drawPolygon(arrow)

        painter.setPen(QPen(color, 1))
        painter.setFont(QFont("Sans", 9, QFont.Bold))
        painter.drawText(end_px + QPoint(6, -6), label)

    def _draw_ned_compass(self, painter):
        """Draw a fixed screen-space compass so screen direction is unambiguous."""
        box_x = max(8, self.width() - 126)
        box_y = 12
        origin = QPoint(box_x + 52, box_y + 48)
        painter.setPen(QPen(QColor("#52636b"), 1))
        painter.setBrush(QColor(16, 21, 26, 220))
        painter.drawRoundedRect(box_x, box_y, 116, 82, 6, 6)
        painter.setPen(QPen(QColor("#cfd8dc"), 1))
        painter.setFont(QFont("Sans", 8, QFont.Bold))
        painter.drawText(box_x + 8, box_y + 14, "NED 固定方向")

        def draw_compass_axis(end, color, label):
            painter.setPen(QPen(color, 2))
            painter.drawLine(origin, end)
            painter.setBrush(color)
            painter.drawEllipse(end, 2, 2)
            painter.setPen(QPen(color, 1))
            painter.drawText(end + QPoint(4, 4), label)

        draw_compass_axis(origin + QPoint(0, -25), QColor("#ef5350"), "N")
        draw_compass_axis(origin + QPoint(28, 0), QColor("#66bb6a"), "E")
        draw_compass_axis(origin + QPoint(0, 25), QColor("#42a5f5"), "D")

    def paintEvent(self, event):
        del event
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.fillRect(self.rect(), QColor("#10151a"))

        center = self._center()
        scale = self._scene_scale()

        if self.show_grid:
            # A compact ground plane in the NED X/Y plane.
            grid_radius = max(2.0, 4.0 / max(self.zoom, 0.2))
            step = max(1.0, round(grid_radius / 4.0))
            grid_radius = step * 4.0
            grid_color = QColor("#26343b")
            for i in range(-4, 5):
                offset = i * step
                self._draw_line_3d(
                    painter, (-grid_radius, offset, 0.0),
                    (grid_radius, offset, 0.0), center, scale, grid_color)
                self._draw_line_3d(
                    painter, (offset, -grid_radius, 0.0),
                    (offset, grid_radius, 0.0), center, scale, grid_color)

        # World axes are translated to the current vehicle position only for
        # readability; translation does not change their NED orientation.
        axis_len = max(1.5, 2.5 / max(self.zoom, 0.35))
        world_origin = self.pose[:3] if self.pose else (0.0, 0.0, 0.0)
        world_axes = (
            ("+N", (world_origin[0] + axis_len, world_origin[1], world_origin[2]),
             QColor("#ef5350")),
            ("+E", (world_origin[0], world_origin[1] + axis_len, world_origin[2]),
             QColor("#66bb6a")),
            ("+D", (world_origin[0], world_origin[1], world_origin[2] + axis_len),
             QColor("#42a5f5")),
        )
        for label, endpoint, color in world_axes:
            self._draw_axis_3d(painter, world_origin, endpoint, center, scale,
                               color, label, Qt.SolidLine, 2)

        if self.points:
            painter.setPen(QPen(QColor("#00e5ff"), 2))
            projected = [self._project(point, center, scale) for point in self.points]
            for first, second in zip(projected, projected[1:]):
                painter.drawLine(first, second)
            painter.setPen(QPen(QColor("#607d8b"), 1))
            painter.setBrush(QColor("#00e5ff"))
            painter.drawEllipse(projected[0], 3, 3)

        if self.pose:
            x, y, z, roll, pitch, yaw = self.pose
            body_len = max(0.45, 0.8 / max(self.zoom, 0.35))
            # R = Rz(yaw) * Ry(pitch) * Rx(roll), matching the firmware convention.
            cr, sr = math.cos(roll), math.sin(roll)
            cp, sp = math.cos(pitch), math.sin(pitch)
            cy, sy = math.cos(yaw), math.sin(yaw)
            rotation = (
                (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
                (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
                (-sp, cp * sr, cp * cr),
            )
            origin = (x, y, z)

            # 简化 AUV 模型：尖首 + 四边形尾部，随当前位姿旋转。
            model_width = body_len * 0.42
            model_height = body_len * 0.24
            model_body = [
                (body_len, 0.0, 0.0),
                (-body_len * 0.78, -model_width, -model_height),
                (-body_len * 0.78, model_width, -model_height),
                (-body_len * 0.78, model_width, model_height),
                (-body_len * 0.78, -model_width, model_height),
            ]

            def to_world(point):
                return tuple(
                    origin[row] + sum(rotation[row][col] * point[col]
                                      for col in range(3))
                    for row in range(3)
                )

            model_world = [to_world(point) for point in model_body]
            model_faces = ((0, 1, 2), (0, 2, 3), (0, 3, 4), (0, 4, 1),
                           (1, 4, 3, 2))
            painter.setPen(QPen(QColor("#ffd740"), 1))
            painter.setBrush(QColor(255, 215, 64, 55))
            for face in model_faces:
                polygon = QPolygon([
                    self._project(model_world[index], center, scale)
                    for index in face
                ])
                painter.drawPolygon(polygon)

            model_edges = ((0, 1), (0, 2), (0, 3), (0, 4),
                           (1, 2), (2, 3), (3, 4), (4, 1))
            painter.setPen(QPen(QColor("#ffd740"), 2))
            for start, end in model_edges:
                painter.drawLine(
                    self._project(model_world[start], center, scale),
                    self._project(model_world[end], center, scale),
                )

            body_axes = (
                ("bX", (x + rotation[0][0] * body_len,
                         y + rotation[1][0] * body_len,
                         z + rotation[2][0] * body_len), QColor("#ff8a80")),
                ("bY", (x + rotation[0][1] * body_len,
                         y + rotation[1][1] * body_len,
                         z + rotation[2][1] * body_len), QColor("#b9f6ca")),
                ("bZ", (x + rotation[0][2] * body_len,
                         y + rotation[1][2] * body_len,
                         z + rotation[2][2] * body_len), QColor("#82b1ff")),
            )
            for label, endpoint, color in body_axes:
                self._draw_axis_3d(painter, origin, endpoint, center, scale,
                                   color, label, Qt.DashLine, 3)
            current = self._project(origin, center, scale)
            painter.setPen(QPen(QColor("#ffffff"), 2))
            painter.setBrush(QColor("#ffd740"))
            painter.drawEllipse(current, 5, 5)

        painter.setPen(QPen(QColor("#90a4ae"), 1))
        painter.setFont(QFont("Sans", 9))
        painter.drawText(12, 20, "拖动旋转  ·  滚轮缩放  ·  默认视角：N↑  E→  D↓")
        painter.setPen(QPen(QColor("#ef5350"), 2))
        painter.drawText(14, self.height() - 38, "世界 +N")
        painter.setPen(QPen(QColor("#66bb6a"), 2))
        painter.drawText(72, self.height() - 38, "世界 +E")
        painter.setPen(QPen(QColor("#42a5f5"), 2))
        painter.drawText(130, self.height() - 38, "世界 +D")
        painter.setPen(QPen(QColor("#ff8a80"), 2))
        painter.drawText(14, self.height() - 18, "机体 bX")
        painter.setPen(QPen(QColor("#b9f6ca"), 2))
        painter.drawText(82, self.height() - 18, "机体 bY")
        painter.setPen(QPen(QColor("#82b1ff"), 2))
        painter.drawText(150, self.height() - 18, "机体 bZ")
        self._draw_ned_compass(painter)
        painter.end()


class TrajectoryViewerWidget(QWidget):
    """ROS-backed trajectory page embedded in the master console."""

    pose_signal = pyqtSignal(object)
    status_signal = pyqtSignal(object)

    def __init__(self, node):
        super().__init__()
        self.node = node
        self.paused = False
        self.last_pose = None
        self._build_ui()
        self._create_subscriptions()
        self.pose_signal.connect(self._on_pose_main_thread)
        self.status_signal.connect(self._on_status_main_thread)

    def _build_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(8)

        toolbar = QHBoxLayout()
        title = QLabel("🧭 三维轨迹与刚体位姿")
        title.setStyleSheet("font-size: 16px; font-weight: bold; color: #00e5ff;")
        toolbar.addWidget(title)
        toolbar.addStretch()
        self.source_label = QLabel("数据源: /zit6/state/pos · 等待数据")
        self.source_label.setStyleSheet("color: #90a4ae; font-size: 11px;")
        toolbar.addWidget(self.source_label)
        layout.addLayout(toolbar)

        content = QHBoxLayout()
        content.setSpacing(10)
        self.canvas = Pose3DCanvas()
        content.addWidget(self.canvas, 1)

        side = QVBoxLayout()
        side.setSpacing(8)
        controls = QGroupBox("显示控制")
        controls_layout = QFormLayout(controls)
        self.follow_cb = QCheckBox("跟随刚体")
        self.follow_cb.setChecked(True)
        self.follow_cb.toggled.connect(self.canvas.set_follow)
        controls_layout.addRow(self.follow_cb)
        self.grid_cb = QCheckBox("显示网格")
        self.grid_cb.setChecked(True)
        self.grid_cb.toggled.connect(self.canvas.set_grid)
        controls_layout.addRow(self.grid_cb)
        reset_view_button = QPushButton("重置 NED 视角")
        reset_view_button.clicked.connect(self.canvas.reset_view)
        controls_layout.addRow(reset_view_button)
        self.max_points_spin = QSpinBox()
        self.max_points_spin.setRange(100, 20000)
        self.max_points_spin.setSingleStep(100)
        self.max_points_spin.setValue(3000)
        self.max_points_spin.valueChanged.connect(self.canvas.set_max_points)
        controls_layout.addRow("轨迹点数", self.max_points_spin)
        buttons = QHBoxLayout()
        clear_button = QPushButton("清空轨迹")
        clear_button.clicked.connect(self.canvas.clear)
        buttons.addWidget(clear_button)
        self.pause_button = QPushButton("暂停显示")
        self.pause_button.setCheckable(True)
        self.pause_button.toggled.connect(self._set_paused)
        buttons.addWidget(self.pause_button)
        controls_layout.addRow(buttons)
        side.addWidget(controls)

        pose_group = QGroupBox("当前刚体位姿 · 世界系 NED")
        pose_layout = QGridLayout(pose_group)
        self.pose_labels = {}
        for row, (key, label, unit) in enumerate([
            ("x", "X / 北向", "m"), ("y", "Y / 东向", "m"), ("z", "Z / 深度", "m"),
            ("roll", "Roll / 横滚", "rad"), ("pitch", "Pitch / 俯仰", "rad"),
            ("yaw", "Yaw / 航向", "rad"),
        ]):
            pose_layout.addWidget(QLabel(label), row, 0)
            value = QLabel("--")
            value.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            value.setStyleSheet("color: #ffffff; font-family: monospace; font-size: 12px;")
            self.pose_labels[key] = value
            pose_layout.addWidget(value, row, 1)
            pose_layout.addWidget(QLabel(unit), row, 2)
        self.euler_deg_label = QLabel("R/P/Y: -- / -- / -- °")
        self.euler_deg_label.setStyleSheet("color: #ffd740; font-family: monospace;")
        pose_layout.addWidget(self.euler_deg_label, 6, 0, 1, 3)
        side.addWidget(pose_group)

        state_group = QGroupBox("系统状态")
        state_layout = QFormLayout(state_group)
        self.armed_label = QLabel("未知")
        self.nav_label = QLabel("未知")
        self.control_label = QLabel("未知")
        state_layout.addRow("ARM", self.armed_label)
        state_layout.addRow("导航", self.nav_label)
        state_layout.addRow("控制层", self.control_label)
        side.addWidget(state_group)
        side.addStretch()
        content.addLayout(side, 0)
        layout.addLayout(content, 1)
        self._apply_style()

    def _apply_style(self):
        self.setStyleSheet("""
            QWidget { background-color: #121212; color: #b0bec5; }
            QGroupBox { color: #00e5ff; font-weight: bold; border: 1px solid #2d2d2d;
                        border-radius: 8px; margin-top: 10px; padding-top: 10px; }
            QGroupBox::title { subcontrol-origin: margin; left: 12px; padding: 0 4px; }
            QPushButton, QSpinBox { background-color: #1e1e1e; color: #eeeeee;
                                    border: 1px solid #444; border-radius: 4px; padding: 5px; }
            QPushButton:hover { background-color: #263238; }
            QCheckBox { spacing: 6px; }
        """)

    def _create_subscriptions(self):
        self.pos_sub = self.node.create_subscription(
            Float32MultiArray, "/zit6/state/pos", self._on_pose_thread, 10)
        self.status_sub = self.node.create_subscription(
            ZitStatus, "/zit6/state/status", self._on_status_thread, 10)

    def _on_pose_thread(self, msg):
        pose = _finite_pose(msg.data)
        if pose is not None:
            self.pose_signal.emit(pose)

    def _on_status_thread(self, msg):
        self.status_signal.emit(msg)

    def _on_pose_main_thread(self, pose):
        self.last_pose = pose
        if self.paused:
            return
        self.canvas.set_pose(pose)
        self.source_label.setText(
            f"数据源: /zit6/state/pos · {len(self.canvas.points)} 点")
        keys = ("x", "y", "z", "roll", "pitch", "yaw")
        for key, value in zip(keys, pose):
            self.pose_labels[key].setText(f"{value:+.4f}")
        degrees = [math.degrees(value) for value in pose[3:6]]
        self.euler_deg_label.setText(
            "R/P/Y: " + " / ".join(f"{value:+.2f}" for value in degrees) + " °")

    def _on_status_main_thread(self, msg):
        self.armed_label.setText("已解锁 🟢" if msg.is_armed else "已锁定 🔴")
        self.nav_label.setText("就绪 🟢" if msg.navigation_ready else "未就绪 🔴")
        levels = {0: "NONE", 1: "POSITION", 2: "VELOCITY", 3: "FORCE"}
        self.control_label.setText(levels.get(msg.control_level, str(msg.control_level)))

    def _set_paused(self, paused):
        self.paused = bool(paused)
        self.pause_button.setText("继续显示" if self.paused else "暂停显示")

    def closeEvent(self, event):
        self.node.destroy_subscription(self.pos_sub)
        self.node.destroy_subscription(self.status_sub)
        super().closeEvent(event)
