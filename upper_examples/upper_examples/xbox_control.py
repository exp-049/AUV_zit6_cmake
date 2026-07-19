#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import argparse
import threading
import time

import rclpy
from rclpy.node import Node
from zit6_interfaces.msg import ZitSetpoint

# 导入共享的心跳面板
from .heartbeat import FloatingHeartbeatPanel

os.environ["SDL_VIDEODRIVER"] = "dummy"

try:
    import pygame
except ImportError:
    pygame = None

# Qt imports
try:
    from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, 
                                 QPushButton, QLabel, QComboBox, QCheckBox, QDoubleSpinBox, QGroupBox, QGridLayout)
    from PyQt5.QtCore import Qt, QTimer
    from PyQt5.QtGui import QPainter, QColor, QPen, QBrush, QPainterPath, QFont
except ImportError:
    from PySide6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, 
                                   QPushButton, QLabel, QComboBox, QCheckBox, QDoubleSpinBox, QGroupBox, QGridLayout)
    from PySide6.QtCore import Qt, QTimer
    from PySide6.QtGui import QPainter, QColor, QPen, QBrush, QPainterPath, QFont


class GamepadVisualizer(QWidget):
    """
    自研矢量 Xbox 手柄动态可视化组件，使用 QPainter 绘制，支持实时按键与摇杆动画发光反馈。
    """
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(420, 270)
        self.axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.btn_states = {'A': False, 'B': False, 'X': False, 'Y': False, 'LB': False, 'RB': False, 'back': False, 'start': False}
        self.hat_state = (0, 0)
        self.connected = False

    def update_state(self, axes, btn_states, hat_state, connected):
        self.axes = axes
        self.btn_states = btn_states
        self.hat_state = hat_state
        self.connected = connected
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        
        # 居中比例缩放绘制 (基于 400x250 虚拟画布)
        width = self.width()
        height = self.height()
        scale = min(width / 400.0, height / 250.0)
        
        painter.translate((width - 400 * scale) / 2, (height - 250 * scale) / 2)
        painter.scale(scale, scale)
        
        # 基础色彩定义
        bg_color = QColor(25, 25, 25) if self.connected else QColor(18, 18, 18)
        border_color = QColor(0, 229, 255) if self.connected else QColor(70, 70, 70)
        
        # 1. 绘制 Xbox 经典手柄轮廓 (Symmetric Wings Path)
        body_path = QPainterPath()
        body_path.moveTo(100, 50)
        body_path.quadTo(200, 42, 300, 50)      # 上边缘圆弧
        body_path.quadTo(355, 60, 375, 120)     # 右侧外翼
        body_path.quadTo(390, 195, 340, 225)    # 右手柄底部
        body_path.quadTo(300, 240, 250, 185)    # 右手柄内侧内凹
        body_path.quadTo(200, 195, 150, 185)    # 下方凹槽
        body_path.quadTo(100, 240, 60, 225)     # 左手柄内侧内凹
        body_path.quadTo(10, 195, 25, 120)      # 左手柄底部及外翼
        body_path.quadTo(45, 60, 100, 50)       # 左侧边缘
        
        # 填充手柄背板
        painter.setPen(Qt.NoPen)
        painter.setBrush(QBrush(bg_color))
        painter.drawPath(body_path)
        
        # 绘制手柄边缘发光圈 (霓虹描边)
        pen = QPen(border_color, 2)
        painter.setPen(pen)
        painter.setBrush(Qt.NoBrush)
        painter.drawPath(body_path)
        
        # 2. 绘制顶部 LB / RB 按钮
        # LB
        lb_color = QColor(0, 229, 255, 200) if self.btn_states.get('LB') else QColor(45, 45, 45)
        painter.setPen(QPen(QColor(80, 80, 80), 1))
        painter.setBrush(QBrush(lb_color))
        painter.drawRoundedRect(65, 35, 60, 11, 3, 3)
        
        # RB
        rb_color = QColor(0, 229, 255, 200) if self.btn_states.get('RB') else QColor(45, 45, 45)
        painter.setBrush(QBrush(rb_color))
        painter.drawRoundedRect(275, 35, 60, 11, 3, 3)

        # 3. 绘制 Start / Back (菜单/视口键)
        back_color = QColor(0, 229, 255, 200) if self.btn_states.get('back') else QColor(50, 50, 50)
        painter.setBrush(QBrush(back_color))
        painter.drawEllipse(170, 95, 11, 11)
        
        start_color = QColor(0, 229, 255, 200) if self.btn_states.get('start') else QColor(50, 50, 50)
        painter.setBrush(QBrush(start_color))
        painter.drawEllipse(219, 95, 11, 11)
        
        # 4. 绘制左摇杆 (LS)
        ls_cx, ls_cy = 120, 100
        # 摇杆凹槽边界
        painter.setPen(QPen(QColor(60, 60, 60), 1.5))
        painter.setBrush(QBrush(QColor(20, 20, 20)))
        painter.drawEllipse(ls_cx - 24, ls_cy - 24, 48, 48)
        # 导向刻度十字
        painter.setPen(QPen(QColor(50, 50, 50, 80), 1))
        painter.drawLine(ls_cx - 24, ls_cy, ls_cx + 24, ls_cy)
        painter.drawLine(ls_cx, ls_cy - 24, ls_cx, ls_cy + 24)
        
        # 摇杆模型动画回显使用默认物理位置 (Surge=轴1Y, Sway=轴0X)
        # 这里为了可视化，采用常规方向，不随反转配置而颠倒可视化的方向，以便忠实地回显手柄物理位置
        ls_x = -self.axes[1] if self.connected else 0.0
        ls_y = self.axes[0] if self.connected else 0.0
        
        # 摇杆帽
        cap_x = ls_cx + int(ls_x * 16)
        cap_y = ls_cy + int(ls_y * 16)
        painter.setPen(QPen(border_color, 1.5))
        painter.setBrush(QBrush(QColor(40, 40, 40)))
        painter.drawEllipse(cap_x - 14, cap_y - 14, 28, 28)
        
        # 中心指示点
        painter.setBrush(QBrush(QColor(0, 229, 255) if self.connected else QColor(90, 90, 90)))
        painter.drawEllipse(cap_x - 3, cap_y - 3, 6, 6)
        
        # 5. 绘制右摇杆 (RS)
        rs_cx, rs_cy = 230, 150
        painter.setPen(QPen(QColor(60, 60, 60), 1.5))
        painter.setBrush(QBrush(QColor(20, 20, 20)))
        painter.drawEllipse(rs_cx - 24, rs_cy - 24, 48, 48)
        painter.setPen(QPen(QColor(50, 50, 50, 80), 1))
        painter.drawLine(rs_cx - 24, rs_cy, rs_cx + 24, rs_cy)
        painter.drawLine(rs_cx, rs_cy - 24, rs_cx, rs_cy + 24)
        
        # 右摇杆默认物理位置 (Yaw=轴5X, Heave=轴2Y)
        rs_x = self.axes[5] if self.connected else 0.0
        rs_y = self.axes[2] if self.connected else 0.0
        
        cap_rx = rs_cx + int(rs_x * 16)
        cap_ry = rs_cy + int(rs_y * 16)
        painter.setPen(QPen(border_color, 1.5))
        painter.setBrush(QBrush(QColor(40, 40, 40)))
        painter.drawEllipse(cap_rx - 14, cap_ry - 14, 28, 28)
        painter.setBrush(QBrush(QColor(0, 229, 255) if self.connected else QColor(90, 90, 90)))
        painter.drawEllipse(cap_rx - 3, cap_ry - 3, 6, 6)
        
        # 6. 绘制 D-Pad (十字方向键)
        dpad_cx, dpad_cy = 170, 150
        d_size = 13
        
        # 绘制背景
        painter.setPen(QPen(QColor(65, 65, 65), 1.5))
        painter.setBrush(QBrush(QColor(24, 24, 24)))
        
        dpad_path = QPainterPath()
        dpad_path.moveTo(dpad_cx - d_size/2, dpad_cy - d_size*1.5)
        dpad_path.lineTo(dpad_cx + d_size/2, dpad_cy - d_size*1.5)
        dpad_path.lineTo(dpad_cx + d_size/2, dpad_cy - d_size/2)
        dpad_path.lineTo(dpad_cx + d_size*1.5, dpad_cy - d_size/2)
        dpad_path.lineTo(dpad_cx + d_size*1.5, dpad_cy + d_size/2)
        dpad_path.lineTo(dpad_cx + d_size/2, dpad_cy + d_size/2)
        dpad_path.lineTo(dpad_cx + d_size/2, dpad_cy + d_size*1.5)
        dpad_path.lineTo(dpad_cx - d_size/2, dpad_cy + d_size*1.5)
        dpad_path.lineTo(dpad_cx - d_size/2, dpad_cy + d_size/2)
        dpad_path.lineTo(dpad_cx - d_size*1.5, dpad_cy + d_size/2)
        dpad_path.lineTo(dpad_cx - d_size*1.5, dpad_cy - d_size/2)
        dpad_path.lineTo(dpad_cx - d_size/2, dpad_cy - d_size/2)
        dpad_path.closeSubpath()
        painter.drawPath(dpad_path)
        
        # 高亮方向状态
        hx, hy = self.hat_state
        painter.setPen(Qt.NoPen)
        painter.setBrush(QBrush(QColor(0, 229, 255)))
        if hy > 0:   # Up
            painter.drawRect(dpad_cx - d_size/2 + 1, dpad_cy - d_size*1.5 + 1, d_size - 1, d_size - 1)
        if hy < 0:   # Down
            painter.drawRect(dpad_cx - d_size/2 + 1, dpad_cy + d_size/2, d_size - 1, d_size - 1)
        if hx < 0:   # Left
            painter.drawRect(dpad_cx - d_size*1.5 + 1, dpad_cy - d_size/2 + 1, d_size - 1, d_size - 1)
        if hx > 0:   # Right
            painter.drawRect(dpad_cx + d_size/2, dpad_cy - d_size/2 + 1, d_size - 1, d_size - 1)
            
        # 7. 绘制 ABXY 动作按键组
        btn_cx, btn_cy = 280, 100
        btn_dist = 22
        
        buttons = [
            ('Y', QColor(255, 235, 59), 0, -btn_dist),   # 黄
            ('A', QColor(76, 175, 80), 0, btn_dist),    # 绿
            ('X', QColor(33, 150, 243), -btn_dist, 0),   # 蓝
            ('B', QColor(244, 67, 54), btn_dist, 0)     # 红
        ]
        
        for name, col, ox, oy in buttons:
            bx, by = btn_cx + ox, btn_cy + oy
            is_pressed = self.btn_states.get(name, False)
            
            painter.setPen(QPen(QColor(55, 55, 55), 1))
            if is_pressed:
                painter.setBrush(QBrush(col))
                text_color = Qt.black
            else:
                dim_col = QColor(col.red(), col.green(), col.blue(), 55)
                painter.setBrush(QBrush(dim_col))
                text_color = QColor(190, 190, 190)
                
            painter.drawEllipse(bx - 9, by - 9, 18, 18)
            
            # 绘制字母
            painter.setPen(text_color)
            font = QFont("Arial", 8, QFont.Bold)
            painter.setFont(font)
            painter.drawText(bx - 9, by - 9, 18, 18, Qt.AlignCenter, name)


class XboxControlWidget(QWidget):
    """
    手柄遥控可视化及指令控制发布组件 (支持配置通道映射与死区)
    """
    def __init__(self, node):
        super().__init__()
        self.node = node
        
        self.joystick = None
        self.joystick_connected = False
        self.control_active = False
        
        # 内部底层映射的摇杆原始/目标物理轴值 [Surge,Sway,Heave,Roll,Pitch,Yaw]
        self.axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.btn_states = {'A': False, 'B': False, 'X': False, 'Y': False, 'LB': False, 'RB': False, 'back': False, 'start': False}
        self.hat_state = (0, 0)
        self.last_a_state = False
        self.input_thread = None
        self.num_axes = 0
        self.num_buttons = 0
        
        self.setpoint_pub = self.node.create_publisher(ZitSetpoint, '/zit6/cmd/setpoint', 10)
        
        # 负责手柄指令的 10Hz 定时下发
        self.control_timer = self.node.create_timer(0.1, self.ros_control_timer_callback)
        
        if pygame:
            try:
                pygame.init()
                pygame.joystick.init()
            except Exception as e:
                print(f"初始化 Pygame 手柄库失败: {e}")
                
        self.init_style()
        self.init_ui()
        self.check_joystick_connection()
        if pygame and self.joystick_connected:
            self.input_thread = threading.Thread(target=self.read_joystick, daemon=True)
            self.input_thread.start()
            
        self.joy_timer = QTimer()
        self.joy_timer.timeout.connect(self.update_gui_joystick)
        self.joy_timer.start(50)

    def init_style(self):
        self.setStyleSheet("""
            QWidget {
                background-color: #121212;
            }
            QLabel {
                color: #b0bec5;
                font-size: 12px;
            }
            QGroupBox {
                color: #00e5ff;
                font-weight: bold;
                font-size: 12px;
                border: 1px solid #2d2d2d;
                border-radius: 8px;
                margin-top: 10px;
                padding-top: 10px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                subcontrol-position: top left;
                left: 12px;
                padding: 0 4px;
            }
            QComboBox {
                background-color: #1e1e1e;
                color: #ffffff;
                border: 1px solid #444444;
                border-radius: 4px;
                padding: 3px 6px;
                min-height: 24px;
                font-size: 11px;
            }
            QComboBox QAbstractItemView {
                background-color: #1e1e1e;
                color: #ffffff;
                selection-background-color: #006064;
                selection-color: #00e5ff;
                border: 1px solid #444444;
            }
            QDoubleSpinBox {
                background-color: #1e1e1e;
                color: #ffffff;
                border: 1px solid #444444;
                border-radius: 4px;
                padding: 3px 6px;
                min-height: 24px;
                font-size: 11px;
            }
            QDoubleSpinBox::up-button, QDoubleSpinBox::down-button {
                width: 14px;
                background-color: #2b2b2b;
                border: none;
            }
            QCheckBox {
                color: #ffffff;
                font-size: 11px;
                spacing: 5px;
            }
            QCheckBox::indicator {
                width: 16px;
                height: 16px;
                background-color: #1e1e1e;
                border: 1px solid #444444;
                border-radius: 3px;
            }
            QCheckBox::indicator:checked {
                background-color: #00e5ff;
                border: 1px solid #00e5ff;
            }
        """)

    def init_ui(self):
        # 整体布局：左右分栏
        main_layout = QHBoxLayout(self)
        main_layout.setContentsMargins(15, 15, 15, 15)
        main_layout.setSpacing(15)
        
        # =========================================================================
        # 左侧控制与通道配置面板
        # =========================================================================
        left_widget = QWidget()
        left_layout = QVBoxLayout(left_widget)
        left_layout.setContentsMargins(0, 0, 0, 0)
        left_layout.setSpacing(10)
        
        title = QLabel("🎮 Xbox 手柄控制台")
        title.setStyleSheet("font-size: 16px; font-weight: bold; color: #00e5ff;")
        left_layout.addWidget(title)
        
        self.joy_status_label = QLabel("正在检测手柄连接...")
        self.joy_status_label.setStyleSheet("font-size: 13px; color: #888888;")
        left_layout.addWidget(self.joy_status_label)
        
        self.joy_active_label = QLabel("控制状态: 未启用")
        self.joy_active_label.setStyleSheet("font-size: 13px; font-weight: bold; color: #ff9800;")
        left_layout.addWidget(self.joy_active_label)
        
        # 1. 通道映射与反转 GroupBox (替代原本生硬的进度条)
        cfg_group = QGroupBox("通道映射与极性配置")
        cfg_grid = QGridLayout(cfg_group)
        cfg_grid.setContentsMargins(12, 12, 12, 12)
        cfg_grid.setSpacing(8)
        
        cfg_grid.addWidget(QLabel("控制通道"), 0, 0)
        cfg_grid.addWidget(QLabel("物理手柄轴"), 0, 1)
        cfg_grid.addWidget(QLabel("反转极性"), 0, 2)
        
        # 前进 Surge
        cfg_grid.addWidget(QLabel("前进 (Surge):"), 1, 0)
        self.combo_surge = QComboBox()
        self.combo_surge.addItems([f"轴 {i}" for i in range(6)])
        self.combo_surge.setCurrentIndex(1) # 默认 轴 1
        self.chk_surge_inv = QCheckBox()
        self.chk_surge_inv.setChecked(True)  # 默认反转，因为 pygame 轴1 向上为负
        cfg_grid.addWidget(self.combo_surge, 1, 1)
        cfg_grid.addWidget(self.chk_surge_inv, 1, 2)
        
        # 左右 Sway
        cfg_grid.addWidget(QLabel("左右 (Sway):"), 2, 0)
        self.combo_sway = QComboBox()
        self.combo_sway.addItems([f"轴 {i}" for i in range(6)])
        self.combo_sway.setCurrentIndex(0) # 默认 轴 0
        self.chk_sway_inv = QCheckBox()
        self.chk_sway_inv.setChecked(True)
        cfg_grid.addWidget(self.combo_sway, 2, 1)
        cfg_grid.addWidget(self.chk_sway_inv, 2, 2)
        
        # 升降 Heave
        cfg_grid.addWidget(QLabel("升降 (Heave):"), 3, 0)
        self.combo_heave = QComboBox()
        self.combo_heave.addItems([f"轴 {i}" for i in range(6)])
        self.combo_heave.setCurrentIndex(4) # 默认 轴 4
        self.chk_heave_inv = QCheckBox()
        self.chk_heave_inv.setChecked(False)
        cfg_grid.addWidget(self.combo_heave, 3, 1)
        cfg_grid.addWidget(self.chk_heave_inv, 3, 2)

        # 横滚 Roll
        cfg_grid.addWidget(QLabel("横滚 (Roll，旁路):"), 4, 0)
        self.combo_roll = QComboBox()
        self.combo_roll.addItems(["未使用"] + [f"轴 {i}" for i in range(6)])
        self.combo_roll.setCurrentIndex(0)
        self.chk_roll_inv = QCheckBox()
        self.combo_roll.setEnabled(False)
        self.chk_roll_inv.setEnabled(False)
        cfg_grid.addWidget(self.combo_roll, 4, 1)
        cfg_grid.addWidget(self.chk_roll_inv, 4, 2)

        # 俯仰 Pitch
        cfg_grid.addWidget(QLabel("俯仰 (Pitch，旁路):"), 5, 0)
        self.combo_pitch = QComboBox()
        self.combo_pitch.addItems(["未使用"] + [f"轴 {i}" for i in range(6)])
        self.combo_pitch.setCurrentIndex(0)
        self.chk_pitch_inv = QCheckBox()
        self.combo_pitch.setEnabled(False)
        self.chk_pitch_inv.setEnabled(False)
        cfg_grid.addWidget(self.combo_pitch, 5, 1)
        cfg_grid.addWidget(self.chk_pitch_inv, 5, 2)
        
        # 转向 Yaw
        cfg_grid.addWidget(QLabel("转向 (Yaw):"), 6, 0)
        self.combo_yaw = QComboBox()
        self.combo_yaw.addItems([f"轴 {i}" for i in range(6)])
        self.combo_yaw.setCurrentIndex(3) # 默认 轴 3
        self.chk_yaw_inv = QCheckBox()
        self.chk_yaw_inv.setChecked(False)
        cfg_grid.addWidget(self.combo_yaw, 6, 1)
        cfg_grid.addWidget(self.chk_yaw_inv, 6, 2)
        
        left_layout.addWidget(cfg_group)
        
        # 2. 死区设置 GroupBox
        dz_group = QGroupBox("死区配置参数 (Deadzone)")
        dz_grid = QGridLayout(dz_group)
        dz_grid.setContentsMargins(12, 12, 12, 12)
        dz_grid.setSpacing(8)
        
        dz_grid.addWidget(QLabel("前进死区 (Surge DZ):"), 0, 0)
        self.spin_surge_dz = QDoubleSpinBox()
        self.spin_surge_dz.setRange(0.0, 0.5)
        self.spin_surge_dz.setValue(0.15) # 前进高滤噪死区
        self.spin_surge_dz.setSingleStep(0.01)
        dz_grid.addWidget(self.spin_surge_dz, 0, 1)
        
        dz_grid.addWidget(QLabel("常规轴死区 (Other DZ):"), 1, 0)
        self.spin_general_dz = QDoubleSpinBox()
        self.spin_general_dz.setRange(0.0, 0.5)
        self.spin_general_dz.setValue(0.08)
        self.spin_general_dz.setSingleStep(0.01)
        dz_grid.addWidget(self.spin_general_dz, 1, 1)
        
        left_layout.addWidget(dz_group)
        
        # 启用开关
        self.btn_joy_toggle = QPushButton("启用手柄控制")
        self.btn_joy_toggle.setCheckable(True)
        self.btn_joy_toggle.clicked.connect(self.on_joy_toggle_clicked)
        self.btn_joy_toggle.setStyleSheet("""
            QPushButton {
                background-color: #37474f; 
                color: white;
                font-weight: bold;
                padding: 12px;
                border: none;
                border-radius: 6px;
                margin-top: 5px;
            }
            QPushButton:hover {
                background-color: #455a64;
            }
        """)
        left_layout.addWidget(self.btn_joy_toggle)
        left_layout.addStretch()
        
        main_layout.addWidget(left_widget, stretch=4)
        
        # =========================================================================
        # 右侧手柄矢量动态模型
        # =========================================================================
        right_widget = QWidget()
        right_layout = QVBoxLayout(right_widget)
        right_layout.setContentsMargins(0, 0, 0, 0)
        
        model_lbl = QLabel("手柄实时动态回显模型 (Gamepad Model)")
        model_lbl.setStyleSheet("font-size: 11px; color: #555555; font-weight: bold; margin-bottom: 5px;")
        right_layout.addWidget(model_lbl)
        
        self.visualizer = GamepadVisualizer(self)
        right_layout.addWidget(self.visualizer)
        
        main_layout.addWidget(right_widget, stretch=6)

    def check_joystick_connection(self):
        if not pygame:
            self.joystick_connected = False
            return
        try:
            pygame.joystick.init()
            count = pygame.joystick.get_count()
            if count > 0:
                if not self.joystick_connected or not self.joystick:
                    self.joystick = pygame.joystick.Joystick(0)
                    self.joystick.init()
                    self.num_axes = self.joystick.get_numaxes()
                    self.num_buttons = self.joystick.get_numbuttons()
                    self.joystick_connected = True
            else:
                self.joystick_connected = False
                self.joystick = None
        except Exception:
            self.joystick_connected = False
            self.joystick = None

    def safe_get_axis(self, axis_index):
        if self.joystick and 0 <= axis_index < self.num_axes:
            try:
                return self.joystick.get_axis(axis_index)
            except Exception:
                pass
        return 0.0

    def safe_get_button(self, btn_index):
        if self.joystick and btn_index < self.num_buttons:
            try:
                return self.joystick.get_button(btn_index)
            except Exception:
                pass
        return False

    def apply_deadzone(self, val, deadzone):
        if abs(val) < deadzone:
            return 0.0
        return val

    def read_joystick(self):
        while rclpy.ok():
            if not self.joystick_connected or not self.joystick:
                time.sleep(0.5)
                continue
            try:
                pygame.event.pump()
                
                # 动态获取配置的通道索引和方向极性并计算控制值
                surge_axis = self.combo_surge.currentIndex()
                sway_axis = self.combo_sway.currentIndex()
                heave_axis = self.combo_heave.currentIndex()
                roll_axis = self.combo_roll.currentIndex() - 1
                pitch_axis = self.combo_pitch.currentIndex() - 1
                yaw_axis = self.combo_yaw.currentIndex()
                
                s_inv = -1.0 if self.chk_surge_inv.isChecked() else 1.0
                sw_inv = -1.0 if self.chk_sway_inv.isChecked() else 1.0
                h_inv = -1.0 if self.chk_heave_inv.isChecked() else 1.0
                r_inv = -1.0 if self.chk_roll_inv.isChecked() else 1.0
                p_inv = -1.0 if self.chk_pitch_inv.isChecked() else 1.0
                y_inv = -1.0 if self.chk_yaw_inv.isChecked() else 1.0
                
                self.axes[0] = self.safe_get_axis(surge_axis) * s_inv
                self.axes[1] = self.safe_get_axis(sway_axis) * sw_inv
                self.axes[2] = self.safe_get_axis(heave_axis) * h_inv
                self.axes[3] = 0.0  # Roll 兼容字段，固件控制路径旁路
                self.axes[4] = 0.0  # Pitch 兼容字段，固件控制路径旁路
                self.axes[5] = self.safe_get_axis(yaw_axis) * y_inv
                
                # 获取各个按键状态（经典 Xbox 布局索引）
                self.btn_states['A'] = self.safe_get_button(0)
                self.btn_states['B'] = self.safe_get_button(1)
                self.btn_states['X'] = self.safe_get_button(2)
                self.btn_states['Y'] = self.safe_get_button(3)
                self.btn_states['LB'] = self.safe_get_button(4)
                self.btn_states['RB'] = self.safe_get_button(5)
                self.btn_states['back'] = self.safe_get_button(6)
                self.btn_states['start'] = self.safe_get_button(7)
                
                if self.joystick.get_numhats() > 0:
                    self.hat_state = self.joystick.get_hat(0)
                else:
                    self.hat_state = (0, 0)
                
                a_state = self.btn_states['A']
                if a_state and not self.last_a_state:
                    self.control_active = not self.control_active
                    if not self.control_active:
                        self.send_stop_command()
                self.last_a_state = a_state
            except Exception:
                pass
            time.sleep(0.02)

    def on_joy_toggle_clicked(self, checked):
        self.control_active = checked
        if not self.control_active:
            self.send_stop_command()

    def update_gui_joystick(self):
        self.check_joystick_connection()
        
        if not self.joystick_connected:
            self.joy_status_label.setText("未连接手柄，正在扫描...")
            self.joy_status_label.setStyleSheet("color: #ff5722;")
            self.joy_active_label.setText("控制状态: 未就绪")
            self.joy_active_label.setStyleSheet("color: #888888;")
            self.btn_joy_toggle.setChecked(False)
            self.btn_joy_toggle.setEnabled(False)
            self.btn_joy_toggle.setText("启用手柄控制")
            self.btn_joy_toggle.setStyleSheet("background-color: #37474f; color: white;")
            
            # 手柄模型更新为离线状态
            self.visualizer.update_state([0.0]*6, {'A': False, 'B': False, 'X': False, 'Y': False, 'LB': False, 'RB': False, 'back': False, 'start': False}, (0,0), False)
            return
            
        self.btn_joy_toggle.setEnabled(True)
        self.joy_status_label.setText(f"已连接: {self.joystick.get_name()}")
        self.joy_status_label.setStyleSheet("color: #00e5ff; font-weight: bold;")
        
        if self.control_active:
            self.joy_active_label.setText("控制状态: 已启用 (ACTIVE) 🟢")
            self.joy_active_label.setStyleSheet("color: #4caf50; font-weight: bold;")
            self.btn_joy_toggle.setChecked(True)
            self.btn_joy_toggle.setText("禁用手柄控制 (A键)")
            self.btn_joy_toggle.setStyleSheet("background-color: #d84315; color: white;")
        else:
            self.joy_active_label.setText("控制状态: 未启用 (STANDBY) 🟡")
            self.joy_active_label.setStyleSheet("color: #ff9800; font-weight: bold;")
            self.btn_joy_toggle.setChecked(False)
            self.btn_joy_toggle.setText("启用手柄控制 (A键)")
            self.btn_joy_toggle.setStyleSheet("background-color: #00838f; color: white;")
            
        # 实时同步数据到右侧手柄绘图模型 (模型本身基于手柄的实际物理状态回显，
        # 我们用 combo box 选择的轴对应数值传给 visualizer 渲染以反映实际映射到的物理操作)
        # 即：左摇杆表示 Surge/Sway；右摇杆回显 Heave/Yaw
        # 这里直接传入 axes 供回显，这会让摇杆模型在视觉上准确反应输出强度。
        self.visualizer.update_state(self.axes, self.btn_states, self.hat_state, True)

    def send_stop_command(self):
        try:
            msg = ZitSetpoint()
            msg.control_key = 16
            msg.x = 0.0
            msg.y = 0.0
            msg.z = 0.0
            msg.roll = 0.0
            msg.pitch = 0.0
            msg.yaw = 0.0
            self.setpoint_pub.publish(msg)
        except Exception:
            pass

    def ros_control_timer_callback(self):
        if self.control_active:
            msg = ZitSetpoint()
            msg.control_key = 50
            
            # 从界面获取死区配置
            s_dz = self.spin_surge_dz.value()
            g_dz = self.spin_general_dz.value()
            
            msg.x = self.apply_deadzone(self.axes[0], s_dz)
            msg.y = self.apply_deadzone(self.axes[1], g_dz)
            msg.z = self.apply_deadzone(self.axes[2], g_dz)
            msg.roll = 0.0  # 兼容字段；固件控制路径旁路
            msg.pitch = 0.0  # 兼容字段；固件控制路径旁路
            msg.yaw = self.apply_deadzone(self.axes[5], g_dz)
            self.setpoint_pub.publish(msg)

    def close(self):
        self.control_active = False
        self.send_stop_command()
        if pygame:
            try:
                pygame.quit()
            except Exception:
                pass


class XboxControlApp(QMainWindow):
    """
    手柄控制器独立 GUI 窗口
    """
    def __init__(self, node, spinner):
        super().__init__()
        self.node = node
        self.spinner = spinner
        self.setWindowTitle("Xbox 手柄遥控器")
        self.resize(800, 420)
        self.setStyleSheet("QMainWindow { background-color: #121212; }")
        
        self.widget = XboxControlWidget(self.node)
        self.setCentralWidget(self.widget)
        
        # 统一的浮动心跳下发面板
        self.floating_hbt = FloatingHeartbeatPanel(self.node, self)
        self.floating_hbt.show()

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self.floating_hbt.setGeometry(self.width() - 280, 15, 260, 42)
        self.floating_hbt.raise_()

    def closeEvent(self, event):
        self.widget.close()
        self.floating_hbt.close()
        super().closeEvent(event)


def main(args=None):
    parser = argparse.ArgumentParser(description="Xbox 手柄控制指令发布程序")
    parser.add_argument("--cli", action="store_true", help="是否在命令行纯控制模式下运行")
    parser.add_argument("--deadzone", type=float, default=0.1, help="手柄摇杆死区")
    
    parsed_args, unknown = parser.parse_known_args(args=args)

    if parsed_args.cli:
        rclpy.init(args=args)
        node = Node('xbox_control_cli')
        
        if not pygame:
            print("Error: pygame is required.")
            sys.exit(1)
            
        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() == 0:
            print("Error: No joystick connected.")
            sys.exit(1)
            
        joystick = pygame.joystick.Joystick(0)
        joystick.init()
        print("=" * 60)
        print(f" [Xbox Control CLI] 手柄控制器已启动！")
        print("=" * 60)
        
        setpoint_pub = node.create_publisher(ZitSetpoint, '/zit6/cmd/setpoint', 10)
        
        active = False
        last_a_state = False
        num_axes = joystick.get_numaxes()
        num_buttons = joystick.get_numbuttons()
        
        def apply_deadzone(val, deadzone):
            if abs(val) < deadzone:
                return 0.0
            return val
            
        try:
            while rclpy.ok():
                pygame.event.pump()
                
                surge = joystick.get_axis(1) if 1 < num_axes else 0.0
                sway = -joystick.get_axis(0) if 0 < num_axes else 0.0
                heave = joystick.get_axis(4) if 4 < num_axes else 0.0
                yaw = joystick.get_axis(3) if 3 < num_axes else 0.0
                
                a_state = joystick.get_button(0) if 0 < num_buttons else False
                if a_state and not last_a_state:
                    active = not active
                    if not active:
                        msg = ZitSetpoint()
                        msg.control_key = 16
                        msg.x = 0.0
                        msg.y = 0.0
                        msg.z = 0.0
                        msg.roll = 0.0
                        msg.pitch = 0.0
                        msg.yaw = 0.0
                        setpoint_pub.publish(msg)
                        
                last_a_state = a_state
                print(f"Surge: {surge:+.2f} | Sway: {sway:+.2f} | Heave: {heave:+.2f} | Yaw: {yaw:+.2f} | Active: {active}", end="\r")
                
                s_msg = ZitSetpoint()
                s_msg.control_key = 50
                s_msg.x = -apply_deadzone(surge, parsed_args.deadzone)
                s_msg.y = apply_deadzone(sway, parsed_args.deadzone)
                s_msg.z = apply_deadzone(heave, parsed_args.deadzone)
                s_msg.roll = 0.0
                s_msg.pitch = 0.0
                s_msg.yaw = apply_deadzone(yaw, parsed_args.deadzone)
                setpoint_pub.publish(s_msg)
                
                time.sleep(0.1)
        except KeyboardInterrupt:
            msg = ZitSetpoint()
            msg.control_key = 16
            msg.x = 0.0
            msg.y = 0.0
            msg.z = 0.0
            msg.roll = 0.0
            msg.pitch = 0.0
            msg.yaw = 0.0
            setpoint_pub.publish(msg)
        finally:
            pygame.quit()
            node.destroy_node()
            rclpy.shutdown()
    else:
        rclpy.init(args=args)
        node = Node('xbox_control_gui_node')
        
        spinner = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
        spinner.start()
        
        app = QApplication(sys.argv)
        app.setStyle("Fusion")
        
        window = XboxControlApp(node, spinner)
        window.show()
        
        exit_code = app.exec_()
        
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(exit_code)

if __name__ == '__main__':
    main()
