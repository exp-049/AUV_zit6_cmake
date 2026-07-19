#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import argparse
import threading
import time

import rclpy
from rclpy.node import Node
from zit6_interfaces.msg import ZitSetpoint, ZitStatus

# Qt imports
try:
    from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, 
                                 QPushButton, QLabel, QDoubleSpinBox, QComboBox, QFrame, QGroupBox, QGridLayout, QSplitter)
    from PyQt5.QtCore import Qt, pyqtSignal, QTimer
except ImportError:
    from PySide6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, 
                                   QPushButton, QLabel, QDoubleSpinBox, QComboBox, QFrame, QGroupBox, QGridLayout, QSplitter)
    from PySide6.QtCore import Qt, Signal as pyqtSignal, QTimer

class MotionControlWidget(QWidget):
    """
    运动控制台组件，整合了指令单次下发与实时状态监控
    """
    status_signal = pyqtSignal(object)

    def __init__(self, node):
        super().__init__()
        self.node = node
        self.seq = 0
        
        # 1. 发布者
        self.pub = self.node.create_publisher(ZitSetpoint, '/zit6/cmd/setpoint', 10)
        
        # 2. 订阅者
        self.status_sub = self.node.create_subscription(
            ZitStatus,
            '/zit6/state/status',
            self.status_callback,
            10
        )
        
        self.init_style()
        self.init_ui()
        self.update_mask_styles()
        
        self.status_signal.connect(self.update_status_ui)

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
                padding: 4px 8px;
                min-height: 28px;
                font-size: 12px;
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
                padding: 4px 8px;
                min-height: 28px;
                font-size: 12px;
            }
            QDoubleSpinBox::up-button, QDoubleSpinBox::down-button {
                width: 16px;
                background-color: #2b2b2b;
                border: none;
            }
        """)

    def init_ui(self):
        # 整体采用左右分栏结构
        main_layout = QHBoxLayout(self)
        main_layout.setContentsMargins(10, 10, 10, 10)
        main_layout.setSpacing(10)
        
        splitter = QSplitter(Qt.Horizontal)
        main_layout.addWidget(splitter)
        
        # =========================================================================
        # 左分栏：指令下发 (Command Dispatch)
        # =========================================================================
        left_widget = QWidget()
        left_layout = QVBoxLayout(left_widget)
        left_layout.setContentsMargins(5, 5, 5, 5)
        left_layout.setSpacing(10)
        
        title_left = QLabel("⚓ 指令下发控制")
        title_left.setStyleSheet("font-size: 15px; font-weight: bold; color: #00e5ff;")
        left_layout.addWidget(title_left)
        
        # 标志组 (control_key)
        ctrl_group = QGroupBox("控制标志设置 (Control Key)")
        ctrl_layout = QGridLayout(ctrl_group)
        
        ctrl_layout.addWidget(QLabel("控制模式 (Mode):"), 0, 0)
        self.mode_combo = QComboBox()
        self.mode_combo.addItems(["位置模式 (POS, 0)", "速度模式 (VEL, 1)", "推力模式 (FORCE, 2)"])
        self.mode_combo.currentIndexChanged.connect(self.update_hex_display)
        ctrl_layout.addWidget(self.mode_combo, 0, 1)
        
        ctrl_layout.addWidget(QLabel("坐标系 (Frame):"), 1, 0)
        self.frame_combo = QComboBox()
        self.frame_combo.addItems(["世界坐标系 (World, 0x00)", "机体坐标系 (Body, 0x10)"])
        self.frame_combo.currentIndexChanged.connect(self.update_hex_display)
        ctrl_layout.addWidget(self.frame_combo, 1, 1)
        
        ctrl_layout.addWidget(QLabel("定位模式 (Increment):"), 2, 0)
        self.inc_combo = QComboBox()
        self.inc_combo.addItems(["绝对目标 (Absolute, 0x00)", "相对增量 (Incremental, 0x20)"])
        self.inc_combo.currentIndexChanged.connect(self.update_hex_display)
        ctrl_layout.addWidget(self.inc_combo, 2, 1)
        
        left_layout.addWidget(ctrl_group)
        
        # 掩码组 (type_mask)
        mask_group = QGroupBox("控制轴掩码状态 (Type Mask)")
        mask_layout = QHBoxLayout(mask_group)
        mask_layout.setContentsMargins(10, 12, 10, 12)
        
        self.btn_mask_x = QPushButton()
        self.btn_mask_y = QPushButton()
        self.btn_mask_z = QPushButton()
        self.btn_mask_roll = QPushButton()
        self.btn_mask_pitch = QPushButton()
        self.btn_mask_yaw = QPushButton()

        for btn in [self.btn_mask_x, self.btn_mask_y, self.btn_mask_z,
                    self.btn_mask_roll, self.btn_mask_pitch, self.btn_mask_yaw]:
            btn.setCheckable(True)
            btn.setChecked(True)
            btn.clicked.connect(self.update_mask_styles)
            mask_layout.addWidget(btn)

        # Roll/Pitch 保留在消息和界面中用于协议兼容，但当前固件控制路径旁路。
        self.btn_mask_roll.setEnabled(False)
        self.btn_mask_pitch.setEnabled(False)
            
        left_layout.addWidget(mask_group)
        
        # 目标值组 (Setpoint Values)
        val_group = QGroupBox("目标值设定 (Setpoint Values)")
        val_layout = QGridLayout(val_group)
        
        val_layout.addWidget(QLabel("X轴目标 (Surge):"), 0, 0)
        self.spin_x = QDoubleSpinBox()
        self.spin_x.setRange(-1000.0, 1000.0)
        self.spin_x.setSingleStep(0.1)
        val_layout.addWidget(self.spin_x, 0, 1)
        
        val_layout.addWidget(QLabel("Y轴目标 (Sway):"), 1, 0)
        self.spin_y = QDoubleSpinBox()
        self.spin_y.setRange(-1000.0, 1000.0)
        self.spin_y.setSingleStep(0.1)
        val_layout.addWidget(self.spin_y, 1, 1)
        
        val_layout.addWidget(QLabel("Z轴目标 (Heave):"), 2, 0)
        self.spin_z = QDoubleSpinBox()
        self.spin_z.setRange(-1000.0, 1000.0)
        self.spin_z.setSingleStep(0.1)
        val_layout.addWidget(self.spin_z, 2, 1)
        
        val_layout.addWidget(QLabel("横滚目标 (Roll, rad，旁路):"), 3, 0)
        self.spin_roll = QDoubleSpinBox()
        self.spin_roll.setRange(-3.1416, 3.1416)
        self.spin_roll.setSingleStep(0.05)
        self.spin_roll.setEnabled(False)
        val_layout.addWidget(self.spin_roll, 3, 1)

        val_layout.addWidget(QLabel("俯仰目标 (Pitch, rad，旁路):"), 4, 0)
        self.spin_pitch = QDoubleSpinBox()
        self.spin_pitch.setRange(-3.1416, 3.1416)
        self.spin_pitch.setSingleStep(0.05)
        self.spin_pitch.setEnabled(False)
        val_layout.addWidget(self.spin_pitch, 4, 1)

        val_layout.addWidget(QLabel("偏航目标 (Yaw, rad):"), 5, 0)
        self.spin_yaw = QDoubleSpinBox()
        self.spin_yaw.setRange(-3.1416, 3.1416)
        self.spin_yaw.setSingleStep(0.05)
        val_layout.addWidget(self.spin_yaw, 5, 1)
        
        left_layout.addWidget(val_group)
        
        # 调试反馈
        debug_frame = QFrame()
        debug_frame.setStyleSheet("background-color: #1e1e1e; border: 1px solid #333333; border-radius: 6px;")
        debug_layout = QHBoxLayout(debug_frame)
        debug_layout.setContentsMargins(10, 8, 10, 8)
        self.lbl_debug_info = QLabel("control_key: 0x00 | type_mask: 0x00 | seq: 0")
        self.lbl_debug_info.setStyleSheet("color: #aaaaaa; font-family: monospace; font-size: 11px; background-color: transparent;")
        debug_layout.addWidget(self.lbl_debug_info)
        left_layout.addWidget(debug_frame)
        
        # 发送按钮
        self.btn_publish = QPushButton("单次下发 Setpoint 指令")
        self.btn_publish.clicked.connect(self.on_publish_clicked)
        self.btn_publish.setStyleSheet("""
            QPushButton {
                background-color: #00838f;
                color: white;
                font-weight: bold;
                font-size: 13px;
                padding: 12px;
                border: none;
                border-radius: 6px;
            }
            QPushButton:hover {
                background-color: #0097a7;
            }
        """)
        left_layout.addWidget(self.btn_publish)
        left_layout.addStretch()
        
        splitter.addWidget(left_widget)
        
        # =========================================================================
        # 右分栏：状态监测 (Status Monitoring)
        # =========================================================================
        right_widget = QWidget()
        right_layout = QVBoxLayout(right_widget)
        right_layout.setContentsMargins(5, 5, 5, 5)
        right_layout.setSpacing(10)
        
        title_right = QLabel("📊 实时状态监测 (/zit6/state/status)")
        title_right.setStyleSheet("font-size: 15px; font-weight: bold; color: #00e5ff;")
        right_layout.addWidget(title_right)
        
        # 核心控制状态组
        status_group = QGroupBox("控制状态 (AUV State)")
        status_layout = QGridLayout(status_group)
        status_layout.setVerticalSpacing(8)
        
        # 解锁状态
        status_layout.addWidget(QLabel("解锁状态 (Armed):"), 0, 0)
        self.lbl_armed = QLabel("未知 (Offline)")
        self.lbl_armed.setStyleSheet("font-weight: bold; font-size: 13px; color: #888888;")
        status_layout.addWidget(self.lbl_armed, 0, 1)
        
        # 解锁模式
        status_layout.addWidget(QLabel("解锁模式 (Arm Mode):"), 1, 0)
        self.lbl_arm_mode = QLabel("未知")
        self.lbl_arm_mode.setStyleSheet("font-weight: bold; color: #ffffff;")
        status_layout.addWidget(self.lbl_arm_mode, 1, 1)
        
        # 控制层级
        status_layout.addWidget(QLabel("控制层级 (Level):"), 2, 0)
        self.lbl_control_level = QLabel("未知")
        self.lbl_control_level.setStyleSheet("font-weight: bold; color: #ffffff;")
        status_layout.addWidget(self.lbl_control_level, 2, 1)
        
        # 惯导状态
        status_layout.addWidget(QLabel("惯导状态 (INS):"), 3, 0)
        self.lbl_ins_state = QLabel("未知")
        self.lbl_ins_state.setStyleSheet("font-weight: bold; color: #ffffff;")
        status_layout.addWidget(self.lbl_ins_state, 3, 1)
        
        # 导航就绪
        status_layout.addWidget(QLabel("导航就绪 (Nav Ready):"), 4, 0)
        self.lbl_nav_ready = QLabel("未知")
        self.lbl_nav_ready.setStyleSheet("font-weight: bold; color: #888888;")
        status_layout.addWidget(self.lbl_nav_ready, 4, 1)
        
        right_layout.addWidget(status_group)
        
        # 推进器实际输出组
        forces_group = QGroupBox("推进器输出 (forces)")
        forces_layout = QGridLayout(forces_group)
        forces_layout.setVerticalSpacing(8)
        self.lbl_forces = []
        for i, name in enumerate(["Fx", "Fy", "Fz", "Mroll", "Mpitch", "Myaw"]):
            lbl_name = QLabel(f"{name} Output:")
            lbl_val = QLabel("0.00 N")
            lbl_val.setStyleSheet("font-family: monospace; font-weight: bold; color: #00e5ff;")
            forces_layout.addWidget(lbl_name, i, 0)
            forces_layout.addWidget(lbl_val, i, 1)
            self.lbl_forces.append(lbl_val)
        right_layout.addWidget(forces_group)
        
        # 系统性能/运行指标
        perf_group = QGroupBox("运行指标 (Metrics)")
        perf_layout = QGridLayout(perf_group)
        
        perf_layout.addWidget(QLabel("循环耗时 (Cycle Time):"), 0, 0)
        self.lbl_cycle = QLabel("0.0 ms")
        self.lbl_cycle.setStyleSheet("font-family: monospace; font-weight: bold; color: #ffffff;")
        perf_layout.addWidget(self.lbl_cycle, 0, 1)
        
        perf_layout.addWidget(QLabel("电池电压 (Battery):"), 1, 0)
        self.lbl_voltage = QLabel("0.00 V")
        self.lbl_voltage.setStyleSheet("font-family: monospace; font-weight: bold; color: #4caf50;")
        perf_layout.addWidget(self.lbl_voltage, 1, 1)
        
        right_layout.addWidget(perf_group)
        
        # 异常诊断指示灯面板
        err_group = QGroupBox("异常诊断故障指示 (Diagnostics)")
        err_layout = QGridLayout(err_group)
        err_layout.setVerticalSpacing(10)
        
        self.err_lbl_fs = QLabel("🔴 致命强停 (FORCE STOP)")
        self.err_lbl_sf = QLabel("⚠️ 传感器故障 (SENSOR FAIL)")
        self.err_lbl_vl = QLabel("⚠️ 电压异常 (VOLTAGE LOW)")
        self.err_lbl_ct = QLabel("⚠️ 通讯超时 (TIMEOUT)")
        
        for i, lbl in enumerate([self.err_lbl_fs, self.err_lbl_sf, self.err_lbl_vl, self.err_lbl_ct]):
            lbl.setStyleSheet("color: #444444; font-weight: bold; font-size: 11px;")
            err_layout.addWidget(lbl, i // 2, i % 2)
            
        right_layout.addWidget(err_group)
        right_layout.addStretch()
        
        splitter.addWidget(right_widget)
        splitter.setSizes([500, 500])

    def get_control_key(self):
        mode = self.mode_combo.currentIndex()
        frame = 0x10 if self.frame_combo.currentIndex() == 1 else 0x00
        inc = 0x20 if self.inc_combo.currentIndex() == 1 else 0x00
        return mode | frame | inc

    def get_type_mask(self):
        mask = 0
        if not self.btn_mask_x.isChecked():
            mask |= 1
        if not self.btn_mask_y.isChecked():
            mask |= 2
        if not self.btn_mask_z.isChecked():
            mask |= 4
        if not self.btn_mask_roll.isChecked():
            mask |= 8
        if not self.btn_mask_pitch.isChecked():
            mask |= 16
        if not self.btn_mask_yaw.isChecked():
            mask |= 32
        return mask

    def update_mask_styles(self):
        buttons = [
            (self.btn_mask_x, "X轴控制中 🟢", "X轴未控制 🔴"),
            (self.btn_mask_y, "Y轴控制中 🟢", "Y轴未控制 🔴"),
            (self.btn_mask_z, "Z轴控制中 🟢", "Z轴未控制 🔴"),
            (self.btn_mask_roll, "Roll旁路 ⚪", "Roll旁路 ⚪"),
            (self.btn_mask_pitch, "Pitch旁路 ⚪", "Pitch旁路 ⚪"),
            (self.btn_mask_yaw, "Yaw控制中 🟢", "Yaw未控制 🔴")
        ]
        
        for btn, active_txt, inactive_txt in buttons:
            if btn.isChecked():
                btn.setText(active_txt)
                btn.setStyleSheet("""
                    QPushButton {
                        background-color: #2e7d32;
                        color: white;
                        font-weight: bold;
                        border: none;
                        border-radius: 4px;
                        padding: 6px 10px;
                        font-size: 11px;
                    }
                """)
            else:
                btn.setText(inactive_txt)
                btn.setStyleSheet("""
                    QPushButton {
                        background-color: #d84315;
                        color: white;
                        font-weight: bold;
                        border: none;
                        border-radius: 4px;
                        padding: 6px 10px;
                        font-size: 11px;
                    }
                """)
        self.update_hex_display()

    def update_hex_display(self):
        ck = self.get_control_key()
        tm = self.get_type_mask()
        self.lbl_debug_info.setText(f"control_key: 0x{ck:02X} | type_mask: 0x{tm:02X} | seq: {self.seq}")

    def on_publish_clicked(self):
        self.publish_setpoint()

    def publish_setpoint(self):
        try:
            msg = ZitSetpoint()
            msg.control_key = self.get_control_key()
            msg.type_mask = self.get_type_mask()
            msg.x = float(self.spin_x.value())
            msg.y = float(self.spin_y.value())
            msg.z = float(self.spin_z.value())
            msg.roll = 0.0  # 兼容字段；固件控制路径旁路
            msg.pitch = 0.0  # 兼容字段；固件控制路径旁路
            msg.yaw = float(self.spin_yaw.value())
            self.seq += 1
            msg.seq = self.seq
            
            self.pub.publish(msg)
            self.update_hex_display()
        except Exception:
            pass

    def send_stop_msg(self):
        try:
            msg = ZitSetpoint()
            msg.control_key = 16
            msg.type_mask = 0
            msg.x = 0.0
            msg.y = 0.0
            msg.z = 0.0
            msg.roll = 0.0
            msg.pitch = 0.0
            msg.yaw = 0.0
            self.pub.publish(msg)
        except Exception:
            pass

    def status_callback(self, msg):
        self.status_signal.emit(msg)

    def update_status_ui(self, msg):
        # 1. 解锁状态
        if msg.is_armed:
            self.lbl_armed.setText("已解锁 (ARMED) 🟢")
            self.lbl_armed.setStyleSheet("font-weight: bold; font-size: 13px; color: #4caf50;")
        else:
            self.lbl_armed.setText("已锁定 (LOCKED) 🔴")
            self.lbl_armed.setStyleSheet("font-weight: bold; font-size: 13px; color: #f44336;")
            
        # 2. 解锁模式
        if msg.arm_mode == 0:
            self.lbl_arm_mode.setText("DEFAULT (0) [导航就绪解锁]")
        elif msg.arm_mode == 3:
            self.lbl_arm_mode.setText("REMOTE (3) [直接强制遥控]")
        else:
            self.lbl_arm_mode.setText(f"UNKNOWN ({msg.arm_mode})")
            
        # 3. 控制层级
        levels = {0: "NONE (0) [安全挂起]", 1: "POS (1) [位置环]", 2: "VEL (2) [速度环]", 3: "FORCE (3) [直接力控]"}
        self.lbl_control_level.setText(levels.get(msg.control_level, f"UNKNOWN ({msg.control_level})"))
        
        # 4. 惯导状态
        ins_states = {
            0: "待机 (0)", 
            1: "粗对准 (1)", 
            2: "精对准 (2)", 
            3: "SINS/GPS/DVL组合 (3)", 
            4: "SINS/DVL组合 (4)", 
            5: "MRU状态 (5)"
        }
        self.lbl_ins_state.setText(ins_states.get(msg.ins_state, f"UNKNOWN ({msg.ins_state})"))
        
        # 5. 导航就绪
        if msg.navigation_ready:
            self.lbl_nav_ready.setText("就绪 (READY) 🟢")
            self.lbl_nav_ready.setStyleSheet("font-weight: bold; color: #4caf50;")
        else:
            self.lbl_nav_ready.setText("未对准 (NOT READY) 🔴")
            self.lbl_nav_ready.setStyleSheet("font-weight: bold; color: #f44336;")
            
        # 6. 推进器输出
        for i in range(min(6, len(msg.forces))):
            val = msg.forces[i]
            self.lbl_forces[i].setText(f"{val:+.2f} N")
            
        # 7. 运行指标
        self.lbl_cycle.setText(f"{msg.cycle_time_ms:.2f} ms")
        self.lbl_voltage.setText(f"{msg.battery_voltage:.2f} V")
        
        # 8. 异常诊断灯解析
        flags = msg.error_flags
        
        # Bit0: 强制强停 (FORCE_STOP)
        if flags & 1:
            self.err_lbl_fs.setStyleSheet("color: #f44336; font-weight: bold; font-size: 11px;")
        else:
            self.err_lbl_fs.setStyleSheet("color: #2e7d32; font-weight: bold; font-size: 11px;")
            
        # Bit1: 传感器故障 (SENSOR_FAIL)
        if flags & 2:
            self.err_lbl_sf.setStyleSheet("color: #ff9800; font-weight: bold; font-size: 11px;")
        else:
            self.err_lbl_sf.setStyleSheet("color: #2e7d32; font-weight: bold; font-size: 11px;")
            
        # Bit2: 电压异常 (VOLTAGE_LOW)
        if flags & 4:
            self.err_lbl_vl.setStyleSheet("color: #ff9800; font-weight: bold; font-size: 11px;")
        else:
            self.err_lbl_vl.setStyleSheet("color: #2e7d32; font-weight: bold; font-size: 11px;")
            
        # Bit3: 通讯超时 (TIMEOUT)
        if flags & 8:
            self.err_lbl_ct.setStyleSheet("color: #ff9800; font-weight: bold; font-size: 11px;")
        else:
            self.err_lbl_ct.setStyleSheet("color: #2e7d32; font-weight: bold; font-size: 11px;")

    def close(self):
        pass


class MotionControlApp(QMainWindow):
    """
    独立运行运动控制台窗口
    """
    def __init__(self, node, spinner):
        super().__init__()
        self.node = node
        self.spinner = spinner
        self.setWindowTitle("Zit6 AUV 运动控制台")
        self.resize(1000, 600)
        self.setStyleSheet("QMainWindow { background-color: #121212; }")
        
        self.widget = MotionControlWidget(self.node)
        self.setCentralWidget(self.widget)
        
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
    parser = argparse.ArgumentParser(description="Zit6 AUV 运动控制与状态监测工具")
    parser.add_argument("--cli", action="store_true", help="单次下发并监听状态一次退出")
    
    parsed_args, unknown = parser.parse_known_args(args=args)

    if parsed_args.cli:
        rclpy.init(args=args)
        node = Node('motion_control_cli')
        
        pub = node.create_publisher(ZitSetpoint, '/zit6/cmd/setpoint', 10)
        
        # 默认下发一帧空指令
        time.sleep(0.5)
        msg = ZitSetpoint()
        msg.control_key = 0
        msg.type_mask = 63
        msg.x = 0.0
        msg.y = 0.0
        msg.z = 0.0
        msg.roll = 0.0
        msg.pitch = 0.0
        msg.yaw = 0.0
        msg.seq = 1
        pub.publish(msg)
        print("已通过命令行下发默认空状态指令。")
        time.sleep(0.5)
        
        node.destroy_node()
        rclpy.shutdown()
    else:
        rclpy.init(args=args)
        node = Node('motion_control_gui_node')
        
        spinner = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
        spinner.start()
        
        app = QApplication(sys.argv)
        app.setStyle("Fusion")
        
        window = MotionControlApp(node, spinner)
        window.show()
        
        exit_code = app.exec_()
        
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(exit_code)

if __name__ == '__main__':
    main()
