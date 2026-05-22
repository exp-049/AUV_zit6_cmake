#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import argparse
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt32

# Qt imports
try:
    from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QFrame, QHBoxLayout, QLabel, QComboBox, QPushButton
    from PyQt5.QtCore import Qt, QTimer
except ImportError:
    from PySide6.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QFrame, QHBoxLayout, QLabel, QComboBox, QPushButton
    from PySide6.QtCore import Qt, QTimer

class FloatingHeartbeatPanel(QFrame):
    """
    一个可拖拽的、全局统一的解锁心跳下发浮动面板。
    可以放置在任何主窗口或独立窗口之上。
    """
    def __init__(self, node, parent=None):
        super().__init__(parent)
        self.node = node
        self.active = False
        self.mode = 1
        self.pub_timer = None
        self.draggable = True
        
        self.pub = self.node.create_publisher(UInt32, '/zit6/cmd/agxhbt', 10)
        self.drag_position = None
        
        self.init_ui()

    def init_ui(self):
        self.setObjectName("FloatingHbtPanel")
        self.setStyleSheet("""
            #FloatingHbtPanel {
                background-color: rgba(25, 25, 25, 0.95);
                border: 2px solid #00e5ff;
                border-radius: 8px;
            }
            QLabel {
                color: #00e5ff;
                font-weight: bold;
                font-size: 11px;
            }
            QComboBox {
                background-color: #1e1e1e;
                color: white;
                border: 1px solid #444444;
                border-radius: 4px;
                font-size: 11px;
                padding: 3px 6px;
                min-width: 80px;
            }
            QComboBox QAbstractItemView {
                background-color: #1e1e1e;
                color: white;
                selection-background-color: #006064;
                selection-color: #00e5ff;
                border: 1px solid #444444;
            }
        """)
        
        layout = QHBoxLayout(self)
        layout.setContentsMargins(8, 6, 8, 6)
        layout.setSpacing(10)
        
        # 拖拽句柄
        self.handle = QLabel("⋮⋮")
        self.handle.setStyleSheet("color: #888888; font-size: 13px; font-weight: bold;")
        
        title = QLabel("心跳解锁:")
        
        self.mode_combo = QComboBox()
        self.mode_combo.addItems(["常规 (1)", "推力 (3)"])
        self.mode_combo.currentIndexChanged.connect(self.on_mode_changed)
        
        self.btn_toggle = QPushButton("开始下发")
        self.btn_toggle.setCheckable(True)
        self.btn_toggle.setStyleSheet("""
            QPushButton {
                background-color: #2e7d32;
                color: white;
                font-weight: bold;
                font-size: 11px;
                border: none;
                border-radius: 4px;
                padding: 5px 12px;
            }
        """)
        self.btn_toggle.clicked.connect(self.on_toggle_clicked)
        
        layout.addWidget(self.handle)
        layout.addWidget(title)
        layout.addWidget(self.mode_combo)
        layout.addWidget(self.btn_toggle)

    def on_mode_changed(self, index):
        self.mode = 1 if index == 0 else 3

    def on_toggle_clicked(self, checked):
        self.active = checked
        if checked:
            self.btn_toggle.setText("停止 (急停) 🛑")
            self.btn_toggle.setStyleSheet("""
                QPushButton {
                    background-color: #d84315;
                    color: white;
                    font-weight: bold;
                    font-size: 11px;
                    border: none;
                    border-radius: 4px;
                    padding: 5px 12px;
                }
            """)
            self.pub_timer = self.node.create_timer(0.1, self.send_heartbeat)
        else:
            self.btn_toggle.setText("开始下发")
            self.btn_toggle.setStyleSheet("""
                QPushButton {
                    background-color: #2e7d32;
                    color: white;
                    font-weight: bold;
                    font-size: 11px;
                    border: none;
                    border-radius: 4px;
                    padding: 5px 12px;
                }
            """)
            if self.pub_timer:
                self.node.destroy_timer(self.pub_timer)
                self.pub_timer = None

    def send_heartbeat(self):
        try:
            msg = UInt32()
            msg.data = self.mode
            self.pub.publish(msg)
        except Exception:
            pass

    def mousePressEvent(self, event):
        if self.draggable and event.button() == Qt.LeftButton:
            self.drag_position = event.globalPos() - self.frameGeometry().topLeft()
            event.accept()
        else:
            super().mousePressEvent(event)

    def mouseMoveEvent(self, event):
        if self.draggable and event.buttons() == Qt.LeftButton and self.drag_position is not None:
            self.move(event.globalPos() - self.drag_position)
            event.accept()
        else:
            super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event):
        self.drag_position = None
        super().mouseReleaseEvent(event)

    def close(self):
        if self.pub_timer:
            self.node.destroy_timer(self.pub_timer)
            self.pub_timer = None


class HeartbeatApp(QMainWindow):
    """
    心跳发布器独立 GUI 窗口，直接复用 FloatingHeartbeatPanel
    """
    def __init__(self, node, spinner):
        super().__init__()
        self.node = node
        self.spinner = spinner
        self.setWindowTitle("ROS 2 心跳发布控制台")
        self.resize(320, 100)
        self.setStyleSheet("QMainWindow { background-color: #121212; }")
        
        # 居中显示
        central = QWidget()
        self.setCentralWidget(central)
        layout = QVBoxLayout(central)
        layout.setContentsMargins(15, 15, 15, 15)
        
        # 实例化共享的心跳浮动面板并嵌入 (静态展示，取消拖拽/句柄)
        self.panel = FloatingHeartbeatPanel(self.node, self)
        self.panel.draggable = False
        self.panel.handle.hide()  # 隐藏拖拽句柄
        
        # 移除浮空阴影/边框限制以适应主窗口内嵌样式
        self.panel.setStyleSheet("""
            #FloatingHbtPanel {
                background-color: transparent;
                border: none;
            }
            QLabel {
                color: #00e5ff;
                font-weight: bold;
                font-size: 12px;
            }
        """)
        
        layout.addWidget(self.panel)

    def closeEvent(self, event):
        self.panel.close()
        super().closeEvent(event)


def main(args=None):
    # 处理 CLI 命令行参数
    parser = argparse.ArgumentParser(description="ROS 2 心跳下发程序 (常规/推力解锁)")
    parser.add_argument("--cli", action="store_true", help="是否在命令行纯控制模式下运行")
    parser.add_argument("--mode", type=int, choices=[1, 3], default=1, help="心跳解锁模式 (1: 常规arm, 3: 推力arm)")
    parser.add_argument("--freq", type=float, default=10.0, help="心跳发送频率 (Hz)")
    
    parsed_args, unknown = parser.parse_known_args(args=args)

    if parsed_args.cli:
        rclpy.init(args=args)
        node = Node('heartbeat_cli_node')
        
        # CLI 模式：持续下发心跳
        print("=" * 60)
        print(f" [Heartbeat CLI] 心跳发布程序已启动 (纯终端模式)")
        print(f" - 解锁模式 (agxhbt): {parsed_args.mode} ({'常规' if parsed_args.mode == 1 else '推力'})")
        print(f" - 发送频率 (Hz): {parsed_args.freq}")
        print(" 按下 Ctrl+C 可停止下发...")
        print("=" * 60)

        pub = node.create_publisher(UInt32, '/zit6/cmd/agxhbt', 10)
        msg = UInt32()
        msg.data = parsed_args.mode
        interval = 1.0 / parsed_args.freq
        
        try:
            while rclpy.ok():
                pub.publish(msg)
                time.sleep(interval)
        except KeyboardInterrupt:
            print("\n已停止心跳下发...")
        finally:
            node.destroy_node()
            rclpy.shutdown()
    else:
        rclpy.init(args=args)
        node = Node('heartbeat_gui_node')
        
        # GUI 模式：拉起 Qt 界面
        spinner = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
        spinner.start()
        
        app = QApplication(sys.argv)
        app.setStyle("Fusion")
        
        window = HeartbeatApp(node, spinner)
        window.show()
        
        exit_code = app.exec_()
        
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(exit_code)

if __name__ == '__main__':
    main()
