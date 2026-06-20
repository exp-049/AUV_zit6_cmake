#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import threading

import rclpy
from rclpy.node import Node

# 导入各个独立模块的 Widget
from .image_viewer import ImageViewerWidget
from .xbox_control import XboxControlWidget
from .config_setter import ConfigWidget
from .motion_control import MotionControlWidget
from .heartbeat import FloatingHeartbeatPanel
from .log_viewer import LogViewerWidget

# Qt imports
try:
    from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
                                 QPushButton, QLabel, QStackedWidget, QFrame, QSplitter, QSizePolicy)
    from PyQt5.QtCore import Qt
    from PyQt5.QtGui import QIcon
except ImportError:
    from PySide6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
                                   QPushButton, QLabel, QStackedWidget, QFrame, QSplitter, QSizePolicy)
    from PySide6.QtCore import Qt, QAction as QIcon  # 兼容性定义
class MasterConsoleApp(QMainWindow):
    """
    Zit6 AUV 统一主控制台，整合所有控制和监测板块于一体
    """
    def __init__(self, node, spinner):
        super().__init__()
        self.node = node
        self.spinner = spinner
        
        self.setWindowTitle("Zit6 AUV 综合主控制台")
        self.resize(1200, 780)
        self.init_style()
        
        # 主布局：垂直分割，上部内容区 + 底部日志面板
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        self.main_layout = QVBoxLayout(main_widget)
        self.main_layout.setContentsMargins(0, 0, 0, 0)
        self.main_layout.setSpacing(0)

        # ── 上部：内容区（侧边栏 + 页面堆栈） ──
        content_area = QWidget()
        content_layout = QHBoxLayout(content_area)
        content_layout.setContentsMargins(0, 0, 0, 0)
        content_layout.setSpacing(0)

        # 1. 左侧导航侧边栏
        self.sidebar = QFrame()
        self.sidebar.setObjectName("Sidebar")
        self.sidebar.setFixedWidth(220)
        sidebar_layout = QVBoxLayout(self.sidebar)
        sidebar_layout.setContentsMargins(10, 20, 10, 20)
        sidebar_layout.setSpacing(10)
        
        # 标题标志
        logo_lbl = QLabel("ZIT6 CONSOLE")
        logo_lbl.setStyleSheet("font-size: 18px; font-weight: bold; color: #00e5ff; letter-spacing: 1px; padding: 10px 0 20px 5px;")
        sidebar_layout.addWidget(logo_lbl)
        
        # 导航按钮
        self.nav_buttons = []
        nav_items = [
            ("📷 图像监控", 0),
            ("🎮 手柄遥控", 1),
            ("⚙️ 参数配置", 2),
            ("⚓ 运动控制台", 3),
        ]
        
        for text, index in nav_items:
            btn = QPushButton(text)
            btn.setCheckable(True)
            btn.setFixedHeight(45)
            btn.clicked.connect(lambda checked, idx=index: self.switch_page(idx))
            sidebar_layout.addWidget(btn)
            self.nav_buttons.append(btn)
            
        sidebar_layout.addStretch()
        
        # 状态标语
        footer = QLabel("Status: Online")
        footer.setStyleSheet("color: #4caf50; font-size: 11px; padding-left: 10px;")
        sidebar_layout.addWidget(footer)
        
        content_layout.addWidget(self.sidebar)
        
        # 2. 右侧页面堆栈 (QStackedWidget)
        self.stacked_widget = QStackedWidget()
        content_layout.addWidget(self.stacked_widget)
        
        # 初始化各个业务 Widget 并加入堆栈
        self.image_widget = ImageViewerWidget(self.node)
        self.xbox_widget = XboxControlWidget(self.node)
        self.config_widget = ConfigWidget(self.node)
        self.motion_widget = MotionControlWidget(self.node)
        
        self.stacked_widget.addWidget(self.image_widget)
        self.stacked_widget.addWidget(self.xbox_widget)
        self.stacked_widget.addWidget(self.config_widget)
        self.stacked_widget.addWidget(self.motion_widget)

        # 将内容区加入主布局
        self.main_layout.addWidget(content_area, 1)

        # ── 底部：日志监控面板（始终显示） ──
        self.log_widget = LogViewerWidget(self.node)
        self.log_widget.setMaximumHeight(300)
        self.main_layout.addWidget(self.log_widget, 0)
        
        # 图像置顶特殊联动逻辑
        self.image_widget.pin_toggled_signal.connect(self.on_pin_toggled)
        
        # --- 创建全局唯一的浮动心跳面板（挂载到主窗口最上层） ---
        self.floating_hbt = FloatingHeartbeatPanel(self.node, self)
        self.floating_hbt.show()
        
        # 默认选中第一个导航页 (图像监控)
        self.switch_page(0)

    def init_style(self):
        self.setStyleSheet("""
            QMainWindow {
                background-color: #121212;
            }
            #Sidebar {
                background-color: #1a1a1f;
                border-right: 1px solid #2a2a2a;
            }
            #Sidebar QPushButton {
                background-color: transparent;
                color: #b0bec5;
                border: none;
                border-radius: 6px;
                font-size: 14px;
                font-weight: bold;
                text-align: left;
                padding-left: 20px;
            }
            #Sidebar QPushButton:hover {
                background-color: #263238;
                color: #ffffff;
            }
            #Sidebar QPushButton:checked {
                background-color: #006064;
                color: #00e5ff;
                border-left: 4px solid #00e5ff;
            }
            QStackedWidget {
                background-color: #121212;
            }
        """)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        # 固定将可拖拽心跳面板初始放置在右上角 (宽 260px，高 40px)
        self.floating_hbt.setGeometry(self.width() - 280, 15, 260, 42)
        # 强制将心跳面板置于所有层级最前方
        self.floating_hbt.raise_()

    def switch_page(self, index):
        self.stacked_widget.setCurrentIndex(index)
        for i, btn in enumerate(self.nav_buttons):
            btn.setChecked(i == index)
        # 切换页面时，确保浮动面板在最前部显示，防止被新载入的 Widget 压在下方
        self.floating_hbt.raise_()

    def on_pin_toggled(self, checked):
        """始终置顶联动"""
        geom = self.geometry()
        flags = self.windowFlags()
        if checked:
            self.setWindowFlags(flags | Qt.WindowStaysOnTopHint)
        else:
            self.setWindowFlags(flags & ~Qt.WindowStaysOnTopHint)
        self.show()
        self.setGeometry(geom)

    def closeEvent(self, event):
        # 窗口关闭时关闭各个组件的后台线程或定时器
        self.image_widget.close()
        self.xbox_widget.close()
        self.config_widget.close()
        self.motion_widget.close()
        self.log_widget.close()
        self.floating_hbt.close()
        super().closeEvent(event)


def main(args=None):
    rclpy.init(args=args)
    node = Node('master_console_gui_node')
    
    spinner = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spinner.start()
    
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    
    window = MasterConsoleApp(node, spinner)
    window.show()
    
    exit_code = app.exec_()
    
    node.destroy_node()
    rclpy.shutdown()
    
    sys.exit(exit_code)

if __name__ == '__main__':
    main()
