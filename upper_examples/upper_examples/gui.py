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
from .trajectory_viewer import TrajectoryViewerWidget

# Qt imports
try:
    from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
                                 QPushButton, QLabel, QStackedWidget, QFrame, QSplitter, QSizePolicy)
    from PyQt5.QtCore import Qt, QObject
    from PyQt5.QtGui import QFont, QFontDatabase, QFontMetrics, QIcon
except ImportError:
    from PySide6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
                                   QPushButton, QLabel, QStackedWidget, QFrame, QSplitter, QSizePolicy)
    from PySide6.QtCore import Qt, QObject
    from PySide6.QtGui import QFont, QFontDatabase, QFontMetrics, QIcon


def configure_application_font(app):
    """Select a font with CJK coverage when one is available in the image."""
    try:
        available = set(QFontDatabase().families())
    except Exception:
        return

    for family in (
        "Noto Sans CJK SC",
        "Noto Sans CJK TC",
        "WenQuanYi Zen Hei",
        "Droid Sans Fallback",
    ):
        if family in available:
            font = QFont(family)
            # A family can be registered without covering the glyphs we use.
            if QFontMetrics(font).inFontUcs4(ord("中")):
                app.setFont(font)
                return

    sys.stderr.write(
        "[ZIT6 GUI] No CJK-capable font found; install fonts-noto-cjk.\n"
    )


def configure_runtime_dir():
    """Provide Qt with a private runtime directory in minimal containers."""
    if os.environ.get("XDG_RUNTIME_DIR"):
        return

    uid = getattr(os, "getuid", lambda: 0)()
    runtime_dir = os.path.join("/tmp", f"runtime-{uid}")
    try:
        os.makedirs(runtime_dir, mode=0o700, exist_ok=True)
        os.chmod(runtime_dir, 0o700)
    except OSError:
        return
    os.environ["XDG_RUNTIME_DIR"] = runtime_dir


class _EmbeddedRqtContext(QObject):
    """Small PluginContext adapter for embedding rqt_console in our stack."""

    def __init__(self, node, layout):
        super().__init__()
        self.node = node
        self._layout = layout
        self.widget = None

    def serial_number(self):
        return 1

    def add_widget(self, widget):
        self.widget = widget
        self._layout.addWidget(widget)


class RqtConsolePage(QWidget):
    """Embedded Foxy rqt_console page, sharing the master ROS node."""

    def __init__(self, node):
        super().__init__()
        self._plugin = None
        self._context = None
        self._filter_splitter = None

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(4)

        try:
            from rqt_console.console import Console

            # rqt_console is normally shown in its own window.  Its filter
            # panes are very tall when embedded, so keep them collapsed and
            # expose them through a small explicit toggle.
            self._filter_toggle = QPushButton("显示筛选器")
            self._filter_toggle.setCheckable(True)
            self._filter_toggle.setFixedHeight(30)
            self._filter_toggle.setStyleSheet("""
                QPushButton {
                    background-color: #263238;
                    color: #e0f7fa;
                    border: 1px solid #455a64;
                    border-radius: 4px;
                    padding: 3px 10px;
                }
                QPushButton:hover {
                    background-color: #37474f;
                    border-color: #00bcd4;
                }
                QPushButton:checked {
                    background-color: #006064;
                    color: #ffffff;
                }
            """)
            self._filter_toggle.clicked.connect(self._toggle_filters)
            layout.addWidget(self._filter_toggle, 0, Qt.AlignLeft)

            self._context = _EmbeddedRqtContext(node, layout)
            self._plugin = Console(self._context)
            self._style_console_widget(self._context.widget)
        except Exception as exc:
            # Keep the main console usable on images that do not include the
            # optional rqt_console Debian package.
            message = QLabel(
                "rqt_console 不可用\n\n"
                f"{type(exc).__name__}: {exc}\n\n"
                "请安装 ros-foxy-rqt-console 后重新启动 GUI。")
            message.setAlignment(Qt.AlignCenter)
            message.setStyleSheet("color: #ffb74d; font-size: 14px;")
            layout.addWidget(message)

    def _style_console_widget(self, widget):
        """Make the stock rqt_console controls fit the dark master console."""
        if widget is None:
            return

        widget.setObjectName("EmbeddedRqtConsole")
        widget.setStyleSheet("""
            QWidget#EmbeddedRqtConsole {
                background-color: #121212;
                color: #d7e0e3;
            }
            QWidget#EmbeddedRqtConsole QLabel,
            QWidget#EmbeddedRqtConsole QGroupBox {
                color: #aebdc2;
            }
            QWidget#EmbeddedRqtConsole QPushButton {
                background-color: #263238;
                color: #e0f7fa;
                border: 1px solid #455a64;
                border-radius: 4px;
                padding: 4px 10px;
                min-height: 24px;
            }
            QWidget#EmbeddedRqtConsole QPushButton:hover {
                background-color: #37474f;
                border-color: #00bcd4;
            }
            QWidget#EmbeddedRqtConsole QPushButton:pressed,
            QWidget#EmbeddedRqtConsole QPushButton:checked {
                background-color: #006064;
                color: #ffffff;
            }
            QWidget#EmbeddedRqtConsole QTableView,
            QWidget#EmbeddedRqtConsole QTableWidget {
                background-color: #17191b;
                alternate-background-color: #1d2224;
                color: #d7e0e3;
                gridline-color: #303a3e;
                selection-background-color: #005662;
                selection-color: #ffffff;
                border: 1px solid #303a3e;
            }
            QWidget#EmbeddedRqtConsole QHeaderView::section {
                background-color: #263238;
                color: #d8f3f5;
                border: 1px solid #37474f;
                padding: 4px 6px;
            }
            QWidget#EmbeddedRqtConsole QLineEdit,
            QWidget#EmbeddedRqtConsole QComboBox,
            QWidget#EmbeddedRqtConsole QDateTimeEdit,
            QWidget#EmbeddedRqtConsole QTextEdit,
            QWidget#EmbeddedRqtConsole QPlainTextEdit {
                background-color: #1e2224;
                color: #d7e0e3;
                border: 1px solid #455a64;
                border-radius: 3px;
                padding: 3px;
            }
            QWidget#EmbeddedRqtConsole QComboBox QAbstractItemView {
                background-color: #1e2224;
                color: #d7e0e3;
                selection-background-color: #006064;
            }
            QWidget#EmbeddedRqtConsole QSplitter::handle {
                background-color: #263238;
            }
            QWidget#EmbeddedRqtConsole QScrollBar:vertical,
            QWidget#EmbeddedRqtConsole QScrollBar:horizontal {
                background-color: #17191b;
            }
        """)

        # rqt_console replaces several labels with theme icons.  Minimal
        # Docker images often have no icon theme, which leaves blank white
        # buttons.  Use text labels and clear the optional icons.
        button_labels = {
            "load_button": "加载",
            "save_button": "保存",
            "pause_button": "暂停",
            "record_button": "继续",
            "clear_button": "清空",
            "column_resize_button": "自适应列",
            "add_exclude_button": "+",
            "highlight_exclude_button": "仅高亮",
            "add_highlight_button": "+",
        }
        for object_name, label in button_labels.items():
            button = getattr(widget, object_name, None)
            if button is None:
                continue
            button.setIcon(QIcon())
            button.setText(label)
            if object_name in {"add_exclude_button", "add_highlight_button"}:
                button.setMaximumWidth(44)
                button.setMinimumWidth(36)
            elif object_name == "highlight_exclude_button":
                button.setMaximumWidth(80)
                button.setMinimumWidth(68)

        # The master console has a floating heartbeat panel in the upper
        # right corner.  Reserve that area in rqt_console's toolbar so its
        # Clear/Fit buttons remain clickable instead of being covered.
        root_layout = widget.layout()
        if root_layout is not None and root_layout.count():
            toolbar_layout = root_layout.itemAt(0).layout()
            if toolbar_layout is not None:
                margins = toolbar_layout.contentsMargins()
                toolbar_layout.setContentsMargins(
                    margins.left(), margins.top(),
                    max(margins.right(), 280), margins.bottom())

        self._filter_splitter = getattr(widget, "filter_splitter", None)
        table_splitter = getattr(widget, "table_splitter", None)
        if self._filter_splitter is not None:
            self._filter_splitter.setVisible(False)
        if table_splitter is not None:
            table_splitter.setSizes([1, 0])

    def _toggle_filters(self, checked):
        if self._filter_splitter is None:
            return

        self._filter_splitter.setVisible(checked)
        self._filter_toggle.setText("隐藏筛选器" if checked else "显示筛选器")
        table_splitter = getattr(self._context.widget, "table_splitter", None)
        if table_splitter is None:
            return
        table_splitter.setSizes([2, 1] if checked else [1, 0])

    def close(self):
        if self._plugin is not None:
            try:
                self._plugin.shutdown_plugin()
            except Exception as exc:
                sys.stderr.write(f"[ZIT6 GUI] rqt_console shutdown: {exc}\n")
            self._plugin = None
        super().close()


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
        # Qt's stylesheet engine does not implement CSS character spacing;
        # leaving that unsupported rule here only produces a warning.
        logo_lbl.setStyleSheet("font-size: 18px; font-weight: bold; color: #00e5ff; padding: 10px 0 20px 5px;")
        sidebar_layout.addWidget(logo_lbl)
        
        # 导航按钮
        self.nav_buttons = []
        nav_items = [
            ("图像监控", 0),
            ("手柄遥控", 1),
            ("参数配置", 2),
            ("运动控制台", 3),
            ("三维轨迹 / 位姿", 4),
            ("日志控制台", 5),
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
        self.trajectory_widget = TrajectoryViewerWidget(self.node)
        self.rqt_console_widget = RqtConsolePage(self.node)
        
        self.stacked_widget.addWidget(self.image_widget)
        self.stacked_widget.addWidget(self.xbox_widget)
        self.stacked_widget.addWidget(self.config_widget)
        self.stacked_widget.addWidget(self.motion_widget)
        self.stacked_widget.addWidget(self.trajectory_widget)
        self.stacked_widget.addWidget(self.rqt_console_widget)

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
        self.trajectory_widget.close()
        self.log_widget.close()
        self.rqt_console_widget.close()
        self.floating_hbt.close()
        super().closeEvent(event)


def main(args=None):
    configure_runtime_dir()
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    configure_application_font(app)

    rclpy.init(args=args)
    node = Node('master_console_gui_node')

    spinner = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spinner.start()
    
    window = MasterConsoleApp(node, spinner)
    window.show()
    
    exit_code = app.exec_()
    
    rclpy.shutdown()
    spinner.join(timeout=2.0)
    node.destroy_node()
    
    sys.exit(exit_code)

if __name__ == '__main__':
    main()
