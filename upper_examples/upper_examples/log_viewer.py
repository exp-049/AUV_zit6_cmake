#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import datetime

from rcl_interfaces.msg import Log as RosLog

# Qt imports
try:
    from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QTextEdit, QHBoxLayout,
                                 QPushButton, QLabel, QCheckBox, QComboBox)
    from PyQt5.QtCore import Qt, QTimer, pyqtSignal
    from PyQt5.QtGui import QTextCursor, QColor
except ImportError:
    from PySide6.QtWidgets import (QWidget, QVBoxLayout, QTextEdit, QHBoxLayout,
                                   QPushButton, QLabel, QCheckBox, QComboBox)
    from PySide6.QtCore import Qt, QTimer, Signal as pyqtSignal
    from PySide6.QtGui import QTextCursor, QColor


# 日志级别 → 颜色映射
LEVEL_COLORS = {
    10: QColor(150, 150, 150),   # DEBUG: 灰色
    20: QColor(76, 175, 80),     # INFO:  绿色
    30: QColor(255, 193, 7),     # WARN:  黄色
    40: QColor(244, 67, 54),     # ERROR: 红色
    50: QColor(156, 39, 176),    # FATAL: 紫色
}

LEVEL_LABELS = {
    10: "[DEBUG]",
    20: "[INFO] ",
    30: "[WARN] ",
    40: "[ERROR]",
    50: "[FATAL]",
}


class LogViewerWidget(QWidget):
    """
    底部日志监控面板：订阅 /zit6/log，彩色显示，自动滚动，实时写入文件
    """

    # 线程安全信号：ROS spin 线程 → Qt 主线程
    _log_signal = pyqtSignal(object)

    def __init__(self, node):
        super().__init__()
        self.node = node
        self.auto_scroll = True
        self.level_filter = 0          # 0=全部
        self.log_file_path = os.path.expanduser("~/auv_run_log.txt")
        self._file_handle = None

        self.init_ui()
        self.init_subscription()
        self._open_log_file()

        # 将信号连接到主线程的槽
        self._log_signal.connect(self._on_log_main_thread)

    # ── 文件操作 ──
    def _open_log_file(self):
        """打开日志文件（追加模式）"""
        try:
            os.makedirs(os.path.dirname(self.log_file_path) or '.', exist_ok=True)
            self._file_handle = open(self.log_file_path, 'a', encoding='utf-8')
            ts = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            self._file_handle.write(f"\n{'='*60}\n")
            self._file_handle.write(f" Session start: {ts}\n")
            self._file_handle.write(f"{'='*60}\n")
            self._file_handle.flush()
        except Exception as e:
            print(f"[LogViewer] 无法打开日志文件: {e}")
            self._file_handle = None

    def _write_log(self, text: str):
        """实时写入一行日志到文件"""
        if self._file_handle is None:
            return
        try:
            self._file_handle.write(text + "\n")
            self._file_handle.flush()
        except Exception:
            pass

    def init_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(6, 4, 6, 4)
        layout.setSpacing(4)

        # ── 顶部工具栏 ──
        toolbar = QHBoxLayout()
        toolbar.setSpacing(8)

        title = QLabel("📋 /zit6/log")
        title.setStyleSheet("font-size: 13px; font-weight: bold; color: #00e5ff;")
        toolbar.addWidget(title)

        toolbar.addStretch()

        # 级别过滤
        self.filter_combo = QComboBox()
        self.filter_combo.addItems(["全部", "INFO+", "WARN+", "ERROR+"])
        self.filter_combo.setMinimumWidth(80)
        self.filter_combo.setMaximumHeight(24)
        self.filter_combo.currentIndexChanged.connect(self.on_filter_changed)
        toolbar.addWidget(QLabel("过滤:"))
        toolbar.addWidget(self.filter_combo)

        # 自动滚动开关
        self.scroll_cb = QCheckBox("自动滚动")
        self.scroll_cb.setChecked(True)
        self.scroll_cb.stateChanged.connect(
            lambda s: setattr(self, 'auto_scroll', bool(s)))
        toolbar.addWidget(self.scroll_cb)

        # 清空按钮
        clear_btn = QPushButton("清空")
        clear_btn.setFixedWidth(50)
        clear_btn.setFixedHeight(24)
        clear_btn.clicked.connect(self.on_clear)
        clear_btn.setStyleSheet("""
            QPushButton {
                background-color: #424242; color: #eee;
                border: none; border-radius: 3px; padding: 2px 6px;
                font-size: 12px;
            }
            QPushButton:hover { background-color: #616161; }
        """)
        toolbar.addWidget(clear_btn)

        layout.addLayout(toolbar)

        # ── 日志显示区域 ──
        self.log_view = QTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setMaximumHeight(220)
        self.log_view.setStyleSheet("""
            QTextEdit {
                background-color: #1a1a1f;
                color: #e0e0e0;
                font-family: 'Consolas', 'Courier New', monospace;
                font-size: 12px;
                border: 1px solid #333;
                border-radius: 3px;
                padding: 4px;
            }
        """)
        layout.addWidget(self.log_view)

    def init_subscription(self):
        self.sub = self.node.create_subscription(
            RosLog, '/zit6/log', self._on_log_thread, 10)

    def _on_log_thread(self, msg: RosLog):
        """ROS spin 线程中收到消息 → 通过信号派发到 Qt 主线程"""
        self._log_signal.emit(msg)

    def _on_log_main_thread(self, msg: RosLog):
        """Qt 主线程中处理 GUI 更新"""
        # 级别过滤
        if self.level_filter > 0 and msg.level < self.level_filter:
            return

        # 格式化时间
        ts = msg.stamp.sec + msg.stamp.nanosec / 1e9
        dt = datetime.datetime.fromtimestamp(ts)
        time_str = dt.strftime("%H:%M:%S.%f")[:12]

        level_label = LEVEL_LABELS.get(msg.level, f"[LVL{msg.level}]")
        color = LEVEL_COLORS.get(msg.level, QColor(200, 200, 200))

        line = f"{time_str} {level_label} ({msg.name}): {msg.msg}"

        # ★ 实时写入文件（防止崩溃丢失）
        self._write_log(line)

        # 添加到 QTextEdit（带颜色）—— 现在安全地在主线程操作
        cursor = self.log_view.textCursor()
        cursor.movePosition(QTextCursor.End)

        cursor.insertHtml(
            f'<span style="color:#888;">{time_str}</span> '
            f'<span style="color:{color.name()};">{level_label}</span> '
            f'<span style="color:#666;">({msg.name}):</span> '
            f'<span style="color:#e0e0e0;">{msg.msg}</span><br>'
        )

        if self.auto_scroll:
            self.log_view.verticalScrollBar().setValue(
                self.log_view.verticalScrollBar().maximum())

    def on_filter_changed(self, index):
        """过滤下拉框变化"""
        mapping = {0: 0, 1: 20, 2: 30, 3: 40}
        self.level_filter = mapping.get(index, 0)

    def on_clear(self):
        self.log_view.clear()

    def closeEvent(self, event):
        """关闭时写入结束标记并关闭文件句柄"""
        if self._file_handle:
            ts = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            try:
                self._file_handle.write(f" Session end: {ts}\n")
                self._file_handle.write(f"{'='*60}\n\n")
                self._file_handle.close()
            except Exception:
                pass
            self._file_handle = None
        self.node.destroy_subscription(self.sub)
        super().closeEvent(event)
