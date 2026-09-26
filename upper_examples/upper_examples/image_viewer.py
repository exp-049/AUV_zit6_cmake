#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Preview the iceoryx2 camera streams exposed by go2rtc.

Camera pixels arrive at camera_streamer over iceoryx2 and are encoded as
H.264 before go2rtc exposes them. The GUI uses HTTP/WebRTC for preview and the HTTP MP4 API for recording;
pixels do not travel through ROS/DDS.
"""

import argparse
from datetime import datetime
import os
from pathlib import Path
import signal
import sys
import subprocess
import threading
import time
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

import rclpy
from rclpy.node import Node

from .heartbeat import FloatingHeartbeatPanel

os.environ.pop("QT_QPA_PLATFORM_PLUGIN_PATH", None)

try:
    from PyQt5.QtWidgets import (
        QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
        QListWidget, QListWidgetItem, QPushButton, QLabel, QSplitter,
        QSizePolicy, QMenu, QAction, QComboBox, QLineEdit, QStackedWidget,
        QCheckBox, QSpinBox,
    )
    from PyQt5.QtCore import Qt, pyqtSignal, QThread, QTimer, QUrl
    _QT_BINDING = "PyQt5"
    from PyQt5.QtGui import QImage, QPixmap
except ImportError:
    from PySide6.QtWidgets import (
        QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
        QListWidget, QListWidgetItem, QPushButton, QLabel, QSplitter,
        QSizePolicy, QMenu, QComboBox, QLineEdit, QStackedWidget,
        QCheckBox, QSpinBox,
    )
    from PySide6.QtCore import Qt, Signal as pyqtSignal, QThread, QTimer, QUrl
    _QT_BINDING = "PySide6"
    from PySide6.QtGui import QImage, QPixmap, QAction

try:
    if _QT_BINDING == "PyQt5":
        from PyQt5.QtWebEngineWidgets import (
            QWebEngineView, QWebEngineSettings, QWebEngineScript,
        )
    else:
        from PySide6.QtWebEngineWidgets import QWebEngineView, QWebEngineSettings
        from PySide6.QtWebEngineCore import QWebEngineScript
except ImportError:
    QWebEngineView = None
    QWebEngineSettings = None
    QWebEngineScript = None


DEFAULT_ENDPOINT = os.environ.get(
    "ZIT6_GO2RTC_ENDPOINT", "192.168.16.10:1984")
STREAMS = (
    ("front", "前视", "raw", "原始"),
    ("front_annotated", "前视", "annotated", "标注"),
    ("down", "下视", "raw", "原始"),
    ("down_annotated", "下视", "annotated", "标注"),
)

GO2RTC_WEBRTC_COMPAT_JS = r"""
(() => {
    const prototype = window.RTCPeerConnection &&
        window.RTCPeerConnection.prototype;
    if (!prototype || "connectionState" in prototype) return;

    // QtWebEngine 5.12 (Chromium 69) lacks the connectionState API used by
    // go2rtc's player. Map its connection events to the available ICE state.
    Object.defineProperty(prototype, "connectionState", {
        configurable: true,
        get() {
            const state = this.iceConnectionState;
            if (state === "completed") return "connected";
            if (state === "checking") return "connecting";
            return state;
        },
    });

    const addEventListener = prototype.addEventListener;
    prototype.addEventListener = function(type, listener, options) {
        if (type !== "connectionstatechange") {
            return addEventListener.call(this, type, listener, options);
        }
        return addEventListener.call(
            this,
            "iceconnectionstatechange",
            function(event) {
                if (typeof listener === "function") {
                    listener.call(this, event);
                } else if (listener && typeof listener.handleEvent === "function") {
                    listener.handleEvent(event);
                }
            },
            options,
        );
    };
})();
"""


def _extract_jpeg_frames(buffer):
    """Extract complete JPEGs from an arbitrary MJPEG byte chunk."""
    frames = []
    while True:
        start = buffer.find(b"\xff\xd8")
        if start < 0:
            # Keep one byte in case the next chunk begins with FF D8.
            if len(buffer) > 1:
                del buffer[:-1]
            break

        end = buffer.find(b"\xff\xd9", start + 2)
        if end < 0:
            if start:
                del buffer[:start]
            break

        frames.append(bytes(buffer[start:end + 2]))
        del buffer[:end + 2]
    return frames


class MjpegStreamThread(QThread):
    """Read one MJPEG response and emit individual JPEG frames."""

    frame_received = pyqtSignal(bytes)
    stream_error = pyqtSignal(str)

    def __init__(self, url, parent=None):
        super().__init__(parent)
        self.url = url
        self._stop_event = threading.Event()
        self._response = None
        self._response_lock = threading.Lock()

    def stop(self):
        self._stop_event.set()
        with self._response_lock:
            response = self._response
        if response is not None:
            try:
                response.close()
            except OSError:
                pass
        if self.isRunning():
            self.wait(2500)

    def run(self):
        request = Request(
            self.url,
            headers={
                "Accept": "multipart/x-mixed-replace",
                "Cache-Control": "no-cache",
                "User-Agent": "ZIT6-console-image-monitor/1.0",
            },
        )
        try:
            response = urlopen(request, timeout=8.0)
            with self._response_lock:
                self._response = response

            content_type = response.headers.get("Content-Type", "").lower()
            if response.getcode() != 200:
                raise RuntimeError(f"HTTP {response.getcode()}")
            if "multipart/x-mixed-replace" not in content_type:
                raise RuntimeError(
                    f"不是 MJPEG 流 (Content-Type: {content_type or 'unknown'})")

            buffer = bytearray()
            while not self._stop_event.is_set():
                chunk = response.read(64 * 1024)
                if not chunk:
                    break
                buffer.extend(chunk)
                for frame in _extract_jpeg_frames(buffer):
                    if self._stop_event.is_set():
                        break
                    self.frame_received.emit(frame)
        except (HTTPError, URLError, OSError, TimeoutError, RuntimeError) as exc:
            if not self._stop_event.is_set():
                reason = getattr(exc, "reason", exc)
                self.stream_error.emit(str(reason))
        except Exception as exc:
            if not self._stop_event.is_set():
                self.stream_error.emit(str(exc))
        finally:
            with self._response_lock:
                response = self._response
                self._response = None
            if response is not None:
                try:
                    response.close()
                except OSError:
                    pass


class ClickableLabel(QLabel):
    double_clicked = pyqtSignal()

    def mouseDoubleClickEvent(self, event):
        self.double_clicked.emit()
        super().mouseDoubleClickEvent(event)


class ImageViewerWidget(QWidget):
    """Show two independently selectable network video streams."""

    pin_toggled_signal = pyqtSignal(bool)

    def __init__(self, node, auto_connect=True):
        super().__init__()
        self.node = node  # Kept for the shared heartbeat panel/API.
        self.aspect_ratio_mode = Qt.KeepAspectRatio
        self._source_selects = []
        self._transport_selects = []
        self._video_panes = []
        self._video_stacks = []
        self._video_labels = []
        self._video_webviews = []
        self._video_status = []
        self._stream_threads = [None, None]
        self._current_streams = [None, None]
        self._last_pixmaps = [QPixmap(), QPixmap()]
        self._frame_counts = [0, 0]
        self._active_endpoint = None
        self._endpoint_dirty = False
        self._record_process = None
        self._record_output_dir = None
        self._record_started_at = None
        self._record_streams = []
        self._record_format = "mkv"
        self._record_log_path = None
        self._record_log_handle = None
        self._record_stop_requested = False

        self.init_ui()

        self.record_timer = QTimer(self)
        self.record_timer.timeout.connect(self._poll_recording)
        self.record_timer.start(1000)
        if auto_connect:
            QTimer.singleShot(0, self.connect_selected_streams)

    def init_ui(self):
        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        self.splitter = QSplitter(Qt.Horizontal)
        layout.addWidget(self.splitter)

        self.sidebar_widget = QWidget()
        sidebar_layout = QVBoxLayout(self.sidebar_widget)
        sidebar_layout.setContentsMargins(5, 5, 5, 5)

        title_label = QLabel("网络图像源（go2rtc）")
        title_label.setStyleSheet(
            "font-size: 15px; font-weight: bold; color: #00e5ff; "
            "margin-bottom: 5px;")
        sidebar_layout.addWidget(title_label)

        hint_label = QLabel(
            "预览和录像都使用 go2rtc HTTP 地址（默认端口 1984）；"
            "预览可选 WebRTC、MSE 或自动选择")
        hint_label.setStyleSheet("font-size: 11px; color: #9aa7ad;")
        hint_label.setWordWrap(True)
        sidebar_layout.addWidget(hint_label)

        endpoint_label = QLabel("预览/API 地址（HTTP，默认端口 1984）")
        endpoint_label.setStyleSheet("color: #00e5ff; font-weight: bold;")
        sidebar_layout.addWidget(endpoint_label)

        self.endpoint_edit = QLineEdit(DEFAULT_ENDPOINT)
        self.endpoint_edit.setPlaceholderText("例如 192.168.16.10:1984")
        self.endpoint_edit.setStyleSheet(
            "QLineEdit { background: #1e1e1e; color: #e0e0e0; "
            "padding: 6px; border: 1px solid #444; border-radius: 5px; }")
        self.endpoint_edit.textEdited.connect(self._on_endpoint_edited)
        self.endpoint_edit.returnPressed.connect(self.connect_selected_streams)
        sidebar_layout.addWidget(self.endpoint_edit)

        self.btn_connect = QPushButton("连接/应用地址")
        self.btn_connect.clicked.connect(self.connect_selected_streams)
        sidebar_layout.addWidget(self.btn_connect)

        for slot in range(2):
            default_camera = "前视" if slot == 0 else "下视"
            slot_label = QLabel(f"画面 {slot + 1} 来源")
            slot_label.setStyleSheet("color: #00e5ff; font-weight: bold;")
            sidebar_layout.addWidget(slot_label)

            combo = QComboBox()
            combo.setMinimumContentsLength(18)
            combo.setStyleSheet(
                "QComboBox { background: #1e1e1e; color: #e0e0e0; "
                "padding: 6px; border: 1px solid #444; border-radius: 5px; }"
            )
            for path, camera, mode, mode_label in STREAMS:
                label = f"{camera} / {mode_label}"
                source = {
                    "path": path,
                    "camera": camera,
                    "mode": mode,
                    "mode_label": mode_label,
                }
                combo.addItem(label, source)
            default_index = next(
                index for index, source in enumerate(STREAMS)
                if source[1] == default_camera and source[2] == "raw")
            combo.setCurrentIndex(default_index)
            combo.currentIndexChanged.connect(
                lambda index, s=slot: self._on_source_selected(s, index))
            self._source_selects.append(combo)
            sidebar_layout.addWidget(combo)

            transport_label = QLabel(f"画面 {slot + 1} 接收方式")
            transport_label.setStyleSheet("color: #9aa7ad;")
            sidebar_layout.addWidget(transport_label)
            transport_combo = QComboBox()
            transport_combo.addItem("WebRTC（低延迟）", "webrtc")
            transport_combo.addItem("MSE（H.264）", "mse")
            transport_combo.addItem("自动选择", "auto")
            transport_combo.setStyleSheet(
                "QComboBox { background: #1e1e1e; color: #e0e0e0; "
                "padding: 6px; border: 1px solid #444; border-radius: 5px; }")
            transport_combo.currentIndexChanged.connect(
                lambda index, s=slot: self._on_source_selected(
                    s, self._source_selects[s].currentIndex()))
            self._transport_selects.append(transport_combo)
            sidebar_layout.addWidget(transport_combo)

        self.address_status_label = QLabel("默认地址将在启动时尝试连接")
        self.address_status_label.setStyleSheet(
            "font-size: 11px; color: #888888; padding: 5px;")
        self.address_status_label.setWordWrap(True)
        sidebar_layout.addWidget(self.address_status_label)

        record_options_label = QLabel("录制设置")
        record_options_label.setStyleSheet("color: #00e5ff; font-weight: bold;")
        sidebar_layout.addWidget(record_options_label)

        self.record_stream_checks = []
        for slot in range(2):
            check = QCheckBox()
            check.setChecked(True)
            check.setStyleSheet("color: #cfd8dc; padding: 2px;")
            self.record_stream_checks.append(check)
            self._update_record_stream_label(slot)
            sidebar_layout.addWidget(check)

        format_label = QLabel("保存格式")
        format_label.setStyleSheet("color: #9aa7ad;")
        sidebar_layout.addWidget(format_label)
        self.record_format_combo = QComboBox()
        self.record_format_combo.addItem("MKV（保留原编码）", "mkv")
        self.record_format_combo.addItem("MP4（H.264，音频有则 AAC）", "mp4")
        self.record_format_combo.setToolTip(
            "MKV 和 MP4 都保留 H.264 视频，不做二次压缩；若有音频，MP4 转为 AAC。"
            "停止时会将首个媒体时间戳归零并重新封装。")
        sidebar_layout.addWidget(self.record_format_combo)

        record_path_label = QLabel(
            "录像通过 go2rtc HTTP MP4 接口（1984）接收 H.264；MKV/MP4 都保留原视频码流，"
            "与预览选择 WebRTC、MSE 或自动选择无关。")
        record_path_label.setStyleSheet("font-size: 11px; color: #9aa7ad;")
        record_path_label.setWordWrap(True)
        sidebar_layout.addWidget(record_path_label)

        duration_label = QLabel("录制时长（墙上时钟秒；0 表示手动停止）")
        duration_label.setStyleSheet("color: #9aa7ad;")
        duration_label.setToolTip(
            "计时从收到首帧后开始。输出文件保留视频源时间轴；"
            "仿真速度低于实时速率时，文件时长会短于此设置。")
        sidebar_layout.addWidget(duration_label)
        self.record_duration_spin = QSpinBox()
        self.record_duration_spin.setRange(0, 86400)
        self.record_duration_spin.setValue(0)
        self.record_duration_spin.setSuffix(" 秒")
        self.record_duration_spin.setSpecialValueText("手动停止")
        sidebar_layout.addWidget(self.record_duration_spin)

        self.btn_record = QPushButton("开始录制")
        self.btn_record.setEnabled(True)
        self.btn_record.setToolTip(
            "录制勾选的当前画面源；通过 go2rtc HTTP API 复制 H.264，停止后封装成 MKV 或 MP4，"
            "不依赖 RTSP，也不受预览选择 WebRTC 或 MSE 影响。")
        self.btn_record.clicked.connect(self.toggle_recording)
        self.btn_record.setStyleSheet(
            "background-color: #455a64; color: white; padding: 7px;")
        sidebar_layout.addWidget(self.btn_record)

        self.record_status_label = QLabel("未录制")
        self.record_status_label.setWordWrap(True)
        self.record_status_label.setStyleSheet(
            "font-size: 11px; color: #9aa7ad; padding: 2px;")
        sidebar_layout.addWidget(self.record_status_label)

        self.btn_pin = QPushButton("始终置顶")
        self.btn_pin.setCheckable(True)
        self.btn_pin.clicked.connect(self.on_pin_clicked)
        self.btn_pin.setStyleSheet("background-color: #455a64; color: white;")
        sidebar_layout.addWidget(self.btn_pin)

        sidebar_layout.addStretch()
        self.splitter.addWidget(self.sidebar_widget)

        display_widget = QWidget()
        display_layout = QVBoxLayout(display_widget)
        display_layout.setContentsMargins(0, 0, 0, 0)
        display_layout.setSpacing(6)

        for slot in range(2):
            pane = QWidget()
            pane_layout = QVBoxLayout(pane)
            pane_layout.setContentsMargins(0, 0, 0, 0)

            title = QLabel(f"画面 {slot + 1}")
            title.setStyleSheet(
                "font-size: 14px; font-weight: bold; color: #00e5ff; "
                "padding: 3px;")
            pane_layout.addWidget(title)

            image_label = ClickableLabel("等待连接图像流...")
            image_label.setAlignment(Qt.AlignCenter)
            image_label.setStyleSheet(
                "background-color: #1a1a1a; border: 1px solid #333333; "
                "border-radius: 8px;")
            image_label.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Ignored)
            image_label.setContextMenuPolicy(Qt.CustomContextMenu)
            image_label.customContextMenuRequested.connect(
                lambda pos, s=slot: self.show_context_menu(s, pos))
            image_label.double_clicked.connect(self.toggle_fullscreen)
            video_stack = QStackedWidget()
            video_stack.addWidget(image_label)
            web_view = None
            if QWebEngineView is not None:
                web_view = QWebEngineView()
                if QWebEngineScript is not None:
                    compatibility = QWebEngineScript()
                    compatibility.setName("go2rtc-chromium69-webrtc-state")
                    compatibility.setInjectionPoint(
                        QWebEngineScript.DocumentCreation)
                    compatibility.setWorldId(QWebEngineScript.MainWorld)
                    compatibility.setRunsOnSubFrames(False)
                    compatibility.setSourceCode(GO2RTC_WEBRTC_COMPAT_JS)
                    web_view.page().scripts().insert(compatibility)
                web_view.setStyleSheet("background-color: #000000;")
                if QWebEngineSettings is not None:
                    settings = web_view.settings()
                    autoplay = getattr(
                        QWebEngineSettings, "PlaybackRequiresUserGesture", None)
                    if autoplay is not None:
                        settings.setAttribute(autoplay, False)
                web_view.loadFinished.connect(
                    lambda ok, s=slot: self._on_web_view_loaded(s, ok))
                video_stack.addWidget(web_view)
            pane_layout.addWidget(video_stack, 1)

            status = QLabel("未连接")
            status.setStyleSheet(
                "font-size: 11px; color: #888888; padding: 3px;")
            status.setWordWrap(True)
            pane_layout.addWidget(status)

            self._video_panes.append(pane)
            self._video_stacks.append(video_stack)
            self._video_labels.append(image_label)
            self._video_webviews.append(web_view)
            self._video_status.append(status)
            display_layout.addWidget(pane, 1)

        self.splitter.addWidget(display_widget)
        self.splitter.setSizes([300, 820])

    @staticmethod
    def _workspace_root():
        """Find the workspace containing install/ and the recording script."""
        probes = [Path.cwd(), Path(__file__).resolve().parent]
        for probe in probes:
            current = probe if probe.is_dir() else probe.parent
            for parent in (current, *current.parents):
                if ((parent / "install").is_dir()
                        and (parent / "scripts" / "record_go2rtc.sh").is_file()):
                    return parent
        # Keep the fallback deterministic for source-tree launches outside
        # the ROS workspace.  The directory is created when recording starts.
        return Path.cwd()

    def _update_record_stream_label(self, slot):
        if slot >= len(getattr(self, "record_stream_checks", [])):
            return
        source = self._source_selects[slot].currentData()
        if source:
            description = f"{source['camera']} / {source['mode_label']}"
        else:
            description = "未选择来源"
        self.record_stream_checks[slot].setText(
            f"录制画面 {slot + 1}：{description}")

    def _recording_source(self):
        """Return checked GUI sources and the applied go2rtc HTTP endpoint."""
        if self._endpoint_dirty:
            self.record_status_label.setText(
                "预览地址已修改；先点击“连接/应用地址”再开始录制")
            return None
        if self._active_endpoint is None:
            self.record_status_label.setText("先连接 go2rtc 地址，再开始录制")
            return None

        streams = []
        for slot, check in enumerate(self.record_stream_checks):
            if not check.isChecked():
                continue
            source = self._source_selects[slot].currentData()
            if source and source["path"] not in streams:
                streams.append(source["path"])

        return {
            "endpoint": self._active_endpoint,
            "streams": streams,
        }

    def toggle_recording(self):
        if self._record_process is not None and self._record_process.poll() is None:
            self.stop_recording()
        else:
            self.start_recording()

    def start_recording(self):
        if self._record_process is not None:
            if self._record_process.poll() is None:
                return
            self._record_process = None
            self._close_record_log()

        source = self._recording_source()
        if source is None:
            return
        streams = source["streams"]
        if not streams:
            self.record_status_label.setText("请至少勾选一个画面源")
            return

        root = self._workspace_root()
        script = root / "scripts" / "record_go2rtc.sh"
        if not script.is_file():
            self.record_status_label.setText(f"找不到录制脚本：{script}")
            return

        output_root = root / "video_record"
        record_timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        output_dir = output_root / record_timestamp
        suffix = 1
        while output_dir.exists():
            output_dir = output_root / f"{record_timestamp}_{suffix:02d}"
            suffix += 1
        try:
            output_root.mkdir(parents=True, exist_ok=True)
            output_dir.mkdir(parents=True, exist_ok=False)
        except OSError as exc:
            self.record_status_label.setText(
                f"无法创建录制目录：{output_dir}\n{exc}")
            return

        endpoint = source["endpoint"]
        record_format = self.record_format_combo.currentData()
        duration = self.record_duration_spin.value()
        self._record_log_path = output_dir / "recorder.log"
        environment = os.environ.copy()
        environment.update({
            "GORTC_HOST": endpoint["host"],
            "GORTC_PORT": str(endpoint["port"]),
            "GORTC_STREAMS": " ".join(streams),
            "OUT_DIR": str(output_root),
            "RECORD_DIR": str(output_dir),
            "RECORD_TIMESTAMP": output_dir.name,
            "RECORD_FORMAT": str(record_format),
        })

        try:
            self._record_log_handle = open(
                self._record_log_path, "w", encoding="utf-8", buffering=1)
            self._record_process = subprocess.Popen(
                ["bash", str(script), str(duration)],
                cwd=str(root),
                env=environment,
                stdin=subprocess.DEVNULL,
                stdout=self._record_log_handle,
                stderr=subprocess.STDOUT,
            )
        except OSError as exc:
            self._record_process = None
            self._close_record_log()
            self.record_status_label.setText(f"启动录制失败：{exc}")
            return

        self._record_output_dir = output_dir
        self._record_streams = streams
        self._record_format = record_format
        self._record_started_at = time.monotonic()
        self._record_stop_requested = False
        self.btn_record.setEnabled(True)
        self.btn_record.setText("停止录制")
        self.btn_record.setStyleSheet(
            "background-color: #b71c1c; color: white; padding: 7px;")
        self.record_status_label.setStyleSheet(
            "font-size: 11px; color: #ffcc80; padding: 2px;")
        self.record_status_label.setText(
            f"正在检查并录制 {len(streams)} 路 {record_format.upper()}\n"
            f"HTTP {endpoint['authority']}\n"
            f"来源：{' / '.join(streams)}\n"
            f"输出 {output_dir}")

    def stop_recording(self):
        process = self._record_process
        if process is None:
            return
        if process.poll() is None:
            try:
                process.send_signal(signal.SIGINT)
            except OSError:
                try:
                    process.terminate()
                except OSError:
                    pass
            self._record_stop_requested = True
            self.btn_record.setEnabled(False)
            self.record_status_label.setStyleSheet(
                "font-size: 11px; color: #ffcc80; padding: 2px;")
            self.record_status_label.setText(
                f"正在停止并验证录像文件…\n目录：{self._record_output_dir}")
            return
        self._poll_recording()

    def _poll_recording(self):
        process = self._record_process
        if process is None:
            return
        if process.poll() is None:
            if self._record_stop_requested:
                self.record_status_label.setText(
                    f"正在停止并验证录像文件…\n目录：{self._record_output_dir}")
                return
            elapsed = int(time.monotonic() - (
                self._record_started_at or time.monotonic()))
            log_lines = []
            if self._record_log_path is not None:
                try:
                    log_lines = self._record_log_path.read_text(
                        encoding="utf-8", errors="replace").splitlines()
                except OSError:
                    pass
            ready = sum(line.startswith("已收到画面：") for line in log_lines)
            phase = ("录制中" if ready >= len(self._record_streams)
                     else f"等待视频流 ({ready}/{len(self._record_streams)})")
            errors = [line for line in log_lines
                      if line.startswith("错误：") or line.startswith("不可用：")]
            detail = f"\n{errors[-1]}" if errors else ""
            self.record_status_label.setText(
                f"{phase}：{len(self._record_streams)} 路 "
                f"{self._record_format.upper()} ({elapsed}s)\n"
                f"来源：{' / '.join(self._record_streams)}\n"
                f"输出 {self._record_output_dir}{detail}")
            return

        return_code = process.returncode
        self._record_process = None
        self._record_started_at = None
        self._record_stop_requested = False
        self._close_record_log()
        self.btn_record.setEnabled(True)
        self.btn_record.setText("开始录制")
        self.btn_record.setStyleSheet(
            "background-color: #455a64; color: white; padding: 7px;")
        if return_code == 0:
            self.record_status_label.setStyleSheet(
                "font-size: 11px; color: #81c784; padding: 2px;")
            self.record_status_label.setText(
                f"录制已完成\n文件目录：{self._record_output_dir}")
        else:
            details = self._record_log_tail()
            self.record_status_label.setStyleSheet(
                "font-size: 11px; color: #ff8a80; padding: 2px;")
            self.record_status_label.setText(
                f"录制进程已退出（代码 {return_code}）\n{details}\n"
                f"日志：{self._record_log_path}")

    def _close_record_log(self):
        handle = self._record_log_handle
        self._record_log_handle = None
        if handle is not None:
            try:
                handle.flush()
                handle.close()
            except OSError:
                pass

    def _record_log_tail(self, limit=16):
        if self._record_log_path is None:
            return ""
        try:
            lines = self._record_log_path.read_text(
                encoding="utf-8", errors="replace").splitlines()
        except OSError:
            return "无法读取录制日志"
        return "\n".join(lines[-limit:])

    @staticmethod
    def _stream_item_text(stream):
        return (
            f"{stream['host_label']}:{stream['port']}  /  "
            f"{stream['camera']}  /  {stream['mode_label']}  /  "
            f"{stream.get('transport_label', '视频')}")

    @staticmethod
    def _parse_endpoint(value):
        """Parse a go2rtc host:port address; HTTP is used by the recorder too."""
        from urllib.parse import urlsplit

        value = value.strip()
        if not value:
            raise ValueError("请输入 go2rtc 地址，例如 192.168.16.10:1984")
        if "://" not in value:
            value = "http://" + value

        try:
            parsed = urlsplit(value)
            port = parsed.port
        except ValueError as exc:
            raise ValueError("地址格式错误，请使用 IP:端口，例如 192.168.16.10:1984") from exc

        if parsed.scheme.lower() != "http":
            raise ValueError("当前只支持 HTTP go2rtc 地址")
        if not parsed.hostname or port is None or not 1 <= port <= 65535:
            raise ValueError("地址需要包含有效端口，例如 192.168.16.10:1984")
        if (parsed.username or parsed.password or parsed.path not in ("", "/")
                or parsed.query or parsed.fragment):
            raise ValueError("这里只填写主机和端口，不要添加路径或账号信息")

        host = parsed.hostname
        authority = parsed.netloc
        recorder_host = f"[{host}]" if ":" in host else host
        return {
            "host": host,
            "recorder_host": recorder_host,
            "port": port,
            "authority": authority,
        }

    def _on_endpoint_edited(self, text):
        self._endpoint_dirty = True
        self.address_status_label.setText("地址已修改；点击“连接/应用地址”后生效")
        self.address_status_label.setStyleSheet(
            "font-size: 11px; color: #ffcc80; padding: 5px;")

    def _stream_for_slot(self, slot, endpoint):
        source = self._source_selects[slot].currentData()
        if not source:
            return None
        path = source["path"]
        authority = endpoint["authority"]
        transport = self._transport_selects[slot].currentData()
        base_url = f"http://{authority}"
        if transport == "auto":
            url = f"{base_url}/stream.html?src={path}"
            transport_label = "go2rtc 自动选择"
        elif transport == "mse":
            url = f"{base_url}/stream.html?src={path}&mode=mse"
            transport_label = "MSE / H.264"
        else:
            url = f"{base_url}/stream.html?src={path}&mode=webrtc"
            transport_label = "WebRTC"
        return {
            **source,
            "host_label": endpoint["host"],
            "host": endpoint["host"],
            "port": endpoint["port"],
            "transport": transport,
            "transport_label": transport_label,
            "id": f"{authority}/{path}?mode={transport}",
            "url": url,
        }

    def connect_selected_streams(self):
        try:
            endpoint = self._parse_endpoint(self.endpoint_edit.text())
        except ValueError as exc:
            self.address_status_label.setText(str(exc))
            self.address_status_label.setStyleSheet(
                "font-size: 11px; color: #ff8a80; padding: 5px;")
            return

        self._active_endpoint = endpoint
        self._endpoint_dirty = False
        self.address_status_label.setText(
            f"已应用地址 {endpoint['authority']}；正在连接两个画面")
        self.address_status_label.setStyleSheet(
            "font-size: 11px; color: #9aa7ad; padding: 5px;")
        for slot in range(2):
            stream = self._stream_for_slot(slot, endpoint)
            if stream is not None:
                self.start_stream(stream, slot)

    def _on_source_selected(self, slot, index):
        if index < 0 or slot >= len(self._source_selects):
            return
        self._update_record_stream_label(slot)
        if self._endpoint_dirty:
            self.address_status_label.setText(
                "地址已修改；点击“连接/应用地址”后，新选择才会连接")
            return
        if self._active_endpoint is None:
            return
        stream = self._stream_for_slot(slot, self._active_endpoint)
        if stream:
            self.start_stream(stream, slot)

    def start_stream(self, stream, slot=0):
        if slot not in (0, 1):
            return
        current = self._current_streams[slot]
        if (current is not None and current.get("id") == stream.get("id")
                and self._stream_threads[slot] is not None):
            return

        self._stop_stream(slot)
        self._current_streams[slot] = stream
        self._frame_counts[slot] = 0
        self._last_pixmaps[slot] = QPixmap()
        self._video_labels[slot].setPixmap(QPixmap())
        self._video_labels[slot].setText(f"正在连接 {stream['url']}...")
        self._video_status[slot].setText(
            f"正在连接 {self._stream_item_text(stream)}...")

        if stream.get("transport") in ("webrtc", "mse", "auto"):
            web_view = self._video_webviews[slot]
            if web_view is None:
                self._video_stacks[slot].setCurrentWidget(self._video_labels[slot])
                self._video_labels[slot].setText(
                    "此环境缺少 QtWebEngine，无法播放网页视频")
                self._video_status[slot].setText(
                    "安装 python3-pyqt5.qtwebengine 后重启 GUI")
                return
            self._video_stacks[slot].setCurrentWidget(web_view)
            web_view.setUrl(QUrl(stream["url"]))
            return

        self._video_stacks[slot].setCurrentWidget(self._video_labels[slot])
        reader = MjpegStreamThread(stream["url"], self)
        self._stream_threads[slot] = reader
        reader.frame_received.connect(
            lambda data, s=slot: self.update_image(s, data))
        reader.stream_error.connect(
            lambda error, s=slot: self._on_stream_error(s, error))
        reader.finished.connect(
            lambda r=reader, s=slot: self._on_stream_finished(s, r))
        reader.start()

    def _stop_stream(self, slot=None):
        slots = range(2) if slot is None else (slot,)
        for current_slot in slots:
            reader = self._stream_threads[current_slot]
            self._stream_threads[current_slot] = None
            if reader is not None:
                reader.stop()
            if current_slot < len(self._video_webviews):
                web_view = self._video_webviews[current_slot]
                if web_view is not None:
                    web_view.stop()
                    web_view.setUrl(QUrl("about:blank"))

    def _on_web_view_loaded(self, slot, ok):
        stream = self._current_streams[slot]
        if stream is None or stream.get("transport") not in ("webrtc", "mse", "auto"):
            return
        if ok:
            self._video_status[slot].setText(
                f"{self._stream_item_text(stream)} | 播放页面已载入")
        else:
            self._video_status[slot].setText(
                f"视频页面载入失败：{stream['url']}")

    def _on_stream_error(self, slot, error):
        stream = self._current_streams[slot]
        if stream is None:
            return
        self._video_status[slot].setText(
            f"图像流连接失败：{stream['url']} | {error}")
        self._video_labels[slot].setPixmap(QPixmap())
        self._video_labels[slot].setText("图像流连接失败；检查地址后点击连接重试")

    def _on_stream_finished(self, slot, reader):
        if self._stream_threads[slot] is not reader:
            return
        self._stream_threads[slot] = None
        if self._current_streams[slot] is not None and self._frame_counts[slot]:
            self._video_status[slot].setText("图像流已断开；点击连接/应用地址重试")

    def update_image(self, slot, jpeg_data):
        image = QImage()
        if not image.loadFromData(jpeg_data, "JPEG"):
            return

        self._last_pixmaps[slot] = QPixmap.fromImage(image)
        self._frame_counts[slot] += 1
        self._display_pixmap(slot)
        stream_label = self._stream_item_text(self._current_streams[slot])
        self._video_status[slot].setText(
            f"{stream_label} | {image.width()}x{image.height()} | "
            f"JPEG 帧 {self._frame_counts[slot]}")

    def _display_pixmap(self, slot=None):
        slots = range(2) if slot is None else (slot,)
        for current_slot in slots:
            pixmap = self._last_pixmaps[current_slot]
            if pixmap.isNull():
                continue
            self._video_labels[current_slot].setPixmap(pixmap.scaled(
                self._video_labels[current_slot].size(),
                self.aspect_ratio_mode,
                Qt.SmoothTransformation,
            ))

    def toggle_fullscreen(self):
        is_visible = self.sidebar_widget.isVisible()
        self.sidebar_widget.setVisible(not is_visible)
        self.address_status_label.setVisible(not is_visible)

        if is_visible:
            self.main_window().setWindowTitle("图像监控（双击画面恢复）")
            for label in self._video_labels:
                label.setStyleSheet(
                    "background-color: #000000; border: none; border-radius: 0px;")
        else:
            self.main_window().setWindowTitle("图像监控")
            for label in self._video_labels:
                label.setStyleSheet(
                    "background-color: #1a1a1a; border: 1px solid #333333; "
                    "border-radius: 8px;")
        self._display_pixmap()

    def main_window(self):
        target = self
        while target.parent():
            target = target.parent()
        return target

    def on_pin_clicked(self, checked):
        self.pin_toggled_signal.emit(checked)
        if checked:
            self.btn_pin.setText("已置顶")
            self.btn_pin.setStyleSheet("background-color: #00acc1; color: white;")
        else:
            self.btn_pin.setText("始终置顶")
            self.btn_pin.setStyleSheet("background-color: #455a64; color: white;")

    def show_context_menu(self, slot, pos):
        menu = QMenu(self)

        pin_action = QAction("始终置顶", self)
        pin_action.setCheckable(True)
        is_pinned = bool(self.main_window().windowFlags() & Qt.WindowStaysOnTopHint)
        pin_action.setChecked(is_pinned)
        pin_action.triggered.connect(lambda: self.btn_pin.click())
        menu.addAction(pin_action)

        fs_action = QAction("切换双画面全屏显示", self)
        fs_action.triggered.connect(self.toggle_fullscreen)
        menu.addAction(fs_action)

        menu.addSeparator()

        keep_action = QAction("保持宽高比 (黑边填充)", self)
        keep_action.setCheckable(True)
        keep_action.setChecked(self.aspect_ratio_mode == Qt.KeepAspectRatio)
        keep_action.triggered.connect(
            lambda: self.change_aspect_ratio(Qt.KeepAspectRatio))
        menu.addAction(keep_action)

        stretch_action = QAction("拉伸填充画面", self)
        stretch_action.setCheckable(True)
        stretch_action.setChecked(self.aspect_ratio_mode == Qt.IgnoreAspectRatio)
        stretch_action.triggered.connect(
            lambda: self.change_aspect_ratio(Qt.IgnoreAspectRatio))
        menu.addAction(stretch_action)

        crop_action = QAction("裁剪填充画面 (无拉伸变形)", self)
        crop_action.setCheckable(True)
        crop_action.setChecked(
            self.aspect_ratio_mode == Qt.KeepAspectRatioByExpanding)
        crop_action.triggered.connect(
            lambda: self.change_aspect_ratio(Qt.KeepAspectRatioByExpanding))
        menu.addAction(crop_action)

        exec_menu = getattr(menu, "exec_", None) or menu.exec
        exec_menu(self._video_labels[slot].mapToGlobal(pos))

    def change_aspect_ratio(self, mode):
        self.aspect_ratio_mode = mode
        self._display_pixmap()

    def close(self):
        self.record_timer.stop()
        self.stop_recording()
        self._stop_stream()


class ImageViewerApp(QMainWindow):
    """Standalone network image viewer window."""

    def __init__(self, node, spinner, initial_url=None, start_fullscreen=False):
        super().__init__()
        self.node = node
        self.spinner = spinner
        self.setWindowTitle("网络图像查看器")
        self.resize(1120, 720)
        self.setStyleSheet("QMainWindow { background-color: #121212; }")

        self.widget = ImageViewerWidget(self.node, auto_connect=not bool(initial_url))
        self.setCentralWidget(self.widget)

        self.floating_hbt = FloatingHeartbeatPanel(self.node, self)
        self.floating_hbt.show()
        self.widget.pin_toggled_signal.connect(self.on_pin_toggled)

        if initial_url:
            self.widget.start_stream({
                "id": f"custom:{initial_url}",
                "host_label": "自定义",
                "host": "",
                "port": "",
                "camera": "网络",
                "mode": "stream",
                "mode_label": "自定义 MJPEG",
                "transport": "custom",
                "transport_label": "自定义",
                "path": initial_url,
                "url": initial_url,
            }, slot=0)

        if start_fullscreen:
            QTimer.singleShot(500, self.widget.toggle_fullscreen)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self.floating_hbt.setGeometry(self.width() - 280, 15, 260, 42)
        self.floating_hbt.raise_()

    def on_pin_toggled(self, checked):
        geom = self.geometry()
        flags = self.windowFlags()
        if checked:
            self.setWindowFlags(flags | Qt.WindowStaysOnTopHint)
        else:
            self.setWindowFlags(flags & ~Qt.WindowStaysOnTopHint)
        self.show()
        self.setGeometry(geom)
        self.floating_hbt.raise_()

    def closeEvent(self, event):
        self.widget.close()
        self.floating_hbt.close()
        super().closeEvent(event)


def main(args=None):
    parser = argparse.ArgumentParser(description="网络视频图像查看 GUI")
    parser.add_argument(
        "--url", type=str, default=None,
        help="可选：直接连接兼容的 MJPEG URL；默认连接 go2rtc H.264 流")
    parser.add_argument("--fullscreen", action="store_true", help="启动时全屏")

    parsed_args, unknown = parser.parse_known_args(args=args)

    rclpy.init(args=args)
    node = Node("image_viewer_gui_node")

    spinner = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spinner.start()

    app = QApplication(sys.argv)
    app.setStyle("Fusion")

    window = ImageViewerApp(
        node=node,
        spinner=spinner,
        initial_url=parsed_args.url,
        start_fullscreen=parsed_args.fullscreen,
    )
    window.show()

    exit_code = app.exec_()

    node.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
