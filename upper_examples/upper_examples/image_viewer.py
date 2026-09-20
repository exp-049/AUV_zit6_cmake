#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Network MJPEG image monitor.

The image monitor deliberately does not subscribe to ROS image topics. The
vision process already exposes raw and annotated frames as MJPEG streams;
using HTTP here keeps large video frames out of DDS and lets the console
choose between the local machine and the vehicle computer.
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
from concurrent.futures import ThreadPoolExecutor, as_completed
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
        QSizePolicy, QMenu, QAction, QComboBox,
    )
    from PyQt5.QtCore import Qt, pyqtSignal, QThread, QTimer
    from PyQt5.QtGui import QImage, QPixmap
except ImportError:
    from PySide6.QtWidgets import (
        QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
        QListWidget, QListWidgetItem, QPushButton, QLabel, QSplitter,
        QSizePolicy, QMenu, QComboBox,
    )
    from PySide6.QtCore import Qt, Signal as pyqtSignal, QThread, QTimer
    from PySide6.QtGui import QImage, QPixmap, QAction


DEFAULT_MJPEG_PORT = 8090
DEFAULT_GO2RTC_PORT = 1984
NETWORK_HOSTS = (
    ("localhost", "127.0.0.1"),
    ("192.168.16.10", "192.168.16.10"),
)
STREAMS = (
    ("front", "前视", "raw", "原始"),
    ("front_annotated", "前视", "annotated", "标注"),
    ("down", "下视", "raw", "原始"),
    ("down_annotated", "下视", "annotated", "标注"),
)


def _configured_ports():
    """Return ports worth probing, with explicit environment values first.

    uv_camera uses 8090 for its direct MJPEG source and starts go2rtc on
    1984, falling back through the following ports when 1984 is occupied.
    The environment override also lets an operator use a custom preview
    port without changing the GUI.
    """
    ports = []

    def add(value):
        try:
            value = int(value)
        except (TypeError, ValueError):
            return
        if 1 <= value <= 65535 and value not in ports:
            ports.append(value)

    for name in ("ZIT6_IMAGE_PORT", "ZIT6_GO2RTC_PORT"):
        add(os.environ.get(name))
    for value in os.environ.get("ZIT6_IMAGE_PORTS", "").split(","):
        add(value.strip())

    add(DEFAULT_MJPEG_PORT)
    # Match composed.py's automatic go2rtc fallback window.
    for port in range(DEFAULT_GO2RTC_PORT, DEFAULT_GO2RTC_PORT + 101):
        add(port)
    # Cover common manually selected MJPEG preview ports too.
    for port in range(8080, 8101):
        add(port)
    return ports


def _probe_port(host, port, timeout=0.6):
    """Identify a port only when it returns a real MJPEG stream."""
    probes = (
        ("mjpeg", f"http://{host}:{port}/front"),
        ("go2rtc", f"http://{host}:{port}/api/stream.mjpeg?src=front"),
    )
    for transport, url in probes:
        request = Request(
            url,
            headers={
                "Accept": "multipart/x-mixed-replace",
                "Cache-Control": "no-cache",
                "User-Agent": "ZIT6-console-image-monitor/1.0",
            },
        )
        try:
            response = urlopen(request, timeout=timeout)
            try:
                content_type = response.headers.get("Content-Type", "").lower()
                if (response.getcode() == 200
                        and "multipart/x-mixed-replace" in content_type):
                    return transport
            finally:
                response.close()
        except (HTTPError, URLError, OSError, TimeoutError):
            pass
    return None


def _probe_http_mjpeg(url, timeout=1.5):
    """Return whether an HTTP URL exposes an MJPEG response."""
    request = Request(
        url,
        headers={
            "Accept": "multipart/x-mixed-replace",
            "Cache-Control": "no-cache",
            "User-Agent": "ZIT6-console-image-monitor/1.0",
        },
    )
    try:
        response = urlopen(request, timeout=timeout)
        try:
            return (response.getcode() == 200
                    and "multipart/x-mixed-replace" in
                    response.headers.get("Content-Type", "").lower())
        finally:
            response.close()
    except (HTTPError, URLError, OSError, TimeoutError):
        return False


def _stream_definitions(open_ports=None):
    """Return direct-MJPEG and go2rtc-MJPEG endpoint candidates."""
    if open_ports is None:
        open_ports = [
            (host_label, host, port)
            for host_label, host in NETWORK_HOSTS
            for port in _configured_ports()
        ]

    streams = []
    for endpoint in open_ports:
        host_label, host, port = endpoint[:3]
        transport_hint = endpoint[3] if len(endpoint) > 3 else None
        for path, camera, mode, mode_label in STREAMS:
            base = {
                "host_label": host_label,
                "host": host,
                "port": port,
                "path": path,
                "camera": camera,
                "mode": mode,
                "mode_label": mode_label,
            }
            transports = ((transport_hint,) if transport_hint else
                          ("mjpeg", "go2rtc"))
            for transport in transports:
                if transport == "mjpeg":
                    streams.append(dict(
                        base,
                        transport="mjpeg",
                        transport_label="直接 MJPEG",
                        id=f"{host_label}:{port}/{path}",
                        url=f"http://{host}:{port}/{path}",
                    ))
                else:
                    streams.append(dict(
                        base,
                        transport="go2rtc",
                        transport_label="go2rtc MJPEG",
                        id=f"{host_label}:{port}/api/stream.mjpeg?src={path}",
                        url=(f"http://{host}:{port}/api/stream.mjpeg?src={path}"),
                    ))
    return streams


def _probe_stream(stream, timeout=1.5):
    """Check HTTP/MJPEG headers without waiting for a video frame."""
    try:
        available = _probe_http_mjpeg(stream["url"], timeout=timeout)
        return dict(stream, available=available, error="" if available else (
            "不是 MJPEG 流"
        ))
    except HTTPError as exc:
        return dict(stream, available=False, error=f"HTTP {exc.code}")
    except (URLError, OSError, TimeoutError) as exc:
        reason = getattr(exc, "reason", exc)
        return dict(stream, available=False, error=str(reason))
    except Exception as exc:  # Keep one bad endpoint from stopping discovery.
        return dict(stream, available=False, error=str(exc))


class StreamProbeThread(QThread):
    """Probe all local/remote endpoints without blocking the Qt event loop."""

    streams_ready = pyqtSignal(object)

    def __init__(self, streams, parent=None):
        super().__init__(parent)
        self._streams = list(streams or [])
        self._stop_event = threading.Event()

    def stop(self):
        self._stop_event.set()

    def run(self):
        results = []
        port_candidates = [
            (host_label, host, port)
            for host_label, host in NETWORK_HOSTS
            for port in _configured_ports()
        ]
        open_ports = []
        with ThreadPoolExecutor(max_workers=32) as executor:
            futures = {
                executor.submit(_probe_port, host, port): (host_label, host, port)
                for host_label, host, port in port_candidates
            }
            for future in as_completed(futures):
                if self._stop_event.is_set():
                    return
                endpoint = futures[future]
                try:
                    if future.result():
                        open_ports.append(endpoint)
                except Exception:
                    pass

        streams = _stream_definitions(open_ports)
        if not streams:
            self.streams_ready.emit([])
            return

        with ThreadPoolExecutor(max_workers=min(32, len(streams))) as executor:
            futures = {
                executor.submit(_probe_stream, stream): stream
                for stream in streams
            }
            for future in as_completed(futures):
                if self._stop_event.is_set():
                    return
                try:
                    result = future.result()
                    if result.get("available"):
                        results.append(result)
                except Exception:
                    pass
        self.streams_ready.emit(results)


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
    """Show two independently selectable network MJPEG streams."""

    pin_toggled_signal = pyqtSignal(bool)

    def __init__(self, node):
        super().__init__()
        self.node = node  # Kept for the shared heartbeat panel/API.
        self.aspect_ratio_mode = Qt.KeepAspectRatio
        self._probe_thread = None
        self._available_streams = []
        self._source_selects = []
        self._video_panes = []
        self._video_labels = []
        self._video_status = []
        self._stream_threads = [None, None]
        self._current_streams = [None, None]
        self._last_pixmaps = [QPixmap(), QPixmap()]
        self._frame_counts = [0, 0]
        self._record_process = None
        self._record_output_dir = None
        self._record_started_at = None

        self.init_ui()

        self.scan_timer = QTimer(self)
        self.scan_timer.timeout.connect(self.scan_streams)
        self.scan_timer.start(5000)
        self.record_timer = QTimer(self)
        self.record_timer.timeout.connect(self._poll_recording)
        self.record_timer.start(1000)
        QTimer.singleShot(0, self.scan_streams)

    def init_ui(self):
        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        self.splitter = QSplitter(Qt.Horizontal)
        layout.addWidget(self.splitter)

        self.sidebar_widget = QWidget()
        sidebar_layout = QVBoxLayout(self.sidebar_widget)
        sidebar_layout.setContentsMargins(5, 5, 5, 5)

        title_label = QLabel("网络图像源（非 ROS 话题）")
        title_label.setStyleSheet(
            "font-size: 15px; font-weight: bold; color: #00e5ff; "
            "margin-bottom: 5px;")
        sidebar_layout.addWidget(title_label)

        hint_label = QLabel(
            "自动探测 localhost 和 192.168.16.10\n"
            "端口：8090 及 go2rtc 1984 的自动回退端口")
        hint_label.setStyleSheet("font-size: 11px; color: #9aa7ad;")
        hint_label.setWordWrap(True)
        sidebar_layout.addWidget(hint_label)

        for slot in range(2):
            slot_label = QLabel(f"画面 {slot + 1} 来源")
            slot_label.setStyleSheet("color: #00e5ff; font-weight: bold;")
            sidebar_layout.addWidget(slot_label)

            combo = QComboBox()
            combo.setEnabled(False)
            combo.setMinimumContentsLength(18)
            combo.setStyleSheet(
                "QComboBox { background: #1e1e1e; color: #e0e0e0; "
                "padding: 6px; border: 1px solid #444; border-radius: 5px; }"
            )
            combo.currentIndexChanged.connect(
                lambda index, s=slot: self._on_source_selected(s, index))
            self._source_selects.append(combo)
            sidebar_layout.addWidget(combo)

        self.scan_status_label = QLabel("正在探测网络图像源...")
        self.scan_status_label.setStyleSheet(
            "font-size: 11px; color: #888888; padding: 5px;")
        self.scan_status_label.setWordWrap(True)
        sidebar_layout.addWidget(self.scan_status_label)

        self.btn_refresh = QPushButton("刷新网络源")
        self.btn_refresh.clicked.connect(self.scan_streams)
        sidebar_layout.addWidget(self.btn_refresh)

        self.btn_record = QPushButton("开始录制四路")
        # Keep the button clickable even before discovery finishes.  When a
        # required stream is missing, start_recording() explains the reason
        # in the status label instead of making the click appear ignored.
        self.btn_record.setEnabled(True)
        self.btn_record.setToolTip(
            "按当前发现的可用 MJPEG 路数录制；不存在的路会自动跳过")
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
        # Stack the two camera panes vertically.  This preserves the
        # left-hand source controls while giving each 16:9 stream a useful
        # width and enough height to inspect the image.
        display_layout = QVBoxLayout(display_widget)
        display_layout.setContentsMargins(0, 0, 0, 0)
        display_layout.setSpacing(6)

        for slot in range(2):
            pane = QWidget()
            pane_layout = QVBoxLayout(pane)
            pane_layout.setContentsMargins(0, 0, 0, 0)

            title = QLabel(f"画面 {slot + 1}（等待来源）")
            title.setStyleSheet(
                "font-size: 14px; font-weight: bold; color: #00e5ff; "
                "padding: 3px;")
            pane_layout.addWidget(title)

            image_label = ClickableLabel("正在探测图像流...")
            image_label.setAlignment(Qt.AlignCenter)
            image_label.setStyleSheet(
                "background-color: #1a1a1a; border: 1px solid #333333; "
                "border-radius: 8px;")
            image_label.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Ignored)
            image_label.setContextMenuPolicy(Qt.CustomContextMenu)
            image_label.customContextMenuRequested.connect(
                lambda pos, s=slot: self.show_context_menu(s, pos))
            image_label.double_clicked.connect(self.toggle_fullscreen)
            pane_layout.addWidget(image_label, 1)

            status = QLabel("未连接")
            status.setStyleSheet(
                "font-size: 11px; color: #888888; padding: 3px;")
            status.setWordWrap(True)
            pane_layout.addWidget(status)

            self._video_panes.append(pane)
            self._video_labels.append(image_label)
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

    def _recording_source(self):
        """Return one endpoint and all available direct MJPEG streams."""
        groups = {}
        for stream in self._available_streams:
            if stream.get("transport") != "mjpeg":
                continue
            key = (stream.get("host_label"), stream.get("host"),
                   stream.get("port"))
            groups.setdefault(key, set()).add(stream.get("path"))

        candidates = [key for key, paths in groups.items() if paths]
        candidates.sort(key=lambda key: (
            0 if key[0] == "localhost" else 1, key[2]))
        if not candidates:
            return None

        key = candidates[0]
        preferred_order = (
            "front", "front_annotated", "down", "down_annotated")
        paths = groups[key]
        streams = [path for path in preferred_order if path in paths]
        return key[1], key[2], streams

    def toggle_recording(self):
        if self._record_process is not None and self._record_process.poll() is None:
            self.stop_recording()
        else:
            self.start_recording()

    def start_recording(self):
        if self._record_process is not None:
            self._record_process = None

        source = self._recording_source()
        if source is None:
            self.record_status_label.setText(
                "未发现可用的直接 MJPEG 流，暂不开始录制")
            return

        root = self._workspace_root()
        script = root / "scripts" / "record_go2rtc.sh"
        if not script.is_file():
            self.record_status_label.setText(
                f"找不到录制脚本：{script}")
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
        except OSError as exc:
            self.record_status_label.setText(f"无法创建录制目录：{output_root}\n{exc}")
            return

        host, port, streams = source
        environment = os.environ.copy()
        environment.update({
            "GORTC_HOST": str(host),
            "GORTC_PORT": str(port),
            "GORTC_STREAMS": " ".join(streams),
            "OUT_DIR": str(output_root),
            "RECORD_DIR": str(output_dir),
            "RECORD_TIMESTAMP": output_dir.name,
            "VIDEO_FPS": os.environ.get("ZIT6_RECORD_FPS", "10"),
            "SEGMENT_SECONDS": os.environ.get("ZIT6_RECORD_SEGMENT_SECONDS", "2"),
        })

        try:
            self._record_process = subprocess.Popen(
                ["bash", str(script), "0"],
                cwd=str(root),
                env=environment,
                stdin=subprocess.DEVNULL,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.STDOUT,
            )
        except OSError as exc:
            self._record_process = None
            self.record_status_label.setText(f"启动录制失败：{exc}")
            return

        self._record_output_dir = output_dir
        self._record_started_at = time.monotonic()
        self.btn_record.setText("停止录制")
        self.btn_record.setStyleSheet(
            "background-color: #b71c1c; color: white; padding: 7px;")
        self.record_status_label.setStyleSheet(
            "font-size: 11px; color: #ff8a80; padding: 2px;")
        self.record_status_label.setText(
            f"录制中：{len(streams)} 路 TS\n"
            f"来源 {host}:{port}\n"
            f"流：{' / '.join(streams)}\n"
            f"输出 {output_dir}")

    def stop_recording(self):
        process = self._record_process
        if process is None:
            return
        if process.poll() is None:
            try:
                # The script traps SIGINT and forwards it to all four ffmpeg
                # workers, allowing each MPEG-TS segment to close cleanly.
                process.send_signal(signal.SIGINT)
                process.wait(timeout=8.0)
            except (OSError, subprocess.TimeoutExpired):
                try:
                    process.terminate()
                    process.wait(timeout=2.0)
                except (OSError, subprocess.TimeoutExpired):
                    try:
                        process.kill()
                    except OSError:
                        pass

        self._record_process = None
        self._record_started_at = None
        self.btn_record.setText("开始录制四路")
        self.btn_record.setStyleSheet(
            "background-color: #455a64; color: white; padding: 7px;")
        if self._record_output_dir is not None:
            self.record_status_label.setStyleSheet(
                "font-size: 11px; color: #81c784; padding: 2px;")
            self.record_status_label.setText(
                f"录制已停止\n文件目录：{self._record_output_dir}")

    def _poll_recording(self):
        process = self._record_process
        if process is None:
            return
        if process.poll() is None:
            elapsed = int(time.monotonic() - (self._record_started_at or time.monotonic()))
            self.record_status_label.setText(
                f"录制中：4 路 TS ({elapsed}s)\n"
                f"输出 {self._record_output_dir}")
            return

        return_code = process.returncode
        self._record_process = None
        self._record_started_at = None
        self.btn_record.setText("开始录制四路")
        self.btn_record.setStyleSheet(
            "background-color: #455a64; color: white; padding: 7px;")
        self.record_status_label.setStyleSheet(
            "font-size: 11px; color: #81c784; padding: 2px;")
        if return_code == 0:
            self.record_status_label.setText(
                f"录制已完成\n文件目录：{self._record_output_dir}")
        else:
            self.record_status_label.setStyleSheet(
                "font-size: 11px; color: #ff8a80; padding: 2px;")
            self.record_status_label.setText(
                f"录制进程已退出（代码 {return_code}）\n"
                f"目录：{self._record_output_dir}")

    @staticmethod
    def _stream_item_text(stream):
        return (
            f"{stream['host_label']}:{stream['port']}  /  "
            f"{stream['camera']}  /  {stream['mode_label']}  /  "
            f"{stream.get('transport_label', 'MJPEG')}")

    @staticmethod
    def _stream_sort_key(stream):
        return (
            0 if stream["host_label"] == "localhost" else 1,
            0 if stream["camera"] == "前视" else 1,
            0 if stream["mode"] == "raw" else 1,
            0 if stream.get("transport") == "mjpeg" else 1,
            stream["port"],
        )

    @classmethod
    def _default_stream(cls, streams, camera):
        matching = [stream for stream in streams if stream["camera"] == camera]
        return min(matching, key=cls._stream_sort_key) if matching else None

    def scan_streams(self):
        if self._probe_thread is not None and self._probe_thread.isRunning():
            return

        self.btn_refresh.setEnabled(False)
        self.scan_status_label.setText(
            "正在探测 localhost 和 192.168.16.10 的端口和视频流...")
        probe = StreamProbeThread(None, self)
        self._probe_thread = probe
        probe.streams_ready.connect(self._on_probe_results)
        probe.finished.connect(lambda p=probe: self._on_probe_finished(p))
        probe.start()

    def _on_probe_finished(self, probe):
        if self._probe_thread is probe:
            self._probe_thread = None
        self.btn_refresh.setEnabled(True)

    def _on_probe_results(self, results):
        available = [stream for stream in results if stream.get("available")]
        available.sort(key=self._stream_sort_key)
        self._available_streams = available

        current_ids = [
            stream.get("id") if stream is not None else None
            for stream in self._current_streams
        ]
        selections = []
        for slot, combo in enumerate(self._source_selects):
            combo.blockSignals(True)
            combo.clear()
            for stream in available:
                combo.addItem(self._stream_item_text(stream), stream)
            combo.setEnabled(bool(available))

            selected = next(
                (stream for stream in available
                 if stream.get("id") == current_ids[slot]),
                None,
            )
            if selected is None:
                selected = self._default_stream(
                    available, "前视" if slot == 0 else "下视")
            if selected is not None:
                selected_index = next(
                    (index for index, stream in enumerate(available)
                     if stream.get("id") == selected.get("id")),
                    -1,
                )
                combo.setCurrentIndex(selected_index)
            else:
                combo.setCurrentIndex(-1)
            combo.blockSignals(False)
            selections.append(selected)

        if not available:
            self.btn_record.setEnabled(True)
            self.scan_status_label.setText(
                "未发现可用图像流（已扫描 8090 和 go2rtc 回退端口）")
            for slot, label in enumerate(self._video_labels):
                if self._stream_threads[slot] is None:
                    label.setPixmap(QPixmap())
                    label.setText("未发现可用图像流")
                    self._video_status[slot].setText("等待自动重连")
            return

        self.btn_record.setEnabled(True)
        recording_source = self._recording_source()
        if recording_source is None:
            self.scan_status_label.setText(
                f"发现 {len(available)} 路视频流，但没有可录制的直接 MJPEG 流")
        else:
            record_count = len(recording_source[2])
            self.scan_status_label.setText(
                f"发现 {len(available)} 路视频流，可录制 {record_count} 路；"
                "缺失的路会自动跳过")
        for slot, stream in enumerate(selections):
            if stream is None:
                continue
            current = self._current_streams[slot]
            if (current is None or current.get("id") != stream.get("id")
                    or self._stream_threads[slot] is None):
                self.start_stream(stream, slot)

    def _on_source_selected(self, slot, index):
        if index < 0 or slot >= len(self._source_selects):
            return
        stream = self._source_selects[slot].itemData(index)
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

    def _on_stream_error(self, slot, error):
        stream = self._current_streams[slot]
        if stream is None:
            return
        self._video_status[slot].setText(
            f"图像流连接失败：{stream['url']} | {error}")
        self._video_labels[slot].setPixmap(QPixmap())
        self._video_labels[slot].setText("图像流连接失败，稍后将自动重探")

    def _on_stream_finished(self, slot, reader):
        if self._stream_threads[slot] is not reader:
            return
        self._stream_threads[slot] = None
        if self._current_streams[slot] is not None and self._frame_counts[slot]:
            self._video_status[slot].setText("图像流已断开，等待自动重连...")

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
            f"MJPEG 帧 {self._frame_counts[slot]}")

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
        self.scan_status_label.setVisible(not is_visible)

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
        self.scan_timer.stop()
        self.record_timer.stop()
        self.stop_recording()
        if self._probe_thread is not None:
            self._probe_thread.stop()
            self._probe_thread.wait(2500)
            self._probe_thread = None
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

        self.widget = ImageViewerWidget(self.node)
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
                "mode_label": "MJPEG",
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
    parser = argparse.ArgumentParser(description="网络 MJPEG 图像查看 GUI")
    parser.add_argument(
        "--url", type=str, default=None,
        help="可选：直接连接一个 MJPEG URL；不指定时自动探测两个默认地址")
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
