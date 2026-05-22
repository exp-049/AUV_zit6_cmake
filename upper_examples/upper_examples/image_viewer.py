#!/usr/bin/env encoding
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import argparse
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2

# 导入共享的浮动心跳面板
from .heartbeat import FloatingHeartbeatPanel

os.environ.pop("QT_QPA_PLATFORM_PLUGIN_PATH", None)

try:
    from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, 
                                 QListWidget, QPushButton, QLabel, QMessageBox, QSplitter, QComboBox, 
                                 QSizePolicy, QMenu, QAction)
    from PyQt5.QtCore import Qt, pyqtSignal, QTimer
    from PyQt5.QtGui import QImage, QPixmap
except ImportError:
    from PySide6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, 
                                   QListWidget, QPushButton, QLabel, QMessageBox, QSplitter, QComboBox, 
                                   QSizePolicy, QMenu)
    from PySide6.QtCore import Qt, Signal as pyqtSignal, QTimer
    from PySide6.QtGui import QImage, QPixmap, QAction

class ClickableLabel(QLabel):
    double_clicked = pyqtSignal()
    
    def mouseDoubleClickEvent(self, event):
        self.double_clicked.emit()
        super().mouseDoubleClickEvent(event)

class ImageViewerWidget(QWidget):
    """
    图像接收与展示组件，支持在主控制台中嵌入或在独立窗口中显示
    """
    image_received_signal = pyqtSignal(object)
    pin_toggled_signal = pyqtSignal(bool)

    def __init__(self, node):
        super().__init__()
        self.node = node
        self.bridge = CvBridge()
        self.subscription = None
        self.current_topic = None
        self.current_type = None
        self.topic_types_map = {}
        
        self.aspect_ratio_mode = Qt.KeepAspectRatio
        
        self.init_ui()
        self.image_received_signal.connect(self.update_image)
        
        self.scan_timer = QTimer()
        self.scan_timer.timeout.connect(self.scan_topics)
        self.scan_timer.start(2000)
        self.scan_topics()

    def init_ui(self):
        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        
        self.splitter = QSplitter(Qt.Horizontal)
        layout.addWidget(self.splitter)
        
        self.sidebar_widget = QWidget()
        sidebar_layout = QVBoxLayout(self.sidebar_widget)
        sidebar_layout.setContentsMargins(5, 5, 5, 5)
        
        title_label = QLabel("图像话题列表")
        title_label.setStyleSheet("font-size: 15px; font-weight: bold; color: #00e5ff; margin-bottom: 5px;")
        sidebar_layout.addWidget(title_label)
        
        self.topic_list = QListWidget()
        self.topic_list.setStyleSheet("""
            QListWidget {
                background-color: #1e1e1e;
                color: #e0e0e0;
                border: 1px solid #333333;
                border-radius: 8px;
            }
            QListWidget::item {
                background-color: #1e1e1e;
                color: #e0e0e0;
                padding: 8px;
                border-bottom: 1px solid #2a2a2a;
            }
            QListWidget::item:hover {
                background-color: #2c2c2c;
                color: #ffffff;
            }
            QListWidget::item:selected {
                background-color: #005662;
                color: #00e5ff;
                font-weight: bold;
            }
        """)
        self.topic_list.itemClicked.connect(self.on_topic_selected)
        sidebar_layout.addWidget(self.topic_list)
        
        qos_layout = QHBoxLayout()
        qos_label = QLabel("QoS 配置:")
        qos_label.setStyleSheet("font-size: 12px;")
        self.qos_combo = QComboBox()
        self.qos_combo.addItems(["Sensor Data (Best Effort)", "Reliable"])
        self.qos_combo.currentIndexChanged.connect(self.on_qos_changed)
        qos_layout.addWidget(qos_label)
        qos_layout.addWidget(self.qos_combo)
        sidebar_layout.addLayout(qos_layout)
        
        btn_layout = QHBoxLayout()
        self.btn_refresh = QPushButton("刷新话题")
        self.btn_refresh.clicked.connect(self.scan_topics)
        
        self.btn_pin = QPushButton("始终置顶")
        self.btn_pin.setCheckable(True)
        self.btn_pin.clicked.connect(self.on_pin_clicked)
        self.btn_pin.setStyleSheet("background-color: #455a64; color: white;")
        
        btn_layout.addWidget(self.btn_refresh)
        btn_layout.addWidget(self.btn_pin)
        sidebar_layout.addLayout(btn_layout)
        
        self.splitter.addWidget(self.sidebar_widget)
        
        display_widget = QWidget()
        display_layout = QVBoxLayout(display_widget)
        display_layout.setContentsMargins(0, 0, 0, 0)
        
        self.image_label = ClickableLabel("请在左侧选择一个活跃 of 图像话题开始查看...")
        self.image_label.setAlignment(Qt.AlignCenter)
        self.image_label.setStyleSheet("background-color: #1a1a1a; border: 1px solid #333333; border-radius: 8px;")
        self.image_label.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Ignored)
        
        self.image_label.setContextMenuPolicy(Qt.CustomContextMenu)
        self.image_label.customContextMenuRequested.connect(self.show_context_menu)
        self.image_label.double_clicked.connect(self.toggle_fullscreen)
        
        display_layout.addWidget(self.image_label)
        
        self.status_label = QLabel("当前无活跃的数据流")
        self.status_label.setStyleSheet("font-size: 12px; color: #888888; padding: 5px;")
        display_layout.addWidget(self.status_label)
        
        self.splitter.addWidget(display_widget)
        self.splitter.setSizes([260, 840])

    def scan_topics(self):
        try:
            topic_names_and_types = self.node.get_topic_names_and_types()
        except Exception as e:
            return
            
        image_topics = []
        for name, types in topic_names_and_types:
            for t in types:
                if t in ['sensor_msgs/msg/Image', 'sensor_msgs/msg/CompressedImage']:
                    image_topics.append((name, t))
                    break
        
        current_topics = [self.topic_list.item(i).text() for i in range(self.topic_list.count())]
        new_topics = [t[0] for t in image_topics]
        
        if set(current_topics) != set(new_topics):
            selected = self.topic_list.currentItem()
            selected_text = selected.text() if selected else None
            
            self.topic_list.clear()
            self.topic_types_map.clear()
            
            for name, t in image_topics:
                self.topic_list.addItem(name)
                self.topic_types_map[name] = t
                
            if selected_text:
                items = self.topic_list.findItems(selected_text, Qt.MatchExactly)
                if items:
                    self.topic_list.setCurrentItem(items[0])
                else:
                    self.reset_stream()

    def reset_stream(self):
        if self.subscription:
            self.node.destroy_subscription(self.subscription)
            self.subscription = None
        self.current_topic = None
        self.current_type = None
        self.image_label.setPixmap(QPixmap())
        self.image_label.setText("请在左侧选择一个活跃的图像话题开始查看...")
        self.status_label.setText("当前无活跃的数据流")

    def on_topic_selected(self, item):
        topic_name = item.text()
        topic_type = self.topic_types_map.get(topic_name)
        if topic_name != self.current_topic or topic_type != self.current_type:
            self.subscribe_to_topic(topic_name, topic_type)

    def on_qos_changed(self, index):
        if self.current_topic and self.current_type:
            self.subscribe_to_topic(self.current_topic, self.current_type)

    def get_qos_profile(self):
        if self.qos_combo.currentIndex() == 1:
            return QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        else:
            return qos_profile_sensor_data

    def subscribe_to_topic(self, topic_name, topic_type):
        if self.subscription:
            self.node.destroy_subscription(self.subscription)
            self.subscription = None
            
        self.current_topic = topic_name
        self.current_type = topic_type
        qos = self.get_qos_profile()
        self.status_label.setText(f"正在连接到 {topic_name}...")
        
        try:
            if topic_type == 'sensor_msgs/msg/Image':
                self.subscription = self.node.create_subscription(
                    Image,
                    topic_name,
                    self.image_callback,
                    qos
                )
            elif topic_type == 'sensor_msgs/msg/CompressedImage':
                self.subscription = self.node.create_subscription(
                    CompressedImage,
                    topic_name,
                    self.compressed_image_callback,
                    qos
                )
            self.status_label.setText(f"成功订阅 {topic_name}，正在等待图像帧...")
            
            items = self.topic_list.findItems(topic_name, Qt.MatchExactly)
            if items:
                self.topic_list.setCurrentItem(items[0])
        except Exception as e:
            QMessageBox.critical(self, "订阅失败", f"无法订阅话题 {topic_name}:\n{e}")
            self.reset_stream()

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.image_received_signal.emit(cv_image)
        except Exception:
            pass

    def compressed_image_callback(self, msg):
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.image_received_signal.emit(cv_image)
        except Exception:
            pass

    def update_image(self, cv_image):
        if cv_image is None or self.current_topic is None:
            return
            
        rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        height, width, channel = rgb_image.shape
        bytes_per_line = channel * width
        
        q_img = QImage(rgb_image.data, width, height, bytes_per_line, QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(q_img)
        
        scaled_pixmap = pixmap.scaled(
            self.image_label.size(), 
            self.aspect_ratio_mode, 
            Qt.SmoothTransformation
        )
        self.image_label.setPixmap(scaled_pixmap)
        
        qos_text = "Reliable" if self.qos_combo.currentIndex() == 1 else "Sensor Data"
        self.status_label.setText(f"话题: {self.current_topic} | 分辨率: {width}x{height} | QoS: {qos_text}")

    def toggle_fullscreen(self):
        is_visible = self.sidebar_widget.isVisible()
        self.sidebar_widget.setVisible(not is_visible)
        self.status_label.setVisible(not is_visible)
        
        if is_visible:
            self.main_window().setWindowTitle("图像监控 (双击画面恢复)")
            self.image_label.setStyleSheet("background-color: #000000; border: none; border-radius: 0px;")
        else:
            self.main_window().setWindowTitle("图像监控")
            self.image_label.setStyleSheet("background-color: #1a1a1a; border: 1px solid #333333; border-radius: 8px;")

    def main_window(self):
        target = self
        while target.parent():
            target = target.parent()
        return target

    def on_pin_clicked(self, checked):
        self.pin_toggled_signal.emit(checked)
        if checked:
            self.btn_pin.setText("已置顶 📌")
            self.btn_pin.setStyleSheet("background-color: #00acc1; color: white;")
        else:
            self.btn_pin.setText("始终置顶")
            self.btn_pin.setStyleSheet("background-color: #455a64; color: white;")

    def show_context_menu(self, pos):
        menu = QMenu(self)
        
        pin_action = QAction("始终置顶 📌", self)
        pin_action.setCheckable(True)
        is_pinned = bool(self.main_window().windowFlags() & Qt.WindowStaysOnTopHint)
        pin_action.setChecked(is_pinned)
        pin_action.triggered.connect(lambda: self.btn_pin.click())
        menu.addAction(pin_action)
        
        fs_action = QAction("切换全屏显示", self)
        fs_action.triggered.connect(self.toggle_fullscreen)
        menu.addAction(fs_action)
        
        menu.addSeparator()
        
        keep_action = QAction("保持宽高比 (黑边填充)", self)
        keep_action.setCheckable(True)
        keep_action.setChecked(self.aspect_ratio_mode == Qt.KeepAspectRatio)
        keep_action.triggered.connect(lambda: self.change_aspect_ratio(Qt.KeepAspectRatio))
        menu.addAction(keep_action)
        
        stretch_action = QAction("拉伸填充画面", self)
        stretch_action.setCheckable(True)
        stretch_action.setChecked(self.aspect_ratio_mode == Qt.IgnoreAspectRatio)
        stretch_action.triggered.connect(lambda: self.change_aspect_ratio(Qt.IgnoreAspectRatio))
        menu.addAction(stretch_action)
        
        crop_action = QAction("裁剪填充画面 (无拉伸变形)", self)
        crop_action.setCheckable(True)
        crop_action.setChecked(self.aspect_ratio_mode == Qt.KeepAspectRatioByExpanding)
        crop_action.triggered.connect(lambda: self.change_aspect_ratio(Qt.KeepAspectRatioByExpanding))
        menu.addAction(crop_action)
        
        menu.exec_(self.image_label.mapToGlobal(pos))

    def change_aspect_ratio(self, mode):
        self.aspect_ratio_mode = mode

    def close(self):
        self.reset_stream()


class ImageViewerApp(QMainWindow):
    """
    图像查看器独立 GUI 窗口
    """
    def __init__(self, node, spinner, initial_topic=None, initial_qos='sensor_data', start_fullscreen=False):
        super().__init__()
        self.node = node
        self.spinner = spinner
        self.setWindowTitle("ROS 2 图像查看器")
        self.resize(1120, 720)
        self.setStyleSheet("QMainWindow { background-color: #121212; }")
        
        self.widget = ImageViewerWidget(self.node)
        self.setCentralWidget(self.widget)
        
        # 统一的浮动心跳下发面板
        self.floating_hbt = FloatingHeartbeatPanel(self.node, self)
        self.floating_hbt.show()
        
        self.widget.pin_toggled_signal.connect(self.on_pin_toggled)
        
        if initial_qos == 'reliable':
            self.widget.qos_combo.setCurrentIndex(1)
            
        if initial_topic:
            self.widget.subscribe_to_topic(initial_topic, 'sensor_msgs/msg/Image')
            
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
    parser = argparse.ArgumentParser(description="ROS 2 图像接收查看 GUI")
    parser.add_argument("--topic", type=str, default=None, help="默认直接订阅的话题路径")
    parser.add_argument("--qos", type=str, choices=["sensor_data", "reliable"], default="sensor_data", help="默认采用的 QoS 模式")
    parser.add_argument("--fullscreen", action="store_true", help="是否在启动时直接全屏")
    
    parsed_args, unknown = parser.parse_known_args(args=args)

    rclpy.init(args=args)
    node = Node('image_viewer_gui_node')
    
    spinner = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spinner.start()
    
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    
    window = ImageViewerApp(
        node=node,
        spinner=spinner,
        initial_topic=parsed_args.topic,
        initial_qos=parsed_args.qos,
        start_fullscreen=parsed_args.fullscreen
    )
    window.show()
    
    exit_code = app.exec_()
    
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)

if __name__ == '__main__':
    main()
