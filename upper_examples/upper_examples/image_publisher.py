#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np
import time

class ImagePublisherNode(Node):
    def __init__(self):
        super().__init__('image_publisher_node')
        self.bridge = CvBridge()
        
        # 创建图像话题发布者：包含原始图像与压缩图像
        self.pub_raw = self.create_publisher(Image, '/test_cam/raw', 10)
        self.pub_compressed = self.create_publisher(CompressedImage, '/test_cam/raw/compressed', 10)
        
        # 尝试打开默认的物理摄像头 0
        self.cap = cv2.VideoCapture(0)
        if self.cap.isOpened():
            self.get_logger().info("成功打开本地摄像头 (Device 0)")
            self.use_camera = True
        else:
            self.get_logger().warn("未检测到本地摄像头，将自动使用程序动态生成测试测试卡画面...")
            self.use_camera = False
            # 初始化动态测试图的物理移动参数
            self.ball_x = 100
            self.ball_y = 100
            self.dx = 6
            self.dy = 5
            self.width = 640
            self.height = 480

        # 定时器：30 FPS (约 33.3ms 发布一次)
        self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)
        self.frame_id = 0
        self.get_logger().info("图像发布器节点已启动！")
        self.get_logger().info("发布话题：/test_cam/raw (类型: sensor_msgs/msg/Image)")
        self.get_logger().info("发布话题：/test_cam/raw/compressed (类型: sensor_msgs/msg/CompressedImage)")

    def timer_callback(self):
        if self.use_camera:
            ret, frame = self.cap.read()
            if not ret:
                self.get_logger().error("读取摄像头帧失败，切换为动态测试图...")
                self.use_camera = False
                return
        else:
            # 动态生成一帧精美的测试卡图像
            frame = self.generate_test_pattern()

        # 1. 转换并发布原始图像 (sensor_msgs/msg/Image)
        try:
            raw_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            raw_msg.header.stamp = self.get_clock().now().to_msg()
            raw_msg.header.frame_id = "test_camera_frame"
            self.pub_raw.publish(raw_msg)
        except Exception as e:
            self.get_logger().error(f"原始图像转换/发布失败: {e}")

        # 2. 转换并发布压缩图像 (sensor_msgs/msg/CompressedImage)
        try:
            compressed_msg = self.bridge.cv2_to_compressed_imgmsg(frame, dst_format="jpg")
            compressed_msg.header.stamp = self.get_clock().now().to_msg()
            compressed_msg.header.frame_id = "test_camera_frame"
            self.pub_compressed.publish(compressed_msg)
        except Exception as e:
            self.get_logger().error(f"压缩图像转换/发布失败: {e}")

        self.frame_id += 1

    def generate_test_pattern(self):
        # 创建暗灰色渐变背景底图
        img = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        for y in range(self.height):
            img[y, :, 0] = int(y / self.height * 50)  # Blue
            img[y, :, 1] = int(y / self.height * 20)  # Green
            img[y, :, 2] = 30                          # Red
            
        # 绘制参考坐标网格
        for x in range(0, self.width, 80):
            cv2.line(img, (x, 0), (x, self.height), (80, 80, 80), 1)
        for y in range(0, self.height, 60):
            cv2.line(img, (0, y), (self.width, y), (80, 80, 80), 1)

        # 移动并绘制一个七彩弹球
        self.ball_x += self.dx
        self.ball_y += self.dy
        if self.ball_x < 35 or self.ball_x > self.width - 35:
            self.dx = -self.dx
        if self.ball_y < 35 or self.ball_y > self.height - 35:
            self.dy = -self.dy

        # 外圈青色，内圈深青色
        cv2.circle(img, (self.ball_x, self.ball_y), 30, (0, 229, 255), -1)
        cv2.circle(img, (self.ball_x, self.ball_y), 15, (0, 131, 143), -1)

        # 绘制动态文本（含时间戳和帧号）
        current_time = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
        cv2.putText(img, "ROS 2 Image Publisher Test", (30, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)
        cv2.putText(img, f"Time: {current_time}", (30, 95), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 200, 200), 2)
        cv2.putText(img, f"Frame ID: {self.frame_id}", (30, 135), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 229, 255), 2)
        
        # 绘制底部动态正弦波形
        points = []
        for x in range(self.width):
            y = int(self.height - 50 + 20 * np.sin((x + self.frame_id * 5) * 0.02))
            points.append([x, y])
        cv2.polylines(img, [np.array(points)], False, (0, 255, 0), 2)

        return img

    def destroy_node(self):
        if self.cap.isOpened():
            self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ImagePublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("正在退出图像发布节点...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
