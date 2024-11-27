#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from PyQt5.QtWidgets import QApplication, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QGridLayout, QWidget
from PyQt5.QtGui import QPixmap, QImage
from PyQt5.QtCore import Qt, QTimer
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
import cv2

class ImageDisplayNode(Node):
    def __init__(self):
        super().__init__('image_display_node')
        self.bridge = CvBridge()
        self.latest_image = None

        # 이미지 토픽 구독
        self.image_subscriber = self.create_subscription(
            Image,
            '/yolov8_output',  # ROS2 이미지 토픽 이름
            self.image_callback,
            10
        )
        self.get_logger().info("Subscribed to /yolov8_output")

        # 시간과 거리 토픽 구독
        self.time_subscriber = self.create_subscription(
            String,
            '/time_print',
            self.time_callback,
            10
        )
        self.distance_subscriber = self.create_subscription(
            String,
            '/distance_print',
            self.distance_callback,
            10
        )

        # 최신 시간과 거리 데이터 저장
        self.latest_time = "00:00"
        self.latest_distance = "00 m"

    def image_callback(self, msg):
        """이미지 콜백 함수: ROS2 메시지를 OpenCV 이미지로 변환."""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.latest_image = cv_image
        except CvBridgeError as e:
            self.get_logger().error(f"Error converting image: {e}")

    def time_callback(self, msg):
        """시간 데이터 콜백 함수."""
        self.latest_time = msg.data

    def distance_callback(self, msg):
        """거리 데이터 콜백 함수."""
        self.latest_distance = msg.data


class ImageDisplayGUI(QWidget):
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node

        self.setWindowTitle('ROS2 Image Viewer')
        self.setGeometry(100, 100, 800, 600)

        # 전체 레이아웃
        self.main_layout = QVBoxLayout()

        # 이미지 영역
        self.image_label = QLabel("Image")
        self.image_label.setAlignment(Qt.AlignCenter)
        self.image_label.setStyleSheet("border: 1px solid black;")
        self.image_label.setMinimumSize(600, 400)
        self.main_layout.addWidget(self.image_label)

        # 하단 영역
        self.bottom_layout = QHBoxLayout()

        # 버튼 영역
        self.button_layout = QVBoxLayout()
        self.start_button = QPushButton("START")
        self.start_button.setStyleSheet("background-color: #005580; color: white; font-size: 16px;")
        self.stop_button = QPushButton("STOP")
        self.stop_button.setStyleSheet("background-color: #005580; color: white; font-size: 16px;")
        self.button_layout.addWidget(self.start_button)
        self.button_layout.addWidget(self.stop_button)

        self.bottom_layout.addLayout(self.button_layout)

        # 정보 표시 영역
        self.info_layout = QGridLayout()
        self.time_label = QLabel("Time")
        self.time_label.setStyleSheet("font-size: 16px;")
        self.time_value = QLabel("00:00")
        self.time_value.setStyleSheet("background-color: #add8e6; font-size: 16px; padding: 5px;")
        self.distance_label = QLabel("Distance")
        self.distance_label.setStyleSheet("font-size: 16px;")
        self.distance_value = QLabel("00 m")
        self.distance_value.setStyleSheet("background-color: #add8e6; font-size: 16px; padding: 5px;")

        self.info_layout.addWidget(self.time_label, 0, 0)
        self.info_layout.addWidget(self.time_value, 0, 1)
        self.info_layout.addWidget(self.distance_label, 1, 0)
        self.info_layout.addWidget(self.distance_value, 1, 1)

        self.bottom_layout.addLayout(self.info_layout)

        # 하단 레이아웃 추가
        self.main_layout.addLayout(self.bottom_layout)

        # 최종 레이아웃 설정
        self.setLayout(self.main_layout)

        # 타이머 설정
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_gui)
        self.timer.start(100)

    def update_gui(self):
        """GUI 업데이트: 이미지, 시간 및 거리 값을 업데이트."""
        rclpy.spin_once(self.ros_node, timeout_sec=0.01)

        # 이미지 업데이트
        if self.ros_node.latest_image is not None:
            cv_image = cv2.cvtColor(self.ros_node.latest_image, cv2.COLOR_BGR2RGB)
            height, width, channel = cv_image.shape
            bytes_per_line = channel * width
            qt_image = QImage(cv_image.data, width, height, bytes_per_line, QImage.Format_RGB888)
            pixmap = QPixmap.fromImage(qt_image)
            self.image_label.setPixmap(pixmap)

        # 시간 및 거리 값 업데이트
        self.time_value.setText(self.ros_node.latest_time)
        self.distance_value.setText(self.ros_node.latest_distance)


def main():
    # ROS2 및 PyQt5 초기화
    rclpy.init()
    app = QApplication(sys.argv)

    # ROS2 노드 생성
    ros_node = ImageDisplayNode()

    # PyQt5 GUI 생성
    gui = ImageDisplayGUI(ros_node)

    # GUI 표시
    gui.show()

    # Qt 애플리케이션 실행
    app.exec()

    # ROS2 종료
    ros_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
