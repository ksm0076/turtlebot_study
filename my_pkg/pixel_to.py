import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
import pyrealsense2 as rs2

class PersonFollower(Node):
    def __init__(self, image_topic, depth_image_topic, depth_info_topic):
        super().__init__('person_follower')
        
        # 카메라 및 YOLO 모델 초기화
        self.image_subscriber = self.create_subscription(Image, image_topic, self.image_callback, 10)
        self.depth_subscriber = self.create_subscription(Image, depth_image_topic, self.depth_callback, 10)
        self.depth_info_subscriber = self.create_subscription(CameraInfo, depth_info_topic, self.depth_info_callback, 10)

        self.bridge = CvBridge()
        self.model = YOLO("yolov8n.pt")  # YOLOv8 모델 경로
        self.intrinsics = None  # 카메라 내부 파라미터 저장
        self.target_pixel = None  # 타겟 픽셀 위치
        self.target_distance = None  # 타겟 거리

    def image_callback(self, msg):
        # YOLO를 통한 사람 검출
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        results = self.model.predict(source=cv_image, device='cpu')[0].cpu()
        
        if results.boxes:
            for bbox_info in results.boxes:
                if int(bbox_info.cls) == 0:  # 사람이 검출된 경우
                    bbox = bbox_info.xywh[0]
                    center_x, center_y = int(bbox[0]), int(bbox[1])
                    self.target_pixel = (center_x, center_y)
                    self.get_logger().info(f"Person detected at pixel: {self.target_pixel}")
                    break
        else:
            self.target_pixel = None
            self.get_logger().info("No person detected.")

    def depth_callback(self, depth_msg):
        if self.target_pixel and self.intrinsics:
            # 깊이 이미지를 OpenCV 형식으로 변환
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, depth_msg.encoding)
            center_x, center_y = self.target_pixel
            depth_value = depth_image[center_y, center_x]

            # 깊이 값이 유효하면 3D 좌표 계산
            if depth_value > 0:
                # 픽셀 좌표와 깊이 값을 3D 공간 좌표로 변환
                result = rs2.rs2_deproject_pixel_to_point(self.intrinsics, [center_x, center_y], depth_value)
                self.get_logger().info(f"3D Position of person: x={result[0]:.2f} m, y={result[1]:.2f} m, z={result[2]:.2f} m")
            else:
                self.get_logger().warn("Invalid depth value at target pixel.")

    def depth_info_callback(self, camera_info_msg):
        # 카메라 내재 파라미터 설정
        if not self.intrinsics:
            self.intrinsics = rs2.intrinsics()
            self.intrinsics.width = camera_info_msg.width
            self.intrinsics.height = camera_info_msg.height
            self.intrinsics.ppx = camera_info_msg.k[2]
            self.intrinsics.ppy = camera_info_msg.k[5]
            self.intrinsics.fx = camera_info_msg.k[0]
            self.intrinsics.fy = camera_info_msg.k[4]
            if camera_info_msg.distortion_model == 'plumb_bob':
                self.intrinsics.model = rs2.distortion.brown_conrady
            elif camera_info_msg.distortion_model == 'equidistant':
                self.intrinsics.model = rs2.distortion.kannala_brandt4
            self.intrinsics.coeffs = [i for i in camera_info_msg.d]

def main(args=None):
    rclpy.init(args=args)
    image_topic = '/camera/color/image_raw'
    depth_image_topic = '/camera/depth/image_rect_raw'
    depth_info_topic = '/camera/depth/camera_info'
    
    node = PersonFollower(image_topic, depth_image_topic, depth_info_topic)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
