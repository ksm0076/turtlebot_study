import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
import random

class PersonSegmentationNode(Node):
    def __init__(self):
        super().__init__('person_segmentation_node')
        self.img_pub = self.create_publisher(Image, '/yolov8_output', 10)
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_listener_callback,
            10)
        self.bridge = CvBridge()
        self.model = YOLO("yolov8n.pt")  # YOLOv8 모델 경로
        self._class_to_color = {}  # 각 클래스별 색상 지정

    def image_listener_callback(self, msg):
        # ROS image massage -> convert -> OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        results = self.model.predict(source=cv_image, device='cpu')[0].cpu()  # CPU에서 추론

        if not results.boxes:
            print('No objects detected')
            self.img_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8"))
            return
        
        ###
        sorted_boxes = sorted(
            results.boxes,
            key=lambda box: (box.xywh[0][2] * box.xywh[0][3]),  # width * height
            reverse=True  # 크기가 큰 순서대로 정렬
        )
        ### 
        bbox_info = sorted_boxes[0]
        class_name = self.model.names[int(bbox_info.cls)] # bbox_info.cls : class id, human = 0            
        bbox = bbox_info.xywh[0] # (x, y)-coordinate and size of bbox
        center_x = float(bbox[0]) 
        center_y = float(bbox[1])
        size_x = float(bbox[2])
        size_y = float(bbox[3])

        if class_name not in self._class_to_color:
            color = (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255))
            self._class_to_color[class_name] = color
        else:
            color = self._class_to_color[class_name]

        pt1 = (round(center_x - size_x / 2.0), round(center_y - size_y / 2.0))
        pt2 = (round(center_x + size_x / 2.0), round(center_y + size_y / 2.0))
        cv2.rectangle(cv_image, pt1, pt2, color, 2)
        # cv2.putText(cv_image, class_name, (pt1[0], pt1[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        
        ###
        print(bbox_info.conf.numpy())
        bbox_size = size_x * size_y
        x1, y1, x2, y2 = bbox_info.xyxy[0].numpy() # 좌상단, 우하단
        
        # print(f"x: {center_x}, y: {center_y}, size: {bbox_size}")
        print(f"pt1: {pt1}, pt2: {pt2}")
        print(f"x1y1: {x1}, {y1}, x2y2: {x2}, {y2}")
        ###
        
        cv2.putText(cv_image, class_name, (round(center_x), round(center_y)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        # cv2.putText(image, text, org, font, fontScale, color, thickness)
        # org : 좌측하단 모서리 좌표
        self.img_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8"))

def main(args=None):
    rclpy.init(args=args)
    person_segmentation_node = PersonSegmentationNode()
    rclpy.spin(person_segmentation_node)
    person_segmentation_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
