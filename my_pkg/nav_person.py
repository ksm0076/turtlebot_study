import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from cv_bridge import CvBridge
from ultralytics import YOLO
import numpy as np

class PersonFollower(Node):
    def __init__(self, image_topic='/camera/image_raw', scan_topic='/scan'):
        super().__init__('person_follower')
        
        # YOLO 모델 초기화 및 ROS 설정
        self.image_subscriber = self.create_subscription(Image, image_topic, self.image_callback, 10)
        self.scan_subscriber = self.create_subscription(LaserScan, scan_topic, self.scan_callback, 10)
        
        self.bridge = CvBridge()
        self.model = YOLO("yolov8n.pt")  # YOLOv8 모델 경로
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # 변수 초기화
        self.angle_to_person = None
        self.distance_to_person = None
        self.camera_fov = 62.2  # 카메라 수평 시야각
        self.image_width = 640  # 카메라 이미지의 너비 640 * 480
        
    def image_callback(self, msg):
        # 이미지에서 사람을 감지
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        results = self.model.predict(source=cv_image, device='cpu')[0].cpu()
        
        if results.boxes:
            for bbox_info in results.boxes:
                if int(bbox_info.cls) == 0:  # 사람이 검출된 경우
                    bbox = bbox_info.xywh[0]
                    center_x = float(bbox[0])

                    # 중심 좌표에서 카메라 기준 각도를 계산
                    self.angle_to_person = (center_x - self.image_width / 2) * (self.camera_fov / self.image_width)
                    self.get_logger().info(f"Person detected at angle: {self.angle_to_person:.2f} degrees")
                    return
        else:
            self.get_logger().info("No person detected.")
            self.angle_to_person = None

    def scan_callback(self, msg):
        # 사람의 방향이 설정되었을 때 해당 방향에서의 거리를 계산
        if self.angle_to_person is not None:
            scan_index = int((self.angle_to_person + self.camera_fov / 2) / self.camera_fov * len(msg.ranges))
            self.distance_to_person = msg.ranges[scan_index]
            self.get_logger().info(f"Distance to person: {self.distance_to_person:.2f} meters")
            
            # 거리와 방향을 기반으로 목표 좌표 전송
            self.move_towards_person()

    def move_towards_person(self):
        # 사람을 향해 이동할 목표 설정
        if self.distance_to_person is None or self.distance_to_person == float('inf'):
            self.get_logger().warn("No valid distance to person.")
            return
        
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'base_link'
        goal_pose.header.stamp = self.get_clock().now().to_msg()

        # 사람 방향과 거리로 목표 위치 설정
        goal_pose.pose.position.x = self.distance_to_person * 0.9  # 약간의 여유 거리 유지
        goal_pose.pose.position.y = np.tan(np.radians(self.angle_to_person)) * self.distance_to_person
        goal_pose.pose.orientation.w = 1.0
        
        self.send_goal(goal_pose)

    def send_goal(self, pose):
        if not self.nav_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().warn("Navigation server not available.")
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self.nav_client.send_goal_async(goal_msg)

def main(args=None):
    rclpy.init(args=args)
    person_follower = PersonFollower()
    rclpy.spin(person_follower)
    person_follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
