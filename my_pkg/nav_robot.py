import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped

class NavToPoseClient(Node):
    def __init__(self):
        super().__init__('nav_to_pose_client')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.feedback_msg = None
        self.timer = self.create_timer(1.0, self.timer_callback)
        
    # 로봇을 목표 지점으로 이동시키기 위한 함수
    def send_goal(self, pose):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose # 목표지점

        self._action_client.wait_for_server()
        
        # 비동기적으로 액션 서버에 목표 전송
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        
    # 서버에서 목표 수락 여부 확인
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected.')
            return

        self.get_logger().info('Goal accepted.')
        # 비동기적으로 목표 수행의 결과를 가져옴
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
        
    # 목표 수행 결과를 처리하는 함수
    def get_result_callback(self, future):
        result = future.result()
        if result.status == 3:  # 3 means SUCCEEDED in action client status
            self.get_logger().info('Goal reached.')
        else:
            self.get_logger().warn(f'Goal failed with status: {result.status}')
            
        # 목표 도달 시 타이머 취소
        self.timer.cancel()
        self.feedback_msg = None
        
    def feedback_callback(self, feedback_msg):
        self.feedback_msg = feedback_msg.feedback
        # self.get_logger().info(f'Feedback: {feedback_msg.feedback}')
        
    def timer_callback(self):
    # 1초마다 호출되는 이 함수에서 피드백 메시지를 출력합니다.
        if self.feedback_msg is not None:
            self.get_logger().info(f'Feedback: {self.feedback_msg}')

def main(args=None):
    rclpy.init(args=args)

    node = NavToPoseClient()

    # 목표 지점을 설정합니다 (예: x=2.0, y=3.0 위치로 이동)
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = node.get_clock().now().to_msg()
    pose.pose.position.x = 3.8
    pose.pose.position.y = 0.8
    pose.pose.orientation.w = 1.0

    node.send_goal(pose)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()