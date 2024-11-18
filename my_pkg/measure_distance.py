#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import math

class DistanceTracker(Node):
    def __init__(self):
        super().__init__('distance_tracker')
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',  # Odometry 토픽 이름
            self.odom_callback,
            10
        )
        
        self.prev_x = None
        self.prev_y = None
        self.total_distance = 0.0

    def odom_callback(self, msg):
        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y

        if self.prev_x is not None and self.prev_y is not None:
            distance = math.sqrt((current_x - self.prev_x)**2 + (current_y - self.prev_y)**2)
            self.total_distance += distance

            self.get_logger().info(f'Traveled distance: {self.total_distance:.2f} meters')

        self.prev_x = current_x
        self.prev_y = current_y

def main(args=None):
    rclpy.init(args=args)
    distance_tracker = DistanceTracker()
    rclpy.spin(distance_tracker)

    distance_tracker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
