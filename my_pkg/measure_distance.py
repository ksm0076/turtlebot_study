#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import math
from std_msgs.msg import String

class DistanceTracker(Node):
    def __init__(self):
        super().__init__('distance_tracker')
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',  # Odometry 토픽 이름
            self.odom_callback,
            10
        )
        self.command_sub = self.create_subscription(
            String,
            '/distance_command',
            self.command_callback,
            10
        )
        
        self.prev_x = None
        self.prev_y = None
        self.total_distance = 0.0
        self.is_measuring_distance = False
        
        self.create_timer(1, self.show_distance)
        
    def odom_callback(self, msg):
        if self.is_measuring_distance:
            current_x = msg.pose.pose.position.x
            current_y = msg.pose.pose.position.y
            if self.prev_x is not None and self.prev_y is not None:
                distance = math.sqrt((current_x - self.prev_x)**2 + (current_y - self.prev_y)**2)
                self.total_distance += distance

                

            self.prev_x = current_x
            self.prev_y = current_y
        
    def command_callback(self, msg):
        command = msg.data
        if command == "start":
            self.start_distance_measurement()
        elif command == "stop":
            self.stop_distance_measurement()
    
    def start_distance_measurement(self):
        self.is_measuring_distance = True
        self.prev_x = None
        self.prev_y = None
        self.total_distance = 0.0
        print("Distance measurement START!!")
        
    def stop_distance_measurement(self):
        self.is_measuring_distance = False
        print(f"Distance measurement STOP, Total distance : {self.total_distance:.2f} m")
    
    def show_distance(self):
        if self.is_measuring_distance:
            self.get_logger().info(f'Traveled distance: {self.total_distance:.2f} m')
        
def main(args=None):
    rclpy.init(args=args)
    distance_tracker = DistanceTracker()
    rclpy.spin(distance_tracker)

    distance_tracker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
