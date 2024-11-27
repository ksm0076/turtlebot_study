#!/usr/bin/env python
import rclpy
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf_transformations import euler_from_quaternion
import math
from geometry_msgs.msg import Quaternion
import time

class OdomSubscription(Node):
    def __init__(self):
        super().__init__('Odom_Subscription')
        
        self.odom_pub = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10
        )
        self.current_pose = Pose()
        self.create_timer(1, self.time_callback)
    
    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose
    
    def time_callback(self):
        print("x:", self.current_pose.orientation.x)
        print("y:", self.current_pose.orientation.y)
        print("z:", self.current_pose.orientation.z)
        print("w:",self.current_pose.orientation.w)
        print("--------------------------------")
        
def main():
    rclpy.init()
    
    odom_sub = OdomSubscription()
    rclpy.spin(odom_sub)
    
    odom_sub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()