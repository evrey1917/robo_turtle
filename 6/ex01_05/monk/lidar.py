from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from rclpy.node import Node
import rclpy
import math
import sys

class LidarMovement(Node):

    def __init__(self):
        super().__init__('lidar_movement')

        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/robot/cmd_vel',
            10
        )
        
        self.subscription = self.create_subscription(
            LaserScan,
            '/robot/scan',
            self.listener_callback,
            10
        )
        
        #self.radius = radius
        #self.radius = self.declare_parameter('radius', radius).get_parameter_value().double_value
        #self.control_loop_timer = self.create_timer(0.1, self.control_loop)

    def listener_callback(self, msg):
        cmd_vel_msg = Twist()
        
        len_int = len(msg.ranges)
        
        window = 13
        
        ranges = msg.ranges[len_int // 2 - window : len_int // 2 + window]
        for i in ranges:
            if i < 0.8:
                linear_speed = 0.0
                break
            linear_speed = 1.0
        
        angular_speed = 0.0
        
        cmd_vel_msg.linear.x = linear_speed
        cmd_vel_msg.angular.z = angular_speed

        self.cmd_vel_publisher.publish(cmd_vel_msg)


def main(args=None):
    rclpy.init(args=args)

    lidar_movement = LidarMovement()
    try:
        rclpy.spin(lidar_movement)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == '__main__':
    main()
