from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from rclpy.node import Node
import rclpy
import math
import sys

class StarMovement(Node):

    def __init__(self, speed):
        super().__init__('star_movement')

        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        self.speed = float(speed)
        self.control_loop_timer = self.create_timer(0.01, self.control_loop)

    def control_loop(self):
        cmd_vel_msg = Twist()
        
        linear_speed = self.speed
        
        seconds, nanoseconds = self.get_clock().now().seconds_nanoseconds()
        x = seconds % 5 + nanoseconds / (1000 * 1000 * 1000)
        if x >= 2.3:
            cmd_vel_msg.linear.x = 0.0
            cmd_vel_msg.angular.z = math.pi / 3
        else:
            cmd_vel_msg.linear.x = linear_speed / 3
            cmd_vel_msg.angular.z = 0.0
        
        self.cmd_vel_publisher.publish(cmd_vel_msg)

def main(args=None):
    rclpy.init(args=args)
    
    speed = 2.0

    star_movement = StarMovement(speed)
    try:
        rclpy.spin(star_movement)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == '__main__':
    main()
