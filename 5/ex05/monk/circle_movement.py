from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from rclpy.node import Node
import rclpy
import math
import sys

class CircleMovement(Node):

    def __init__(self, radius):
        super().__init__('circle_move')

        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        self.radius = radius
        self.radius = self.declare_parameter('radius', radius).get_parameter_value().double_value
        self.control_loop_timer = self.create_timer(0.5, self.control_loop)

    def control_loop(self):
        cmd_vel_msg = Twist()
        
        linear_speed = self.radius

        angular_speed = 1.0
        
        cmd_vel_msg.linear.x = linear_speed
        cmd_vel_msg.angular.z = angular_speed

        self.cmd_vel_publisher.publish(cmd_vel_msg)


def main(args=None):
    rclpy.init(args=args)
    
    if len(sys.argv) >= 2:
        radius = float(sys.argv[1])
    else:
        radius = float(1.0)

    circle_movement = CircleMovement(radius)
    try:
        rclpy.spin(circle_movement)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == '__main__':
    main()
