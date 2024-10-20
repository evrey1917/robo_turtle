from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from rclpy.node import Node
import rclpy
import math
import sys

class MoveToGoal(Node):

    def __init__(self, x_goal, y_goal, theta_goal):
        super().__init__('move_to_goal')

        self.x_goal = x_goal
        self.y_goal = y_goal
        self.theta_goal = theta_goal

        self.pose_subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10
        )

        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/turtle1/cmd_vel',
            10
        )

        self.current_pose = None
        self.control_loop_timer = self.create_timer(0.5, self.control_loop)

    def pose_callback(self, msg):
        self.current_pose = msg

    def control_loop(self):
        if self.current_pose is None:
            return

        distance_to_goal = math.sqrt(
            (self.x_goal - self.current_pose.x) ** 2 +
            (self.y_goal - self.current_pose.y) ** 2
        )

        distance_epsilon = 0.1

        if distance_to_goal > distance_epsilon:
            linear_speed = distance_to_goal

            angle_to_goal = math.atan2(self.y_goal - self.current_pose.y, self.x_goal - self.current_pose.x)
            angular_speed = 2.0 * (angle_to_goal - self.current_pose.theta)

            cmd_vel_msg = Twist()
            cmd_vel_msg.linear.x = linear_speed
            cmd_vel_msg.angular.z = angular_speed
            self.cmd_vel_publisher.publish(cmd_vel_msg)

        else:
            angle_difference = self.theta_goal - self.current_pose.theta
            if abs(angle_difference) > 0.01:
                cmd_vel_msg = Twist()
                cmd_vel_msg.angular.z = angle_difference
                self.cmd_vel_publisher.publish(cmd_vel_msg)
            else:
                self.cmd_vel_publisher.publish(Twist())
                self.get_logger().info("Goal Reached")
                self.destroy_timer(self.control_loop_timer)

def main(args=None):
    rclpy.init(args=args)
    
    if len(sys.argv) != 4:
        return
    
    x_goal = float(sys.argv[1])
    y_goal = float(sys.argv[2])
    theta_goal = float(sys.argv[3])

    move_to_goal_node = MoveToGoal(x_goal, y_goal, theta_goal)
    rclpy.spin(move_to_goal_node)
    move_to_goal_node.get_logger().info("DESTROYED")
    move_to_goal_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
