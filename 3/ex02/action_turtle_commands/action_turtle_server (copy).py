from rclpy.action import ActionServer
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from rclpy.node import Node
import rclpy
import math

from my_action_msg.action import MessageTurtleCommands


class MyActionServer(Node):

    def __init__(self):
        super().__init__('action_server')
        
        self._action_server = ActionServer(
            self,
            MessageTurtleCommands,
            'turtle_action',
            self.execute_callback
        )
        
        self._pose_subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10
        )

        self._cmd_vel_publisher = self.create_publisher(
            Twist,
            '/turtle1/cmd_vel',
            10
        )

        self.current_pose = Pose()
    
    def pose_callback(self, pose):
        self.current_pose = pose

    def execute_callback(self, goal_handle):
        self.get_logger().info('Commanda: ' + str(goal_handle.request.command) + str(goal_handle.request.s) + ' meters, angle:' + str(goal_handle.request.angle))
        
        self.get_logger().info('Executing goal...')
        
        feedback_msg = MessageTurtleCommands.Feedback()

        if self.current_pose is None:
            self.get_logger().error('No pose now')
            goal_handle.abort()
            return MessageTurtleCommands.Result(result=False)

        twist = Twist()

        if goal_handle.request.command == "forward":
            self.get_logger().info('Moving forward ' + str(goal_handle.request.s) + ' meters.')
            start_x = self.current_pose.x
            start_y = self.current_pose.y
            odometry = 0.0

            twist.linear.x = 1.0
            k = 0
            while odometry < goal_handle.request.s:
                self._cmd_vel_publisher.publish(twist)
                #rclpy.spin_once(self)
                odometry = math.sqrt(
                    (self.current_pose.x - start_x) ** 2 +
                    (self.current_pose.y - start_y) ** 2
                )
                feedback_msg.odom = int(odometry)
                goal_handle.publish_feedback(feedback_msg)

            twist.linear.x = 0.0
            self._cmd_vel_publisher.publish(twist)

        elif goal_handle.request.command in ["turn_left", "turn_right"]:
            angle_to_turn = goal_handle.request.angle
            self.get_logger().info('Turning ' + str(angle_to_turn) + ' degrees.')

            twist.angular.z = 1.0 if goal_handle.request.command == "turn_left" else -1.0

            initial_angle = self.current_pose.theta
            angle_turned = 0.0
            k = 0
            while angle_turned < math.radians(abs(angle_to_turn)):
                self._cmd_vel_publisher.publish(twist)
                #rclpy.spin_once(self)
                angle_turned = abs(self.current_pose.theta - initial_angle)

            twist.angular.z = 0.0
            self._cmd_vel_publisher.publish(twist)

        goal_handle.succeed()
        
        return MessageTurtleCommands.Result(result=True)


def main(args=None):
    rclpy.init(args=args)

    action_server = MyActionServer()

    rclpy.spin(action_server)
    
    #action_server.destroy_node()
    
    #rclpy.shutdown()


if __name__ == '__main__':
    main()
