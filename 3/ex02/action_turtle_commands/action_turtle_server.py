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
        self.get_logger().info('Goal has received!')
        
        command = goal_handle.request.command
        result = MessageTurtleCommands.Result()
        feedback_msg = MessageTurtleCommands.Feedback()
        twist = Twist()
        
        start_pose = self.current_pose
        
        if start_pose is None:
            result.result = False
            goal_handle.succeed()
            self.get_logger().error("TurtleSim is not running")
            return result
        try:
            if goal_handle.is_cancel_requested:
                result.result = False
                goal_handle.succeed()
                self.get_logger().error("Goal was canceled at runtime")
                return result
            
            if command == 'forward':
                twist.linear.x = float(goal_handle.request.s)
            elif command == 'turn_left':
                twist.angular.z = 1 * goal_handle.request.angle * math.pi / 180
            elif command == 'turn_right':
                twist.angular.z = -1 * goal_handle.request.angle * math.pi / 180
            
            self._cmd_vel_publisher.publish(twist)
            need_distance = int(goal_handle.request.s)
            distance_moved = 0
            feedback_msg.odom = distance_moved            
            goal_handle.publish_feedback(feedback_msg)
            
            while distance_moved < need_distance:
                rclpy.spin_once(self)
                distance_moved = int(math.sqrt(
                    (self.current_pose.x - start_pose.x)**2 +
                    (self.current_pose.y - start_pose.y)**2
                ))
                feedback_msg.odom = distance_moved
                goal_handle.publish_feedback(feedback_msg)
            result.result = True
        
        except Exception as e:
            self.get_logger().error("Something went wrong...")
            self.get_logger().error(f"{e}")
            goal_handle.succeed()
            result.result = False
            return result
        
        goal_handle.succeed()
        
        #self.create_timer(0.1, self.my_timer_callback)
        rclpy.spin(self)
        
        return result

    def my_timer_callback(self):
        for i in self.timers:
            self.destroy_timer(i)
        self.create_timer(1, self.my_timer_callback)
        self.get_logger().error("BAM")
        rclpy.spin(self)

def main(args=None):
    rclpy.init(args=args)

    action_server = MyActionServer()

    rclpy.spin(action_server)
    
    action_server.destroy_node()
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()
