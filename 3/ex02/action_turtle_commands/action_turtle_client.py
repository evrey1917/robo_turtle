from rclpy.action import ActionClient
from std_msgs.msg import String
from rclpy.node import Node
import rclpy
import time

from my_action_msg.action import MessageTurtleCommands


class MyActionClient(Node):

    def __init__(self):
        super().__init__('action_client')
        
        self.action_client = ActionClient(
            self,
            MessageTurtleCommands,
            'turtle_action'
        )
        
        self.cmd_text_sub = self.create_subscription(
            String,
            'cmd_text',
            self.send_goal,
            10
        )

    def send_goal(self, msg):
        goal_msg = MessageTurtleCommands.Goal()
        
        splitted_data = msg.data.split()
        
        if len(splitted_data) == 3:
            if not splitted_data[0] in ["forward", "turn_left", "turn_right"]:
                self.get_logger().info(f'Wrong command: {splitted_data[0]}')
                return
        else:
            self.get_logger().info('Wrong command parameters')
            return
            
        goal_msg.command = splitted_data[0]
        goal_msg.s = int(splitted_data[1])
        goal_msg.angle = int(splitted_data[2])
        
        self.action_client.wait_for_server()

        self._send_goal_future = self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result.result}')
            
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: {feedback.odom}')


def main(args=None):
    rclpy.init(args=args)

    action_client = MyActionClient()
    rclpy.spin(action_client)
    
    action_client.destroy_node()
    
    rclpy.shutdown()

#ros2 topic pub -t 1 /cmd_text std_msgs/msg/String "data: 'forward 2 0'"
#ros2 topic pub -t 1 /cmd_text std_msgs/msg/String "data: 'turn_right 0 90'"
#ros2 topic pub -t 1 /cmd_text std_msgs/msg/String "data: 'forward 1 0'"

if __name__ == '__main__':
    main()
