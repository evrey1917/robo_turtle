# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('text_to_cmd_vel')
        self.subscription = self.create_subscription(String, 'cmd_text', self.cmd_text_callback, 1)
        self.subscription  # prevent unused variable warning
        
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 1)

    def cmd_text_callback(self, msg):
        twist = Twist()
        
        if msg.data == "turn_right":
            twist.linear.x = 0.0
            twist.angular.z = -3.14 / 2
        elif msg.data == "turn_left":
            twist.linear.x = 0.0
            twist.angular.z = 3.14 / 2
        elif msg.data == "move_forward":
            twist.linear.x = 1.0
            twist.angular.z = 0.0
        elif msg.data == "move_backward":
            twist.linear.x = -1.0
            twist.angular.z = 0.0
        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.get_logger().info('I don\'t know such command: "%s"' % msg.data)
                
        self.publisher_.publish(twist)


class Loh(Node):

    def __init__(self):
        super().__init__('text_to_cmd_vel')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        twist = Twist()
        twist.linear.x = 1.0
        twist.angular.z = 1.0
        self.publisher_.publish(twist)
        self.get_logger().info('I heard:')

def main(args=None):
    
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()
    #minimal_publisher.cmd_text_callback("a")

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
