import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np


class DepthImageSubscriber(Node):
    def __init__(self):
        super().__init__('depth_image_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/depth/image',
            self.listener_callback,
            10)
        self.br = CvBridge()

    def listener_callback(self, msg):
        try:
            # For depth images, the encoding is usually '32FC1' or '16UC1'
            cv_image = self.br.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge Error: {e}')
            return

        # Normalize the depth image to visualize it (clip between 0 and 10 meters)
        depth_array = np.array(cv_image, dtype=np.float32).clip(min=0, max=10)

        depth_image = cv2.normalize(depth_array, None, 0, 255, cv2.NORM_MINMAX)
        depth_image = np.uint8(depth_image)

        cv2.imshow('Depth Image', depth_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    depth_image_subscriber = DepthImageSubscriber()
    rclpy.spin(depth_image_subscriber)
    depth_image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
