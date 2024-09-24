#from example_interfaces.srv import AddTwoInts
from interface_name.srv import FullNameSumService

import rclpy
from rclpy.node import Node


class MinimalService(Node):

    def __init__(self):
        super().__init__('service_name')
        self.srv = self.create_service(FullNameSumService, 'SummFullName', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.full_name = request.last_name + " " + request.name + " " + request.first_name
        self.get_logger().info('Incoming request\nlast name: %s, name: %s, first name: %s' % (request.last_name, request.name, request.first_name))

        return response


def main():
    rclpy.init()

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
