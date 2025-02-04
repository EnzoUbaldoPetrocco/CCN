#!/usr/bin/env python3.10
import rclpy
from rclpy.node import Node

class Controller(Node):
    def __init__(self):
        super().__init__("Controller")
        self.get_logger().info("Wey, alua?")


def main(args=None):
    rclpy.init(args=args)
    controller = Controller()

    rclpy.spin(controller)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
