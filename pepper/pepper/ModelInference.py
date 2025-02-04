#!/usr/bin/env python3.10
import rclpy
from rclpy.node import Node
from PIL import Image
import sys
sys.path.insert(1, '../')
from robot_interfaces.srv import Inference
from sensor_msgs.msg import Image


#TODO: connect this node to flask via client and server
#TODO: connect this node to the model for inference

class ModelInference(Node):
    def __init__(self):
        super().__init__("ModelInference")
        self.srv = self.create_service(Inference, 'inference_image', self.inference_clbk)
    
    
    def inference_clbk(self, request, response):
        img = response.image
        shape = response.shape
        response.res = "BELIIIIIIIIIIIn"
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.image, request.shape))

        return response


def main(args=None):
    rclpy.init(args=args)
    mi = ModelInference()
    rclpy.spin(mi)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
