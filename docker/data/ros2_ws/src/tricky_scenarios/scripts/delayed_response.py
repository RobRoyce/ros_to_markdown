#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


class DelayedResponse(Node):
    def __init__(self):
        super().__init__("delayed_response_server")
        self.srv = self.create_service(Trigger, "/delayed_response", self.handle_delayed_request)
        self.get_logger().info("DelayedResponse: service /delayed_response is ready.")

    def handle_delayed_request(self, request, response):
        self.get_logger().info("DelayedResponse: Received request. Sleeping 5 seconds...")
        time.sleep(5)
        response.success = True
        response.message = "DelayedResponse done after 5s"
        return response


def main(args=None):
    rclpy.init(args=args)
    node = DelayedResponse()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
