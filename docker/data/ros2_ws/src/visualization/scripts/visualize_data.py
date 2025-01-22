#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class VisualizeData(Node):
    def __init__(self):
        super().__init__('visualize_data')
        self.sub_processed = self.create_subscription(String, '/data/processed', self.data_callback, 10)

    def data_callback(self, msg):
        self.get_logger().info(f"VISUALIZE: Received processed data => {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = VisualizeData()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
