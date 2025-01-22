#!/usr/bin/env python3

import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class DataProcessor(Node):
    def __init__(self):
        super().__init__('data_processor')
        self.pub_processed = self.create_publisher(String, '/data/processed', 10)
        self.sub_filtered = self.create_subscription(String, '/data/filtered', self.filtered_callback, 10)

    def filtered_callback(self, msg):
        try:
            data = json.loads(msg.data)
            text_out = f"Processed => temp: {data['temperature']:.2f}, hum: {data['humidity']:.2f}, valid: {data['valid']}"
            self.get_logger().info(text_out)
            self.pub_processed.publish(String(data=text_out))
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to parse message: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = DataProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
