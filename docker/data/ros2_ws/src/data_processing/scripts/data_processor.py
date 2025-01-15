#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from data_processing.msg import FilteredData
from std_msgs.msg import String

class DataProcessor(Node):
    def __init__(self):
        super().__init__('data_processor')
        self.pub_processed = self.create_publisher(String, '/data/processed', 10)
        self.sub_filtered = self.create_subscription(FilteredData, '/data/filtered', self.filtered_callback, 10)

    def filtered_callback(self, msg):
        text_out = f"Processed => temp: {msg.temperature:.2f}, hum: {msg.humidity:.2f}, valid: {msg.valid}"
        self.get_logger().info(text_out)
        self.pub_processed.publish(String(data=text_out))

def main(args=None):
    rclpy.init(args=args)
    node = DataProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
