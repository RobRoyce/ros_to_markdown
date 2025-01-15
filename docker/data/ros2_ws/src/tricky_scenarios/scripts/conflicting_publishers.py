#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import random

class ConflictingPublishers(Node):
    def __init__(self):
        super().__init__('conflicting_temperature_publisher')
        self.pub_temp = self.create_publisher(Float64, '/sensor/temperature', 10)
        self.timer = self.create_timer(1.0, self.publish_conflict)

    def publish_conflict(self):
        temp_value = 100.0 + random.uniform(-5.0, 5.0)
        self.get_logger().warn(f"Conflicting publisher: publishing temperature: {temp_value:.2f}")
        self.pub_temp.publish(Float64(data=temp_value))

def main(args=None):
    rclpy.init(args=args)
    node = ConflictingPublishers()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
