#!/usr/bin/env python3

import random

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class TemperaturePublisher(Node):
    def __init__(self):
        super().__init__('temperature_publisher')
        self.pub = self.create_publisher(Float64, '/sensor/temperature', 10)
        self.timer = self.create_timer(1.0, self.publish_temperature)

    def publish_temperature(self):
        temp_value = 20.0 + random.uniform(-1.0, 1.0)
        self.get_logger().info(f"Publishing temperature: {temp_value:.2f}")
        msg = Float64(data=temp_value)
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TemperaturePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
