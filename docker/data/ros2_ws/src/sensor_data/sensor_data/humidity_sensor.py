#!/usr/bin/env python3

import random

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class HumiditySensor(Node):
    def __init__(self):
        super().__init__("humidity_sensor")
        self.publisher = self.create_publisher(Float64, "/sensor/humidity", 10)
        self.timer = self.create_timer(1.0, self.publish_humidity)
        self.get_logger().info("Humidity sensor started")

    def publish_humidity(self):
        msg = Float64()
        msg.data = 50.0 + random.uniform(-10.0, 10.0)  # Random humidity around 50%
        self.publisher.publish(msg)
        self.get_logger().info(f"Published humidity: {msg.data:.2f}%")


def main(args=None):
    rclpy.init(args=args)
    node = HumiditySensor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
