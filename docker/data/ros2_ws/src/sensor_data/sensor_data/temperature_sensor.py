#!/usr/bin/env python3

import random

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class TemperatureSensor(Node):
    def __init__(self):
        super().__init__("temperature_sensor")
        self.publisher = self.create_publisher(Float64, "/sensor/temperature", 10)
        self.timer = self.create_timer(1.0, self.publish_temperature)
        self.get_logger().info("Temperature sensor started")

    def publish_temperature(self):
        msg = Float64()
        msg.data = 20.0 + random.uniform(-2.0, 2.0)  # Random temperature around 20°C
        self.publisher.publish(msg)
        self.get_logger().info(f"Published temperature: {msg.data:.2f}°C")


def main(args=None):
    rclpy.init(args=args)
    node = TemperatureSensor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
