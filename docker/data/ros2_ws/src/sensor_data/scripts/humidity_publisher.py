#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import random

class HumidityPublisher(Node):
    def __init__(self):
        super().__init__('humidity_publisher')
        self.pub = self.create_publisher(Float64, '/sensor/humidity', 10)
        # 2Hz => timer of 0.5 seconds
        self.timer = self.create_timer(0.5, self.publish_humidity)

    def publish_humidity(self):
        humidity_value = 50.0 + random.uniform(-5.0, 5.0)
        self.get_logger().info(f"Publishing humidity: {humidity_value:.2f}")
        msg = Float64(data=humidity_value)
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = HumidityPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
