#!/usr/bin/env python3

from environment_integration.msg import EnvironmentData

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class EnvironmentBuilder(Node):
    def __init__(self):
        super().__init__("environment_builder")
        self.pub_env = self.create_publisher(EnvironmentData, "/environment/data", 10)
        self.sub_processed = self.create_subscription(
            String, "/data/processed", self.processed_callback, 10
        )

    def processed_callback(self, msg):
        env_msg = EnvironmentData()
        env_msg.frame_id = "map"
        # Convert text into ASCII-based map
        env_msg.map_data = [float(ord(c)) for c in msg.data]
        self.pub_env.publish(env_msg)
        self.get_logger().info("Published EnvironmentData")


def main(args=None):
    rclpy.init(args=args)
    node = EnvironmentBuilder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
