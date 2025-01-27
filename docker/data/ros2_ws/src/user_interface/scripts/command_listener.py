#!/usr/bin/env python3

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node


class CommandListener(Node):
    def __init__(self):
        super().__init__("command_listener")
        self.pub_cmd_vel = self.create_publisher(Twist, "/cmd_vel", 10)
        self.timer = self.create_timer(1.0, self.publish_cmd)
        self.forward = True

    def publish_cmd(self):
        twist = Twist()
        if self.forward:
            twist.linear.x = 0.2
            twist.angular.z = 0.0
        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.5

        self.get_logger().info(f"Publishing Twist forward={self.forward}")
        self.pub_cmd_vel.publish(twist)
        self.forward = not self.forward


def main(args=None):
    rclpy.init(args=args)
    node = CommandListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
