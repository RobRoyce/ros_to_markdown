#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class MoveRobot(Node):
    def __init__(self):
        super().__init__('move_robot')
        self.sub_cmd = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.pub_status = self.create_publisher(String, '/robot/status', 10)

    def cmd_vel_callback(self, cmd):
        self.get_logger().info(f"move_robot: Received cmd_vel linear={cmd.linear.x:.2f}, angular={cmd.angular.z:.2f}")
        status_msg = f"MOVING with linear={cmd.linear.x:.2f}, angular={cmd.angular.z:.2f}"
        self.pub_status.publish(String(data=status_msg))

def main(args=None):
    rclpy.init(args=args)
    node = MoveRobot()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
