#!/usr/bin/env python3
from geometry_msgs.msg import Twist
import rospy
from std_msgs.msg import String


class MoveRobotNode:
    def __init__(self):
        rospy.init_node("move_robot", anonymous=True)
        rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_callback)
        self.pub_status = rospy.Publisher("/robot/status", String, queue_size=10)

    def cmd_vel_callback(self, cmd):
        rospy.loginfo(
            "move_robot: Received cmd_vel linear=%.2f, angular=%.2f", cmd.linear.x, cmd.angular.z
        )
        status_msg = f"MOVING with linear={cmd.linear.x:.2f}, angular={cmd.angular.z:.2f}"
        self.pub_status.publish(status_msg)


def main():
    global node
    node = MoveRobotNode()
    rospy.spin()


if __name__ == "__main__":
    main()
