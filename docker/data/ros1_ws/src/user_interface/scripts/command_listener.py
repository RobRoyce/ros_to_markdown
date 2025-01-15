#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

def main():
    rospy.init_node('command_listener', anonymous=True)
    pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(1)
    forward = True

    while not rospy.is_shutdown():
        twist = Twist()
        if forward:
            twist.linear.x = 0.2
            twist.angular.z = 0.0
        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.5

        rospy.loginfo("UserInterface: Publishing Twist forward=%s", forward)
        pub_cmd_vel.publish(twist)

        forward = not forward
        rate.sleep()

if __name__ == '__main__':
    main()
