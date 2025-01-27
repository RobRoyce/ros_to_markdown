#!/usr/bin/env python3
import random

import rospy
from std_msgs.msg import Float64


def main():
    rospy.init_node("temperature_publisher", anonymous=True)
    pub = rospy.Publisher("/sensor/temperature", Float64, queue_size=10)
    rate = rospy.Rate(1)  # 1Hz

    while not rospy.is_shutdown():
        temp_value = 20.0 + random.uniform(-1.0, 1.0)
        rospy.loginfo("Publishing temperature: %.2f", temp_value)
        pub.publish(temp_value)
        rate.sleep()


if __name__ == "__main__":
    main()
