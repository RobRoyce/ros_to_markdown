#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
import random

def main():
    rospy.init_node('humidity_publisher', anonymous=True)
    pub = rospy.Publisher('/sensor/humidity', Float64, queue_size=10)
    rate = rospy.Rate(2)  # 2Hz

    while not rospy.is_shutdown():
        humidity_value = 50.0 + random.uniform(-5.0, 5.0)
        rospy.loginfo("Publishing humidity: %.2f", humidity_value)
        pub.publish(humidity_value)
        rate.sleep()

if __name__ == '__main__':
    main()
