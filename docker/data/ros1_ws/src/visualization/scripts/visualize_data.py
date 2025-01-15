#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

def data_callback(msg):
    rospy.loginfo("VISUALIZE: Received processed data => %s", msg.data)

def main():
    rospy.init_node('visualize_data', anonymous=True)
    rospy.Subscriber('/data/processed', String, data_callback)
    rospy.spin()

if __name__ == '__main__':
    main()
