#!/usr/bin/env python3
from environment_integration.msg import EnvironmentData
import rospy
from std_msgs.msg import String


class EnvironmentBuilderNode:
    def __init__(self):
        rospy.init_node('environment_builder', anonymous=True)
        self.pub_env = rospy.Publisher('/environment/data', EnvironmentData, queue_size=10)
        rospy.Subscriber('/data/processed', String, self.processed_callback)

    def processed_callback(self, msg):
        env_msg = EnvironmentData()
        env_msg.frame_id = "map"
        # Convert the processed string into a dummy array of ASCII values
        env_msg.map_data = [ord(c) for c in msg.data]
        self.pub_env.publish(env_msg)
        rospy.loginfo("EnvironmentBuilder: Published EnvironmentData")

def main():
    node = EnvironmentBuilderNode()
    rospy.spin()

if __name__ == '__main__':
    main()
