#!/usr/bin/env python3
import rospy
from data_processing.msg import FilteredData
from std_msgs.msg import String

class DataProcessorNode:
    def __init__(self):
        rospy.init_node('data_processor', anonymous=True)
        self.pub_processed = rospy.Publisher('/data/processed', String, queue_size=10)
        rospy.Subscriber('/data/filtered', FilteredData, self.filtered_callback)

    def filtered_callback(self, msg):
        text_out = "Processed => temp: {:.2f}, hum: {:.2f}, valid: {}".format(
            msg.temperature, msg.humidity, msg.valid)
        rospy.loginfo("DataProcessor: %s", text_out)
        self.pub_processed.publish(text_out)

def main():
    node = DataProcessorNode()
    rospy.spin()

if __name__ == '__main__':
    main()
