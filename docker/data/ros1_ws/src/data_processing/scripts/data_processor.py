#!/usr/bin/env python3
from data_processing.msg import FilteredData
import rospy
from std_msgs.msg import String


class DataProcessorNode:
    def __init__(self):
        rospy.init_node("data_processor", anonymous=True)
        self.pub_processed = rospy.Publisher("/data/processed", String, queue_size=10)
        rospy.Subscriber("/data/filtered", FilteredData, self.filtered_callback)

    def filtered_callback(self, msg):
        text_out = (
            f"Processed => temp: {msg.temperature:.2f}, "
            f"hum: {msg.humidity:.2f}, "
            f"valid: {msg.valid}"
        )
        rospy.loginfo("DataProcessor: %s", text_out)
        self.pub_processed.publish(text_out)


def main():
    global node
    node = DataProcessorNode()
    rospy.spin()


if __name__ == "__main__":
    main()
