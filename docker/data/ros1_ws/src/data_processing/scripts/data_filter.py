#!/usr/bin/env python3
from data_processing.msg import FilteredData
import rospy
from std_msgs.msg import Float64


class DataFilterNode:
    def __init__(self):
        rospy.init_node("data_filter", anonymous=True)
        self.pub_filtered = rospy.Publisher("/data/filtered", FilteredData, queue_size=10)

        rospy.Subscriber("/sensor/temperature", Float64, self.temp_callback)
        rospy.Subscriber("/sensor/humidity", Float64, self.hum_callback)

        self.temperature = None
        self.humidity = None

    def temp_callback(self, msg):
        self.temperature = msg.data
        self.publish_filtered()

    def hum_callback(self, msg):
        self.humidity = msg.data
        self.publish_filtered()

    def publish_filtered(self):
        if self.temperature is not None and self.humidity is not None:
            filtered = FilteredData()
            filtered.temperature = self.temperature
            filtered.humidity = self.humidity
            filtered.valid = True
            filtered.timestamp = str(rospy.Time.now())
            self.pub_filtered.publish(filtered)
            rospy.loginfo("DataFilter: Published FilteredData")


def main():
    global node
    node = DataFilterNode()
    rospy.spin()


if __name__ == "__main__":
    main()
