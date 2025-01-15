#!/usr/bin/env python3

from data_processing.msg import FilteredData
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class DataFilter(Node):
    def __init__(self):
        super().__init__('data_filter')
        self.temp_sub = self.create_subscription(Float64, '/sensor/temperature', self.temp_callback, 10)
        self.hum_sub = self.create_subscription(Float64, '/sensor/humidity', self.hum_callback, 10)
        self.pub_filtered = self.create_publisher(FilteredData, '/data/filtered', 10)

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
            fmsg = FilteredData()
            fmsg.temperature = float(self.temperature)
            fmsg.humidity = float(self.humidity)
            fmsg.valid = True
            fmsg.timestamp = str(self.get_clock().now().to_msg())
            self.pub_filtered.publish(fmsg)
            self.get_logger().info("Published FilteredData")

def main(args=None):
    rclpy.init(args=args)
    node = DataFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
