#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sensor_data',
            executable='temperature_sensor',
            name='temperature_sensor',
            output='screen'
        ),
        Node(
            package='sensor_data',
            executable='humidity_sensor',
            name='humidity_sensor',
            output='screen'
        )
    ]) 