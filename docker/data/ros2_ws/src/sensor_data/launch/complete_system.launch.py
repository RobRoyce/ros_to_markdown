#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            # Only launch the working sensor nodes
            Node(package="sensor_data", executable="temperature_sensor", name="temperature_sensor"),
            Node(package="sensor_data", executable="humidity_sensor", name="humidity_sensor"),
            # Comment out problematic nodes for now
            # Node(
            #     package='data_processing',
            #     executable='data_filter',
            #     name='data_filter'
            # ),
            # Node(
            #     package='data_processing',
            #     executable='data_processor',
            #     name='data_processor'
            # ),
            # Node(
            #     package='environment_integration',
            #     executable='environment_builder',
            #     name='environment_builder'
            # ),
        ]
    )
