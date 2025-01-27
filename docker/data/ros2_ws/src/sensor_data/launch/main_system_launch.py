#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    # sensor_data
    ld.add_action(
        Node(
            package="sensor_data",
            executable="temperature_publisher",
            name="temperature_publisher",
            output="screen",
        )
    )
    ld.add_action(
        Node(
            package="sensor_data",
            executable="humidity_publisher",
            name="humidity_publisher",
            output="screen",
        )
    )

    # data_processing
    ld.add_action(
        Node(
            package="data_processing", executable="data_filter", name="data_filter", output="screen"
        )
    )
    ld.add_action(
        Node(
            package="data_processing",
            executable="data_processor",
            name="data_processor",
            output="screen",
        )
    )

    # environment_integration
    ld.add_action(
        Node(
            package="environment_integration",
            executable="environment_builder",
            name="environment_builder",
            output="screen",
        )
    )

    # robot_control
    ld.add_action(
        Node(package="robot_control", executable="move_robot", name="move_robot", output="screen")
    )

    # action_server
    ld.add_action(
        Node(
            package="action_server", executable="move_to_goal", name="move_to_goal", output="screen"
        )
    )

    # tricky_scenarios
    ld.add_action(
        Node(
            package="tricky_scenarios",
            executable="conflicting_publishers",
            name="conflicting_publishers",
            output="screen",
        )
    )
    ld.add_action(
        Node(
            package="tricky_scenarios",
            executable="delayed_response",
            name="delayed_response",
            output="screen",
        )
    )

    # visualization
    ld.add_action(
        Node(
            package="visualization",
            executable="visualize_data",
            name="visualize_data",
            output="screen",
        )
    )

    # user_interface
    ld.add_action(
        Node(
            package="user_interface",
            executable="command_listener",
            name="command_listener",
            output="screen",
        )
    )

    return ld
