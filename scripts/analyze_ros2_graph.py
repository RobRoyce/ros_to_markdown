#!/usr/bin/env python3

"""ROS2 graph analysis script for visualizing node relationships."""

import os
import subprocess


def main():
    """Source ROS2 environment and run analyzer."""
    # Source ROS2 setup
    ros_setup = "/opt/ros/humble/setup.bash"
    if not os.path.exists(ros_setup):
        print(f"Error: ROS2 setup file not found at {ros_setup}")
        return

    # Create command that sources ROS2 and runs our analyzer
    cmd = f"bash -c 'source {ros_setup} && python3 scripts/analyze_ros_graph.py'"

    # Run the command
    process = subprocess.run(cmd, shell=True)
    return process.returncode


if __name__ == "__main__":
    main()
