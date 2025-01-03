#!/usr/bin/env python3

"""Launch script for ROS turtlesim demo nodes."""

import signal
import subprocess
import sys
import time

from ros_to_markdown.core.ros_detector import ROSDetector
from ros_to_markdown.models.ros_components import ROSVersion


def ensure_ros_core():
    """Ensure ROS core/master is running."""
    try:
        # Try to run roscore as a subprocess
        roscore = subprocess.Popen(
            ["roscore"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        # Give roscore time to start
        time.sleep(3)
        return roscore
    except FileNotFoundError:
        print("Error: roscore command not found. Is ROS1 installed correctly?")
        return None


def main():
    """Launch turtlesim and teleop using version-appropriate ROS API."""
    ros_version = ROSDetector.detect_ros_version()
    roscore_process = None

    if ros_version == ROSVersion.ROS2:
        import rclpy
        from rclpy.node import Node

        # Initialize ROS context
        rclpy.init()
        node = Node("turtlesim")

        # ROS2-specific launch commands
        turtlesim_cmd = [
            "ros2",
            "run",
            "turtlesim",
            "turtlesim_node",
            "--ros-args",
            "-r",
            "__node:=turtlesim",
            "-r",
            "__ns:=/",
        ]
        teleop_cmd = [
            "ros2",
            "run",
            "turtlesim",
            "turtle_teleop_key",
            "--ros-args",
            "-r",
            "__node:=teleop_turtle",
            "-r",
            "__ns:=/",
        ]

    elif ros_version == ROSVersion.ROS1:
        import rospy

        # Ensure roscore is running for ROS1
        roscore_process = ensure_ros_core()
        if not roscore_process:
            return

        # Initialize ROS node
        rospy.init_node("turtlesim_launcher", anonymous=True)

        # ROS1-specific launch commands
        turtlesim_cmd = ["rosrun", "turtlesim", "turtlesim_node", "__name:=turtlesim"]
        teleop_cmd = ["rosrun", "turtlesim", "turtle_teleop_key", "__name:=teleop_turtle"]

    else:
        print("Error: Unknown ROS version")
        return

    # Start turtlesim using subprocess (but keep it quiet)
    turtlesim = subprocess.Popen(
        turtlesim_cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )

    # Give turtlesim time to start
    time.sleep(2)

    # Start teleop using subprocess to maintain terminal control
    teleop = subprocess.Popen(
        teleop_cmd,
        stdin=sys.stdin,
        stdout=sys.stdout,
        stderr=subprocess.PIPE,
    )

    def cleanup(signum=None, frame=None):
        """Clean up processes on exit."""
        teleop.terminate()
        turtlesim.terminate()

        if roscore_process:
            roscore_process.terminate()

        if ros_version == ROSVersion.ROS2:
            node.destroy_node()
            rclpy.shutdown()
        else:
            rospy.signal_shutdown("Shutting down turtlesim launcher")

        sys.exit(0)

    # Handle interrupts gracefully
    signal.signal(signal.SIGINT, cleanup)
    signal.signal(signal.SIGTERM, cleanup)

    try:
        teleop.wait()
    finally:
        cleanup()


if __name__ == "__main__":
    main()
