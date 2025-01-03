#!/usr/bin/env python3

"""Launch script for ROS turtlesim demo nodes."""

import signal
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node


def main():
    """Launch turtlesim and teleop using ROS2 native API."""
    # Initialize ROS context
    rclpy.init()

    # Create and start turtlesim node
    turtlesim_node = Node("turtlesim")

    # Start turtlesim using subprocess (but keep it quiet)
    turtlesim = subprocess.Popen(
        [
            "ros2",
            "run",
            "turtlesim",
            "turtlesim_node",
            "--ros-args",
            "-r",
            "__node:=turtlesim",
            "-r",
            "__ns:=/",
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )

    # Give turtlesim time to start
    time.sleep(2)

    # Start teleop using subprocess to maintain terminal control
    teleop = subprocess.Popen(
        [
            "ros2",
            "run",
            "turtlesim",
            "turtle_teleop_key",
            "--ros-args",
            "-r",
            "__node:=teleop_turtle",
            "-r",
            "__ns:=/",
        ],
        stdin=sys.stdin,
        stdout=sys.stdout,
        stderr=subprocess.PIPE,
    )

    def cleanup(signum=None, frame=None):
        """Clean up processes on exit."""
        teleop.terminate()
        turtlesim.terminate()
        turtlesim_node.destroy_node()
        rclpy.shutdown()
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
