"""ROS version detection and environment inspection utilities.

This module provides functionality to detect and validate ROS installations,
determine ROS versions, and inspect the ROS environment.
"""

from importlib import import_module
import socket
import subprocess

from ..models.ros_components import ROSDistro, ROSVersion


class ROSDetector:
    """ROS version and distribution detection utilities."""

    @staticmethod
    def detect_ros_distro() -> ROSDistro:
        """Detect the ROS distribution.

        Returns:
            ROSDistro: Enum indicating the ROS distribution
        """
        return ROSDistro.from_env()

    @staticmethod
    def detect_ros_version() -> ROSVersion:
        """Detect which version of ROS is currently active.

        Returns:
            ROSVersion: Enum indicating which ROS version is detected
        """
        ros_distro = ROSDetector.detect_ros_distro()

        if ros_distro in [
            ROSDistro.FURY,
            ROSDistro.GALACTIC,
            ROSDistro.HUMBLE,
            ROSDistro.IRON,
            ROSDistro.JAZZY,
            ROSDistro.ROLLING,
        ]:
            return ROSDetector._check_ros2()
        elif ros_distro in [ROSDistro.NOETIC, ROSDistro.MELODIC]:
            return ROSDetector._check_ros1()

        print("Unknown ROS distribution.")
        return ROSVersion.UNKNOWN

    @staticmethod
    def _check_ros2() -> ROSVersion:
        """Check for ROS2 installation.

        Returns:
            ROSVersion: Enum indicating ROS2 version or UNKNOWN if not found.
        """
        try:
            import_module("rclpy")
            return ROSVersion.ROS2
        except ImportError:
            return ROSVersion.UNKNOWN

    @staticmethod
    def _check_ros1() -> ROSVersion:
        """Check for ROS1 installation.

        Returns:
            ROSVersion: Enum indicating ROS1 version or UNKNOWN if not found.
        """
        try:
            import_module("rospy")
            return ROSVersion.ROS1
        except ImportError:
            return ROSVersion.UNKNOWN

    @staticmethod
    def _fallback_detection() -> ROSVersion:
        """Perform additional checks to determine ROS version.

        Returns:
            ROSVersion: Enum indicating detected version or UNKNOWN.
        """
        if ROSDetector._run_command(["rosversion", "-d"]):
            return ROSVersion.ROS1
        if ROSDetector._run_command(["ros2", "-h"]):
            return ROSVersion.ROS2

        return ROSVersion.UNKNOWN

    @staticmethod
    def _run_command(command: list) -> bool:
        """Run a command and return True if successful.

        Args:
            command (list): The command to run.

        Returns:
            bool: True if the command was successful, False otherwise.
        """
        try:
            subprocess.run(command, capture_output=True, check=True)
            return True
        except (subprocess.SubprocessError, FileNotFoundError) as e:
            print(f"Command '{' '.join(command)}' failed: {e}")
            return False

    @staticmethod
    def is_ros_initialized() -> bool:
        """Check if the ROS master is running.

        Returns:
            bool: True if the ROS master is running, False otherwise.
        """
        try:
            socket.create_connection(("localhost", 11311), timeout=1)
            print("ROS master is running.")
            return True
        except (socket.timeout, ConnectionRefusedError):
            print("ROS master is not running.")
            return False
