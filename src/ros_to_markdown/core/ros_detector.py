"""ROS version detection and environment inspection utilities.

This module provides functionality to detect and validate ROS installations,
determine ROS versions, and inspect the ROS environment.
"""

import os
import subprocess
from importlib import import_module
from typing import Optional

from ..models.ros_components import ROSVersion


class ROSDetector:
    """ROS version and distribution detection utilities."""

    @staticmethod
    def detect_ros_version() -> ROSVersion:
        """Detect which version of ROS is currently active.

        Returns:
            ROSVersion: Enum indicating which ROS version is detected

        The detection is performed by checking environment variables and trying
        to import ROS-specific packages.
        """
        # Check ROS2 first
        if os.environ.get("ROS_DISTRO") in [
            "foxy",
            "galactic",
            "humble",
            "iron",
            "jazzy",
            "rolling",
        ]:
            try:
                import_module("rclpy")
                return ROSVersion.ROS2
            except ImportError:
                pass

        # Check ROS1
        if os.environ.get("ROS_DISTRO") in ["noetic", "melodic"]:
            try:
                import_module("rospy")
                return ROSVersion.ROS1
            except ImportError:
                pass

        # If no clear indicators, try additional checks
        return ROSDetector._fallback_detection()

    @staticmethod
    def _fallback_detection() -> ROSVersion:
        """Perform additional checks to determine ROS version.

        Returns:
            ROSVersion: Enum indicating which ROS version is detected
        """
        # Try to run rosversion (ROS1) command
        try:
            subprocess.run(["rosversion", "-d"], capture_output=True, check=True)
            return ROSVersion.ROS1
        except (subprocess.SubprocessError, FileNotFoundError):
            pass

        # Try to run ros2 command
        try:
            subprocess.run(["ros2", "--help"], capture_output=True, check=True)
            return ROSVersion.ROS2
        except (subprocess.SubprocessError, FileNotFoundError):
            return ROSVersion.UNKNOWN

    @staticmethod
    def get_ros_distro() -> Optional[str]:
        """Get the ROS distribution name.

        Returns:
            Optional[str]: Name of the ROS distribution if found, None otherwise
        """
        return os.environ.get("ROS_DISTRO")
