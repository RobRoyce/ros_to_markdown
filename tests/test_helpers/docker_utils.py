"""Utilities for Docker-related test functionality."""

import os
from typing import Optional


def is_running_in_docker() -> bool:
    """Check if the current process is running inside a Docker container.

    Returns:
        bool: True if running in Docker, False otherwise
    """
    # Check for .dockerenv file
    if os.path.exists("/.dockerenv"):
        return True

    # Check cgroup (backup method)
    try:
        with open("/proc/1/cgroup") as f:
            return "docker" in f.read()
    except (OSError, FileNotFoundError):
        return False


def get_ros_workspace() -> Optional[str]:
    """Get the path to the ROS workspace in the Docker container.

    Returns:
        Optional[str]: Path to the ROS workspace if found, None otherwise
    """
    if not is_running_in_docker():
        return None

    # Common workspace locations in our Docker containers
    workspace_paths = [
        os.path.expanduser("~/ws"),  # User's home directory (preferred)
        "/home/ros/ws",  # Explicit home path
        "/opt/ros/ws",  # System-wide location (fallback)
        "/opt/ros/turtlesim_ws",  # Turtlesim demo workspace
        os.environ.get("ROS_WORKSPACE", ""),  # Environment-specified workspace
    ]

    # Debug workspace detection
    print(f"Checking workspace paths: {workspace_paths}")
    for path in workspace_paths:
        if path and os.path.exists(path):
            print(f"Found workspace at: {path}")
            return path
        elif path:
            print(f"Path does not exist: {path}")

    return None
