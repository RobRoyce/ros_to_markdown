import os
import subprocess
import time
from typing import Optional, List
from contextlib import contextmanager

from ros_to_markdown.core.ros_detector import ROSDetector
from ros_to_markdown.models.ros_components import ROSVersion


class ROSTestEnvironment:
    """Helper class to manage ROS test environments."""
    
    def __init__(self):
        """Initialize the ROS test environment."""
        self.ros_version, self.ros_distro = ROSDetector.detect_ros_version()
        self.processes: List[subprocess.Popen] = []

    def setup(self):
        """Set up the ROS test environment."""
        if self.ros_version == ROSVersion.ROS1:
            self._setup_ros1()
        elif self.ros_version == ROSVersion.ROS2:
            self._setup_ros2()
        else:
            raise RuntimeError("Unknown ROS version")

    def teardown(self):
        """Clean up the ROS test environment."""
        for process in self.processes:
            process.terminate()
            process.wait()
        self.processes.clear()

    def _setup_ros1(self):
        """Set up ROS1 test environment with basic nodes."""
        # Start roscore if not running
        if not ROSDetector.is_ros_initialized():
            roscore = subprocess.Popen(
                ["roscore"],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
            self.processes.append(roscore)
            time.sleep(2)  # Wait for roscore to initialize

        # Start turtlesim node
        turtlesim = subprocess.Popen(
            ["rosrun", "turtlesim", "turtlesim_node"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE
        )
        self.processes.append(turtlesim)

        # Start turtle teleop node
        teleop = subprocess.Popen(
            ["rosrun", "turtlesim", "turtle_teleop_key"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE
        )
        self.processes.append(teleop)

    def _setup_ros2(self):
        """Set up ROS2 test environment with basic nodes."""
        # Start turtlesim node
        turtlesim = subprocess.Popen(
            ["ros2", "run", "turtlesim", "turtlesim_node"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE
        )
        self.processes.append(turtlesim)

        # Start turtle teleop node
        teleop = subprocess.Popen(
            ["ros2", "run", "turtlesim", "turtle_teleop_key"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE
        )
        self.processes.append(teleop)

    @contextmanager
    def running_env(self):
        """Context manager for running a temporary ROS environment.
        
        Usage:
            with ROSTestEnvironment().running_env():
                # Run your tests here
        """
        try:
            self.setup()
            yield self
        finally:
            self.teardown() 

"""Helper functions for ROS test environment setup."""

import os
import time
import subprocess
from ros_to_markdown.core.ros_detector import ROSDetector
from ros_to_markdown.models.ros_components import ROSVersion


def launch_turtlesim():
    """Launch turtlesim node for either ROS1 or ROS2.
    
    Returns:
        tuple: (core_process, turtlesim_process) - Processes that need to be terminated
    """
    ros_version = ROSDetector.detect_ros_version()
    
    if ros_version == ROSVersion.ROS1:
        # Start roscore first
        core_process = subprocess.Popen(
            ["roscore"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE
        )
        
        # Give roscore time to start
        time.sleep(2)
        
        # Launch turtlesim
        turtlesim_process = subprocess.Popen(
            ["rosrun", "turtlesim", "turtlesim_node"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            env=dict(os.environ, DISPLAY=os.environ.get("DISPLAY", ":0"))
        )
        
        return core_process, turtlesim_process
        
    else:  # ROS2
        # No need for core process in ROS2
        turtlesim_process = subprocess.Popen(
            ["ros2", "run", "turtlesim", "turtlesim_node"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            env=dict(os.environ, DISPLAY=os.environ.get("DISPLAY", ":0"))
        )
        
        return None, turtlesim_process


def verify_turtlesim_running():
    """Verify that turtlesim node is running.
    
    Returns:
        bool: True if turtlesim node is running
    """
    ros_version = ROSDetector.detect_ros_version()
    
    try:
        if ros_version == ROSVersion.ROS1:
            result = subprocess.run(
                ["rosnode", "list"],
                capture_output=True,
                text=True,
                check=True
            )
            return "/turtlesim" in result.stdout
        else:
            result = subprocess.run(
                ["ros2", "node", "list"],
                capture_output=True,
                text=True,
                check=True
            )
            return "/turtlesim" in result.stdout
    except subprocess.CalledProcessError:
        return False 