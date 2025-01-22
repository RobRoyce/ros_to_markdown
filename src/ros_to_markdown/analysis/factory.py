from typing import Optional
import importlib
from .interfaces import SystemAnalyzer
from ..config.schema import Config, RosVersion

def get_analyzer(config: Config) -> SystemAnalyzer:
    """
    Dynamically import and create appropriate analyzer based on ROS version.
    """
    if config.ros_version == RosVersion.ROS2:
        # Dynamically import ROS2 analyzer to avoid rclpy import issues
        ros2_module = importlib.import_module(".ros2", package="ros_to_markdown.analysis")
        return ros2_module.Ros2SystemAnalyzer()
    else:
        # Future: Import ROS1 analyzer here
        raise NotImplementedError("ROS1 support not yet implemented") 