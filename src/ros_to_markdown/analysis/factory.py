from ..config.schema import Config, RosVersion
from .interfaces import SystemAnalyzer


def get_analyzer(config: Config) -> SystemAnalyzer:
    """
    Dynamically import and create appropriate analyzer based on ROS version.
    """
    if config.ros_version == RosVersion.ROS2:
        # Dynamically import ROS2 analyzer to avoid rclpy import issues
        from ros_to_markdown.analysis.ros2 import Ros2SystemAnalyzer

        return Ros2SystemAnalyzer()
    # Future: Import ROS1 analyzer here
    raise NotImplementedError("ROS1 support not yet implemented")
