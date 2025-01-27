from ros_to_markdown.analysis.interfaces import SystemAnalyzer
from ros_to_markdown.config.schema import Config, RosVersion
from ros_to_markdown.logging import get_logger

logger = get_logger(__name__)


class AnalyzerInitError(Exception):
    """Raised when analyzer initialization fails."""

    pass


def get_analyzer(config: Config) -> SystemAnalyzer:
    """
    Dynamically import and create appropriate analyzer based on ROS version.
    """
    logger.info("Getting analyzer", config=config)

    try:
        if config.ros_version == RosVersion.ROS2:
            logger.info("Using ROS2 analyzer")
            from ros_to_markdown.analysis.ros2 import Ros2SystemAnalyzer

            return Ros2SystemAnalyzer()

        if config.ros_version == RosVersion.ROS1:
            logger.info("Using ROS1 analyzer")
            from ros_to_markdown.analysis.ros1 import Ros1SystemAnalyzer

            return Ros1SystemAnalyzer()

    except ImportError as e:
        logger.error("Failed to import ROS modules", error=str(e))
        raise AnalyzerInitError(
            f"Failed to import ROS modules. Is ROS {config.ros_version} installed?"
        ) from e
    except Exception as e:
        logger.error("Failed to initialize analyzer", error=str(e))
        raise AnalyzerInitError(f"Failed to initialize ROS {config.ros_version} analyzer") from e

    raise NotImplementedError(f"ROS version {config.ros_version} not supported")
