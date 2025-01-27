"""Environment configuration loading."""

import os
from typing import Optional

from .schema import Config, RosDistro, RosVersion


def get_env_config() -> Optional[Config]:
    """Load configuration from environment variables."""
    # Required ROS environment variables
    ros_version = os.getenv("ROS_VERSION")
    ros_distro = os.getenv("ROS_DISTRO")

    if not ros_version or not ros_distro:
        return None

    try:
        config = Config(
            ros_version=RosVersion(int(ros_version)),
            ros_distro=RosDistro(ros_distro),
            output_dir=os.getenv("ROS_TO_MARKDOWN_OUTPUT_DIR"),
            debug=os.getenv("ROS_TO_MARKDOWN_DEBUG", "").lower() == "true",
            perspective=os.getenv("ROS_TO_MARKDOWN_PERSPECTIVE"),
        )

        # Update runtime config if namespace specified
        if namespace := os.getenv("ROS_TO_MARKDOWN_NAMESPACE"):
            config.runtime.namespace = namespace

        return config

    except (ValueError, TypeError) as e:
        raise ValueError(f"Invalid environment configuration: {e}") from e
