from enum import Enum, IntEnum
from typing import List, Optional

from pydantic import BaseModel, Field


class RosVersion(IntEnum):
    """ROS distribution version."""
    ROS1 = 1
    ROS2 = 2


class RosDistro(str, Enum):
    """ROS distribution."""
    HUMBLE = "humble"
    NOETIC = "noetic"
    IRON = "iron"
    JAZZY = "jazzy"


class RuntimeConfig(BaseModel):
    """Configuration for runtime analysis."""
    namespace: Optional[str] = Field(
        default="/",
        description="ROS namespace to analyze"
    )
    node_filter: Optional[List[str]] = Field(
        default=None,
        description="List of node name patterns to include"
    )
    topic_filter: Optional[List[str]] = Field(
        default=None,
        description="List of topic name patterns to include"
    )


class Config(BaseModel):
    """Configuration for ros-to-markdown."""
    ros_distro: RosDistro
    ros_version: RosVersion
    output_dir: Optional[str] = None
    debug: bool = False
    perspective: Optional[str] = None  # Name of perspective to use
    runtime: RuntimeConfig = Field(default_factory=RuntimeConfig)
