from enum import Enum, IntEnum
from typing import Optional, List
from pydantic import BaseModel, Field, model_validator
import os

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
    """Root configuration schema."""
    ros_version: RosVersion = Field(
        default=RosVersion.ROS2,
        description="ROS version to use"
    )
    ros_distro: RosDistro = Field(
        default=RosDistro.HUMBLE,
        description="ROS distribution to use"
    )
    runtime: RuntimeConfig = Field(
        default_factory=RuntimeConfig,
        description="Runtime analysis configuration"
    )
    output_dir: str = Field(
        default="./docs",
        description="Directory to write markdown output"
    )
    debug: bool = Field(
        default=False,
        description="Enable debug logging"
    )
