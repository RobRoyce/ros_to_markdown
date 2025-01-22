"""Configuration schema definitions."""
from enum import Enum, IntEnum
from typing import List, Optional

from pydantic import BaseModel, Field, model_validator


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
    
    @classmethod
    def ros1_distros(cls) -> set:
        """Get ROS1 distributions."""
        return {cls.NOETIC}
    
    @classmethod
    def ros2_distros(cls) -> set:
        """Get ROS2 distributions."""
        return {cls.HUMBLE, cls.IRON, cls.JAZZY}


class RuntimeConfig(BaseModel):
    """Configuration for runtime analysis."""
    namespace: str = Field(
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
    ros_version: RosVersion
    ros_distro: RosDistro
    output_dir: Optional[str] = None
    debug: bool = False
    perspective: Optional[str] = None
    runtime: RuntimeConfig = Field(default_factory=RuntimeConfig)
    
    @model_validator(mode='after')
    def validate_ros_version_distro(self) -> 'Config':
        """Validate ROS version and distribution compatibility."""
        if self.ros_version == RosVersion.ROS1 and self.ros_distro not in RosDistro.ros1_distros():
            raise ValueError(f"Invalid ROS1 distribution: {self.ros_distro}")
        if self.ros_version == RosVersion.ROS2 and self.ros_distro not in RosDistro.ros2_distros():
            raise ValueError(f"Invalid ROS2 distribution: {self.ros_distro}")
        return self
