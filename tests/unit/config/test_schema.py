"""Test configuration schema validation."""

from ros_to_markdown.config.schema import Config, RosDistro, RosVersion, RuntimeConfig

from pydantic import ValidationError
import pytest


def test_runtime_config_validation() -> None:
    """Test runtime configuration validation."""
    # Test defaults
    config = RuntimeConfig()
    assert config.namespace == "/"
    assert config.node_filter is None
    assert config.topic_filter is None

    # Test custom values
    config = RuntimeConfig(
        namespace="/test", node_filter=["node1", "node2"], topic_filter=["topic1"]
    )
    assert config.namespace == "/test"

    if not config.node_filter:
        raise ValueError("Node filter is empty")

    if not config.topic_filter:
        raise ValueError("Topic filter is empty")

    assert len(config.node_filter) == 2
    assert len(config.topic_filter) == 1


def test_ros_version_enum() -> None:
    """Test ROS version enumeration."""
    assert RosVersion.ROS1 == 1
    assert RosVersion.ROS2 == 2

    # Test conversion from int
    assert RosVersion(1) == RosVersion.ROS1
    assert RosVersion(2) == RosVersion.ROS2

    # Test invalid version
    with pytest.raises(ValueError):
        RosVersion(3)


def test_ros_distro_enum() -> None:
    """Test ROS distribution enumeration."""
    assert RosDistro.HUMBLE.value == "humble"
    assert RosDistro.IRON.value == "iron"
    assert RosDistro.JAZZY.value == "jazzy"
    assert RosDistro.NOETIC.value == "noetic"

    # Test conversion from string
    assert RosDistro("humble") == RosDistro.HUMBLE

    # Test invalid distro
    with pytest.raises(ValueError):
        RosDistro("invalid")


def test_config_validation() -> None:
    """Test main configuration validation."""
    # Test minimal config with required fields
    config = Config(ros_version=RosVersion.ROS2, ros_distro=RosDistro.HUMBLE)
    assert config.ros_version == RosVersion.ROS2
    assert config.ros_distro == RosDistro.HUMBLE
    assert config.output_dir is None
    assert config.debug is False
    assert config.perspective is None

    # Test full config
    config = Config(
        ros_version=RosVersion.ROS2,
        ros_distro=RosDistro.HUMBLE,
        output_dir="/tmp/output",
        debug=True,
        perspective="custom",
        runtime=RuntimeConfig(namespace="/test"),
    )
    assert config.output_dir == "/tmp/output"
    assert config.debug is True
    assert config.perspective == "custom"
    assert config.runtime.namespace == "/test"

    # Test missing required fields
    with pytest.raises(ValidationError):
        Config()  # type: ignore[call-arg]  # Intentionally missing required fields


def test_config_validation_errors() -> None:
    """Test configuration validation errors."""
    # Test invalid ROS version/distro combination
    with pytest.raises(ValidationError):
        Config(
            ros_version=RosVersion.ROS2,
            ros_distro=RosDistro.NOETIC,  # ROS1 distro with ROS2
        )
