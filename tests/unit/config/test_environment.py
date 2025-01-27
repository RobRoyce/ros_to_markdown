"""Test environment configuration loading."""
from ros_to_markdown.config.environment import get_env_config
from ros_to_markdown.config.schema import Config, RosDistro, RosVersion

import pytest


def test_get_env_config_empty(monkeypatch):
    """Test environment config with no variables set."""
    # Explicitly unset ROS environment variables
    monkeypatch.delenv("ROS_VERSION", raising=False)
    monkeypatch.delenv("ROS_DISTRO", raising=False)

    config = get_env_config()
    assert config is None


def test_get_env_config_basic(monkeypatch):
    """Test basic environment configuration."""
    monkeypatch.setenv("ROS_VERSION", "2")
    monkeypatch.setenv("ROS_DISTRO", "humble")
    monkeypatch.setenv("ROS_TO_MARKDOWN_DEBUG", "false")

    config = get_env_config()
    assert isinstance(config, Config)
    assert config.ros_version == RosVersion.ROS2
    assert config.ros_distro == RosDistro.HUMBLE


def test_get_env_config_full(monkeypatch):
    """Test full environment configuration."""
    monkeypatch.setenv("ROS_VERSION", "2")
    monkeypatch.setenv("ROS_DISTRO", "humble")
    monkeypatch.setenv("ROS_TO_MARKDOWN_OUTPUT_DIR", "/test/output")
    monkeypatch.setenv("ROS_TO_MARKDOWN_DEBUG", "true")
    monkeypatch.setenv("ROS_TO_MARKDOWN_PERSPECTIVE", "custom")
    monkeypatch.setenv("ROS_TO_MARKDOWN_NAMESPACE", "/test")

    config = get_env_config()
    assert config.output_dir == "/test/output"
    assert config.debug is True
    assert config.perspective == "custom"
    assert config.runtime.namespace == "/test"


def test_get_env_config_invalid(monkeypatch):
    """Test invalid environment configuration."""
    monkeypatch.setenv("ROS_VERSION", "invalid")
    monkeypatch.setenv("ROS_DISTRO", "humble")
    monkeypatch.setenv("ROS_TO_MARKDOWN_DEBUG", "false")

    with pytest.raises(ValueError, match="Invalid environment configuration"):
        get_env_config()
