"""Test CLI functionality."""

import os
from pathlib import Path
from typing import Any

import pytest

# Skip CLI tests in ROS1 environment
pytestmark = pytest.mark.skipif(
    os.getenv("ROS_VERSION") == "1",
    reason="CLI tests require Click which is not available in ROS1 environment",
)

try:
    from ros_to_markdown.cli import cli, get_config

    from click.testing import CliRunner
except ImportError:
    CliRunner = None
    cli = None
    get_config = None


@pytest.fixture
def mock_analyzer(monkeypatch: Any, mock_system_snapshot: Any) -> Any:
    """Mock ROS analyzer."""

    class MockAnalyzer:
        async def get_snapshot(self) -> Any:
            return mock_system_snapshot

    def mock_get_analyzer(*args: Any, **kwargs: Any) -> Any:
        return MockAnalyzer()

    monkeypatch.setattr("ros_to_markdown.cli.get_analyzer", mock_get_analyzer)


def test_get_config_defaults(monkeypatch: Any) -> None:
    """Test default configuration."""
    # Clear environment variables to test true defaults
    monkeypatch.delenv("ROS_VERSION", raising=False)
    monkeypatch.delenv("ROS_DISTRO", raising=False)

    config = get_config()
    assert config.ros_version.value == 2  # Default to ROS2
    # Don't test specific distro as it depends on environment
    assert config.output_dir is None
    assert config.debug is False


def test_get_config_cli_overrides() -> None:
    """Test CLI options override defaults."""
    config = get_config({"output_dir": "/test/output", "debug": True, "perspective": "custom"})
    assert config.output_dir == "/test/output"
    assert config.debug is True
    assert config.perspective == "custom"


def test_cli_help() -> None:
    """Test CLI help output."""
    runner = CliRunner()
    result = runner.invoke(cli, ["--help"])
    assert result.exit_code == 0
    assert "ROS to Markdown" in result.output


def test_cli_runtime_basic() -> None:
    """Test basic runtime command."""
    runner = CliRunner()
    with runner.isolated_filesystem():
        result = runner.invoke(cli, ["runtime"])
        assert result.exit_code == 0


def test_cli_runtime_with_options(mock_analyzer: Any) -> None:
    """Test runtime command with options."""
    runner = CliRunner()
    with runner.isolated_filesystem():
        # Create output directory
        Path("test_output").mkdir(parents=True)

        result = runner.invoke(
            cli,
            [
                "--output-dir",
                "test_output",
                "--debug",
                "--perspective",
                "basic",
                "runtime",
                "--namespace",
                "/test",
                "--node-filter",
                "node1",
                "--topic-filter",
                "topic1",
            ],
        )
        assert result.exit_code == 0
        assert Path("test_output").exists()


def test_cli_invalid_options() -> None:
    """Test CLI with invalid options."""
    runner = CliRunner()
    result = runner.invoke(cli, ["--invalid-option"])
    assert result.exit_code != 0
    assert "Error" in result.output
