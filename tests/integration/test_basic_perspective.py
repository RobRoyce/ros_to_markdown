"""Integration tests for basic perspective."""

from pathlib import Path
from typing import Any

from ros_to_markdown.perspectives.engine import PerspectiveEngine
from ros_to_markdown.perspectives.loader import load_perspective

import pytest


@pytest.fixture
def mock_ros_analyzer(mock_system_snapshot: Any) -> Any:
    """Create a mock ROS analyzer."""

    class MockAnalyzer:
        async def get_snapshot(self) -> Any:
            return mock_system_snapshot

    return MockAnalyzer()


async def test_basic_perspective_execution(mock_ros_analyzer: Any, tmp_path: Path) -> None:
    """Test end-to-end execution of basic perspective."""
    # Load perspective
    perspective = load_perspective("basic")
    engine = PerspectiveEngine(perspective)

    # Execute pipeline
    result = await engine.execute({"analyzer": mock_ros_analyzer})

    # Verify output
    assert "overview_doc" in result
    markdown = result["overview_doc"]

    # Basic content checks
    assert "# ROS System Overview" in markdown
    assert "Generated at:" in markdown
    assert "```mermaid" in markdown

    # Write output and verify file
    output_file = tmp_path / "basic.md"
    output_file.write_text(markdown)
    assert output_file.exists()
