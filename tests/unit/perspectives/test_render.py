"""Test render stage functionality."""

from pathlib import Path
from typing import Any

from ros_to_markdown.perspectives.stages.render import MarkdownRendererStage

import jinja2
import pytest


@pytest.fixture
def mock_graph_data() -> Any:
    """Create mock graph data for testing."""
    return {
        "nodes": [
            {
                "name": "/test_node",
                "namespace": "/",
                "publishers": ["/test_topic"],
                "subscribers": [],
            }
        ],
        "topics": [
            {
                "name": "/test_topic",
                "type": "std_msgs/msg/String",
                "publishers": ["/test_node"],
                "subscribers": [],
            }
        ],
        "connections": [{"from": "/test_node", "to": "/test_topic", "type": "publishes"}],
        "timestamp": "2025-01-01 12:00:00",
    }


async def test_markdown_renderer(mock_graph_data: Any) -> None:
    """Test markdown rendering."""
    stage = MarkdownRendererStage()
    result = await stage.execute({"system_graph": mock_graph_data}, {})

    # Verify basic content
    assert "# ROS System Overview" in result
    assert "Generated at: 2025-01-01 12:00:00" in result
    assert "/test_node" in result
    assert "/test_topic" in result
    assert "std_msgs/msg/String" in result

    # Verify Mermaid graph
    assert "```mermaid" in result
    assert "graph TD" in result


async def test_markdown_renderer_empty_graph() -> None:
    """Test markdown rendering with empty graph."""
    stage = MarkdownRendererStage()
    empty_graph = {"nodes": [], "topics": [], "connections": [], "timestamp": "2025-01-01 12:00:00"}

    result = await stage.execute({"system_graph": empty_graph}, {})
    assert "No connections found" in result


async def test_markdown_renderer_custom_template(tmp_path: Path) -> None:
    """Test markdown rendering with custom template."""
    # Create test template
    template_dir = tmp_path / "templates"
    template_dir.mkdir()
    test_template = template_dir / "test.md.j2"
    test_template.write_text("Test: {{ graph.nodes | length }} nodes")

    stage = MarkdownRendererStage()
    stage.env = jinja2.Environment(loader=jinja2.FileSystemLoader(str(template_dir)))

    result = await stage.execute(
        {"system_graph": {"nodes": [{"name": "test"}]}}, {"template": "test.md.j2"}
    )
    assert "Test: 1 nodes" in result


async def test_markdown_renderer_invalid_template() -> None:
    """Test markdown rendering with invalid template."""
    stage = MarkdownRendererStage()

    with pytest.raises(jinja2.TemplateNotFound):
        await stage.execute({"system_graph": {}}, {"template": "nonexistent.md.j2"})
