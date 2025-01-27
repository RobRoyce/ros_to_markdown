"""Test transform stage functionality."""
from datetime import datetime

from ros_to_markdown.analysis.interfaces import NodeInfo, SystemSnapshot
from ros_to_markdown.perspectives.stages.transform import GraphBuilderStage


async def test_graph_builder(mock_system_snapshot):
    """Test graph builder transformation."""
    stage = GraphBuilderStage()
    result = await stage.execute({"system_state": mock_system_snapshot}, {})

    # Verify basic structure
    assert "nodes" in result
    assert "topics" in result
    assert "connections" in result
    assert "timestamp" in result

    # Verify node normalization
    nodes = {node["name"] for node in result["nodes"]}
    assert "/test_node" in nodes
    assert "/other_node" in nodes

    # Verify connections
    connections = result["connections"]
    assert any(c["from"] == "/test_node" and c["to"] == "/test_topic" for c in connections)
    assert any(c["from"] == "/test_topic" and c["to"] == "/other_node" for c in connections)


async def test_graph_builder_filtering(mock_system_snapshot):
    """Test graph builder with filtering options."""
    stage = GraphBuilderStage()

    # Test nodes-only
    result = await stage.execute(
        {"system_state": mock_system_snapshot},
        {"include_nodes": True, "include_topics": False}
    )
    assert len(result["nodes"]) > 0
    assert len(result["topics"]) == 0

    # Test topics-only
    result = await stage.execute(
        {"system_state": mock_system_snapshot},
        {"include_nodes": False, "include_topics": True}
    )
    assert len(result["nodes"]) == 0
    assert len(result["topics"]) > 0


async def test_graph_builder_name_normalization(mock_system_snapshot):
    """Test node/topic name normalization."""
    stage = GraphBuilderStage()

    # Modify snapshot with various name formats
    mock_system_snapshot.nodes["//double/slashed"] = NodeInfo(
        name="double/slashed",
        namespace="//",
        publishers=[],
        subscribers=[]
    )

    result = await stage.execute({"system_state": mock_system_snapshot}, {})
    nodes = {node["name"] for node in result["nodes"]}

    assert "/double/slashed" in nodes  # Should normalize double slashes
    assert not any(name.startswith("//") for name in nodes)


async def test_graph_builder_empty_snapshot():
    """Test graph builder with empty snapshot."""
    stage = GraphBuilderStage()
    empty_snapshot = SystemSnapshot(
        timestamp=datetime(2025, 1, 1),
        nodes={},
        topics={}
    )

    result = await stage.execute({"system_state": empty_snapshot}, {})
    assert len(result["nodes"]) == 0
    assert len(result["topics"]) == 0
    assert len(result["connections"]) == 0
