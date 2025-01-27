"""Common test fixtures and configuration."""
from datetime import datetime

from ros_to_markdown.analysis.interfaces import NodeInfo, SystemSnapshot, TopicInfo

import pytest


@pytest.fixture
def mock_system_snapshot() -> SystemSnapshot:
    """Create a mock system snapshot for testing."""
    return SystemSnapshot(
        timestamp=datetime(2025, 1, 1, 12, 0, 0),
        nodes={
            "/test_node": NodeInfo(
                name="test_node",
                namespace="/",
                publishers=["/test_topic"],
                subscribers=["/other_topic"]
            ),
            "/other_node": NodeInfo(
                name="other_node",
                namespace="/",
                publishers=["/other_topic"],
                subscribers=["/test_topic"]
            )
        },
        topics={
            "/test_topic": TopicInfo(
                name="/test_topic",
                type="std_msgs/msg/String",
                publishers=["/test_node"],
                subscribers=["/other_node"]
            ),
            "/other_topic": TopicInfo(
                name="/other_topic",
                type="std_msgs/msg/Int32",
                publishers=["/other_node"],
                subscribers=["/test_node"]
            )
        }
    )


@pytest.fixture
def basic_perspective_dict():
    """Create a basic perspective configuration for testing."""
    return {
        "name": "test_perspective",
        "version": "1.0",
        "description": "Test perspective",
        "pipeline": {
            "collect": [{
                "type": "system_snapshot",
                "name": "collect_system_state",
                "inputs": ["analyzer"],
                "output": "system_state"
            }],
            "transform": [{
                "type": "graph_builder",
                "name": "build_system_graph",
                "inputs": ["system_state"],
                "output": "system_graph"
            }],
            "render": [{
                "type": "markdown",
                "name": "generate_overview",
                "inputs": ["system_graph"],
                "output": "overview_doc"
            }]
        },
        "compatibility": {
            "min_version": "1.0",
            "max_version": "2.0",
            "deprecated_features": []
        }
    }
