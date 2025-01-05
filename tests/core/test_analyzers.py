"""Tests for ROS graph analyzers."""

from typing import Generator
from unittest.mock import MagicMock, patch

from ros_to_markdown.analyzers.base import GraphAnalyzer
from ros_to_markdown.core.ros_detector import ROSDetector
from ros_to_markdown.models.ros_components import ROSGraph, ROSVersion

import pytest

try:
    from ros_to_markdown.analyzers.ros1 import ROS1Analyzer
except ModuleNotFoundError:
    ROS1Analyzer = None

try:
    from ros_to_markdown.analyzers.ros2 import ROS2Analyzer
except ModuleNotFoundError:
    ROS2Analyzer = None


class DummyAnalyzer(GraphAnalyzer):
    """Dummy analyzer for testing the base class."""

    def analyze(self) -> ROSGraph:
        """Dummy analyze method."""
        return ROSGraph(
            nodes={},
            edges=set(),
            topics={},
            services={},
            actions={},
            parameters={},
            version="unknown",
            distro="unknown",
        )

    def get_message_type(self, topic_name: str) -> str:
        """Dummy get_message_type method."""
        return "dummy_type"


def test_graph_analyzer_abstract_methods() -> None:
    """Test that the abstract methods are enforced."""
    with pytest.raises(TypeError):
        GraphAnalyzer()  # Cannot instantiate abstract class

    # Test that the dummy analyzer can be instantiated
    analyzer = DummyAnalyzer()
    assert analyzer.analyze() is not None
    assert analyzer.get_message_type("test_topic") == "dummy_type"


@pytest.fixture
def mock_rospy() -> Generator:
    """Mock rospy module for testing."""
    with patch("ros_to_markdown.analyzers.ros1.rospy") as mock_rospy:
        mock_rospy.get_master.return_value.getSystemState.return_value = (
            1,
            "",
            (
                [
                    ("/topic1", ["/node1"]),
                    ("/topic2", ["/node2", "/node3"]),
                ],  # Publishers
                [
                    ("/topic1", ["/node2"]),
                    ("/topic2", ["/node4"]),
                ],  # Subscribers
                [
                    ("/service1", ["/node1"]),
                    ("/service2", ["/node3"]),
                ],  # Services
            ),
        )
        yield mock_rospy


@pytest.fixture
def mock_rostopic() -> Generator:
    """Mock rostopic module for testing."""
    with patch("ros_to_markdown.analyzers.ros1.get_topic_type") as mock_get_topic_type:
        mock_get_topic_type.side_effect = lambda topic: {
            "/topic1": ("std_msgs/String", "", ""),
            "/topic2": ("sensor_msgs/JointState", "", ""),
        }.get(topic, ("unknown", "", ""))
        yield mock_get_topic_type


@pytest.mark.skipif(
    ROSDetector.detect_ros_version() != ROSVersion.ROS1 or ROS1Analyzer is None,
    reason="Test requires ROS1 environment",
)
def test_ros1_analyzer_analyze(mock_rospy: MagicMock, mock_rostopic: MagicMock) -> None:
    """Test the ROS1 analyzer."""
    analyzer = ROS1Analyzer()
    graph = analyzer.analyze()

    assert graph is not None
    assert graph.version == ROSVersion.ROS1
    assert graph.distro == "noetic"

    # Check nodes
    assert len(graph.nodes) == 4
    assert "/node1" in graph.nodes
    assert "/node2" in graph.nodes
    assert "/node3" in graph.nodes
    assert "/node4" in graph.nodes

    # Check edges
    assert len(graph.edges) == 3
    edge_types = {edge.topic_name for edge in graph.edges}
    assert "/topic1" in edge_types
    assert "/topic2" in edge_types

    # Check topics
    assert len(graph.topics) == 2
    assert "/topic1" in graph.topics
    assert "/topic2" in graph.topics
    assert graph.topics["/topic1"].type == "std_msgs/String"
    assert graph.topics["/topic2"].type == "sensor_msgs/JointState"

    # Check services
    assert len(graph.services) == 2
    assert "/service1" in graph.services
    assert "/service2" in graph.services


@pytest.mark.skipif(
    ROSDetector.detect_ros_version() != ROSVersion.ROS1 or ROS1Analyzer is None,
    reason="Test requires ROS1 environment",
)
def test_ros1_analyzer_no_nodes(mock_rospy: MagicMock) -> None:
    """Test ROS1 analyzer when no nodes are found."""
    mock_rospy.get_master.return_value.getSystemState.return_value = (
        1,
        "",
        ([], [], []),
    )
    analyzer = ROS1Analyzer()
    graph = analyzer.analyze()
    assert graph is None


@pytest.mark.skipif(
    ROSDetector.detect_ros_version() != ROSVersion.ROS1 or ROS1Analyzer is None,
    reason="Test requires ROS1 environment",
)
def test_ros1_analyzer_get_message_type(mock_rostopic: MagicMock) -> None:
    """Test getting message type for a topic."""
    analyzer = ROS1Analyzer()
    assert analyzer.get_message_type("/topic1") == "std_msgs/String"
    assert analyzer.get_message_type("/topic2") == "sensor_msgs/JointState"
    assert analyzer.get_message_type("/unknown_topic") == "unknown"


@pytest.mark.skipif(
    ROSDetector.detect_ros_version() != ROSVersion.ROS1 or ROS1Analyzer is None,
    reason="Test requires ROS1 environment",
)
def test_ros1_analyzer_error_handling(mock_rospy: MagicMock) -> None:
    """Test error handling in ROS1 analyzer."""
    mock_rospy.get_master.return_value.getSystemState.side_effect = Exception("Test Exception")
    analyzer = ROS1Analyzer()
    graph = analyzer.analyze()
    assert graph is None
