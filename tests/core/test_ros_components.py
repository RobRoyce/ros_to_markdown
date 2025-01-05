from ros_to_markdown.models.ros_components import (
    ConnectionType,
    ROSGraph,
    ROSGraphEdge,
    ROSNode,
    ROSParameter,
    ROSTopic,
    ROSVersion,
)

import pytest


@pytest.fixture
def sample_graph() -> ROSGraph:
    """Create a sample ROS graph for testing."""
    nodes = {
        "/talker": ROSNode(
            name="/talker",
            package="demo_nodes_py",
            publishers=["/chatter"],
        ),
        "/listener": ROSNode(
            name="/listener",
            package="demo_nodes_py",
            subscribers=["/chatter"],
        ),
        "/parameter_server": ROSNode(
            name="/parameter_server",
            package="demo_nodes_py",
            parameters=[ROSParameter(name="max_delay", type="double", value=1.0)],
        ),
    }

    edges = {
        ROSGraphEdge(
            source="/talker",
            target="/listener",
            connection_type="topic",
            topic_name="/chatter",
            message_type="std_msgs/String",
        )
    }

    topics = {
        "/chatter": ROSTopic(
            name="/chatter",
            type="std_msgs/String",
            publishers=["/talker"],
            subscribers=["/listener"],
        )
    }

    return ROSGraph(
        nodes=nodes,
        edges=edges,
        topics=topics,
        services={},
        actions={},
        parameters={},
        version=ROSVersion.ROS2,
        distro="humble",
    )


@pytest.fixture
def cyclic_graph() -> ROSGraph:
    """Create a ROS graph with cyclic references for testing."""
    nodes = {
        "/controller": ROSNode(
            name="/controller",
            package="control_nodes",
            publishers=["/cmd"],
            subscribers=["/feedback"],
        ),
        "/robot": ROSNode(
            name="/robot", package="robot_nodes", publishers=["/feedback"], subscribers=["/cmd"]
        ),
    }

    edges = {
        ROSGraphEdge(
            source="/controller",
            target="/robot",
            connection_type="topic",
            topic_name="/cmd",
            message_type="geometry_msgs/Twist",
        ),
        ROSGraphEdge(
            source="/robot",
            target="/controller",
            connection_type="topic",
            topic_name="/feedback",
            message_type="sensor_msgs/JointState",
        ),
    }

    topics = {
        "/cmd": ROSTopic(
            name="/cmd",
            type="geometry_msgs/Twist",
            publishers=["/controller"],
            subscribers=["/robot"],
        ),
        "/feedback": ROSTopic(
            name="/feedback",
            type="sensor_msgs/JointState",
            publishers=["/robot"],
            subscribers=["/controller"],
        ),
    }

    return ROSGraph(
        nodes=nodes,
        edges=edges,
        topics=topics,
        services={},
        actions={},
        parameters={},
        version=ROSVersion.ROS2,
        distro="humble",
    )


def test_to_mermaid(sample_graph: ROSGraph) -> None:
    """Test conversion of ROS graph to Mermaid format."""
    mermaid = sample_graph.to_mermaid()

    # Check basic structure
    assert mermaid.startswith("graph LR")

    # Check nodes are included
    assert '_talker["/talker"]' in mermaid
    assert '_listener["/listener"]' in mermaid

    # Check edge is included with correct format - update to match new format
    # Empty label when show_message_types=False
    assert '_talker -- "" --> _listener:::topicEdge' in mermaid


def test_get_node_connections(sample_graph: ROSGraph) -> None:
    """Test retrieving connections for a specific node."""
    # Test publisher connections
    talker_connections = sample_graph.get_node_connections("/talker")
    assert len(talker_connections["publishers"]) == 1
    assert talker_connections["publishers"][0].topic_name == "/chatter"
    assert not talker_connections["subscribers"]

    # Test subscriber connections
    listener_connections = sample_graph.get_node_connections("/listener")
    assert len(listener_connections["subscribers"]) == 1
    assert listener_connections["subscribers"][0].topic_name == "/chatter"
    assert not listener_connections["publishers"]


def test_get_subgraph(sample_graph: ROSGraph) -> None:
    """Test extracting a subgraph with specific nodes."""
    # Get subgraph with just talker and listener
    subgraph = sample_graph.get_subgraph(["/talker", "/listener"])

    # Check nodes
    assert len(subgraph.nodes) == 2
    assert "/talker" in subgraph.nodes
    assert "/listener" in subgraph.nodes
    assert "/parameter_server" not in subgraph.nodes

    # Check edges
    assert len(subgraph.edges) == 1
    edge = next(iter(subgraph.edges))
    assert edge.source == "/talker"
    assert edge.target == "/listener"

    # Check topics
    assert len(subgraph.topics) == 1
    assert "/chatter" in subgraph.topics


def test_get_subgraph_empty(sample_graph: ROSGraph) -> None:
    """Test subgraph with non-existent nodes."""
    subgraph = sample_graph.get_subgraph(["/nonexistent"])
    assert not subgraph.nodes
    assert not subgraph.edges
    assert not subgraph.topics


def test_get_node_connections_nonexistent(sample_graph: ROSGraph) -> None:
    """Test getting connections for non-existent node."""
    connections = sample_graph.get_node_connections("/nonexistent")
    assert not any(connections.values())  # All lists should be empty


def test_complex_connections(sample_graph: ROSGraph) -> None:
    """Test graph with multiple connection types."""
    # Add a service edge
    service_edge = ROSGraphEdge(
        source="/parameter_server",
        target="/talker",
        connection_type="service",
        topic_name="/set_parameters",
        message_type="rcl_interfaces/SetParameters",
    )
    sample_graph.edges.add(service_edge)

    # Test connections
    server_connections = sample_graph.get_node_connections("/parameter_server")
    assert len(server_connections["services"]) == 1
    assert server_connections["services"][0].topic_name == "/set_parameters"

    talker_connections = sample_graph.get_node_connections("/talker")
    assert len(talker_connections["clients"]) == 1
    assert talker_connections["clients"][0].topic_name == "/set_parameters"


def test_cyclic_references(cyclic_graph: ROSGraph) -> None:
    """Test handling of cyclic references in the graph."""
    # Test that we can detect cycles
    controller_conns = cyclic_graph.get_node_connections("/controller")
    robot_conns = cyclic_graph.get_node_connections("/robot")

    # Verify bidirectional communication
    assert len(controller_conns["publishers"]) == 1
    assert len(controller_conns["subscribers"]) == 1
    assert len(robot_conns["publishers"]) == 1
    assert len(robot_conns["subscribers"]) == 1

    # Verify the cycle is properly represented in Mermaid - update to match new format
    mermaid = cyclic_graph.to_mermaid()
    assert '_controller -- "" --> _robot:::topicEdge' in mermaid


def test_find_cycles(cyclic_graph: ROSGraph) -> None:
    """Test cycle detection in the graph."""
    cycles = cyclic_graph.find_cycles()

    # Should find one cycle
    assert len(cycles) == 1
    cycle = cycles[0]

    # Cycle should contain both nodes
    assert len(cycle) == 2
    assert "/controller" in cycle
    assert "/robot" in cycle


def test_analyze_cycle(cyclic_graph: ROSGraph) -> None:
    """Test analysis of cycle connections."""
    cycles = cyclic_graph.find_cycles()
    cycle = cycles[0]

    analysis = cyclic_graph.analyze_cycle(cycle)

    # Should find two topic connections
    assert len(analysis["topics"]) == 2
    assert len(analysis["services"]) == 0
    assert len(analysis["actions"]) == 0

    # Verify the specific topics
    topic_names = {edge.topic_name for edge in analysis["topics"]}
    assert topic_names == {"/cmd", "/feedback"}


def test_deadlock_risk(cyclic_graph: ROSGraph) -> None:
    """Test deadlock risk detection."""
    cycles = cyclic_graph.find_cycles()
    cycle = cycles[0]

    # Initially we have different topics (/cmd and /feedback) - should be safe
    has_risk, reason = cyclic_graph.has_deadlock_risk(cycle)
    assert not has_risk
    assert "No immediate deadlock risks detected" in reason

    # Add a bidirectional dependency using the same topic
    risky_edge1 = ROSGraphEdge(
        source="/controller",
        target="/robot",
        connection_type="topic",
        topic_name="/state",
        message_type="std_msgs/String",
    )
    risky_edge2 = ROSGraphEdge(
        source="/robot",
        target="/controller",
        connection_type="topic",
        topic_name="/state",  # Same topic name creates the risk
        message_type="std_msgs/String",
    )
    cyclic_graph.edges.add(risky_edge1)
    cyclic_graph.edges.add(risky_edge2)

    has_risk, reason = cyclic_graph.has_deadlock_risk(cycle)
    assert has_risk
    assert "Bidirectional dependency on topic /state" in reason


def test_mermaid_cycle_highlighting(cyclic_graph: ROSGraph) -> None:
    """Test that cycles are properly highlighted in Mermaid output."""
    mermaid = cyclic_graph.to_mermaid(highlight_cycles=True)

    # Check for style definitions
    assert "classDef cycleNode" in mermaid
    assert "classDef cycleEdge" in mermaid
    assert "classDef riskEdge" in mermaid

    # Update to check for cycleEdge style instead of cycleNode
    assert '_robot -- "" --> _controller:::cycleEdge' in mermaid
    assert '_controller -- "" --> _robot:::cycleEdge' in mermaid

    # Initially edges should be marked as normal cycles
    assert ":::cycleEdge" in mermaid

    # Add a bidirectional pattern using same topic
    risky_edge1 = ROSGraphEdge(
        source="/controller",
        target="/robot",
        connection_type="topic",
        topic_name="/state",
        message_type="std_msgs/String",
    )
    risky_edge2 = ROSGraphEdge(
        source="/robot",
        target="/controller",
        connection_type="topic",
        topic_name="/state",
        message_type="std_msgs/String",
    )
    cyclic_graph.edges.add(risky_edge1)
    cyclic_graph.edges.add(risky_edge2)

    mermaid = cyclic_graph.to_mermaid(highlight_cycles=True)
    # These edges should also be marked as cycle edges
    assert '_controller -- "" --> _robot:::cycleEdge' in mermaid
    assert '_robot -- "" --> _controller:::cycleEdge' in mermaid


def test_markdown_generation(cyclic_graph: ROSGraph) -> None:
    """Test complete markdown documentation generation."""
    # Add a risky pattern first
    risky_edge1 = ROSGraphEdge(
        source="/controller",
        target="/robot",
        connection_type="topic",
        topic_name="/state",
        message_type="std_msgs/String",
    )
    risky_edge2 = ROSGraphEdge(
        source="/robot",
        target="/controller",
        connection_type="topic",
        topic_name="/state",
        message_type="std_msgs/String",
    )
    cyclic_graph.edges.add(risky_edge1)
    cyclic_graph.edges.add(risky_edge2)

    markdown = cyclic_graph.to_markdown()

    # Check sections
    assert "# ROS System Documentation" in markdown
    assert "## System Information" in markdown
    assert "## System Architecture" in markdown
    assert "## Communication Patterns" in markdown

    # Check cycle documentation
    assert "Pattern 1:" in markdown
    assert "Topic Communications:" in markdown
    assert "/cmd" in markdown
    assert "/feedback" in markdown
    assert "/state" in markdown

    # Check risk assessment
    assert "Risk Assessment" in markdown
    assert "Warning" in markdown  # Should warn about bidirectional dependency on /state


def test_markdown_no_cycles(sample_graph: ROSGraph) -> None:
    """Test markdown generation for graph without cycles."""
    markdown = sample_graph.to_markdown()

    # Should not have communication patterns section
    assert "## Communication Patterns" not in markdown

    # Should have basic sections
    assert "## System Information" in markdown
    assert "## System Architecture" in markdown


def test_connection_type_enum() -> None:
    """Test ConnectionType enum functionality."""
    # Test values
    assert ConnectionType.TOPIC.value == "topic"
    assert ConnectionType.SERVICE.value == "service"
    assert ConnectionType.ACTION.value == "action"

    # Test values() classmethod
    expected_values = {"topic", "service", "action"}
    assert ConnectionType.values() == expected_values


def test_edge_validation() -> None:
    """Test ROSGraphEdge validation."""
    # Test valid edge
    valid_edge = ROSGraphEdge(
        source="/node1",
        target="/node2",
        connection_type="topic",
        topic_name="/test_topic",
        message_type="std_msgs/String",
    )
    assert valid_edge  # Should not raise any exceptions

    # Test invalid connection type
    with pytest.raises(ValueError, match="Invalid connection_type"):
        ROSGraphEdge(
            source="/node1",
            target="/node2",
            connection_type="invalid_type",
            topic_name="/test_topic",
            message_type="std_msgs/String",
        )

    # Test empty strings
    for field in ["source", "target", "topic_name", "message_type"]:
        with pytest.raises(
            ValueError, match=f"{field.replace('_', ' ')} must be a non-empty string"
        ):
            kwargs = {
                "source": "/node1",
                "target": "/node2",
                "connection_type": "topic",
                "topic_name": "/test_topic",
                "message_type": "std_msgs/String",
                field: "",  # Empty string should raise error
            }
            ROSGraphEdge(**kwargs)

    # Test invalid frequency
    with pytest.raises(ValueError, match="Frequency must be a number"):
        ROSGraphEdge(
            source="/node1",
            target="/node2",
            connection_type="topic",
            topic_name="/test_topic",
            message_type="std_msgs/String",
            frequency="invalid",
        )


def test_qos_profile_validation() -> None:
    """Test QoS profile validation in ROSGraphEdge."""
    # Test valid QoS profile
    valid_qos = {"reliability": "RELIABLE", "durability": "VOLATILE", "history": "KEEP_LAST"}
    edge = ROSGraphEdge(
        source="/node1",
        target="/node2",
        connection_type="topic",
        topic_name="/test_topic",
        message_type="std_msgs/String",
        qos_profile=valid_qos,
    )
    assert edge  # Should not raise any exceptions

    # Test missing required fields
    invalid_qos = {
        "reliability": "RELIABLE",
        # Missing durability and history
    }
    with pytest.raises(ValueError, match="QoS profile missing required fields"):
        ROSGraphEdge(
            source="/node1",
            target="/node2",
            connection_type="topic",
            topic_name="/test_topic",
            message_type="std_msgs/String",
            qos_profile=invalid_qos,
        )


def test_mermaid_visualization_styles() -> None:
    """Test enhanced Mermaid visualization styles."""
    # Create a simple non-cyclic graph to test base styles
    nodes = {
        "/publisher": ROSNode(name="/publisher", package="test_pkg"),
        "/subscriber": ROSNode(name="/subscriber", package="test_pkg"),
        "/server": ROSNode(name="/server", package="test_pkg"),
        "/client": ROSNode(name="/client", package="test_pkg"),
    }

    edges = {
        ROSGraphEdge(  # Topic edge
            source="/publisher",
            target="/subscriber",
            connection_type="topic",
            topic_name="/test_topic",
            message_type="std_msgs/String",
        ),
        ROSGraphEdge(  # Service edge
            source="/server",
            target="/client",
            connection_type="service",
            topic_name="/test_service",
            message_type="test_msgs/Srv",
        ),
    }

    graph = ROSGraph(
        nodes=nodes,
        edges=edges,
        topics={},
        services={},
        actions={},
        parameters={},
        version=ROSVersion.ROS2,
        distro="humble",
    )

    mermaid = graph.to_mermaid(highlight_cycles=True)

    # Check style definitions
    assert "classDef default" in mermaid
    assert "classDef cycleNode" in mermaid
    assert "classDef riskNode" in mermaid
    assert "classDef topicEdge" in mermaid
    assert "classDef serviceEdge" in mermaid
    assert "classDef actionEdge" in mermaid
    assert "classDef cycleEdge" in mermaid
    assert "classDef riskEdge" in mermaid

    # Check edge styling
    assert ":::topicEdge" in mermaid  # Topic edges should have topic style
    assert ":::serviceEdge" in mermaid  # Service edges should have service style

    # Now test cycle highlighting with a cyclic graph
    cyclic_edges = {
        ROSGraphEdge(
            source="/publisher",
            target="/subscriber",
            connection_type="topic",
            topic_name="/cycle_topic",
            message_type="std_msgs/String",
        ),
        ROSGraphEdge(
            source="/subscriber",
            target="/publisher",
            connection_type="topic",
            topic_name="/feedback",
            message_type="std_msgs/String",
        ),
    }

    cyclic_graph = ROSGraph(
        nodes={k: v for k, v in nodes.items() if k in {"/publisher", "/subscriber"}},
        edges=cyclic_edges,
        topics={},
        services={},
        actions={},
        parameters={},
        version=ROSVersion.ROS2,
        distro="humble",
    )

    cyclic_mermaid = cyclic_graph.to_mermaid(highlight_cycles=True)

    # Update to check for cycleEdge instead of cycleNode
    assert ":::cycleEdge" in cyclic_mermaid
    assert '_publisher -- "" --> _subscriber:::cycleEdge' in cyclic_mermaid
    assert '_subscriber -- "" --> _publisher:::cycleEdge' in cyclic_mermaid


def test_mermaid_message_type_display(sample_graph: ROSGraph) -> None:
    """Test message type display in Mermaid visualization."""
    # Test without message types
    mermaid_without = sample_graph.to_mermaid(show_message_types=False)
    assert "std_msgs/String" not in mermaid_without
    assert '_talker -- "" --> _listener:::topicEdge' in mermaid_without

    # Test with message types
    mermaid_with = sample_graph.to_mermaid(show_message_types=True)
    assert "|std_msgs/String|" in mermaid_with


def test_risky_cycle_visualization() -> None:
    """Test visualization of risky cycles with services/actions."""
    # Create a graph with a service cycle
    nodes = {
        "/service_a": ROSNode(name="/service_a", package="test_pkg"),
        "/service_b": ROSNode(name="/service_b", package="test_pkg"),
    }

    edges = {
        ROSGraphEdge(
            source="/service_a",
            target="/service_b",
            connection_type="service",
            topic_name="/srv1",
            message_type="test_msgs/Srv",
        ),
        ROSGraphEdge(
            source="/service_b",
            target="/service_a",
            connection_type="service",
            topic_name="/srv2",
            message_type="test_msgs/Srv",
        ),
    }

    graph = ROSGraph(
        nodes=nodes,
        edges=edges,
        topics={},
        services={},
        actions={},
        parameters={},
        version=ROSVersion.ROS2,
        distro="humble",
    )

    mermaid = graph.to_mermaid(highlight_cycles=True)

    # Update to check for cycleEdge instead of riskNode
    assert ":::cycleEdge" in mermaid
    assert '_service_a -- "" --> _service_b:::cycleEdge' in mermaid
    assert '_service_b -- "" --> _service_a:::cycleEdge' in mermaid
