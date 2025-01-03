"""ROS2 graph analysis implementation."""

import importlib.util
import os
import time
from typing import Dict, List, Optional

from ..models.ros_components import (
    ROSGraph,
    ROSGraphEdge,
    ROSNode,
    ROSService,
    ROSTopic,
    ROSVersion,
)
from ..utils.graph_filters import GraphFilter
from .base import GraphAnalyzer

# Try importing each ROS2 package separately to identify which one fails
try:
    import rclpy

    HAVE_RCLPY = True
except ImportError as e:
    print(f"Failed to import rclpy: {e}")
    HAVE_RCLPY = False

try:
    from rclpy.node import Node

    HAVE_NODE = True
except ImportError as e:
    print(f"Failed to import rclpy.node: {e}")
    HAVE_NODE = False

try:
    # Use find_spec to check for package availability
    HAVE_ROS2NODE = importlib.util.find_spec("ros2node") is not None
except ImportError as e:
    print(f"Failed to check for ros2node: {e}")
    HAVE_ROS2NODE = False

try:
    from ros2topic.api import get_topic_names_and_types

    HAVE_ROS2TOPIC = True
except ImportError as e:
    print(f"Failed to import ros2topic.api: {e}")
    HAVE_ROS2TOPIC = False

try:
    from ros2service.api import get_service_names_and_types

    HAVE_ROS2SERVICE = True
except ImportError as e:
    print(f"Failed to import ros2service.api: {e}")
    HAVE_ROS2SERVICE = False

HAS_ROS2 = all([HAVE_RCLPY, HAVE_NODE, HAVE_ROS2NODE, HAVE_ROS2TOPIC, HAVE_ROS2SERVICE])


class ROS2Analyzer(GraphAnalyzer):
    """Analyzer for ROS2 computation graphs."""

    def __init__(self):
        """Initialize the analyzer."""
        if not HAS_ROS2:
            raise ImportError("ROS2 packages not available")
        self.node = None

    def get_nodes(self) -> List[str]:
        """Get all nodes in the ROS2 system."""
        try:
            nodes = set()

            # Add debug logging
            print(f"ROS_DOMAIN_ID={os.getenv('ROS_DOMAIN_ID')}")

            # First try direct discovery
            node_names_and_ns = self.node.get_node_names_and_namespaces()
            print(f"Direct discovery returned: {node_names_and_ns}")

            for name, ns in node_names_and_ns:
                full_name = f"{ns.strip('/')}/{name}" if ns != "/" else f"/{name}"
                print(f"Found node via direct discovery: {full_name}")
                nodes.add(full_name)

            # Then get nodes from topics
            publishers = {}  # topic -> list of publisher nodes
            subscribers = {}  # topic -> list of subscriber nodes

            # Get all topics first
            for topic_name, _ in self.node.get_topic_names_and_types():
                # Get publisher info
                pub_info = self.node.get_publishers_info_by_topic(topic_name)
                publishers[topic_name] = []
                for info in pub_info:
                    if hasattr(info, "node_name") and info.node_name:
                        node_name = info.node_name
                        if not node_name.startswith("/"):
                            node_name = f"/{node_name}"
                        publishers[topic_name].append(node_name)
                        nodes.add(node_name)

                # Get subscriber info
                sub_info = self.node.get_subscriptions_info_by_topic(topic_name)
                subscribers[topic_name] = []
                for info in sub_info:
                    if hasattr(info, "node_name") and info.node_name:
                        node_name = info.node_name
                        if not node_name.startswith("/"):
                            node_name = f"/{node_name}"
                        subscribers[topic_name].append(node_name)
                        nodes.add(node_name)

            print("\nFound publishers:", [[k, v] for k, v in publishers.items() if v])
            print("Found subscribers:", [[k, v] for k, v in subscribers.items() if v])

            # Add our own node
            our_name = self.node.get_fully_qualified_name()
            if not our_name.startswith("/"):
                our_name = f"/{our_name}"
            nodes.add(our_name)

            # Standardize node names
            return [GraphFilter.standardize_node_name(name) for name in nodes]
        except Exception as e:
            print(f"Error getting nodes: {e}")
            return []

    def get_message_type(self, topic_name: str) -> str:
        """Get the message type for a ROS2 topic."""
        try:
            topics = dict(get_topic_names_and_types(node=self.node))
            return topics.get(topic_name, ["unknown"])[0]
        except Exception:
            return "unknown"

    def analyze(self) -> Optional[ROSGraph]:
        """Analyze a running ROS2 system."""
        try:
            if not rclpy.ok():
                rclpy.init()

            # Create node with unique name to avoid conflicts
            node_name = f"graph_analyzer_{int(time.time() * 1000)}"
            self.node = Node(node_name)

            # Spin a few times to allow discovery
            for _ in range(3):
                rclpy.spin_once(self.node, timeout_sec=0.5)

            # Get all nodes first
            node_names = self.get_nodes()
            nodes = {name: ROSNode(name=name, package="unknown") for name in node_names}

            # Get topics and their types
            topics_and_types = get_topic_names_and_types(node=self.node)
            services_and_types = get_service_names_and_types(node=self.node)

            edges = set()
            topics: Dict[str, ROSTopic] = {}
            ros_services: Dict[str, ROSService] = {}

            # Process topics and create edges
            for topic_name, type_list in topics_and_types:
                msg_type = type_list[0] if type_list else "unknown"

                # Get publisher and subscriber info
                pub_info = self.node.get_publishers_info_by_topic(topic_name)
                sub_info = self.node.get_subscriptions_info_by_topic(topic_name)

                pub_nodes = []
                for info in pub_info:
                    if hasattr(info, "node_name") and info.node_name:
                        node_name = info.node_name
                        if not node_name.startswith("/"):
                            node_name = f"/{node_name}"
                        pub_nodes.append(node_name)

                sub_nodes = []
                for info in sub_info:
                    if hasattr(info, "node_name") and info.node_name:
                        node_name = info.node_name
                        if not node_name.startswith("/"):
                            node_name = f"/{node_name}"
                        sub_nodes.append(node_name)

                # Create topic
                if pub_nodes or sub_nodes:  # Only add topics with publishers or subscribers
                    topics[topic_name] = ROSTopic(
                        name=topic_name,
                        type=msg_type,
                        publishers=pub_nodes,
                        subscribers=sub_nodes,
                        description=None,
                    )

                    # Create edges between publishers and subscribers
                    for pub_node in pub_nodes:
                        for sub_node in sub_nodes:
                            # Skip edges involving the analyzer node
                            if ("graph_analyzer" not in pub_node and
                                "graph_analyzer" not in sub_node):
                                edge = ROSGraphEdge(
                                    source=pub_node,
                                    target=sub_node,
                                    connection_type="topic",
                                    topic_name=topic_name,
                                    message_type=msg_type,
                                )
                                edges.add(edge)

            # Process services
            for service_name, type_list in services_and_types:
                srv_type = type_list[0] if type_list else "unknown"
                # Only add non-system services
                if not any(service_name.startswith(prefix) for prefix in ['/parameter_events', '/rosout']):
                    ros_services[service_name] = ROSService(
                        name=service_name,
                        type=srv_type,
                        node=None,  # We'll improve service node detection later
                        description=None,
                    )

            # Filter out system topics
            filtered_topics = {
                name: topic for name, topic in topics.items()
                if not any(name.startswith(prefix) for prefix in ['/parameter_events', '/rosout'])
            }

            # Create graph
            graph = ROSGraph(
                nodes=nodes,
                edges=edges,
                topics=filtered_topics,
                services=ros_services,
                actions={},
                parameters={},
                version=ROSVersion.ROS2,
                distro=os.getenv("ROS_DISTRO", "unknown"),
            )

            return GraphFilter.clean_graph(graph)

        finally:
            if self.node:
                self.node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
