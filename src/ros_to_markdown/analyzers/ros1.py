"""ROS1-specific graph analyzer."""

from typing import Optional

import rospy
from rostopic import get_topic_type

from ..models.ros_components import (
    ROSGraph,
    ROSGraphEdge,
    ROSNode,
    ROSService,
    ROSTopic,
    ROSVersion,
)
from .base import GraphAnalyzer


class ROS1Analyzer(GraphAnalyzer):
    """Analyzer for ROS1 computation graphs."""

    def get_message_type(self, topic_name: str) -> str:
        """Get the message type for a ROS1 topic."""
        try:
            msg_type, _, _ = get_topic_type(topic_name)
            return msg_type if msg_type else "unknown"
        except Exception:
            return "unknown"

    def analyze(self) -> Optional[ROSGraph]:
        """Analyze a running ROS1 system."""
        # Initialize ROS node
        rospy.init_node("graph_analyzer", anonymous=True)

        try:
            code, msg, state = rospy.get_master().getSystemState()
            if code != 1:
                print(f"Failed to get system state: {msg}")
                return None

            publishers, subscribers, services = state

        except Exception as e:
            print(f"Error getting system state: {e}")
            return None

        # Create data structures
        nodes = {}
        edges = set()
        topics = {}
        ros_services = {}

        # Process publishers and subscribers
        for topic_info in publishers:
            topic_name = topic_info[0]
            pub_nodes = topic_info[1]

            # Find subscribers
            sub_nodes = []
            for sub_info in subscribers:
                if sub_info[0] == topic_name:
                    sub_nodes.extend(sub_info[1])

            # Get message type
            message_type = self.get_message_type(topic_name)

            # Create topic
            topics[topic_name] = ROSTopic(
                name=topic_name,
                type=message_type,
                publishers=pub_nodes,
                subscribers=sub_nodes,
                description=None,
            )

            # Add nodes and edges
            for pub_node in pub_nodes:
                if pub_node not in nodes:
                    nodes[pub_node] = ROSNode(name=pub_node, package="unknown")

                for sub_node in sub_nodes:
                    if sub_node not in nodes:
                        nodes[sub_node] = ROSNode(name=sub_node, package="unknown")

                    edge = ROSGraphEdge(
                        source=pub_node,
                        target=sub_node,
                        connection_type="topic",
                        topic_name=topic_name,
                        message_type=message_type,
                    )
                    edges.add(edge)

        # Process services
        for service_info in services:
            service_name = service_info[0]
            provider_nodes = service_info[1]

            ros_services[service_name] = ROSService(
                name=service_name,
                type="unknown",  # Would need rosservice info
                node=provider_nodes[0] if provider_nodes else None,
                description=None,
            )

            # Add service nodes
            for node in provider_nodes:
                if node not in nodes:
                    nodes[node] = ROSNode(name=node, package="unknown")

        if not nodes:
            print("No nodes found in the ROS graph")
            return None

        return ROSGraph(
            nodes=nodes,
            edges=edges,
            topics=topics,
            services=ros_services,
            actions={},  # ROS1 doesn't have actions in the same way
            parameters={},
            version=ROSVersion.ROS1,
            distro="noetic",
        )
