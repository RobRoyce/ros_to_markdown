"""Utilities for filtering and standardizing ROS computation graphs."""

import re
from typing import ClassVar, Dict, List, Set

from ..models.ros_components import ROSGraph, ROSGraphEdge, ROSNode, ROSTopic


class GraphFilter:
    """Filter and standardize ROS computation graphs."""

    # Common system topics/nodes to filter out
    FILTERED_TOPICS: ClassVar[Set[str]] = {
        "/parameter_events",  # ROS2 parameter events
        "/rosout",  # ROS1 logging
        "/clock",  # Simulation time
    }

    FILTERED_NODE_PATTERNS: ClassVar[List[str]] = [
        r"^/graph_analyzer_\d+",  # Our analyzer nodes
        r".*_launcher_\d+",  # Launcher nodes
        r"^/rosout$",  # ROS1 logging node
    ]

    @classmethod
    def clean_graph(cls, graph: ROSGraph) -> ROSGraph:
        """Clean and standardize a ROS graph.

        Args:
            graph: Original ROSGraph object

        Returns:
            Cleaned ROSGraph object
        """
        # Filter nodes
        filtered_nodes: Dict[str, ROSNode] = {}
        for name, node in graph.nodes.items():
            if not any(re.match(pattern, name) for pattern in cls.FILTERED_NODE_PATTERNS):
                filtered_nodes[name] = node

        # Filter topics
        filtered_topics: Dict[str, ROSTopic] = {}
        for name, topic in graph.topics.items():
            if name not in cls.FILTERED_TOPICS:
                filtered_topics[name] = topic

        # Filter edges
        filtered_edges: Set[ROSGraphEdge] = set()
        for edge in graph.edges:
            if (
                edge.source in filtered_nodes
                and edge.target in filtered_nodes
                and edge.topic_name not in cls.FILTERED_TOPICS
            ):
                filtered_edges.add(edge)

        # Create new graph with filtered components
        return ROSGraph(
            nodes=filtered_nodes,
            edges=filtered_edges,
            topics=filtered_topics,
            services=graph.services,  # Keep services for now
            actions=graph.actions,
            parameters=graph.parameters,
            version=graph.version,
            distro=graph.distro,
        )

    @staticmethod
    def standardize_node_name(name: str) -> str:
        """Standardize node names across ROS versions."""
        # Ensure leading slash
        if not name.startswith("/"):
            name = f"/{name}"
        return name
