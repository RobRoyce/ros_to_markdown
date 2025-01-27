import asyncio
from datetime import datetime, timezone
import time
from typing import Dict, List

import rclpy
from rclpy.node import Node

from .interfaces import NodeInfo, SystemAnalyzer, SystemSnapshot, TopicInfo


class Ros2SystemAnalyzer(SystemAnalyzer):
    """ROS2 implementation of system analyzer."""

    def __init__(self) -> None:
        """Initialize ROS2 node for analysis."""
        if not rclpy.ok():
            rclpy.init()
        self.node = Node("system_analyzer")

    async def get_snapshot(self) -> SystemSnapshot:
        """Get current system state using ROS2 APIs."""
        # Get list of nodes
        node_names_ns = self.node.get_node_names_and_namespaces()

        nodes: Dict[str, NodeInfo] = {}
        topics: Dict[str, TopicInfo] = {}

        # Build node and topic information
        for node_name, namespace in node_names_ns:
            # Get node info
            node_pubs = []
            node_subs = []

            try:
                # Get publisher and subscriber info
                pub_info = self.node.get_publisher_names_and_types_by_node(node_name, namespace)
                sub_info = self.node.get_subscriber_names_and_types_by_node(node_name, namespace)

                # Process publishers
                for topic_name, topic_types in pub_info:
                    node_pubs.append(topic_name)
                    if topic_name not in topics:
                        topics[topic_name] = TopicInfo(
                            name=topic_name,
                            type=topic_types[0],  # Use first type if multiple
                            publishers=[f"{namespace}/{node_name}"],
                            subscribers=[],
                        )
                    else:
                        topics[topic_name].publishers.append(f"{namespace}/{node_name}")

                # Process subscribers
                for topic_name, topic_types in sub_info:
                    node_subs.append(topic_name)
                    if topic_name not in topics:
                        topics[topic_name] = TopicInfo(
                            name=topic_name,
                            type=topic_types[0],
                            publishers=[],
                            subscribers=[f"{namespace}/{node_name}"],
                        )
                    else:
                        topics[topic_name].subscribers.append(f"{namespace}/{node_name}")

                # Create node info
                nodes[f"{namespace}/{node_name}"] = NodeInfo(
                    name=node_name, namespace=namespace, publishers=node_pubs, subscribers=node_subs
                )

            except Exception as e:
                self.node.get_logger().warn(f"Error getting info for node {node_name}: {e}")
                continue

        return SystemSnapshot(timestamp=datetime.now(tz=timezone.utc), nodes=nodes, topics=topics)

    async def analyze_frequency(self, topic_name: str, duration: float = 1.0) -> float:
        """Analyze message frequency using ROS2 subscription."""
        # Get topic type
        topic_info = self.node.get_publishers_info_by_topic(topic_name)
        if not topic_info:
            raise ValueError(f"Topic {topic_name} not found")

        msg_type = topic_info[0].topic_type

        # Set up subscription
        msg_count = 0
        start_time = None

        def callback(msg: object) -> None:
            nonlocal msg_count, start_time
            if start_time is None:
                start_time = time.time()
            msg_count += 1

        # Create subscription
        sub = self.node.create_subscription(
            msg_type,
            topic_name,
            callback,
            10,  # QoS depth of 10
        )

        # Wait for duration
        await asyncio.sleep(duration)

        # Calculate frequency
        if start_time is None or msg_count == 0:
            frequency = 0.0
        else:
            elapsed = time.time() - start_time
            frequency = msg_count / elapsed

        # Cleanup
        self.node.destroy_subscription(sub)

        return frequency

    async def analyze_latency(self, source: str, target: str, duration: float = 1.0) -> float:
        """
        Analyze latency between nodes using ROS2 subscription.
        Note: This is a simplified implementation that measures processing time
        between receiving a message and republishing it.
        """
        # Get topic connecting source and target
        source_pubs = self.node.get_publisher_names_and_types_by_node(
            source.split("/")[-1], "/".join(source.split("/")[:-1])
        )
        target_subs = self.node.get_subscriber_names_and_types_by_node(
            target.split("/")[-1], "/".join(target.split("/")[:-1])
        )

        # Find common topics
        source_topics = {name for name, _ in source_pubs}
        target_topics = {name for name, _ in target_subs}
        common_topics = source_topics.intersection(target_topics)

        if not common_topics:
            raise ValueError(f"No common topics found between {source} and {target}")

        # Use first common topic for latency measurement
        topic_name = next(iter(common_topics))

        # Measure message timing
        timestamps: List[float] = []

        def callback(msg: object) -> None:
            timestamps.append(time.time())

        # Subscribe to topic
        topic_info = self.node.get_publishers_info_by_topic(topic_name)
        msg_type = topic_info[0].topic_type
        sub = self.node.create_subscription(
            msg_type,
            topic_name,
            callback,
            10,  # QoS depth of 10
        )

        # Wait for duration
        await asyncio.sleep(duration)

        # Calculate average latency
        if len(timestamps) < 2:
            latency = 0.0
        else:
            # Calculate average time between messages
            deltas = [t2 - t1 for t1, t2 in zip(timestamps[:-1], timestamps[1:])]
            latency = sum(deltas) / len(deltas) * 1000  # Convert to milliseconds

        # Cleanup
        self.node.destroy_subscription(sub)

        return latency

    def __del__(self) -> None:
        """Cleanup ROS2 node."""
        try:
            self.node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass  # Ignore shutdown errors
