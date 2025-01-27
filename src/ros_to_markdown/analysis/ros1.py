import asyncio
from datetime import datetime, timezone
import os
import socket
import time
from typing import Dict, List

from ros_to_markdown.analysis.interfaces import NodeInfo, SystemAnalyzer, SystemSnapshot, TopicInfo
from ros_to_markdown.logging import get_logger

logger = get_logger(__name__)


def check_roscore() -> bool:
    """Check if roscore is running by attempting to connect to the ROS master port."""
    logger.debug("Checking roscore connection...")
    ros_master_uri = os.environ.get("ROS_MASTER_URI", "http://localhost:11311")
    if not ros_master_uri.startswith("http://"):
        return False

    try:
        host = ros_master_uri.split("//")[1].split(":")[0]
        port = int(ros_master_uri.split(":")[-1])

        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(2.0)
        result = sock.connect_ex((host, port))
        sock.close()
        return result == 0
    except Exception as e:
        logger.error(f"Failed to check roscore: {e}")
        return False


class Ros1SystemAnalyzer(SystemAnalyzer):
    """ROS1 implementation of system analyzer."""

    def __init__(self) -> None:
        """Initialize ROS1 node for analysis."""
        if not check_roscore():
            raise RuntimeError("roscore is not running")

        try:
            # Use the already imported rospy from cli.py
            from rosgraph.masterapi import Master
            import rospy
            import rostopic

            # Store modules as instance variables
            self.rospy = rospy
            self.rostopic = rostopic

            # Initialize node if not already initialized
            if not rospy.core.is_initialized():
                rospy.init_node(
                    "system_analyzer",
                    anonymous=True,
                    disable_rosout=True,
                    disable_signals=True,
                    log_level=rospy.FATAL,
                )

            # Initialize master connection
            self.master = Master("/system_analyzer")
            self.master.getSystemState()

            logger.info("ROS1 analyzer initialized successfully")

        except Exception as e:
            logger.error(f"Failed to initialize ROS1 analyzer: {e}")
            raise RuntimeError(f"Failed to initialize ROS1 analyzer: {e}") from e

    async def get_snapshot(self) -> SystemSnapshot:
        """Get current system state using ROS1 APIs."""
        nodes: Dict[str, NodeInfo] = {}
        topics: Dict[str, TopicInfo] = {}

        try:
            # Get system state in one call to avoid race conditions
            publishers, subscribers, _ = self.master.getSystemState()
            topic_types = dict(self.master.getTopicTypes())

            # Build topic information
            for topic_name, pub_nodes in publishers:
                sub_nodes = []
                for sub_topic, sub_list in subscribers:
                    if sub_topic == topic_name:
                        sub_nodes = sub_list
                        break

                topic_type = topic_types.get(topic_name, "unknown_type")
                topics[topic_name] = TopicInfo(
                    name=topic_name,
                    type=topic_type,
                    publishers=pub_nodes,
                    subscribers=sub_nodes,
                )

            # Build node information
            all_nodes = set()
            for topic_info in topics.values():
                all_nodes.update(topic_info.publishers)
                all_nodes.update(topic_info.subscribers)

            for node_name in all_nodes:
                node_pubs = []
                node_subs = []
                for topic_name, topic_info in topics.items():
                    if node_name in topic_info.publishers:
                        node_pubs.append(topic_name)
                    if node_name in topic_info.subscribers:
                        node_subs.append(topic_name)

                nodes[node_name] = NodeInfo(
                    name=node_name,
                    namespace="/",  # ROS1 doesn't have explicit namespaces like ROS2
                    publishers=node_pubs,
                    subscribers=node_subs,
                )

            return SystemSnapshot(
                timestamp=datetime.now(tz=timezone.utc), nodes=nodes, topics=topics
            )

        except Exception as e:
            raise RuntimeError(f"Failed to get system snapshot: {e}") from e

    async def analyze_frequency(self, topic_name: str, duration: float = 1.0) -> float:
        """Analyze message frequency using ROS1 subscription."""
        msg_type = self.rostopic.get_topic_type(topic_name)[0]
        if not msg_type:
            raise ValueError(f"Topic {topic_name} not found")

        # Set up subscription
        msg_count = 0
        start_time = None

        def callback(msg: object) -> None:
            nonlocal msg_count, start_time
            if start_time is None:
                start_time = time.time()
            msg_count += 1

        # Create subscription
        sub = self.rospy.Subscriber(topic_name, msg_type, callback)

        # Wait for duration
        await asyncio.sleep(duration)

        # Calculate frequency
        if start_time is None or msg_count == 0:
            frequency = 0.0
        else:
            elapsed = time.time() - start_time
            frequency = msg_count / elapsed

        # Cleanup
        sub.unregister()

        return frequency

    async def analyze_latency(self, source: str, target: str, duration: float = 1.0) -> float:
        """Analyze latency between nodes."""
        system_state = self.master.getSystemState()
        publishers = system_state[0]

        # Find topics where source is publisher and target is subscriber
        common_topics = []
        for topic_data in publishers:
            topic_name = topic_data[0]
            pub_nodes = topic_data[1]
            if source in pub_nodes:
                for sub_data in system_state[1]:
                    if sub_data[0] == topic_name and target in sub_data[1]:
                        common_topics.append(topic_name)
                        break

        if not common_topics:
            raise ValueError(f"No common topics found between {source} and {target}")

        topic_name = common_topics[0]
        timestamps: List[float] = []

        def callback(msg: object) -> None:
            timestamps.append(time.time())

        msg_type = self.rostopic.get_topic_type(topic_name)[0]
        sub = self.rospy.Subscriber(topic_name, msg_type, callback)

        await asyncio.sleep(duration)

        if len(timestamps) < 2:
            latency = 0.0
        else:
            deltas = [t2 - t1 for t1, t2 in zip(timestamps[:-1], timestamps[1:])]
            latency = sum(deltas) / len(deltas) * 1000

        sub.unregister()
        return latency
