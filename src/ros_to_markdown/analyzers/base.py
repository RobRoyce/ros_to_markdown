"""Base class for ROS graph analyzers."""

from abc import ABC, abstractmethod
from typing import Optional

from ..models.ros_components import ROSGraph


class GraphAnalyzer(ABC):
    """Base class for analyzing ROS computation graphs."""

    @abstractmethod
    def analyze(self) -> Optional[ROSGraph]:
        """Analyze the running ROS system and return a graph representation.

        Returns:
            Optional[ROSGraph]: The analyzed graph, or None if analysis fails
        """
        pass

    @abstractmethod
    def get_message_type(self, topic_name: str) -> str:
        """Get the message type for a topic.

        Args:
            topic_name: Name of the topic

        Returns:
            str: Message type or "unknown" if not found
        """
        pass
