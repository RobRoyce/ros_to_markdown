from abc import ABC, abstractmethod
from dataclasses import dataclass
from datetime import datetime
from typing import Dict, List, Optional


@dataclass
class NodeInfo:
    """Basic information about a ROS node."""
    name: str
    namespace: str
    publishers: List[str]  # Topic names
    subscribers: List[str]  # Topic names

@dataclass
class TopicInfo:
    """Basic information about a ROS topic."""
    name: str
    type: str
    publishers: List[str]  # Node names
    subscribers: List[str]  # Node names
    frequency: Optional[float] = None  # Hz
    latency: Optional[float] = None   # milliseconds

@dataclass
class SystemSnapshot:
    """Snapshot of system state at a point in time."""
    timestamp: datetime
    nodes: Dict[str, NodeInfo]
    topics: Dict[str, TopicInfo]

class SystemAnalyzer(ABC):
    """Base interface for ROS system analyzers."""

    @abstractmethod
    async def get_snapshot(self) -> SystemSnapshot:
        """Get current system state."""
        pass

    @abstractmethod
    async def analyze_frequency(self, topic_name: str, duration: float = 1.0) -> float:
        """Analyze message frequency for a topic."""
        pass

    @abstractmethod
    async def analyze_latency(self, source: str, target: str, duration: float = 1.0) -> float:
        """Analyze latency between two nodes."""
        pass
