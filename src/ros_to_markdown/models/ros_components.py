from dataclasses import dataclass, field
from typing import Dict, List, Optional, Union
from enum import Enum


class ROSVersion(Enum):
    """Enum representing ROS versions."""
    ROS1 = "ROS1"
    ROS2 = "ROS2"
    UNKNOWN = "UNKNOWN"


@dataclass
class ROSParameter:
    """Represents a ROS parameter.
    
    Attributes:
        name: Parameter name
        type: Parameter type (str, int, etc.)
        value: Current parameter value
        description: Optional description of the parameter
    """
    name: str
    type: str
    value: Union[str, int, float, bool, List, Dict]
    description: Optional[str] = None


@dataclass
class ROSNode:
    """Represents a ROS node.
    
    Attributes:
        name: Node name
        package: Package the node belongs to
        description: Optional node description
        publishers: List of topics this node publishes to
        subscribers: List of topics this node subscribes to
        services: List of services this node provides/calls
        actions: List of actions this node provides/calls
        parameters: List of parameters this node uses
    """
    name: str
    package: str
    description: Optional[str] = None
    publishers: List[str] = field(default_factory=list)
    subscribers: List[str] = field(default_factory=list)
    services: List[str] = field(default_factory=list)
    actions: List[str] = field(default_factory=list)
    parameters: List[ROSParameter] = field(default_factory=list)


@dataclass
class ROSTopic:
    """Represents a ROS topic.
    
    Attributes:
        name: Topic name
        type: Message type
        description: Optional topic description
        publishers: List of nodes publishing to this topic
        subscribers: List of nodes subscribing to this topic
        frequency: Optional publishing frequency in Hz
    """
    name: str
    type: str
    description: Optional[str] = None
    publishers: List[str] = field(default_factory=list)
    subscribers: List[str] = field(default_factory=list)
    frequency: Optional[float] = None


@dataclass
class ROSService:
    """Represents a ROS service.
    
    Attributes:
        name: Service name
        type: Service type
        description: Optional service description
        providers: List of nodes providing this service
        clients: List of nodes calling this service
    """
    name: str
    type: str
    description: Optional[str] = None
    providers: List[str] = field(default_factory=list)
    clients: List[str] = field(default_factory=list)


@dataclass
class ROSAction:
    """Represents a ROS action.
    
    Attributes:
        name: Action name
        type: Action type
        description: Optional action description
        providers: List of nodes providing this action
        clients: List of nodes calling this action
    """
    name: str
    type: str
    description: Optional[str] = None
    providers: List[str] = field(default_factory=list)
    clients: List[str] = field(default_factory=list)
