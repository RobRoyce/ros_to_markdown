from dataclasses import dataclass, field
from typing import Dict, List, Optional, Union, Set, Tuple
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
        dynamic_typing: Whether parameter allows type changes (ROS2 only)
        descriptor: Parameter descriptor (ROS2 only)
    """
    name: str
    type: str
    value: Union[str, int, float, bool, List, Dict]
    description: Optional[str] = None
    dynamic_typing: Optional[bool] = None  # ROS2 only
    descriptor: Optional[Dict] = None  # ROS2 only


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
        lifecycle_state: Current lifecycle state (ROS2 only)
        callback_groups: List of callback group names (ROS2 only)
    """
    name: str
    package: str
    description: Optional[str] = None
    publishers: List[str] = field(default_factory=list)
    subscribers: List[str] = field(default_factory=list)
    services: List[str] = field(default_factory=list)
    actions: List[str] = field(default_factory=list)
    parameters: List[ROSParameter] = field(default_factory=list)
    lifecycle_state: Optional[str] = None  # ROS2 only
    callback_groups: List[str] = field(default_factory=list)  # ROS2 only


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
        qos_profile: QoS settings (ROS2 only)
    """
    name: str
    type: str
    description: Optional[str] = None
    publishers: List[str] = field(default_factory=list)
    subscribers: List[str] = field(default_factory=list)
    frequency: Optional[float] = None
    qos_profile: Optional[Dict[str, any]] = None  # ROS2 only


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


@dataclass
class ROSGraphEdge:
    """Represents an edge in the ROS computation graph.
    
    Attributes:
        source: Source node name
        target: Target node name
        connection_type: Type of connection (topic, service, action)
        topic_name: Name of the connecting topic/service/action
        message_type: Type of message/service/action
        qos_profile: QoS profile for ROS2 connections
        transport: Transport protocol used (e.g., TCPROS, UDPROS)
        frequency: Optional message frequency in Hz
    """
    source: str
    target: str
    connection_type: str  # "topic", "service", "action"
    topic_name: str
    message_type: str
    qos_profile: Optional[Dict[str, any]] = None
    transport: Optional[str] = None
    frequency: Optional[float] = None


@dataclass
class ROSGraph:
    """Represents the complete ROS computation graph with support for visualization.
    
    Attributes:
        nodes: Dictionary of node names to ROSNode objects
        edges: Set of graph edges representing connections
        topics: Dictionary of topic names to ROSTopic objects
        services: Dictionary of service names to ROSService objects
        actions: Dictionary of action names to ROSAction objects
        parameters: Dictionary of parameter names to ROSParameter objects
        version: ROS version (ROS1 or ROS2)
        distro: ROS distribution name (e.g., 'noetic', 'humble')
    """
    nodes: Dict[str, ROSNode]
    edges: Set[ROSGraphEdge]
    topics: Dict[str, ROSTopic]
    services: Dict[str, ROSService]
    actions: Dict[str, ROSAction]
    parameters: Dict[str, ROSParameter]
    version: ROSVersion
    distro: str

    def to_mermaid(self) -> str:
        """Convert the ROS graph to Mermaid markdown format.
        
        Returns:
            str: Mermaid graph definition representing the ROS system
        """
        mermaid = ["graph LR"]
        
        # Add nodes
        for node_name, node in self.nodes.items():
            # Escape special characters and add node definition
            safe_name = node_name.replace("/", "_")
            mermaid.append(f'    {safe_name}["{node_name}"]')
        
        # Add edges with different styles for different connection types
        for edge in self.edges:
            src = edge.source.replace("/", "_")
            tgt = edge.target.replace("/", "_")
            
            if edge.connection_type == "topic":
                # Topic connections use solid lines with arrow
                mermaid.append(f'    {src} -- "{edge.topic_name}" --> {tgt}')
            elif edge.connection_type == "service":
                # Service connections use dotted lines
                mermaid.append(f'    {src} -. "{edge.topic_name}" .-> {tgt}')
            elif edge.connection_type == "action":
                # Action connections use thick lines
                mermaid.append(f'    {src} == "{edge.topic_name}" ==> {tgt}')
        
        return "\n".join(mermaid)

    def get_node_connections(self, node_name: str) -> Dict[str, List[ROSGraphEdge]]:
        """Get all connections for a specific node.
        
        Args:
            node_name: Name of the node to get connections for
            
        Returns:
            Dictionary with 'publishers', 'subscribers', 'services', 'clients',
            'action_servers', and 'action_clients' lists
        """
        connections = {
            'publishers': [],
            'subscribers': [],
            'services': [],
            'clients': [],
            'action_servers': [],
            'action_clients': []
        }
        
        for edge in self.edges:
            if edge.source == node_name:
                if edge.connection_type == "topic":
                    connections['publishers'].append(edge)
                elif edge.connection_type == "service":
                    connections['services'].append(edge)
                elif edge.connection_type == "action":
                    connections['action_servers'].append(edge)
            elif edge.target == node_name:
                if edge.connection_type == "topic":
                    connections['subscribers'].append(edge)
                elif edge.connection_type == "service":
                    connections['clients'].append(edge)
                elif edge.connection_type == "action":
                    connections['action_clients'].append(edge)
                    
        return connections

    def get_subgraph(self, node_names: List[str]) -> 'ROSGraph':
        """Extract a subgraph containing only the specified nodes and their connections.
        
        Args:
            node_names: List of node names to include in the subgraph
            
        Returns:
            ROSGraph: New graph containing only the specified nodes and their connections
        """
        nodes = {name: self.nodes[name] for name in node_names if name in self.nodes}
        edges = {edge for edge in self.edges 
                if edge.source in node_names and edge.target in node_names}
        
        return ROSGraph(
            nodes=nodes,
            edges=edges,
            topics={name: topic for name, topic in self.topics.items()
                   if any(edge.topic_name == name for edge in edges)},
            services={name: svc for name, svc in self.services.items()
                     if any(edge.topic_name == name for edge in edges)},
            actions={name: action for name, action in self.actions.items()
                    if any(edge.topic_name == name for edge in edges)},
            parameters=self.parameters,
            version=self.version,
            distro=self.distro
        )


@dataclass
class ROSLaunchFile:
    """Represents a ROS launch file.
    
    Attributes:
        path: Path to the launch file
        format: Launch file format (python/xml/yaml for ROS2, xml for ROS1)
        nodes: List of nodes launched
        parameters: Dictionary of parameters set in the launch file
        included_files: List of other launch files included
        conditions: List of conditional statements
        arguments: Dictionary of launch arguments
        description: Optional description of what this launch file does
    """
    path: str
    format: str  # "python", "xml", or "yaml"
    nodes: List[ROSNode]
    parameters: Dict[str, ROSParameter]
    included_files: List[str] = field(default_factory=list)
    conditions: List[str] = field(default_factory=list)
    arguments: Dict[str, str] = field(default_factory=dict)
    description: Optional[str] = None


@dataclass
class ROSMessage:
    """Represents a ROS message definition.
    
    Attributes:
        name: Full message name (e.g., 'sensor_msgs/Image')
        fields: List of field definitions
        constants: Dictionary of constant definitions
        description: Optional description of the message purpose
        package: Package containing the message definition
        idl_definition: IDL definition (ROS2 only)
        type_hash: Type hash for interface compatibility (ROS2 only)
    """
    name: str
    fields: List[Dict[str, str]]
    constants: Dict[str, Union[str, int, float, bool]] = field(default_factory=dict)
    description: Optional[str] = None
    package: str
    idl_definition: Optional[str] = None  # ROS2 only
    type_hash: Optional[str] = None  # ROS2 only


@dataclass
class ROSBag:
    """Represents a ROS bag file.
    
    Attributes:
        path: Path to the bag file
        duration: Duration of the recording in seconds
        start_time: Start time of the recording
        end_time: End time of the recording
        topics: Dictionary of topic names to message counts
        message_types: Set of message types in the bag
        size: Size of the bag file in bytes
        compression: Compression format used (if any)
        storage_format: Storage format (ROS2 only)
        converter_plugins: List of converter plugins used (ROS2 only)
    """
    path: str
    duration: float
    start_time: float
    end_time: float
    topics: Dict[str, int]
    message_types: set[str]
    size: int
    compression: Optional[str] = None
    storage_format: Optional[str] = None  # ROS2 only
    converter_plugins: List[str] = field(default_factory=list)  # ROS2 only


@dataclass
class ROSWorkspace:
    """Represents a ROS workspace.
    
    Attributes:
        path: Path to the workspace root
        packages: List of packages in the workspace
        build_system: Build system used (catkin or colcon)
        dependencies: List of package dependencies
        version: ROS version used in this workspace
        distro: ROS distribution name
    """
    path: str
    packages: List[str]
    build_system: str
    dependencies: List[str]
    version: ROSVersion
    distro: str
