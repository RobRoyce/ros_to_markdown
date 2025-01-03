"""Data models representing ROS graph components and their relationships."""

from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Dict, List, Optional, Set, Tuple, Union


class ROSVersion(Enum):
    """Enum representing ROS versions."""

    ROS1 = "ROS1"
    ROS2 = "ROS2"
    UNKNOWN = "UNKNOWN"


class ConnectionType(Enum):
    """Valid connection types in the ROS graph."""

    TOPIC = "topic"
    SERVICE = "service"
    ACTION = "action"

    @classmethod
    def values(cls) -> Set[str]:
        """Get all valid connection type values."""
        return {member.value for member in cls}


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
        type: Service type (e.g., std_srvs/Empty)
        node: Node providing the service
        description: Optional service description
    """

    name: str
    type: str
    node: Optional[str] = None
    description: Optional[str] = None


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


@dataclass(frozen=True)
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
    connection_type: str
    topic_name: str
    message_type: str
    qos_profile: Optional[Dict[str, Any]] = None
    transport: Optional[str] = None
    frequency: Optional[float] = None

    def __post_init__(self):
        """Validate edge attributes after initialization."""
        if self.connection_type not in ConnectionType.values():
            raise ValueError(
                f"Invalid connection_type: {self.connection_type}. "
                f"Must be one of: {ConnectionType.values()}"
            )

        if not self.source or not isinstance(self.source, str):
            raise ValueError("source must be a non-empty string")

        if not self.target or not isinstance(self.target, str):
            raise ValueError("target must be a non-empty string")

        if not self.topic_name or not isinstance(self.topic_name, str):
            raise ValueError("topic name must be a non-empty string")

        if not self.message_type or not isinstance(self.message_type, str):
            raise ValueError("message type must be a non-empty string")

        if self.frequency is not None and not isinstance(self.frequency, (int, float)):
            raise ValueError("Frequency must be a number")

        # Validate QoS profile structure if present
        if self.qos_profile is not None:
            self._validate_qos_profile()

    def _validate_qos_profile(self):
        """Validate ROS2 QoS profile structure."""
        required_fields = {"reliability", "durability", "history"}
        if not all(field in self.qos_profile for field in required_fields):
            raise ValueError(f"QoS profile missing required fields: {required_fields}")

    def __hash__(self):
        """Generate hash based on immutable fields.

        Returns:
            int: Hash value for the edge
        """
        return hash(
            (self.source, self.target, self.connection_type, self.topic_name, self.message_type)
        )

    def __eq__(self, other):
        """Compare two ROSGraphEdge objects for equality.

        Args:
            other: Object to compare with

        Returns:
            bool: True if objects are equal, False otherwise
        """
        if not isinstance(other, ROSGraphEdge):
            return NotImplemented
        return (
            self.source == other.source
            and self.target == other.target
            and self.connection_type == other.connection_type
            and self.topic_name == other.topic_name
            and self.message_type == other.message_type
        )


def sanitize_mermaid_id(name: str) -> str:
    """Sanitize a name for use as a Mermaid node ID.

    Args:
        name: The name to sanitize

    Returns:
        A sanitized string safe for use in Mermaid diagrams
    """
    # Replace problematic characters
    sanitized = name.replace("/", "_")
    sanitized = sanitized.replace(" ", "_")
    sanitized = sanitized.replace("-", "_")
    sanitized = sanitized.replace(".", "_")

    # Ensure valid ID by prefixing with 'node_' if starts with number
    if sanitized[0].isdigit():
        sanitized = f"node_{sanitized}"

    return sanitized


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

    def to_mermaid(self, highlight_cycles: bool = True, show_message_types: bool = False) -> str:
        """Convert the ROS graph to Mermaid diagram format.

        Args:
            highlight_cycles: Whether to highlight cycles in the graph
            show_message_types: Whether to show message types on edges

        Returns:
            Mermaid diagram as string
        """
        mermaid = ["graph LR"]

        # Add styling
        mermaid.extend(
            [
                "    %% Node and edge styling",
                "    classDef default fill:#f9f9f9,stroke:#333,stroke-width:1px",
                "    classDef cycleNode fill:#fff0f0,stroke:#d43f3f,stroke-width:2px",
                "    classDef riskNode fill:#ffe0e0,stroke:#ff0000,stroke-width:2px",
                "    classDef topicEdge stroke:#666,stroke-width:1px",
                "    classDef serviceEdge stroke:#666,stroke-width:1px,stroke-dasharray:5 5",
                "    classDef actionEdge stroke:#666,stroke-width:2px",
                "    classDef cycleEdge stroke:#d43f3f,stroke-width:2px",
                "    classDef riskEdge stroke:#ff0000,stroke-width:3px,stroke-dasharray:5 5",
            ]
        )

        # Find cycles and analyze risks
        cycles = set()
        cycle_edges = set()
        risk_nodes = set()
        risky_edges = set()

        if highlight_cycles:
            # First find all cycles
            for cycle in self.find_cycles():
                cycles.update(cycle)
                analysis = self.analyze_cycle(cycle)

                # Track cycle edges
                for edges in analysis.values():
                    cycle_edges.update(edges)

                # Check for risky patterns
                has_risk, _ = self.has_deadlock_risk(cycle)
                if has_risk:
                    # Mark edges in risky cycles
                    for edges in analysis.values():
                        risky_edges.update(edges)
                        # Mark nodes involved in risky cycles
                        for edge in edges:
                            risk_nodes.add(edge.source)
                            risk_nodes.add(edge.target)

        # Add nodes with styling
        for node_name, _node in self.nodes.items():
            safe_id = sanitize_mermaid_id(node_name)
            node_def = f'    {safe_id}["{node_name}"]'

            if highlight_cycles:
                if node_name in risk_nodes:
                    node_def += ":::riskNode"
                elif node_name in cycles:
                    node_def += ":::cycleNode"
            mermaid.append(node_def)

        # Add edges with styling
        for edge in self.edges:
            src_id = sanitize_mermaid_id(edge.source)
            tgt_id = sanitize_mermaid_id(edge.target)

            # Create edge label
            label = edge.topic_name
            if show_message_types and edge.message_type:
                label += f"\\n({edge.message_type})"

            # Determine edge style and format
            edge_def = f"    {src_id} "

            # Set base edge style and format based on type
            if edge.connection_type == "topic":
                edge_def += f'-- "{label}" -->'
                style = ":::topicEdge"
            elif edge.connection_type == "service":
                edge_def += f'-. "{label}" .->'
                style = ":::serviceEdge"
            else:  # action
                edge_def += f'== "{label}" ==>'
                style = ":::actionEdge"

            edge_def += f" {tgt_id}"

            # Override style for risky and cycle edges only if highlighting is enabled
            if highlight_cycles:
                if edge in risky_edges:
                    style = ":::riskEdge"
                elif edge in cycle_edges and edge.connection_type in {"service", "action"}:
                    style = ":::riskEdge"
                elif edge in cycle_edges:
                    style = ":::cycleEdge"

            edge_def += style
            mermaid.append(edge_def)

        return "\n".join(mermaid)

    def to_markdown(self) -> str:
        """Generate a complete markdown documentation of the ROS graph.

        Returns:
            str: Markdown documentation including graph visualization and analysis
        """
        sections = ["# ROS System Documentation\n"]

        # Basic system info
        sections.extend(
            [
                "## System Information",
                f"- ROS Version: {self.version.value}",
                f"- Distribution: {self.distro}",
                f"- Nodes: {len(self.nodes)}",
                f"- Topics: {len(self.topics)}",
                f"- Services: {len(self.services)}",
                f"- Actions: {len(self.actions)}\n",
            ]
        )

        # Graph visualization
        sections.extend(["## System Architecture", "```mermaid", self.to_mermaid(), "```\n"])

        # Cycle analysis
        cycles = self.find_cycles()
        if cycles:
            sections.append("## Communication Patterns\n")

            for i, cycle in enumerate(cycles, 1):
                sections.append(f"### Pattern {i}: {' → '.join(cycle)}")

                # Analyze cycle
                analysis = self.analyze_cycle(cycle)
                has_risk, risk_reason = self.has_deadlock_risk(cycle)

                # Document connections
                if analysis["topics"]:
                    sections.append("\n#### Topic Communications:")
                    for edge in analysis["topics"]:
                        sections.append(f"- {edge.source} → {edge.topic_name} → {edge.target}")

                if analysis["services"]:
                    sections.append("\n#### Service Calls:")
                    for edge in analysis["services"]:
                        sections.append(f"- {edge.source} calls {edge.topic_name} on {edge.target}")

                if analysis["actions"]:
                    sections.append("\n#### Action Calls:")
                    for edge in analysis["actions"]:
                        sections.append(f"- {edge.source} calls {edge.topic_name} on {edge.target}")

                # Risk assessment
                sections.append("\n#### Risk Assessment")
                if has_risk:
                    sections.append(f"⚠️ **Warning**: {risk_reason}")
                else:
                    sections.append("✅ No immediate risks detected")
                sections.append("")  # Empty line between patterns

        return "\n".join(sections)

    def get_node_connections(self, node_name: str) -> Dict[str, List[ROSGraphEdge]]:
        """Get all connections for a specific node.

        Args:
            node_name: Name of the node to get connections for

        Returns:
            Dictionary with 'publishers', 'subscribers', 'services', 'clients',
            'action_servers', and 'action_clients' lists
        """
        connections = {
            "publishers": [],
            "subscribers": [],
            "services": [],
            "clients": [],
            "action_servers": [],
            "action_clients": [],
        }

        for edge in self.edges:
            if edge.source == node_name:
                if edge.connection_type == "topic":
                    connections["publishers"].append(edge)
                elif edge.connection_type == "service":
                    connections["services"].append(edge)
                elif edge.connection_type == "action":
                    connections["action_servers"].append(edge)
            elif edge.target == node_name:
                if edge.connection_type == "topic":
                    connections["subscribers"].append(edge)
                elif edge.connection_type == "service":
                    connections["clients"].append(edge)
                elif edge.connection_type == "action":
                    connections["action_clients"].append(edge)

        return connections

    def get_subgraph(self, node_names: List[str]) -> "ROSGraph":
        """Extract a subgraph containing only the specified nodes and their connections.

        Args:
            node_names: List of node names to include in the subgraph

        Returns:
            ROSGraph: New graph containing only the specified nodes and their connections
        """
        nodes = {name: self.nodes[name] for name in node_names if name in self.nodes}
        edges = {
            edge for edge in self.edges if edge.source in node_names and edge.target in node_names
        }

        return ROSGraph(
            nodes=nodes,
            edges=edges,
            topics={
                name: topic
                for name, topic in self.topics.items()
                if any(edge.topic_name == name for edge in edges)
            },
            services={
                name: svc
                for name, svc in self.services.items()
                if any(edge.topic_name == name for edge in edges)
            },
            actions={
                name: action
                for name, action in self.actions.items()
                if any(edge.topic_name == name for edge in edges)
            },
            parameters=self.parameters,
            version=self.version,
            distro=self.distro,
        )

    def find_cycles(self) -> List[List[str]]:
        """Find all cycles in the graph using Tarjan's algorithm.

        Returns:
            List[List[str]]: List of cycles, where each cycle is a list of node names
        """

        def strong_connect(
            node: str,
            index: Dict[str, int],
            lowlink: Dict[str, int],
            stack: List[str],
            on_stack: Set[str],
            cycles: List[List[str]],
        ):
            # Initialize node
            index[node] = len(index)
            lowlink[node] = index[node]
            stack.append(node)
            on_stack.add(node)

            # Check all neighbors
            for edge in self.edges:
                if edge.source == node:
                    neighbor = edge.target
                    if neighbor not in index:
                        # Recurse on unvisited neighbor
                        strong_connect(neighbor, index, lowlink, stack, on_stack, cycles)
                        lowlink[node] = min(lowlink[node], lowlink[neighbor])
                    elif neighbor in on_stack:
                        # Found a cycle
                        lowlink[node] = min(lowlink[node], index[neighbor])

            # Check if node is root of strongly connected component
            if lowlink[node] == index[node]:
                # Found a strongly connected component
                component = []
                while True:
                    w = stack.pop()
                    on_stack.remove(w)
                    component.append(w)
                    if w == node:
                        break
                if len(component) > 1:  # Only include components with cycles
                    cycles.append(component)

        index: Dict[str, int] = {}
        lowlink: Dict[str, int] = {}
        stack: List[str] = []
        on_stack: Set[str] = set()
        cycles: List[List[str]] = []

        # Run algorithm on all nodes
        for node in self.nodes:
            if node not in index:
                strong_connect(node, index, lowlink, stack, on_stack, cycles)

        return cycles

    def analyze_cycle(self, cycle: List[str]) -> Dict[str, List[ROSGraphEdge]]:
        """Analyze the connections within a cycle.

        Args:
            cycle: List of node names forming a cycle

        Returns:
            Dict mapping connection types to lists of edges in the cycle
        """
        cycle_set = set(cycle)
        cycle_edges: Dict[str, List[ROSGraphEdge]] = {"topics": [], "services": [], "actions": []}

        for edge in self.edges:
            if edge.source in cycle_set and edge.target in cycle_set:
                if edge.connection_type == "topic":
                    cycle_edges["topics"].append(edge)
                elif edge.connection_type == "service":
                    cycle_edges["services"].append(edge)
                elif edge.connection_type == "action":
                    cycle_edges["actions"].append(edge)

        return cycle_edges

    def has_deadlock_risk(self, cycle: List[str]) -> Tuple[bool, str]:
        """Check if a cycle has potential deadlock risks.

        Args:
            cycle: List of node names forming a cycle

        Returns:
            Tuple of (has_risk: bool, reason: str)
        """
        cycle_edges = self.analyze_cycle(cycle)

        # Check for service calls in cycle
        if cycle_edges["services"]:
            return True, "Cycle contains service calls which could deadlock"

        # Check for action calls in cycle
        if cycle_edges["actions"]:
            return True, "Cycle contains action calls which could deadlock"

        # Check for bidirectional topic dependencies on the same topics
        topic_pairs: Dict[str, Set[Tuple[str, str]]] = {}  # topic_name -> set of (src, tgt) pairs
        for edge in cycle_edges["topics"]:
            if edge.topic_name not in topic_pairs:
                topic_pairs[edge.topic_name] = set()
            topic_pairs[edge.topic_name].add((edge.source, edge.target))

        # Look for bidirectional communication using the same topic
        for topic, pairs in topic_pairs.items():
            for src, tgt in pairs:
                if (tgt, src) in pairs:
                    return (
                        True,
                        f"Bidirectional dependency on topic {topic} between {src} and {tgt}",
                    )

        return False, "No immediate deadlock risks detected"


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
        package: Package containing the message definition
        name: Full message name (e.g., 'sensor_msgs/Image')
        fields: List of field definitions
        description: Optional description of the message purpose
        constants: Dictionary of constant definitions
        services: List of ROS services
        actions: List of ROS actions
    """

    package: str
    name: str
    fields: List[Dict[str, str]]
    description: Optional[str] = None
    constants: Dict[str, Union[str, int, float, bool]] = field(default_factory=dict)
    services: List[ROSService] = field(default_factory=list)
    actions: List[ROSAction] = field(default_factory=list)


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
    message_types: Set[str]
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
