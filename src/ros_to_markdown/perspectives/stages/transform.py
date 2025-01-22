from typing import Any, Dict

from ...analysis.interfaces import SystemSnapshot
from ..registry import StageRegistry
from ..stage import PipelineStageImpl


@StageRegistry.register("graph_builder")
class GraphBuilderStage(PipelineStageImpl):
    """Builds a graph representation of the ROS system."""
    
    def _normalize_name(self, name: str, include_leading_slash: bool = True) -> str:
        """Normalize node/topic names to a consistent format."""
        # Remove double slashes and clean up
        name = name.replace('//', '/')
        # Ensure leading slash if requested
        if include_leading_slash and not name.startswith('/'):
            name = f"/{name}"
        elif not include_leading_slash and name.startswith('/'):
            name = name[1:]
        return name
    
    async def execute(self, inputs: Dict[str, Any], config: Dict) -> Dict[str, Any]:
        """Transform system snapshot into a graph representation."""
        snapshot: SystemSnapshot = inputs["system_state"]
        include_nodes = config.get("include_nodes", True)
        include_topics = config.get("include_topics", True)
        
        # Build graph data structure
        graph = {
            "nodes": [],
            "topics": [],
            "connections": [],
            "timestamp": snapshot.timestamp.strftime("%Y-%m-%d %H:%M:%S")
        }
        
        if include_nodes:
            for node_name, node_info in snapshot.nodes.items():
                normalized_name = self._normalize_name(node_name)
                graph["nodes"].append({
                    "name": normalized_name,
                    "namespace": node_info.namespace,
                    "publishers": [self._normalize_name(p) for p in node_info.publishers],
                    "subscribers": [self._normalize_name(s) for s in node_info.subscribers]
                })
        
        if include_topics:
            for topic_name, topic_info in snapshot.topics.items():
                normalized_topic = self._normalize_name(topic_name)
                graph["topics"].append({
                    "name": normalized_topic,
                    "type": topic_info.type,
                    "publishers": [self._normalize_name(p) for p in topic_info.publishers],
                    "subscribers": [self._normalize_name(s) for s in topic_info.subscribers]
                })
                
                # Add node -> topic connections for publishers
                for pub in topic_info.publishers:
                    pub_name = self._normalize_name(pub)
                    graph["connections"].append({
                        "from": pub_name,
                        "to": normalized_topic,
                        "type": "publishes"
                    })
                
                # Add topic -> node connections for subscribers
                for sub in topic_info.subscribers:
                    sub_name = self._normalize_name(sub)
                    graph["connections"].append({
                        "from": normalized_topic,
                        "to": sub_name,
                        "type": "subscribes"
                    })
        
        return graph 