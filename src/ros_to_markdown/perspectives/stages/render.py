from pathlib import Path
from typing import Any, Dict

import jinja2

from ..registry import StageRegistry
from ..stage import PipelineStageImpl


@StageRegistry.register("markdown")
class MarkdownRendererStage(PipelineStageImpl):
    """Renders system information as markdown."""
    
    def __init__(self):
        super().__init__()
        # Set up Jinja environment
        template_dir = Path(__file__).parent.parent / "templates"
        self.env = jinja2.Environment(
            loader=jinja2.FileSystemLoader(str(template_dir)),
            autoescape=False,
            trim_blocks=True,
            lstrip_blocks=True
        )
    
    async def execute(self, inputs: Dict[str, Any], config: Dict) -> str:
        """Render markdown using template."""
        template_name = config.get("template", "overview.md.j2")
        template = self.env.get_template(template_name)
        
        # Get graph data
        graph = inputs["system_graph"]
        
        # Render template
        return template.render(
            graph=graph,
            title="ROS System Overview",
            timestamp=graph.get("timestamp")
        ) 