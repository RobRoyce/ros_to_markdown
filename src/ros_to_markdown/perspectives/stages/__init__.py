"""Stage implementations for the perspective system."""

# Import all stages to register them
from .collect import SystemSnapshotStage
from .render import MarkdownRendererStage
from .transform import GraphBuilderStage

# These imports register the stages with the registry
__all__ = ["SystemSnapshotStage", "GraphBuilderStage", "MarkdownRendererStage"]
