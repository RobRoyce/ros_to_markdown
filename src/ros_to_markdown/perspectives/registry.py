from typing import Dict, Type
from .stage import PipelineStageImpl


class StageRegistry:
    """Registry for pipeline stage implementations."""
    
    _stages: Dict[str, Type[PipelineStageImpl]] = {}
    
    @classmethod
    def register(cls, stage_type: str):
        """Decorator to register a stage implementation."""
        def wrapper(impl_class: Type[PipelineStageImpl]):
            cls._stages[stage_type] = impl_class
            return impl_class
        return wrapper
    
    @classmethod
    def get(cls, stage_type: str) -> Type[PipelineStageImpl]:
        """Get stage implementation by type."""
        if stage_type not in cls._stages:
            raise ValueError(f"No implementation found for stage type: {stage_type}")
        return cls._stages[stage_type]


def get_stage_implementation(stage_type: str) -> PipelineStageImpl:
    """Get instance of stage implementation."""
    impl_class = StageRegistry.get(stage_type)
    return impl_class() 