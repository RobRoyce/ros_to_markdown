from typing import Any, Dict, Optional

from ..logging import get_logger
from .models import Perspective, PipelineStage, StageType
from .registry import get_stage_implementation


class PipelineContext:
    """Holds data and state during pipeline execution."""

    def __init__(self):
        self.data: Dict[str, Any] = {}
        self.logger = get_logger(__name__)

    def get(self, key: str) -> Optional[Any]:
        """Get data by key."""
        return self.data.get(key)

    def put(self, key: str, value: Any):
        """Store data by key."""
        self.data[key] = value


class PerspectiveEngine:
    """Executes perspectives on collected data."""

    def __init__(self, perspective: Perspective):
        self.perspective = perspective
        self.logger = get_logger(__name__)

    async def execute(self, initial_data: Dict[str, Any]) -> Dict[str, Any]:
        """Execute the perspective pipeline."""
        context = PipelineContext()
        context.data.update(initial_data)

        # Execute pipeline stages in order
        for stage_type in StageType:
            stages = getattr(self.perspective.pipeline, stage_type.value, None)
            if not stages:
                continue

            self.logger.debug(f"Executing {stage_type.value} stages")
            for stage in stages:
                await self._execute_stage(stage, context)

        return context.data

    async def _execute_stage(self, stage: PipelineStage, context: PipelineContext):
        """Execute a single pipeline stage."""
        try:
            # Get stage implementation
            stage_impl = get_stage_implementation(stage.type)

            # Prepare inputs
            inputs = {}
            for input_spec in stage.inputs:
                if isinstance(input_spec, str):
                    input_name = input_spec
                    required = True
                else:
                    input_name = input_spec.source
                    required = input_spec.required

                value = context.get(input_name)
                if value is None and required:
                    raise ValueError(f"Required input {input_name} not found")
                inputs[input_name] = value

            # Execute stage
            result = await stage_impl.execute(inputs, stage.config or {})

            # Store result
            context.put(stage.output, result)

        except Exception as e:
            self.logger.error(f"Stage {stage.name} failed: {str(e)}")
            raise
