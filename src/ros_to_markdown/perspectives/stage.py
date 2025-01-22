from abc import ABC, abstractmethod
from typing import Any, Dict


class PipelineStageImpl(ABC):
    """Base class for pipeline stage implementations."""
    
    @abstractmethod
    async def execute(self, inputs: Dict[str, Any], config: Dict) -> Any:
        """Execute the pipeline stage."""
        pass 