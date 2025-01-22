from typing import Any, Dict

from ...analysis.interfaces import SystemSnapshot
from ..registry import StageRegistry
from ..stage import PipelineStageImpl


@StageRegistry.register("system_snapshot")
class SystemSnapshotStage(PipelineStageImpl):
    """Collects system snapshot using analyzer."""

    async def execute(self, inputs: Dict[str, Any], config: Dict) -> SystemSnapshot:
        analyzer = inputs.get("analyzer")
        if not analyzer:
            raise ValueError("Analyzer not provided")

        return await analyzer.get_snapshot()
