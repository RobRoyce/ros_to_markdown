"""Test stage registry functionality."""

from typing import Any

from ros_to_markdown.perspectives.registry import StageRegistry
from ros_to_markdown.perspectives.stage import PipelineStageImpl

import pytest


class MockStage(PipelineStageImpl):
    """Mock stage implementation for testing."""

    async def execute(self, inputs: Any, config: Any) -> Any:
        return {"mock": "result"}


def test_stage_registration() -> None:
    """Test stage registration and retrieval."""
    # Register a test stage
    StageRegistry.register("test_stage")(MockStage)

    # Get registered stage
    impl_class = StageRegistry.get("test_stage")
    assert impl_class == MockStage

    # Test unregistered stage
    with pytest.raises(ValueError, match="No implementation found"):
        StageRegistry.get("nonexistent_stage")


def test_stage_decorator() -> None:
    """Test stage registration decorator."""

    @StageRegistry.register("decorated_stage")
    class TestStage(PipelineStageImpl):
        async def execute(self, inputs: Any, config: Any) -> Any:
            return {"test": "result"}

    # Verify registration
    retrieved_class = StageRegistry.get("decorated_stage")
    assert retrieved_class == TestStage
