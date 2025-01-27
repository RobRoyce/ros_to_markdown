"""Test perspective engine execution."""

from typing import Any
from unittest.mock import AsyncMock, MagicMock

from ros_to_markdown.perspectives.engine import PerspectiveEngine
from ros_to_markdown.perspectives.models import Perspective

import pytest


@pytest.fixture
def mock_stage_impl() -> Any:
    """Create a mock stage implementation."""
    mock = AsyncMock()
    mock.execute = AsyncMock(return_value={"test": "result"})
    return mock


@pytest.fixture
def mock_registry(monkeypatch: Any, mock_stage_impl: Any) -> Any:
    """Mock the stage registry."""

    def mock_get_implementation(stage_type: str) -> Any:
        return mock_stage_impl

    monkeypatch.setattr(
        "ros_to_markdown.perspectives.engine.get_stage_implementation", mock_get_implementation
    )
    return mock_stage_impl


async def test_engine_execution(basic_perspective_dict: Any, mock_registry: Any) -> None:
    """Test perspective engine executes stages correctly."""
    perspective = Perspective.model_validate(basic_perspective_dict)
    engine = PerspectiveEngine(perspective)

    # Execute pipeline
    initial_data = {"analyzer": MagicMock()}
    result = await engine.execute(initial_data)

    # Verify stages were executed
    assert mock_registry.execute.call_count == 3  # collect, transform, render

    # Verify expected outputs are present
    assert "system_state" in result  # From collect stage
    assert "system_graph" in result  # From transform stage
    assert "overview_doc" in result  # From render stage

    # Verify mock results
    assert result["system_state"] == {"test": "result"}
    assert result["system_graph"] == {"test": "result"}
    assert result["overview_doc"] == {"test": "result"}


async def test_engine_missing_input(basic_perspective_dict: Any, mock_registry: Any) -> None:
    """Test engine handles missing inputs correctly."""
    perspective = Perspective.model_validate(basic_perspective_dict)
    engine = PerspectiveEngine(perspective)

    # Execute without required input
    with pytest.raises(ValueError, match="Required input .* not found"):
        await engine.execute({})


async def test_engine_stage_error(basic_perspective_dict: Any, mock_registry: Any) -> None:
    """Test engine handles stage errors correctly."""
    mock_registry.execute.side_effect = ValueError("Stage failed")

    perspective = Perspective.model_validate(basic_perspective_dict)
    engine = PerspectiveEngine(perspective)

    with pytest.raises(ValueError, match="Stage failed"):
        await engine.execute({"analyzer": MagicMock()})
