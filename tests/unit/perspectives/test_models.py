"""Test perspective model validation and configuration."""

from typing import Any

from ros_to_markdown.perspectives.models import Perspective, PipelineStage, StageType

from pydantic import ValidationError
import pytest


def test_pipeline_stage_creation() -> None:
    """Test successful pipeline stage creation with all required fields."""
    stage = PipelineStage(type="test_type", name="test_stage", inputs=["input1"], output="output1")
    assert stage.type == "test_type"
    assert stage.name == "test_stage"


def test_pipeline_stage_missing_fields() -> None:
    """Test pipeline stage validation fails when required fields are missing."""
    # Missing required fields should raise ValidationError
    with pytest.raises(ValidationError):
        PipelineStage(type="test_type")  # type: ignore[call-arg]  # Intentionally missing required fields

    with pytest.raises(ValidationError):
        PipelineStage(name="test_stage", inputs=["input1"], output="output1")  # type: ignore[call-arg]  # Missing type field


def test_pipeline_stage_validation() -> None:
    """Test pipeline stage configuration validation."""
    # Valid configuration
    stage = PipelineStage(type="test_type", name="test_stage", inputs=["input1"], output="output1")
    assert stage.type == "test_type"
    assert stage.name == "test_stage"

    # Missing required fields
    with pytest.raises(ValidationError):
        PipelineStage(type="test_type")  # type: ignore[call-arg]  # Intentionally missing required fields


def test_perspective_validation(basic_perspective_dict: Any) -> None:
    """Test perspective configuration validation."""
    # Valid perspective
    perspective = Perspective.model_validate(basic_perspective_dict)
    assert perspective.name == "test_perspective"
    assert len(perspective.pipeline.collect) == 1

    # Invalid perspective (missing required stages)
    invalid_config = basic_perspective_dict.copy()
    del invalid_config["pipeline"]["collect"]
    with pytest.raises(ValidationError):
        Perspective.model_validate(invalid_config)


def test_stage_type_enum() -> None:
    """Test stage type enumeration."""
    assert StageType.COLLECT.value == "collect"
    assert StageType.TRANSFORM.value == "transform"
    assert StageType.RENDER.value == "render"

    # Test all required stages are present
    required_stages = {"collect", "transform", "render"}
    enum_stages = {stage.value for stage in StageType}
    assert required_stages.issubset(enum_stages)
