"""Test perspective model validation and configuration."""
import pytest
from pydantic import ValidationError

from ros_to_markdown.perspectives.models import (
    Perspective, PipelineStage, StageType
)


def test_pipeline_stage_validation():
    """Test pipeline stage configuration validation."""
    # Valid configuration
    stage = PipelineStage(
        type="test_type",
        name="test_stage",
        inputs=["input1"],
        output="output1"
    )
    assert stage.type == "test_type"
    assert stage.name == "test_stage"
    
    # Missing required fields
    with pytest.raises(ValidationError):
        PipelineStage(type="test_type")


def test_perspective_validation(basic_perspective_dict):
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


def test_stage_type_enum():
    """Test stage type enumeration."""
    assert StageType.COLLECT == "collect"
    assert StageType.TRANSFORM == "transform"
    assert StageType.RENDER == "render"
    
    # Test all required stages are present
    required_stages = {"collect", "transform", "render"}
    enum_stages = {stage.value for stage in StageType}
    assert required_stages.issubset(enum_stages) 