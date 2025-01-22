from enum import Enum
from typing import Dict, List, Optional, Union
from pydantic import BaseModel, Field


class StageType(str, Enum):
    """Types of pipeline stages."""
    COLLECT = "collect"
    TRANSFORM = "transform"
    ANALYZE = "analyze"
    RENDER = "render"


class PipelineInput(BaseModel):
    """Input specification for a pipeline stage."""
    source: str
    required: bool = True
    transform: Optional[str] = None


class PipelineStage(BaseModel):
    """Base configuration for a pipeline stage."""
    type: str
    name: str = Field(..., description="Unique name for this stage")
    inputs: List[Union[str, PipelineInput]]
    output: str
    config: Optional[Dict] = None


class PipelineConfig(BaseModel):
    """Pipeline configuration within a perspective."""
    collect: List[PipelineStage]
    transform: Optional[List[PipelineStage]] = None
    analyze: Optional[List[PipelineStage]] = None
    render: List[PipelineStage]


class CompatibilityConfig(BaseModel):
    """Version compatibility configuration."""
    min_version: str
    max_version: str
    deprecated_features: List[str] = Field(default_factory=list)


class Perspective(BaseModel):
    """Complete perspective definition."""
    name: str
    version: str
    description: str
    pipeline: PipelineConfig
    compatibility: CompatibilityConfig 