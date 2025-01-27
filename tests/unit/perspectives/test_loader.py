"""Test perspective loading functionality."""

from pathlib import Path
from typing import Any

from ros_to_markdown.perspectives.loader import load_perspective
from ros_to_markdown.perspectives.models import Perspective

import pytest
import yaml


def test_load_builtin_perspective(tmp_path: Path, monkeypatch: Any) -> None:
    """Test loading a builtin perspective."""
    # Create mock builtin perspective
    builtin_dir = tmp_path / "definitions"
    builtin_dir.mkdir(parents=True)

    perspective_file = builtin_dir / "basic.yaml"
    perspective_file.write_text("""
name: basic
version: "1.0"
description: "Test perspective"
pipeline:
  collect:
    - type: system_snapshot
      name: collect_system_state
      inputs: ["analyzer"]
      output: system_state
  transform:
    - type: graph_builder
      name: build_system_graph
      inputs: ["system_state"]
      output: system_graph
  render:
    - type: markdown
      name: generate_overview
      inputs: ["system_graph"]
      output: overview_doc
compatibility:
  min_version: "1.0"
  max_version: "2.0"
  deprecated_features: []
    """)

    # Mock the builtin directory path
    def mock_get_builtin_dir(*args: Any) -> Path:
        return tmp_path

    monkeypatch.setattr(
        "ros_to_markdown.perspectives.loader.Path.parent", property(lambda _: tmp_path)
    )

    perspective = load_perspective("basic")
    assert isinstance(perspective, Perspective)
    assert perspective.name == "basic"


def test_load_user_perspective(tmp_path: Path) -> None:
    """Test loading a user-defined perspective."""
    # Create mock user perspective
    user_dir = tmp_path / "user_perspectives"
    user_dir.mkdir()

    perspective_file = user_dir / "custom.yaml"
    perspective_file.write_text("""
name: custom
version: "1.0"
description: "Custom perspective"
pipeline:
  collect:
    - type: system_snapshot
      name: collect_system_state
      inputs: ["analyzer"]
      output: system_state
  transform:
    - type: graph_builder
      name: build_system_graph
      inputs: ["system_state"]
      output: system_graph
  render:
    - type: markdown
      name: generate_overview
      inputs: ["system_graph"]
      output: overview_doc
compatibility:
  min_version: "1.0"
  max_version: "2.0"
  deprecated_features: []
    """)

    with pytest.MonkeyPatch.context() as mp:
        mp.setenv("ROS_TO_MARKDOWN_PERSPECTIVES", str(user_dir))
        perspective = load_perspective("custom")
        assert isinstance(perspective, Perspective)
        assert perspective.name == "custom"


def test_perspective_not_found() -> None:
    """Test error handling when perspective is not found."""
    with pytest.raises(ValueError, match="Perspective not found"):
        load_perspective("nonexistent")


def test_invalid_perspective(tmp_path: Path, monkeypatch: Any) -> None:
    """Test loading an invalid perspective."""
    # Create a temporary builtin directory
    builtin_dir = tmp_path / "definitions"
    builtin_dir.mkdir()

    # Create an invalid perspective file
    perspective_file = builtin_dir / "invalid.yaml"
    perspective_file.write_text("invalid: yaml: : content")

    # Mock the builtin directory path
    monkeypatch.setattr(
        "ros_to_markdown.perspectives.loader.Path.parent", property(lambda _: tmp_path)
    )

    with pytest.raises((ValueError, yaml.scanner.ScannerError)):
        load_perspective("invalid")
