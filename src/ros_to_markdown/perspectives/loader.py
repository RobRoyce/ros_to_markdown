import os
from pathlib import Path
from typing import Optional

from pydantic import ValidationError
import yaml

from ..logging import get_logger
from .models import Perspective

logger = get_logger(__name__)

def get_builtin_perspective_path(name: str) -> Optional[Path]:
    """Get path to a builtin perspective definition."""
    builtin_dir = Path(__file__).parent / "definitions"
    perspective_path = builtin_dir / f"{name}.yaml"

    if perspective_path.exists():
        return perspective_path
    return None

def load_perspective(name: str) -> Perspective:
    """
    Load a perspective definition by name.
    Checks builtin perspectives first, then looks for user-defined ones.
    """
    # Try builtin perspectives first
    if perspective_path := get_builtin_perspective_path(name):
        logger.debug(f"Loading builtin perspective: {name}")
    else:
        # Look for user-defined perspective
        user_perspective_dir = Path(os.getenv("ROS_TO_MARKDOWN_PERSPECTIVES", ""))
        perspective_path = user_perspective_dir / f"{name}.yaml"
        if not perspective_path.exists():
            raise ValueError(f"Perspective not found: {name}")
        logger.debug(f"Loading user perspective: {name}")

    try:
        with open(perspective_path) as f:
            perspective_dict = yaml.safe_load(f)
            return Perspective.model_validate(perspective_dict)
    except ValidationError as e:
        logger.error(f"Invalid perspective definition: {e}")
        raise
    except Exception as e:
        logger.error(f"Error loading perspective: {e}")
        raise
