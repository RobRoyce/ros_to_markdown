import argparse
from typing import Any


def parse_args() -> Any:
    """Parse command line arguments into configuration."""
    parser = argparse.ArgumentParser(description="ROS to Markdown")
    parser.add_argument("--input", type=str, help="Input ROS bag file")
    parser.add_argument("--output", type=str, help="Output Markdown file")
    return parser.parse_args()
