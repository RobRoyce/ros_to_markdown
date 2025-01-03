#!/usr/bin/env python3

"""ROS graph analysis script for visualizing node relationships."""

import os
import subprocess
import sys
import time

from ros_to_markdown.core.ros_detector import ROSDetector
from ros_to_markdown.models.ros_components import ROSVersion


def ensure_ros_core():
    """Ensure ROS core/master is running."""
    try:
        # Try to run roscore as a subprocess
        roscore = subprocess.Popen(
            ["roscore"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        # Give roscore time to start
        time.sleep(3)
        return roscore
    except FileNotFoundError:
        print("Error: roscore command not found. Is ROS1 installed correctly?")
        return None


def ensure_ros_environment():
    """Ensure the appropriate ROS environment is sourced."""
    ros_version = ROSDetector.detect_ros_version()
    ros_distro = ROSDetector.get_ros_distro()

    if not ros_distro:
        print("Error: ROS_DISTRO environment variable not set")
        return False

    # If already sourced, continue with current process
    if os.environ.get("ROS_SOURCED"):
        return True

    # Check if we need to source the environment
    if ros_version in [ROSVersion.ROS1, ROSVersion.ROS2]:
        ros_setup = f"/opt/ros/{ros_distro}/setup.bash"
        if not os.path.exists(ros_setup):
            print(f"Error: ROS setup file not found at {ros_setup}")
            return False

        # Re-run this script with the ROS environment sourced
        cmd = f"bash -c 'source {ros_setup} && ROS_SOURCED=1 python3 {sys.argv[0]}'"
        result = subprocess.run(cmd, shell=True)
        # Exit the original process after launching the sourced one
        sys.exit(result.returncode)

    return True


def print_mermaid_style() -> str:
    """Return consistent Mermaid styling for both ROS1 and ROS2."""
    return """
    classDef default fill:#f9f9f9,stroke:#333,stroke-width:1px
    classDef publisher fill:#e1f3e1,stroke:#4CAF50,stroke-width:1px
    classDef subscriber fill:#e3f2fd,stroke:#2196F3,stroke-width:1px
    classDef pubsub fill:#f3e5f5,stroke:#9c27b0,stroke-width:1px
    classDef cycleNode fill:#fff0f0,stroke:#d43f3f,stroke-width:2px
    classDef riskNode fill:#ffe0e0,stroke:#ff0000,stroke-width:2px
    classDef topicEdge stroke:#666,stroke-width:1px
    classDef serviceEdge stroke:#666,stroke-width:1px,stroke-dasharray:5 5
    classDef actionEdge stroke:#666,stroke-width:2px
    classDef cycleEdge stroke:#d43f3f,stroke-width:2px
    classDef riskEdge stroke:#ff0000,stroke-width:3px,stroke-dasharray:5 5"""


def main():
    """Analyze and visualize the running ROS system."""
    # Ensure ROS environment is properly sourced
    if not ensure_ros_environment():
        return

    ros_version = ROSDetector.detect_ros_version()
    ros_distro = ROSDetector.get_ros_distro()
    roscore_process = None

    try:
        # Start roscore if needed for ROS1
        if ros_version == ROSVersion.ROS1:
            roscore_process = ensure_ros_core()
            if not roscore_process:
                return

        # Import appropriate analyzer based on version
        if ros_version == ROSVersion.ROS1:
            from ros_to_markdown.analyzers.ros1 import ROS1Analyzer
            analyzer = ROS1Analyzer()
        elif ros_version == ROSVersion.ROS2:
            from ros_to_markdown.analyzers.ros2 import ROS2Analyzer
            analyzer = ROS2Analyzer()
        else:
            print("Error: Unknown ROS version")
            return

        # Give ROS time to discover nodes
        time.sleep(2)

        # Analyze system
        graph = analyzer.analyze()
        if not graph:
            return

        # Print statistics
        print("\nSystem Statistics:")
        print(f"ROS Version: {ros_version.name}")
        print(f"ROS Distribution: {ros_distro}")
        print(f"Nodes: {len(graph.nodes)}")
        print(f"Topics: {len(graph.topics)}")
        print(f"Services: {len(graph.services)}")
        print(f"Edges: {len(graph.edges)}")

        # Generate visualizations with consistent styling
        print("\nMermaid Diagram:")
        print("```mermaid")
        print("graph LR")
        print(print_mermaid_style())
        print(
            graph.to_mermaid(highlight_cycles=True, show_message_types=True, include_styles=False)
        )
        print("```")

        print("\nComplete Documentation:")
        print(graph.to_markdown())

    except Exception as e:
        print(f"Error analyzing ROS system: {e}")
    finally:
        if roscore_process:
            roscore_process.terminate()


if __name__ == "__main__":
    main()
