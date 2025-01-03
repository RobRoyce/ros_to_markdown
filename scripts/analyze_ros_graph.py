#!/usr/bin/env python3

"""ROS1 graph analysis script for visualizing node relationships."""

from ros_to_markdown.core.ros_detector import ROSDetector
from ros_to_markdown.models.ros_components import ROSVersion


def main():
    """Analyze and visualize the running ROS system."""
    try:
        # Detect ROS version first
        detector = ROSDetector()
        ros_version = detector.detect_ros_version()

        # Import appropriate analyzer based on version
        if ros_version == ROSVersion.ROS1:
            from ros_to_markdown.analyzers.ros1 import ROS1Analyzer

            analyzer = ROS1Analyzer()
        else:
            from ros_to_markdown.analyzers.ros2 import ROS2Analyzer

            analyzer = ROS2Analyzer()

        # Analyze system
        graph = analyzer.analyze()
        if not graph:
            return

        # Print statistics
        print("\nSystem Statistics:")
        print(f"Nodes: {len(graph.nodes)}")
        print(f"Topics: {len(graph.topics)}")
        print(f"Services: {len(graph.services)}")
        print(f"Edges: {len(graph.edges)}")

        # Generate visualizations
        print("\nMermaid Diagram:")
        print("```mermaid")
        print(graph.to_mermaid(highlight_cycles=True, show_message_types=True))
        print("```")

        print("\nComplete Documentation:")
        print(graph.to_markdown())

    except Exception as e:
        print(f"Error analyzing ROS system: {e}")


if __name__ == "__main__":
    main()
