import os
from pathlib import Path
from typing import Dict, Generator
from unittest.mock import patch

from ros_to_markdown.core.ros_detector import ROSDetector
from ros_to_markdown.models.ros_components import ROSVersion

import pytest

try:
    import rospy
except ImportError:
    pass


@pytest.fixture
def ros_env_config() -> Dict[str, any]:
    """Shared ROS environment configurations."""
    return {
        "ros1": {
            "distro": "noetic",
            "env": {
                "ROS_DISTRO": "noetic",
                "ROS_ROOT": "/opt/ros/noetic/share/ros",
                "ROS_PACKAGE_PATH": "/opt/ros/noetic/share",
            },
            "workspace_dirs": ["devel", "build", "src"],
        },
        "ros2": {
            "distro": "humble",
            "env": {
                "ROS_DISTRO": "humble",
                "AMENT_PREFIX_PATH": "/opt/ros/humble",
                "ROS_PYTHON_VERSION": "3",
            },
            "workspace_dirs": ["install", "build", "src"],
        },
    }


@pytest.fixture
def mock_ros_env(ros_env_config: Dict[str, any]) -> any:
    """Configurable ROS environment fixture."""

    class ROSEnvContext:
        def __init__(self, ros_version: str) -> None:
            self.ros_version = ros_version
            self.env = ros_env_config[ros_version]["env"]

        def __enter__(self) -> "ROSEnvContext":
            self.patcher = patch.dict(os.environ, self.env)
            self.patcher.start()
            return self

        def __exit__(self, exc_type: any, exc_val: any, exc_tb: any) -> None:
            self.patcher.stop()

    def _mock_env(ros_version: str) -> ROSEnvContext:
        return ROSEnvContext(ros_version)

    return _mock_env


@pytest.fixture
def mock_workspace() -> Generator:
    """Configurable workspace structure fixture."""

    class WorkspaceContext:
        def __init__(self, ros_version: str, workspace_path: str = "/workspace") -> None:
            self.ros_version = ros_version
            self.workspace = Path(workspace_path)

            # Map ROS2 distros to their configurations
            self.ros2_configs = {
                "humble": {
                    "python_version": "3.10",
                    "path": "/opt/ros/humble",
                },
                "iron": {
                    "python_version": "3.10",
                    "path": "/opt/ros/iron",
                },
                "jazzy": {
                    "python_version": "3.12",
                    "path": "/opt/ros/jazzy",
                },
            }

        def __enter__(self) -> Path:
            def path_exists(path: str) -> bool:
                path = Path(path)
                if self.ros_version == "ros1":
                    return any(
                        [
                            path == self.workspace / "devel" / "setup.bash",
                            path == self.workspace / "build",
                            path == self.workspace / "src",
                        ]
                    )
                else:
                    return any(
                        [
                            path == self.workspace / "install" / "setup.bash",
                            path == self.workspace / "build",
                            path == self.workspace / "src",
                        ]
                    )

            # Create patchers
            self.path_patcher = patch("os.path.exists")

            # Set up environment based on ROS version
            env_vars = {}
            if self.ros_version == "ros1":
                env_vars.update(
                    {
                        "ROS_DISTRO": "noetic",
                        "ROS_ROOT": "/opt/ros/noetic/share/ros",
                        "ROS_PACKAGE_PATH": "/opt/ros/noetic/share",
                        "AMENT_PREFIX_PATH": "",
                    }
                )
            else:
                # Extract ROS2 distro from version string (e.g., "ros2-humble" -> "humble")
                distro = self.ros_version.split("-")[-1] if "-" in self.ros_version else "humble"
                config = self.ros2_configs.get(distro, self.ros2_configs["humble"])

                env_vars.update(
                    {
                        "ROS_DISTRO": distro,
                        "ROS_ROOT": "",
                        "ROS_PACKAGE_PATH": "",
                        "AMENT_PREFIX_PATH": config["path"],
                        "PYTHONPATH": (
                            f"{config['path']}/lib/python"
                            f"{config['python_version']}/site-packages"
                        ),
                    }
                )

            self.env_patcher = patch.dict(os.environ, env_vars)

            # Start patchers
            mock_exists = self.path_patcher.start()
            mock_exists.side_effect = path_exists
            self.env_patcher.start()

            return self.workspace

        def __exit__(self, exc_type: any, exc_val: any, exc_tb: any) -> None:
            self.path_patcher.stop()
            self.env_patcher.stop()

    def _mock_workspace(ros_version: str, workspace_path: str = "/workspace") -> WorkspaceContext:
        return WorkspaceContext(ros_version, workspace_path)

    return _mock_workspace


@pytest.fixture(scope="class")
def ensure_workspace(request: pytest.FixtureRequest, ros_env_config: Dict[str, any]) -> Path:
    """Create actual workspace based on ROS version."""
    workspace = Path("/home/ros/ws")
    ros_version = ROSDetector.detect_ros_version()
    config = ros_env_config["ros1" if ros_version == ROSVersion.ROS1 else "ros2"]

    # Create base directories
    for dir_name in config["workspace_dirs"]:
        (workspace / dir_name).mkdir(parents=True, exist_ok=True)

    # Create setup.bash
    setup_dir = "devel" if ros_version == ROSVersion.ROS1 else "install"
    setup_path = workspace / setup_dir / "setup.bash"
    setup_path.parent.mkdir(parents=True, exist_ok=True)

    with setup_path.open("w") as f:
        f.write("#!/bin/bash\n")
        if ros_version == ROSVersion.ROS2:
            f.write(f"export WORKSPACE_ROOT={workspace}\n")

    return workspace


@pytest.fixture
def mock_package_xml() -> str:
    """Mock package.xml content."""
    return """<?xml version="1.0"?>
<package format="2">
  <name>test_pkg</name>
  <version>0.0.1</version>
  <description>Test package</description>
  <maintainer email="test@example.com">Test User</maintainer>
  <license>Apache-2.0</license>
</package>
"""


@pytest.fixture(autouse=True)
def rospy_shutdown() -> Generator:
    """Ensure rospy is shutdown after each test."""
    yield
    if "rospy" in globals():
        if rospy.get_node_uri():
            rospy.shutdown()
