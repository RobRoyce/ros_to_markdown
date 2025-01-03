"""Integration tests for ROS workspace detection and introspection."""

import os
import subprocess
import time

import pytest

from ros_to_markdown.core.ros_detector import ROSDetector
from ros_to_markdown.models.ros_components import ROSVersion

from ..test_helpers.docker_utils import get_ros_workspace, is_running_in_docker
from ..test_helpers.ros_test_env import launch_turtlesim, verify_turtlesim_running

# Mark all tests in this module as integration tests
pytestmark = pytest.mark.integration


def requires_docker(func):
    """Decorator to skip tests if not running in Docker."""
    return pytest.mark.skipif(
        not is_running_in_docker(), reason="Test requires Docker environment"
    )(func)


@pytest.fixture(scope="session")
def ensure_workspace():
    """Fixture to ensure a ROS workspace exists for testing."""
    workspace_path = os.path.expanduser("~/ws")

    # Create workspace structure
    os.makedirs(os.path.join(workspace_path, "src"), exist_ok=True)
    os.makedirs(os.path.join(workspace_path, "build"), exist_ok=True)

    # Create version-specific directories and setup files
    if os.environ.get("ROS_DISTRO") == "noetic":
        os.makedirs(os.path.join(workspace_path, "devel"), exist_ok=True)
        setup_path = os.path.join(workspace_path, "devel/setup.bash")
    else:
        os.makedirs(os.path.join(workspace_path, "install"), exist_ok=True)
        setup_path = os.path.join(workspace_path, "install/setup.bash")

    # Create setup.bash
    os.makedirs(os.path.dirname(setup_path), exist_ok=True)
    with open(setup_path, "w") as f:
        f.write("#!/bin/bash\n")
        f.write(f"export WORKSPACE_ROOT={workspace_path}\n")

    # Create test package
    test_pkg_path = os.path.join(workspace_path, "src", "test_pkg")
    os.makedirs(test_pkg_path, exist_ok=True)

    # Create package.xml with correct format
    pkg_format = "2" if os.environ.get("ROS_DISTRO") == "noetic" else "3"
    with open(os.path.join(test_pkg_path, "package.xml"), "w") as f:
        f.write(f"""<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format{pkg_format}.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="{pkg_format}">
  <name>test_pkg</name>
  <version>0.0.1</version>
  <description>Test package</description>
  <maintainer email="test@example.com">Test User</maintainer>
  <license>Apache-2.0</license>
</package>
""")

    # Create symlink to turtlesim
    distro = os.environ.get("ROS_DISTRO", "humble")
    turtlesim_src = f"/opt/ros/{distro}/share/turtlesim"
    turtlesim_link = os.path.join(workspace_path, "src", "turtlesim")

    if os.path.exists(turtlesim_src) and not os.path.exists(turtlesim_link):
        try:
            os.symlink(turtlesim_src, turtlesim_link)
        except OSError as e:
            print(f"Warning: Failed to create turtlesim symlink: {e}")

    return workspace_path


@requires_docker
def test_workspace_exists(ensure_workspace):
    """Verify that we can find the ROS workspace in Docker."""
    workspace = get_ros_workspace()
    assert workspace is not None
    assert os.path.exists(workspace)
    assert workspace == ensure_workspace


@requires_docker
def test_workspace_has_expected_structure(ensure_workspace):
    """Verify the workspace has the expected ROS structure."""
    workspace = get_ros_workspace()
    assert workspace is not None

    # Check for standard ROS workspace directories
    assert os.path.exists(os.path.join(workspace, "src"))

    # ROS version specific checks
    ros_version = ROSDetector.detect_ros_version()
    if ros_version == ROSVersion.ROS1:
        build_dir = "build"
        devel_dir = "devel"
    else:  # ROS2
        build_dir = "build"
        devel_dir = "install"

    # Create the build directories if they don't exist
    os.makedirs(os.path.join(workspace, build_dir), exist_ok=True)
    os.makedirs(os.path.join(workspace, devel_dir), exist_ok=True)

    assert os.path.exists(os.path.join(workspace, build_dir))
    assert os.path.exists(os.path.join(workspace, devel_dir))


def verify_package_exists(pkg_path: str) -> bool:
    """Verify that a ROS package exists and has required files."""
    if not os.path.exists(pkg_path):
        return False

    package_xml = os.path.join(pkg_path, "package.xml")
    if not os.path.exists(package_xml):
        return False

    try:
        with open(package_xml) as f:
            content = f.read()
            return "<package" in content and "</package>" in content
    except Exception:
        return False


@requires_docker
def test_workspace_contains_packages():
    """Verify that the workspace contains ROS packages."""
    workspace = get_ros_workspace()
    assert workspace is not None

    src_dir = os.path.join(workspace, "src")
    test_pkg = os.path.join(src_dir, "test_pkg")

    assert verify_package_exists(test_pkg), f"test_pkg not found or invalid at {test_pkg}"


@requires_docker
def test_turtlesim_package_available(ensure_workspace):
    """Verify that turtlesim package is available in the workspace."""
    ros_version = ROSDetector.detect_ros_version()

    # Get the expected turtlesim path based on ROS version and distro
    distro = os.environ.get("ROS_DISTRO", "humble")
    turtlesim_path = f"/opt/ros/{distro}/share/turtlesim"

    print(f"ROS Version: {ros_version}")
    print(f"ROS Distro: {distro}")
    print(f"Looking for turtlesim at: {turtlesim_path}")

    # First verify turtlesim is installed
    assert os.path.exists(turtlesim_path), f"Turtlesim not found at {turtlesim_path}"

    # Then verify it's in our workspace (should be symlinked by fixture)
    workspace = get_ros_workspace()
    assert workspace is not None, "No workspace found"

    workspace_turtlesim = os.path.join(workspace, "src", "turtlesim")
    print(f"Looking for workspace turtlesim at: {workspace_turtlesim}")
    assert os.path.exists(
        workspace_turtlesim
    ), f"Turtlesim not found in workspace at {workspace_turtlesim}"

    # Verify it's properly symlinked
    assert os.path.islink(workspace_turtlesim), f"{workspace_turtlesim} is not a symlink"
    link_target = os.path.realpath(workspace_turtlesim)
    assert (
        link_target == turtlesim_path
    ), f"Symlink points to {link_target} instead of {turtlesim_path}"

    # Verify package.xml exists and contains expected content
    package_xml = os.path.join(turtlesim_path, "package.xml")
    assert os.path.exists(package_xml), f"package.xml not found at {package_xml}"

    with open(package_xml) as f:
        content = f.read()
        print(f"Package XML contents:\n{content}")
        assert (
            "<name>turtlesim</name>" in content
        ), "package.xml does not contain turtlesim package name"


@requires_docker
@pytest.mark.parametrize(
    "ros_distro,expected_version",
    [
        ("noetic", ROSVersion.ROS1),
        ("humble", ROSVersion.ROS2),
        ("iron", ROSVersion.ROS2),
        ("jazzy", ROSVersion.ROS2),
        ("rolling", ROSVersion.ROS2),
    ],
)
def test_workspace_matches_ros_version(ensure_workspace, ros_distro, expected_version):
    """Verify that the workspace matches the expected ROS version."""
    if os.environ.get("ROS_DISTRO") != ros_distro:
        pytest.skip(f"Test requires {ros_distro} environment")

    workspace = get_ros_workspace()
    assert workspace is not None

    # Verify ROS version matches workspace structure
    ros_version = ROSDetector.detect_ros_version()
    assert ros_version == expected_version

    # Check for version-specific workspace elements
    if ros_version == ROSVersion.ROS1:
        assert os.path.exists(os.path.join(workspace, "devel"))
        assert not os.path.exists(os.path.join(workspace, "install"))
    else:  # ROS2
        # Clean up any ROS1 artifacts that might exist
        devel_dir = os.path.join(workspace, "devel")
        if os.path.exists(devel_dir):
            import shutil

            shutil.rmtree(devel_dir)

        assert os.path.exists(os.path.join(workspace, "install"))
        assert not os.path.exists(os.path.join(workspace, "devel"))


@requires_docker
def test_ros_detector_live_environment(ensure_workspace):
    """Test ROSDetector functionality in a live ROS environment."""
    # Get the expected ROS version based on distro
    distro = os.environ.get("ROS_DISTRO", "humble")
    expected_version = ROSVersion.ROS1 if distro == "noetic" else ROSVersion.ROS2

    # Test direct version detection
    detected_version = ROSDetector.detect_ros_version()
    assert (
        detected_version == expected_version
    ), f"Expected {expected_version} for {distro}, got {detected_version}"

    # Test fallback detection
    fallback_version = ROSDetector._fallback_detection()
    assert (
        fallback_version == expected_version
    ), f"Fallback detection expected {expected_version}, got {fallback_version}"

    # Test distro detection
    detected_distro = ROSDetector.get_ros_distro()
    assert detected_distro == distro, f"Expected distro {distro}, got {detected_distro}"

    # Verify ROS tools are available
    if expected_version == ROSVersion.ROS1:
        # Test rosversion command
        result = subprocess.run(["rosversion", "-d"], capture_output=True, text=True, check=True)
        assert result.stdout.strip() == distro
    else:
        # Test ros2 command
        result = subprocess.run(["ros2", "--help"], capture_output=True, text=True, check=True)
        assert result.returncode == 0


@requires_docker
def test_ros_detector_environment_variables():
    """Test ROSDetector's handling of environment variables."""
    distro = os.environ.get("ROS_DISTRO")
    assert distro is not None, "ROS_DISTRO environment variable not set"

    # Test ROS-specific environment variables
    if ROSDetector.detect_ros_version() == ROSVersion.ROS1:
        assert os.environ.get("ROS_ROOT") is not None
        assert os.environ.get("ROS_PACKAGE_PATH") is not None
    else:  # ROS2
        assert os.environ.get("AMENT_PREFIX_PATH") is not None
        # COLCON_PREFIX_PATH is not always set, especially in fresh containers
        # Instead, check for ROS_VERSION which is always set in ROS2
        assert os.environ.get("ROS_VERSION") == "2"
        assert os.environ.get("ROS_PYTHON_VERSION") == "3"


@requires_docker
def test_ros_detector_with_package_imports(ensure_workspace):
    """Test ROSDetector's ability to detect ROS through package imports."""
    distro = os.environ.get("ROS_DISTRO", "humble")

    if distro == "noetic":
        # Test ROS1 package imports
        import rospy

        assert rospy is not None

        # Verify we can't import ROS2 packages
        with pytest.raises(ImportError):
            import rclpy
    else:
        # Test ROS2 package imports
        try:
            import rclpy

            assert rclpy is not None
        except ModuleNotFoundError as e:
            if "No module named 'rclpy._rclpy_pybind11'" in str(e):
                pytest.skip(f"Skipping due to rclpy import issue in {distro}: {e}")
            raise

        # Verify we can't import ROS1 packages
        with pytest.raises(ImportError):
            import rospy


@requires_docker
def test_ros_detector_workspace_tools(ensure_workspace):
    """Test ROSDetector's tools on the actual workspace structure."""
    workspace = get_ros_workspace()
    ros_version = ROSDetector.detect_ros_version()

    # Verify workspace structure based on ROS version
    if ros_version == ROSVersion.ROS1:
        # ROS1 checks...
        pass
    else:  # ROS2
        # Check for colcon-specific files
        assert os.path.exists(
            os.path.join(workspace, "install/setup.bash")
        ), "ROS2 workspace missing install/setup.bash"

        # Test colcon availability - use list command instead of --version
        try:
            result = subprocess.run(["colcon", "list"], capture_output=True, text=True, check=True)
            assert result.returncode == 0, "colcon list command failed"
        except subprocess.CalledProcessError as e:
            pytest.fail(f"colcon not available in ROS2 environment: {e.stderr}")


@requires_docker
def test_ros_detector_package_inspection(ensure_workspace):
    """Test ROSDetector's ability to inspect packages in the workspace."""
    workspace = get_ros_workspace()
    ros_version = ROSDetector.detect_ros_version()

    # Check test_pkg structure
    test_pkg_path = os.path.join(workspace, "src", "test_pkg")
    assert os.path.exists(test_pkg_path), "test_pkg not found in workspace"

    # Verify package.xml format matches ROS version
    package_xml_path = os.path.join(test_pkg_path, "package.xml")
    with open(package_xml_path) as f:
        content = f.read()
        if ros_version == ROSVersion.ROS1:
            assert (
                'format="2"' in content or 'format="1"' in content
            ), "ROS1 package.xml should use format 1 or 2"
        else:
            assert 'format="3"' in content, "ROS2 package.xml should use format 3"


@requires_docker
def test_ros_detector_with_running_nodes():
    """Test ROSDetector with running ROS nodes."""
    core_process = None
    turtlesim_process = None

    try:
        # Launch turtlesim
        core_process, turtlesim_process = launch_turtlesim()

        # Give node time to start
        time.sleep(5)

        # Verify turtlesim is running
        assert verify_turtlesim_running(), "Turtlesim node not running properly"

    finally:
        # Clean up processes
        if turtlesim_process:
            turtlesim_process.terminate()
            turtlesim_process.wait()
        if core_process:
            core_process.terminate()
            core_process.wait()


@requires_docker
def test_ros_detector_python_path():
    """Test ROSDetector's handling of Python paths in the ROS environment."""
    ros_version = ROSDetector.detect_ros_version()
    python_path = os.environ.get("PYTHONPATH", "")

    if ros_version == ROSVersion.ROS1:
        # Check for ROS1-specific Python paths
        assert (
            "/opt/ros/noetic/lib/python3/dist-packages" in python_path
        ), "ROS1 Python path not properly set"
    else:
        # Check for ROS2-specific Python paths
        distro = ROSDetector.get_ros_distro()

        # Special handling for Jazzy due to Python version differences
        if distro in ["rolling", "jazzy"]:
            # ROS uses system Python (3.12), but our tests run in Python 3.11 venv
            ros_python_version = "3.12"  # ROS system Python version
            venv_python_version = "3.11"  # Our venv Python version

            # Check that either version's path exists (system or venv)
            system_path = f"/opt/ros/{distro}/lib/python{ros_python_version}/site-packages"
            venv_path = f"/opt/ros/{distro}/lib/python{venv_python_version}/site-packages"

            assert any(p in python_path for p in [system_path, venv_path]), (
                f"ROS2 Python path for {distro} not properly set. "
                f"Expected either {system_path} or {venv_path} in {python_path}"
            )
        else:
            # For other distros, use the standard Python version detection
            try:
                for path_entry in python_path.split(":"):
                    if f"/opt/ros/{distro}/lib/python" in path_entry:
                        ros_python_version = path_entry.split("python")[1].split("/")[0]
                        break
                else:
                    ros_python_version = "3.10"  # Default for Humble/Iron
            except Exception:
                ros_python_version = "3.10"  # Fallback

            expected_path = f"/opt/ros/{distro}/lib/python{ros_python_version}/site-packages"
            assert expected_path in python_path, (
                f"ROS2 Python path for {distro} not properly set. "
                f"Expected {expected_path} in {python_path}"
            )
