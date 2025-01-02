"""Integration tests for ROS workspace detection and introspection."""

import os
import pytest
from ros_to_markdown.core.ros_detector import ROSDetector
from ros_to_markdown.models.ros_components import ROSVersion
from ..test_helpers.docker_utils import is_running_in_docker, get_ros_workspace


# Mark all tests in this module as integration tests
pytestmark = pytest.mark.integration

def requires_docker(func):
    """Decorator to skip tests if not running in Docker."""
    return pytest.mark.skipif(
        not is_running_in_docker(),
        reason="Test requires Docker environment"
    )(func)


@pytest.fixture(scope="session")
def ensure_workspace():
    """Fixture to ensure a ROS workspace exists for testing."""
    workspace_path = os.path.expanduser("~/ws")
    
    # Create workspace structure
    os.makedirs(os.path.join(workspace_path, "src"), exist_ok=True)
    
    # Create a dummy package.xml for testing
    pkg_path = os.path.join(workspace_path, "src", "test_pkg")
    os.makedirs(pkg_path, exist_ok=True)
    
    with open(os.path.join(pkg_path, "package.xml"), "w") as f:
        f.write("""<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>test_pkg</name>
  <version>0.0.1</version>
  <description>Test package</description>
  <maintainer email="test@example.com">Test User</maintainer>
  <license>Apache-2.0</license>
</package>
""")

    # Get turtlesim path based on ROS distro
    distro = os.environ.get("ROS_DISTRO", "humble")
    turtlesim_src = f"/opt/ros/{distro}/share/turtlesim"
    
    if os.path.exists(turtlesim_src):
        turtlesim_dest = os.path.join(workspace_path, "src", "turtlesim")
        
        # Remove existing symlink if it's pointing to the wrong place
        if os.path.islink(turtlesim_dest):
            current_target = os.path.realpath(turtlesim_dest)
            if current_target != turtlesim_src:
                os.unlink(turtlesim_dest)
        
        # Create symlink if needed
        if not os.path.exists(turtlesim_dest):
            try:
                os.symlink(turtlesim_src, turtlesim_dest)
                print(f"Created symlink from {turtlesim_src} to {turtlesim_dest}")
            except OSError as e:
                print(f"Failed to create symlink: {e}")
    else:
        print(f"Warning: Turtlesim source not found at {turtlesim_src}")
            
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
    assert os.path.exists(os.path.join(workspace, 'src'))
    
    # ROS version specific checks
    ros_version = ROSDetector.detect_ros_version()
    if ros_version == ROSVersion.ROS1:
        build_dir = 'build'
        devel_dir = 'devel'
    else:  # ROS2
        build_dir = 'build'
        devel_dir = 'install'
    
    # Create the build directories if they don't exist
    os.makedirs(os.path.join(workspace, build_dir), exist_ok=True)
    os.makedirs(os.path.join(workspace, devel_dir), exist_ok=True)
    
    assert os.path.exists(os.path.join(workspace, build_dir))
    assert os.path.exists(os.path.join(workspace, devel_dir))


@requires_docker
def test_workspace_contains_packages():
    """Verify that the workspace contains ROS packages."""
    workspace = get_ros_workspace()
    assert workspace is not None
    
    src_dir = os.path.join(workspace, 'src')
    
    # Look for package.xml files (both ROS1 and ROS2 use these)
    found_packages = False
    for root, _, files in os.walk(src_dir):
        if 'package.xml' in files:
            found_packages = True
            break
    
    assert found_packages, "No ROS packages found in workspace"


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
    assert os.path.exists(workspace_turtlesim), f"Turtlesim not found in workspace at {workspace_turtlesim}"
    
    # Verify it's properly symlinked
    assert os.path.islink(workspace_turtlesim), f"{workspace_turtlesim} is not a symlink"
    link_target = os.path.realpath(workspace_turtlesim)
    assert link_target == turtlesim_path, f"Symlink points to {link_target} instead of {turtlesim_path}"
    
    # Verify package.xml exists and contains expected content
    package_xml = os.path.join(turtlesim_path, "package.xml")
    assert os.path.exists(package_xml), f"package.xml not found at {package_xml}"
    
    with open(package_xml) as f:
        content = f.read()
        print(f"Package XML contents:\n{content}")
        assert "<name>turtlesim</name>" in content, "package.xml does not contain turtlesim package name"


@requires_docker
@pytest.mark.parametrize("ros_distro,expected_version", [
    ("noetic", ROSVersion.ROS1),
    ("humble", ROSVersion.ROS2),
    ("iron", ROSVersion.ROS2),
    ("rolling", ROSVersion.ROS2),
])
def test_workspace_matches_ros_version(ros_distro, expected_version):
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
        assert os.path.exists(os.path.join(workspace, 'devel'))
        assert not os.path.exists(os.path.join(workspace, 'install'))
    else:  # ROS2
        assert os.path.exists(os.path.join(workspace, 'install'))
        assert not os.path.exists(os.path.join(workspace, 'devel')) 