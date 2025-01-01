import subprocess
import sys
from typing import List, Optional


def run_tests_in_container(ros_version: str, test_path: Optional[str] = None) -> bool:
    """Run tests in a specific ROS Docker container.
    
    Args:
        ros_version: The ROS version/distro to test (e.g., 'ros1', 'ros2-humble')
        test_path: Optional specific test path to run
        
    Returns:
        bool: True if tests passed, False otherwise
    """
    cmd = ['./docker/scripts/run-in-docker.sh', ros_version]
    
    if test_path:
        cmd.extend(['pytest', test_path])
    else:
        cmd.extend(['pytest', 'tests/core/test_ros_detector.py'])
    
    try:
        subprocess.run(cmd, check=True)
        return True
    except subprocess.CalledProcessError:
        return False


def main(test_path: Optional[str] = None) -> None:
    """Run tests across all ROS environments.
    
    Args:
        test_path: Optional specific test path to run
    """
    ros_versions = ['ros1', 'ros2-humble', 'ros2-iron']
    failed_versions: List[str] = []
    
    for version in ros_versions:
        print(f"\nRunning tests in {version} environment...")
        if not run_tests_in_container(version, test_path):
            failed_versions.append(version)
    
    if failed_versions:
        print("\nTests failed in the following environments:")
        for version in failed_versions:
            print(f"- {version}")
        sys.exit(1)
    else:
        print("\nAll tests passed in all environments!")


if __name__ == '__main__':
    test_path = sys.argv[1] if len(sys.argv) > 1 else None
    main(test_path) 