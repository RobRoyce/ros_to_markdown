import subprocess
import sys
from typing import List, Optional
import os


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
    
    if os.getenv('PYTEST_ADDOPTS'):
        cmd.extend(os.getenv('PYTEST_ADDOPTS').split())
    
    try:
        result = subprocess.run(cmd, check=False, capture_output=True, text=True)
        if result.returncode != 0:
            print(f"Error in {ros_version} environment:")
            print(result.stderr)
            return False
        print(result.stdout)
        if result.stderr:
            print(result.stderr)
        return True
    except Exception as e:
        print(f"Failed to run tests in {ros_version} environment: {str(e)}")
        return False


def main(test_path: Optional[str] = None) -> None:
    """Run tests across all ROS environments.
    
    Args:
        test_path: Optional specific test path to run
    """
    ros_versions = ['ros1', 'ros2-humble', 'ros2-iron', 'ros2-jazzy']
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