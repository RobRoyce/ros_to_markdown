"""Docker test runner for executing tests across ROS environments."""

import os
import subprocess
import sys
from pathlib import Path
from typing import ClassVar, Dict, List, Optional


class DockerTestRunner:
    """Manages test execution across different ROS Docker environments."""

    ENVIRONMENTS: ClassVar[Dict[str, str]] = {
        "ros1": "noetic",
        "ros2-humble": "humble",
        "ros2-iron": "iron",
        "ros2-jazzy": "jazzy",
    }

    def __init__(self, test_path: Optional[str] = None):
        """Initialize the test runner.

        Args:
            test_path: Optional path to specific test file or directory
        """
        self.test_path = test_path or "tests/"
        self.docker_script = "./docker/scripts/run-in-docker.sh"

    def run_tests(self) -> bool:
        """Run tests in all environments."""
        all_passed = True

        for env, distro in self.ENVIRONMENTS.items():
            print(f"\nRunning tests in {env} environment...")

            # Set environment variables for the test run
            test_env = os.environ.copy()
            test_env["ROS_DISTRO"] = distro
            test_env["PYTEST_ADDOPTS"] = "-v --strict-markers"
            test_env["ROS_WORKSPACE"] = "/home/ros/ws"
            test_env["COVERAGE_FILE"] = ".coverage"

            try:
                # Run pytest with coverage
                subprocess.run(
                    [
                        self.docker_script,
                        env,
                        "python3",
                        "-m",
                        "pytest",
                        "--cov=src/ros_to_markdown",
                        "--cov-config=.coveragerc",
                        "--cov-append",
                        self.test_path,
                    ],
                    env=test_env,
                    check=True,
                )
            except subprocess.CalledProcessError:
                print(f"Tests failed in {env} environment")
                all_passed = False

        return all_passed

    def combine_coverage(self):
        """Combine coverage data from multiple test runs."""
        try:
            # Install coverage if needed (directly, without sudo)
            subprocess.run(
                [
                    self.docker_script,
                    "ros1",
                    "python3",
                    "-m",
                    "pip",
                    "install",
                    "--user",
                    "coverage",
                ],
                check=True,
            )

            # Combine coverage data
            subprocess.run(
                [self.docker_script, "ros1", "python3", "-m", "coverage", "combine"], check=True
            )

            # Generate report
            subprocess.run(
                [self.docker_script, "ros1", "python3", "-m", "coverage", "report"], check=True
            )
        except subprocess.CalledProcessError as e:
            print(f"Warning: Failed to combine coverage data: {e}")


def setup_coverage():
    """Ensure coverage directory exists and is writable."""
    coverage_dir = Path("/workspace")
    coverage_dir.mkdir(parents=True, exist_ok=True)
    os.chmod(coverage_dir, 0o777)


def run_tests_in_container(container_name: str) -> None:
    """Run tests in the specified container with proper coverage setup."""
    setup_coverage()
    # ... rest of the function ...


def main(args: List[str]) -> int:
    """Main entry point for the test runner.

    Args:
        args: Command line arguments

    Returns:
        int: Exit code (0 for success, non-zero for failure)
    """
    test_path = args[0] if len(args) > 0 else None
    runner = DockerTestRunner(test_path)
    success = runner.run_tests()
    runner.combine_coverage()
    return 0 if success else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
