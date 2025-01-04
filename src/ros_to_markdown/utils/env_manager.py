"""Unified environment management for ROS1/ROS2."""

import importlib.util
import logging
import os
import sys
from typing import Dict, Optional

import structlog

from ..core.ros_detector import ROSDetector, ROSVersion

logger = structlog.get_logger(__name__)


class ROSEnvironmentManager:
    """Manages ROS environment setup and validation."""

    @staticmethod
    def ensure_ros_environment() -> bool:
        """Ensure ROS environment is properly configured.

        Returns:
            bool: True if environment is properly configured, False otherwise
        """
        try:
            # Check ROS_VERSION
            ros_version = ROSDetector.detect_ros_version()
            if not ros_version:
                logger.error("Could not detect ROS version")
                return False

            # Check ROS_DISTRO
            ros_distro = os.getenv("ROS_DISTRO")
            if not ros_distro:
                logger.error("ROS_DISTRO not set")
                return False

            # Validate Python environment
            if not ROSEnvironmentManager.validate_python_environment():
                return False

            # Check ROS-specific environment
            if ros_version == ROSVersion.ROS1:
                return ROSEnvironmentManager._check_ros1_env()
            else:
                return ROSEnvironmentManager._check_ros2_env()

        except Exception as e:
            logger.error("Error ensuring ROS environment", error=str(e))
            return False

    @staticmethod
    def setup_logging(verbose: bool = False) -> None:
        """Configure ROS logging.

        Args:
            verbose: Whether to enable verbose logging
        """
        log_level = logging.DEBUG if verbose else logging.INFO

        structlog.configure(
            processors=[
                structlog.processors.add_log_level,
                structlog.processors.TimeStamper(fmt="iso"),
                structlog.dev.ConsoleRenderer(colors=True),
            ],
            wrapper_class=structlog.make_filtering_bound_logger(log_level),
            context_class=dict,
            logger_factory=structlog.PrintLoggerFactory(),
            cache_logger_on_first_use=True,
        )

    @staticmethod
    def validate_python_environment() -> bool:
        """Validate Python environment for current ROS distro.

        Returns:
            bool: True if environment is valid, False otherwise
        """
        try:
            # Check Python version
            ros_distro = os.getenv("ROS_DISTRO", "")
            required_version = ROSEnvironmentManager._get_required_python_version(ros_distro)

            if not required_version:
                logger.error(f"Unknown ROS distribution: {ros_distro}")
                return False

            current_version = f"{sys.version_info.major}.{sys.version_info.minor}"
            if current_version != required_version:
                logger.error(
                    "Python version mismatch", required=required_version, current=current_version
                )
                return False

            # Check required packages
            required_packages = ROSEnvironmentManager._get_required_packages()
            for package, version in required_packages.items():
                if not ROSEnvironmentManager._check_package(package, version):
                    return False

            return True

        except Exception as e:
            logger.error("Error validating Python environment", error=str(e))
            return False

    @staticmethod
    def _check_ros1_env() -> bool:
        """Check ROS1-specific environment."""
        required_vars = ["ROS_MASTER_URI", "ROS_ROOT", "ROS_PACKAGE_PATH"]
        for var in required_vars:
            if not os.getenv(var):
                logger.error(f"Required environment variable not set: {var}")
                return False
        return True

    @staticmethod
    def _check_ros2_env() -> bool:
        """Check ROS2-specific environment."""
        required_vars = ["AMENT_PREFIX_PATH", "COLCON_PREFIX_PATH"]
        for var in required_vars:
            if not os.getenv(var):
                logger.error(f"Required environment variable not set: {var}")
                return False
        return True

    @staticmethod
    def _get_required_python_version(ros_distro: str) -> Optional[str]:
        """Get required Python version for ROS distribution."""
        versions = {"noetic": "3.8", "humble": "3.10", "iron": "3.10", "jazzy": "3.12"}
        return versions.get(ros_distro)

    @staticmethod
    def _get_required_packages() -> Dict[str, str]:
        """Get required Python packages and versions."""
        return {
            "rclpy": ">=0.6.0",
            "rospy": ">=1.15.0",
            "numpy": ">=1.20.0",
            "pytest": ">=6.0.0",
            "mypy": ">=0.800",
            "ruff": ">=0.1.0",
        }

    @staticmethod
    def _check_package(package: str, required_version: str) -> bool:
        """Check if a Python package is installed with correct version."""
        try:
            spec = importlib.util.find_spec(package)
            if not spec:
                logger.error(f"Required package not found: {package}")
                return False

            # Check version if needed
            if required_version:
                import pkg_resources

                version = pkg_resources.get_distribution(package).version
                if not pkg_resources.require(f"{package}{required_version}"):
                    logger.error(
                        "Package version mismatch",
                        package=package,
                        required=required_version,
                        current=version,
                    )
                    return False

            return True

        except Exception as e:
            logger.error(f"Error checking package {package}", error=str(e))
            return False
