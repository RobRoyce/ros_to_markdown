import os
import subprocess
from typing import Optional
from ..models.ros_components import ROSVersion


class ROSDetector:
    """Utility class to detect ROS version in the current environment.
    
    This class provides methods to automatically determine whether ROS1 or ROS2
    is installed and active in the current environment.
    """

    @staticmethod
    def detect_ros_version() -> ROSVersion:
        """Detect which version of ROS is currently active.
        
        Returns:
            ROSVersion: Enum indicating which ROS version is detected
            
        The detection is performed by checking environment variables and trying
        to import ROS-specific packages.
        """
        # Check ROS2 first
        if os.environ.get("ROS_DISTRO") in ["foxy", "galactic", "humble", "iron", "rolling"]:
            try:
                import rclpy
                return ROSVersion.ROS2
            except ImportError:
                pass

        # Check ROS1
        if os.environ.get("ROS_DISTRO") in ["noetic", "melodic"]:
            try:
                import rospy
                return ROSVersion.ROS1
            except ImportError:
                pass

        # If no clear indicators, try additional checks
        return ROSDetector._fallback_detection()

    @staticmethod
    def _fallback_detection() -> ROSVersion:
        """Perform additional checks to determine ROS version.
        
        Returns:
            ROSVersion: Enum indicating which ROS version is detected
        """
        # Try to run rosversion (ROS1) command
        try:
            subprocess.run(["rosversion", "-d"], 
                         stdout=subprocess.PIPE, 
                         stderr=subprocess.PIPE, 
                         check=True)
            return ROSVersion.ROS1
        except (subprocess.SubprocessError, FileNotFoundError):
            pass

        # Try to run ros2 command
        try:
            subprocess.run(["ros2", "--help"], 
                         stdout=subprocess.PIPE, 
                         stderr=subprocess.PIPE, 
                         check=True)
            return ROSVersion.ROS2
        except (subprocess.SubprocessError, FileNotFoundError):
            pass

        return ROSVersion.UNKNOWN


    @staticmethod
    def get_ros_distro() -> Optional[str]:
        """Get the ROS distribution name.
        
        Returns:
            Optional[str]: Name of the ROS distribution if found, None otherwise
        """
        return os.environ.get("ROS_DISTRO")
