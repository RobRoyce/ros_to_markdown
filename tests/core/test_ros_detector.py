import os
import subprocess
import pytest
from unittest.mock import patch, MagicMock
from ros_to_markdown.core.ros_detector import ROSDetector
from ros_to_markdown.models.ros_components import ROSVersion


@pytest.fixture
def mock_ros1_env():
    """Fixture that mocks a ROS1 environment."""
    with patch.dict(os.environ, {
        'ROS_DISTRO': 'noetic',
        'ROS_ROOT': '/opt/ros/noetic/share/ros',
        'ROS_PACKAGE_PATH': '/opt/ros/noetic/share'
    }):
        yield


@pytest.fixture
def mock_ros2_env():
    """Fixture that mocks a ROS2 environment."""
    with patch.dict(os.environ, {
        'ROS_DISTRO': 'humble',
        'AMENT_PREFIX_PATH': '/opt/ros/humble',
        'ROS_PYTHON_VERSION': '3'
    }):
        yield


class TestROSDetector:
    """Test suite for ROSDetector class."""

    def test_detect_ros1_via_env(self, mock_ros1_env):
        """Test ROS1 detection via environment variables."""
        with patch('ros_to_markdown.core.ros_detector.import_module') as mock_import:
            def mock_import_module(name):
                if name == 'rospy':
                    return MagicMock()
                raise ImportError
            mock_import.side_effect = mock_import_module
            version = ROSDetector.detect_ros_version()
            assert version == ROSVersion.ROS1

    def test_detect_ros2_via_env(self, mock_ros2_env):
        """Test ROS2 detection via environment variables."""
        with patch('ros_to_markdown.core.ros_detector.import_module') as mock_import:
            def mock_import_module(name):
                if name == 'rclpy':
                    return MagicMock()
                raise ImportError
            mock_import.side_effect = mock_import_module
            version = ROSDetector.detect_ros_version()
            assert version == ROSVersion.ROS2

    def test_fallback_detection_ros1(self):
        """Test ROS1 detection via fallback method."""
        with patch.dict(os.environ, {'ROS_DISTRO': ''}), \
             patch('subprocess.run') as mock_run:
            # Mock successful rosversion command
            mock_run.side_effect = [
                MagicMock(returncode=0),  # rosversion succeeds
                subprocess.SubprocessError(),  # ros2 fails
            ]
            version = ROSDetector._fallback_detection()
            assert version == ROSVersion.ROS1

    def test_fallback_detection_ros2(self):
        """Test ROS2 detection via fallback method."""
        with patch.dict(os.environ, {'ROS_DISTRO': ''}), \
             patch('subprocess.run') as mock_run:
            # Mock successful ros2 command
            mock_run.side_effect = [
                subprocess.SubprocessError(),  # rosversion fails
                MagicMock(returncode=0),  # ros2 succeeds
            ]
            version = ROSDetector._fallback_detection()
            assert version == ROSVersion.ROS2

    def test_fallback_detection_unknown(self):
        """Test unknown ROS detection when no ROS installation is found."""
        with patch.dict(os.environ, {'ROS_DISTRO': ''}), \
             patch('subprocess.run') as mock_run:
            # Mock both commands failing
            mock_run.side_effect = subprocess.SubprocessError()
            version = ROSDetector._fallback_detection()
            assert version == ROSVersion.UNKNOWN

    def test_get_ros_distro_found(self, mock_ros2_env):
        """Test getting ROS distro when it exists."""
        distro = ROSDetector.get_ros_distro()
        assert distro == 'humble'

    def test_get_ros_distro_not_found(self):
        """Test getting ROS distro when it doesn't exist."""
        with patch.dict(os.environ, {}, clear=True):
            distro = ROSDetector.get_ros_distro()
            assert distro is None

    @pytest.mark.parametrize("ros_distro,expected_version", [
        ('noetic', ROSVersion.ROS1),
        ('melodic', ROSVersion.ROS1),
        ('humble', ROSVersion.ROS2),
        ('iron', ROSVersion.ROS2),
        ('rolling', ROSVersion.ROS2),
    ])
    def test_detect_specific_ros_distros(self, ros_distro, expected_version):
        """Test detection of specific ROS distributions."""
        with patch.dict(os.environ, {'ROS_DISTRO': ros_distro}):
            with patch('ros_to_markdown.core.ros_detector.import_module') as mock_import:
                def mock_import_module(name):
                    if (expected_version == ROSVersion.ROS1 and name == 'rospy') or \
                       (expected_version == ROSVersion.ROS2 and name == 'rclpy'):
                        return MagicMock()
                    raise ImportError
                mock_import.side_effect = mock_import_module
                version = ROSDetector.detect_ros_version()
                assert version == expected_version


@pytest.mark.integration
class TestROSDetectorIntegration:
    """Integration tests for ROSDetector using actual ROS environments."""
    
    def test_detect_actual_ros_version(self):
        """Test detection in the current ROS environment."""
        version = ROSDetector.detect_ros_version()
        distro = ROSDetector.get_ros_distro()
        
        # The test should pass in any ROS environment
        assert version in [ROSVersion.ROS1, ROSVersion.ROS2]
        assert distro is not None
        
        # Verify the version matches the distro
        if distro in ['noetic', 'melodic']:
            assert version == ROSVersion.ROS1
        elif distro in ['humble', 'iron', 'rolling']:
            assert version == ROSVersion.ROS2 