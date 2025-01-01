#!/bin/bash

# Make the script executable and fail on errors
set -e

# Activate virtual environment
source /opt/venv/bin/activate

# Source ROS2 environment
source /opt/ros/${ROS_DISTRO}/setup.bash

# Create and build sample workspace if it doesn't exist
if [ ! -d "/sample_ws/src" ]; then
    echo "Initializing sample workspace..."
    mkdir -p /sample_ws/src
    cd /sample_ws/src
    
    # Clone some common ROS2 packages for testing
    git clone -b ${ROS_DISTRO} https://github.com/ros/ros_tutorials.git || git clone -b humble https://github.com/ros/ros_tutorials.git
    git clone -b ${ROS_DISTRO} https://github.com/ros2/common_interfaces.git || git clone -b humble https://github.com/ros2/common_interfaces.git
    
    # Update package lists and install dependencies
    cd /sample_ws
    echo "Updating package lists..."
    apt-get update

    # Install ROS diagnostic updater and joy packages
    apt-get install -y \
        ros-${ROS_DISTRO}-diagnostic-updater \
        ros-${ROS_DISTRO}-joy \
        ros-${ROS_DISTRO}-teleop-twist-joy

    # Initialize rosdep (ignore if already initialized)
    rosdep init || true
    
    echo "Updating rosdep..."
    rosdep update

    echo "Installing dependencies with rosdep..."
    rosdep install --from-paths src --ignore-src -r -y || {
        echo "Some dependencies failed to install. Continuing anyway..."
    }

    # Build the workspace
    colcon build --symlink-install
fi

# Source the workspace
source /sample_ws/install/setup.bash

# Execute the command passed to the script, or start an interactive shell
if [ $# -eq 0 ]; then
    exec /bin/bash
else
    exec "$@"
fi 