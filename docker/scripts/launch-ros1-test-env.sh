#!/bin/bash

# Make the script executable and fail on errors
set -e

# Source ROS1 environment
source /opt/ros/noetic/setup.bash

# Create and build sample workspace if it doesn't exist
if [ ! -d "/sample_ws/src" ]; then
    echo "Initializing sample workspace..."
    mkdir -p /sample_ws/src
    cd /sample_ws/src
    
    # Clone some common ROS packages for testing
    echo "Cloning ros_tutorials..."
    git clone -b noetic-devel https://github.com/ros/ros_tutorials.git
    
    echo "Cloning common_msgs..."
    git clone -b noetic-devel https://github.com/ros/common_msgs.git
    
    echo "Cloning joystick_drivers..."
    git clone -b ros1 https://github.com/ros-drivers/joystick_drivers.git

    # Update package lists and install dependencies
    cd /sample_ws
    echo "Updating package lists..."
    apt-get update

    # Enable universe repository which contains some of the required packages
    apt-get install -y software-properties-common
    add-apt-repository universe
    apt-get update

    echo "Installing dependencies with rosdep..."
    rosdep install --from-paths src --ignore-src -r -y || {
        echo "Some dependencies failed to install. Continuing anyway..."
    }

    # Initialize and build the workspace
    catkin init
    catkin config --install
    catkin build
fi

# Source the workspace
source /sample_ws/devel/setup.bash

# Execute the command passed to the script, or start an interactive shell
if [ $# -eq 0 ]; then
    exec /bin/bash
else
    exec "$@"
fi 