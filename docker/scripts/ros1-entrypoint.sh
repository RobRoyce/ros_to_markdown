#!/bin/bash
set -e

# Source ROS1 environment
source "/opt/ros/noetic/setup.bash"

# Execute the command passed to the container
exec "$@" 