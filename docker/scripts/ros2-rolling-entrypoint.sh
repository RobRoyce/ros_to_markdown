#!/bin/bash
set -e

# Source ROS2 environment and virtual environment
source "/opt/ros/rolling/setup.bash"
source "/home/ros/.venv/bin/activate"

# Execute the command passed to the container
exec "$@" 