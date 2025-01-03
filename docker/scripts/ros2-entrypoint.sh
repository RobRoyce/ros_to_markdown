#!/bin/bash
set -e

# Source ROS2 environment based on distribution
source "/opt/ros/${ROS_DISTRO}/setup.bash"

# If we're in a development environment, source the workspace
if [ -f "/sample_ws/install/setup.bash" ]; then
    source "/sample_ws/install/setup.bash"
fi

# Add this line to your ros2-entrypoint.sh
export ROS_DOMAIN_ID=0

# Execute the command passed to the container
exec "$@" 