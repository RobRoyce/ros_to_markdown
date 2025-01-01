#!/bin/bash

# Run tests for ROS1
echo "Testing ROS1 (Noetic)..."
docker compose run --rm test-ros1

# Run tests for stable ROS2 distros
for distro in humble iron; do
    echo "Testing ROS2 ($distro)..."
    docker compose run --rm test-ros2-$distro
done

# Note: Jazzy testing temporarily disabled
# echo "Testing ROS2 (Jazzy)..."
# docker compose run --rm test-ros2-jazzy 