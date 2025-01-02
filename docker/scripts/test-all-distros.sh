#!/bin/bash

# Make script exit on first error
set -e

echo "Running ROS Detection Tests across all supported distributions..."

# Use the Python test runner to execute tests
python3 -m tests.test_helpers.docker_test_runner tests/core/test_ros_detector.py

# If specific test file/class/method is provided as argument, run that instead
if [ $# -eq 1 ]; then
    python3 -m tests.test_helpers.docker_test_runner "$1"
fi 