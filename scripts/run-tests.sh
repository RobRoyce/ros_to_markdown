#!/bin/bash

# Make script exit on first error
set -e

# Help message
show_help() {
    echo "Usage: ./scripts/run-tests.sh [OPTIONS] [TEST_PATH]"
    echo ""
    echo "Run tests across all supported ROS distributions"
    echo ""
    echo "Options:"
    echo "  -h, --help     Show this help message"
    echo "  -v, --verbose  Show verbose output"
    echo "  -r, --ros1     Test only ROS1"
    echo "  -2, --ros2     Test only ROS2"
    echo "  -d, --distro   Test specific distribution (noetic|humble|iron|jazzy)"
    echo ""
    echo "Examples:"
    echo "  ./scripts/run-tests.sh                                    # Run all tests"
    echo "  ./scripts/run-tests.sh tests/core/test_ros_detector.py   # Run specific test file"
    echo "  ./scripts/run-tests.sh -d humble                         # Test only on ROS2 Humble"
    echo "  ./scripts/run-tests.sh -r                                # Test only on ROS1"
}

# Default values
VERBOSE=0
ROS1_ONLY=0
ROS2_ONLY=0
SPECIFIC_DISTRO=""
TEST_PATH=""

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        -h|--help)
            show_help
            exit 0
            ;;
        -v|--verbose)
            VERBOSE=1
            shift
            ;;
        -r|--ros1)
            ROS1_ONLY=1
            shift
            ;;
        -2|--ros2)
            ROS2_ONLY=1
            shift
            ;;
        -d|--distro)
            SPECIFIC_DISTRO="$2"
            shift 2
            ;;
        *)
            TEST_PATH="$1"
            shift
            ;;
    esac
done

# Validate arguments
if [ $ROS1_ONLY -eq 1 ] && [ $ROS2_ONLY -eq 1 ]; then
    echo "Error: Cannot specify both --ros1 and --ros2"
    exit 1
fi

if [ ! -z "$SPECIFIC_DISTRO" ]; then
    case $SPECIFIC_DISTRO in
        noetic|humble|iron|jazzy)
            ;;
        *)
            echo "Error: Invalid distribution. Must be one of: noetic, humble, iron, jazzy"
            exit 1
            ;;
    esac
fi

# Set up environment
if [ $VERBOSE -eq 1 ]; then
    export PYTEST_ADDOPTS="-v"
fi

# Run tests based on options
if [ ! -z "$SPECIFIC_DISTRO" ]; then
    case $SPECIFIC_DISTRO in
        noetic)
            ./docker/scripts/run-in-docker.sh ros1 pytest ${TEST_PATH:-tests/}
            ;;
        humble)
            ./docker/scripts/run-in-docker.sh ros2-humble pytest ${TEST_PATH:-tests/}
            ;;
        iron)
            ./docker/scripts/run-in-docker.sh ros2-iron pytest ${TEST_PATH:-tests/}
            ;;
        jazzy)
            ./docker/scripts/run-in-docker.sh ros2-jazzy pytest ${TEST_PATH:-tests/}
            ;;
    esac
elif [ $ROS1_ONLY -eq 1 ]; then
    ./docker/scripts/run-in-docker.sh ros1 pytest ${TEST_PATH:-tests/}
elif [ $ROS2_ONLY -eq 1 ]; then
    for distro in humble iron jazzy; do
        ./docker/scripts/run-in-docker.sh ros2-$distro pytest ${TEST_PATH:-tests/}
    done
else
    # Run all tests using the Python test runner
    python3 -m tests.test_helpers.docker_test_runner ${TEST_PATH}
fi 