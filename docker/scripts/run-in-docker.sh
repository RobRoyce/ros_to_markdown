#!/bin/bash

# At the top of the file, add usage documentation
USAGE="Usage: $0 [options] <ros-version> <command>

Options:
  -i, -t, -it    Docker interactive tty options

Available ROS versions:
  ros1, ros1-dev
  ros2-humble, ros2-humble-dev
  ros2-iron, ros2-iron-dev
  ros2-jazzy, ros2-jazzy-dev

Example:
  $0 -it ros2-humble-dev ros2 run turtlesim turtle_teleop_key"

# Add help option handling
if [[ "$1" == "-h" || "$1" == "--help" ]]; then
    echo "$USAGE"
    exit 0
fi

# Configure X11 display based on OS
configure_display() {
    case "$(uname -s)" in
        Linux*)
            # Check if running in WSL
            if grep -qi microsoft /proc/version; then
                # WSL2 needs special handling
                if grep -qi "WSL2" /proc/version; then
                    export DISPLAY=$(ip route | grep default | awk '{print $3}'):0
                    export DOCKER_EXTRA_HOSTS="host.docker.internal:host-gateway"
                else
                    # WSL1 can use localhost
                    export DISPLAY=localhost:0
                    export DOCKER_EXTRA_HOSTS="host.docker.internal:host-gateway"
                fi
            else
                # Native Linux
                export DISPLAY=${DISPLAY:-:0}
                xhost +local:docker > /dev/null 2>&1
                export X11_VOLUME="/tmp/.X11-unix:/tmp/.X11-unix:rw"
                export XAUTH_VOLUME="$HOME/.Xauthority:/root/.Xauthority:rw"
            fi
            ;;
        Darwin*)
            export DISPLAY=host.docker.internal:0
            if command -v xhost >/dev/null 2>&1; then
                xhost + 127.0.0.1 > /dev/null 2>&1
            fi
            export DOCKER_EXTRA_HOSTS="host.docker.internal:host-gateway"
            ;;
        MINGW*|CYGWIN*|MSYS*)
            export DISPLAY=host.docker.internal:0
            export DOCKER_EXTRA_HOSTS="host.docker.internal:host-gateway"
            ;;
        *)
            echo "Unsupported operating system"
            exit 1
            ;;
    esac
}

# Function to run docker compose command
run_docker_compose() {
    local cmd=$1
    if command -v docker &> /dev/null; then
        if docker compose version &> /dev/null; then
            docker compose $cmd
            return 0
        elif command -v docker-compose &> /dev/null; then
            docker-compose $cmd
            return 0
        fi
    fi
    return 1
}

# Configure display before running
configure_display

# Check if either docker compose command is available
if ! command -v docker &> /dev/null || { ! docker compose version &> /dev/null && ! command -v docker-compose &> /dev/null; }; then
    echo "Error: Docker and/or Docker Compose is not installed..."
    exit 1
fi

# Usage: ./scripts/run-in-docker.sh [ros1|ros1-dev|ros2-humble|...] [command]
# Parse options
DOCKER_OPTS=""
while [[ $# -gt 0 ]]; do
    case $1 in
        -i|-t|-it|-ti)
            DOCKER_OPTS="$DOCKER_OPTS $1"
            shift
            ;;
        *)
            break
            ;;
    esac
done

# Get ROS version and command
ROS_VERSION=$1
shift
COMMAND=$@

# Set ROS_DISTRO based on the service
case $ROS_VERSION in
    "ros1"|"ros1-dev")
        export ROS_DISTRO=noetic
        ;;
    "ros2-humble"|"ros2-humble-dev")
        export ROS_DISTRO=humble
        ;;
    "ros2-iron"|"ros2-iron-dev")
        export ROS_DISTRO=iron
        ;;
    "ros2-jazzy"|"ros2-jazzy-dev")
        export ROS_DISTRO=jazzy
        ;;
    *)
        echo "Error: Invalid ROS version specified"
        exit 1
        ;;
esac

# Add these environment variables before running tests
export COVERAGE_FILE=.coverage
export PYTHONPATH=/workspace/src:$PYTHONPATH

# Run docker compose with the command and environment variables
if ! run_docker_compose "run --rm -e ROS_DISTRO=$ROS_DISTRO $DOCKER_OPTS $ROS_VERSION $COMMAND"; then
    echo "Error: Failed to run docker compose command"
    exit 1
fi

# Cleanup X11 permissions if on Linux
if [[ "$(uname -s)" == "Linux" ]]; then
    xhost -local:docker > /dev/null 2>&1
fi
