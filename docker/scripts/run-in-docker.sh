#!/bin/bash

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
ROS_VERSION=$1
shift
COMMAND=$@

# Validate ROS version/distro argument
case $ROS_VERSION in
    "ros1"|"ros1-dev"|"ros2-humble"|"ros2-humble-dev"|"ros2-iron"|"ros2-iron-dev"|"ros2-jazzy"|"ros2-jazzy-dev")
        ;;
    *)
        echo "Error: First argument must be one of: ros1, ros1-dev..."
        exit 1
        ;;
esac

# Add these environment variables before running tests
export COVERAGE_FILE=.coverage
export PYTHONPATH=/workspace/src:$PYTHONPATH

# Run docker compose with the command
if ! run_docker_compose "run --rm -it $ROS_VERSION $COMMAND"; then
    echo "Error: Failed to run docker compose command"
    exit 1
fi

# Cleanup X11 permissions if on Linux
if [[ "$(uname -s)" == "Linux" ]]; then
    xhost -local:docker > /dev/null 2>&1
fi
