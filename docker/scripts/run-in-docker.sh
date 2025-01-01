#!/bin/bash

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

# Check if either docker compose command is available
if ! command -v docker &> /dev/null || { ! docker compose version &> /dev/null && ! command -v docker-compose &> /dev/null; }; then
    echo "Error: Docker and/or Docker Compose is not installed. Please install:"
    echo "1. Install Docker Engine: https://docs.docker.com/engine/install/"
    echo "2. Install Docker Compose using one of these methods:"
    echo "   - Install Docker Desktop (recommended)"
    echo "   - Install standalone Docker Compose:"
    echo "     sudo apt update && sudo apt install docker-compose-plugin"
    exit 1
fi

# Usage: ./scripts/run-in-docker.sh [ros1|ros1-dev|ros2-humble|ros2-humble-dev|ros2-iron|ros2-iron-dev] [command]
# Example: ./scripts/run-in-docker.sh ros1 pytest tests/

ROS_VERSION=$1
shift
COMMAND=$@

# Validate ROS version/distro argument
case $ROS_VERSION in
    "ros1"|"ros1-dev"|"ros2-humble"|"ros2-humble-dev"|"ros2-iron"|"ros2-iron-dev")
        ;;
    *)
        echo "Error: First argument must be one of: ros1, ros1-dev, ros2-humble, ros2-humble-dev, ros2-iron, ros2-iron-dev"
        echo "Usage: ./scripts/run-in-docker.sh [ros1|ros1-dev|ros2-humble|ros2-humble-dev|ros2-iron|ros2-iron-dev] [command]"
        exit 1
        ;;
esac

# Run docker compose with the appropriate command
if ! run_docker_compose "run --rm --remove-orphans $ROS_VERSION $COMMAND"; then
    echo "Error: Failed to run docker compose command"
    exit 1
fi
