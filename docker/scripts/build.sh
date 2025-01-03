#!/bin/bash

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

# Function to show usage
show_usage() {
    echo "Usage: $0 [options] [service...]"
    echo
    echo "Options:"
    echo "  -c, --clean     Clean before building"
    echo "  -h, --help      Show this help message"
    echo
    echo "Services:"
    echo "  ros1            ROS1 Noetic base"
    echo "  ros1-dev        ROS1 Noetic development"
    echo "  ros2-humble     ROS2 Humble base"
    echo "  ros2-humble-dev ROS2 Humble development"
    echo "  ros2-iron       ROS2 Iron base"
    echo "  ros2-iron-dev   ROS2 Iron development"
    echo "  ros2-jazzy      ROS2 Jazzy base"
    echo "  ros2-jazzy-dev  ROS2 Jazzy development"
    echo
    echo "If no service is specified, all services will be built."
}

# Parse arguments
CLEAN=0
SERVICES=()

while [[ $# -gt 0 ]]; do
    case $1 in
        -c|--clean)
            CLEAN=1
            shift
            ;;
        -h|--help)
            show_usage
            exit 0
            ;;
        *)
            SERVICES+=("$1")
            shift
            ;;
    esac
done

# Clean if requested
if [ $CLEAN -eq 1 ]; then
    echo -e "${YELLOW}Cleaning existing images...${NC}"
    ./docker/scripts/cleanup-docker.sh
fi

# Build services
if [ ${#SERVICES[@]} -eq 0 ]; then
    echo -e "${GREEN}Building all services...${NC}"
    docker compose build --no-cache
else
    echo -e "${GREEN}Building specified services: ${SERVICES[*]}${NC}"
    docker compose build --no-cache "${SERVICES[@]}"
fi

echo -e "${GREEN}Build complete!${NC}" 