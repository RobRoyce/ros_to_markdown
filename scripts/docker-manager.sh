#!/bin/bash

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

# Default values
VERBOSE=0
CLEAN=0

# Function to show usage
show_usage() {
    echo "Usage: $0 [command] [options] [service...]"
    echo
    echo "Commands:"
    echo "  build         Build Docker images"
    echo "  run           Run a command in a container"
    echo "  clean         Clean Docker resources"
    echo "  prune         Remove all project resources"
    echo
    echo "Options:"
    echo "  -v, --verbose  Show verbose output"
    echo "  -c, --clean    Clean before building (build command only)"
    echo "  -h, --help     Show this help message"
    echo
    echo "Services: (for build command)"
    echo "  ros1            ROS1 Noetic base"
    echo "  ros1-dev        ROS1 Noetic development"
    echo "  ros2-humble     ROS2 Humble base"
    echo "  ros2-humble-dev ROS2 Humble development"
    echo "  ros2-iron       ROS2 Iron base"
    echo "  ros2-iron-dev   ROS2 Iron development"
    echo "  ros2-jazzy      ROS2 Jazzy base"
    echo "  ros2-jazzy-dev  ROS2 Jazzy development"
}

# Function to clean Docker resources
clean_resources() {
    echo -e "${YELLOW}Cleaning Docker resources...${NC}"
    
    # Remove containers with our project label
    echo -e "\n${GREEN}Removing stopped containers...${NC}"
    docker container prune -f --filter "label=com.ros-to-markdown=true"
    
    # Remove images with our project label
    echo -e "\n${GREEN}Removing dangling images...${NC}"
    docker image prune -f --filter "label=com.ros-to-markdown=true"
    
    # Remove build cache
    echo -e "\n${GREEN}Removing build cache...${NC}"
    docker builder prune -f --filter "label=com.ros-to-markdown=true"
}

# Function to handle build command
handle_build() {
    local SERVICES=()
    
    # Parse build-specific options
    while [[ $# -gt 0 ]]; do
        case $1 in
            -v|--verbose)
                VERBOSE=1
                shift
                ;;
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
        clean_resources
    fi
    
    # Build services
    if [ ${#SERVICES[@]} -eq 0 ]; then
        echo -e "${GREEN}Building all services...${NC}"
        if [ $VERBOSE -eq 1 ]; then
            docker compose build --no-cache --progress=plain
        else
            docker compose build --no-cache
        fi
    else
        echo -e "${GREEN}Building specified services: ${SERVICES[*]}${NC}"
        if [ $VERBOSE -eq 1 ]; then
            docker compose build --no-cache --progress=plain "${SERVICES[@]}"
        else
            docker compose build --no-cache "${SERVICES[@]}"
        fi
    fi
}

# Function to handle prune command
handle_prune() {
    echo -e "${YELLOW}Performing deep cleanup of all project resources...${NC}"
    
    # Remove all project containers
    echo -e "\n${GREEN}Removing all project containers...${NC}"
    docker ps -a --filter "label=com.ros-to-markdown=true" -q | xargs -r docker rm -f
    
    # Remove all project images
    echo -e "\n${GREEN}Removing all project images...${NC}"
    docker images --filter "label=com.ros-to-markdown=true" -q | xargs -r docker rmi -f
    
    # Remove all project volumes
    echo -e "\n${GREEN}Removing all project volumes...${NC}"
    docker volume ls --filter "label=com.ros-to-markdown=true" -q | xargs -r docker volume rm
    
    # Clean build cache
    echo -e "\n${GREEN}Cleaning build cache...${NC}"
    docker builder prune -f --filter "label=com.ros-to-markdown=true"
}

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
            echo -e "${RED}Unsupported operating system${NC}"
            exit 1
            ;;
    esac
}

# Function to handle run command
handle_run() {
    local SERVICE=$1
    shift
    
    if [ -z "$SERVICE" ]; then
        echo -e "${RED}Error: Service name required${NC}"
        show_usage
        exit 1
    fi

    # Configure display before running
    configure_display
    
    # Set ROS_DISTRO based on the service
    case $SERVICE in
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
            echo -e "${RED}Error: Invalid ROS version specified${NC}"
            exit 1
            ;;
    esac
    
    echo -e "${GREEN}Using DISPLAY variable: $DISPLAY${NC}"
    echo -e "${GREEN}Running command in $SERVICE...${NC}"
    if [ $VERBOSE -eq 1 ]; then
        docker compose run --rm \
            -e DISPLAY=$DISPLAY \
            -e ROS_DISTRO=$ROS_DISTRO \
            -e DOCKER_EXTRA_HOSTS="$DOCKER_EXTRA_HOSTS" \
            -e QT_X11_NO_MITSHM=1 \
            -e NO_AT_BRIDGE=1 \
            $SERVICE "$@"
    else
        docker compose run --rm \
            -e DISPLAY=$DISPLAY \
            -e ROS_DISTRO=$ROS_DISTRO \
            -e DOCKER_EXTRA_HOSTS="$DOCKER_EXTRA_HOSTS" \
            -e QT_X11_NO_MITSHM=1 \
            -e NO_AT_BRIDGE=1 \
            $SERVICE "$@"
    fi

    # Cleanup X11 permissions if on Linux
    if [[ "$(uname -s)" == "Linux" ]]; then
        xhost -local:docker > /dev/null 2>&1
    fi
}

# Parse global options and command
while [[ $# -gt 0 ]]; do
    case $1 in
        build|clean|prune|run)
            COMMAND=$1
            shift
            break
            ;;
        -h|--help)
            show_usage
            exit 0
            ;;
        *)
            echo -e "${RED}Unknown command: $1${NC}"
            show_usage
            exit 1
            ;;
    esac
done

# Execute command
case "$COMMAND" in
    build)
        handle_build "$@"
        ;;
    clean)
        clean_resources
        ;;
    prune)
        handle_prune
        ;;
    run)
        handle_run "$@"
        ;;
    *)
        show_usage
        exit 1
        ;;
esac 