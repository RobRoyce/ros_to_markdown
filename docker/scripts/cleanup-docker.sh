#!/bin/bash

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo -e "${YELLOW}Starting Docker cleanup for ros_to_markdown project...${NC}"

# Remove all containers with our project label
echo -e "\n${GREEN}Removing stopped containers...${NC}"
docker container prune -f --filter "label=com.ros-to-markdown=true"

# Remove all images with our project label
echo -e "\n${GREEN}Removing dangling images...${NC}"
docker image prune -f --filter "label=com.ros-to-markdown=true"

# Remove all project-specific images
echo -e "\n${GREEN}Removing project images...${NC}"
docker images --format "{{.ID}} {{.Repository}}" | grep "ros_to_markdown" | awk '{print $1}' | xargs -r docker rmi -f

# Remove build cache
echo -e "\n${GREEN}Removing build cache...${NC}"
docker builder prune -f --filter "label=com.ros-to-markdown=true"

# Optional: Remove all dangling images (uncomment if needed)
# echo -e "\n${GREEN}Removing all dangling images...${NC}"
# docker image prune -f

echo -e "\n${GREEN}Cleanup complete!${NC}"

# Show remaining project-related images
echo -e "\n${YELLOW}Remaining project images:${NC}"
docker images --filter "label=com.ros-to-markdown=true" 