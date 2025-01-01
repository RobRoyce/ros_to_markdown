# Docker Development Environment

This project uses Docker to provide isolated development environments for both ROS1 and ROS2. The Docker setup consists of:
- A ROS1 (Noetic) environment
- ROS2 environments for supported distributions (Humble, Iron)

## Directory Structure
```
docker/
├── Dockerfile.ros1        # ROS1 Noetic image
├── Dockerfile.ros2-humble # ROS2 Humble image
├── Dockerfile.ros2-iron   # ROS2 Iron image
└── scripts/
    └── run-in-docker.sh   # Helper script for running commands
```

## Prerequisites

You need both Docker Engine and Docker Compose installed on your system. You can install these in one of two ways:

1. **Recommended**: Install [Docker Desktop](https://docs.docker.com/desktop/install/)
   - Includes both Docker Engine and Docker Compose
   - Available for Windows, macOS, and Linux
   - Provides a GUI for container management

2. **Alternative**: Manual Installation
   - Install [Docker Engine](https://docs.docker.com/engine/install/)
   - Install Docker Compose:
     ```bash
     sudo apt update
     sudo apt install docker-compose-plugin
     ```

## Quick Start

1. Build the Docker images:
```bash
# Using new docker compose
docker compose build

# Or using legacy docker-compose
docker-compose build
```

2. Run commands in specific environments:
```bash
# ROS1 Noetic
./docker/scripts/run-in-docker.sh ros1 pytest tests/

# ROS2 Humble
./docker/scripts/run-in-docker.sh ros2-humble pytest tests/

# ROS2 Iron
./docker/scripts/run-in-docker.sh ros2-iron pytest tests/
```

## Helper Script Usage

The `run-in-docker.sh` script provides a convenient way to run commands in any ROS environment:

```bash
./docker/scripts/run-in-docker.sh [ros1|ros2-humble|ros2-iron] [command]
```

Examples:

```bash
# Get an interactive shell in ROS1
./docker/scripts/run-in-docker.sh ros1 bash

# Run pytest in ROS2 Humble
./docker/scripts/run-in-docker.sh ros2-humble pytest tests/

# Execute a Python script in ROS2 Iron
./docker/scripts/run-in-docker.sh ros2-iron python my_script.py
```

## Available Environments

Each ROS version has three service variants:
- Base service (e.g., `ros1`, `ros2-humble`, `ros2-iron`)
- Development environment (e.g., `ros1-dev`, `ros2-humble-dev`)
- Test environment (e.g., `test-ros1`, `test-ros2-humble`)

## Environment Details

| Environment    | Base OS        | Python Version | Status    |
|---------------|----------------|----------------|-----------|
| ROS1 Noetic   | Ubuntu 20.04   | Python 3.8     | Supported |
| ROS2 Humble   | Ubuntu 22.04   | Python 3.10    | Supported |
| ROS2 Iron     | Ubuntu 22.04   | Python 3.10    | Supported |

## Development Workflow

The Docker setup provides:
- Volume mounting for immediate code updates
- Network passthrough for ROS communication
- Display forwarding for GUI applications
- Automatic cleanup of orphaned containers
- Consistent environments across development machines

### Features

- **Hot Reload**: Changes to source code are immediately reflected in containers
- **Resource Isolation**: Each ROS version runs in its own container
- **Consistent Builds**: Uses official ROS Docker images as base
- **Clean Environment**: Automatic orphaned container cleanup
- **Flexible Compose**: Supports both new `docker compose` and legacy `docker-compose`

### Troubleshooting

If you encounter issues:

1. Verify Docker is running:
   ```bash
   docker info
   ```

2. Check Docker Compose installation:
   ```bash
   # Try either
   docker compose version
   # or
   docker-compose --version
   ```

3. Ensure proper permissions:
   ```bash
   # Add your user to the docker group
   sudo usermod -aG docker $USER
   # Log out and back in for changes to take effect
   ```

## Additional Resources

- [Docker Documentation](https://docs.docker.com/)
- [ROS Docker Official Images](https://hub.docker.com/_/ros)
- [ROS2 Docker Official Images](https://hub.docker.com/_/ros)
