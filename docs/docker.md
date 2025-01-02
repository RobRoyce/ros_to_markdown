# Docker Development Environment

This project uses Docker to provide isolated development environments for both ROS1 and ROS2. The Docker setup consists of:
- A ROS1 (Noetic) environment
- ROS2 environments for supported distributions (Humble, Iron, Rolling/Jazzy)

## Directory Structure
```
docker/
├── Dockerfile.ros1         # ROS1 Noetic image
├── Dockerfile.ros2-humble  # ROS2 Humble image
├── Dockerfile.ros2-iron    # ROS2 Iron image
├── Dockerfile.ros2-jazzy   # ROS2 Rolling/Jazzy image
└── scripts/
    └── run-in-docker.sh    # Helper script for running commands
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

# ROS2 Rolling/Jazzy
./docker/scripts/run-in-docker.sh ros2-jazzy pytest tests/
```

## Helper Script Usage

The `run-in-docker.sh` script provides a convenient way to run commands in any ROS environment:

```bash
./docker/scripts/run-in-docker.sh [environment] [command]

# Available environments:
# - ros1, ros1-dev
# - ros2-humble, ros2-humble-dev
# - ros2-iron, ros2-iron-dev
# - ros2-jazzy, ros2-jazzy-dev
```

Examples:

```bash
# Get an interactive shell in ROS1
./docker/scripts/run-in-docker.sh ros1 bash

# Run pytest in ROS2 Humble
./docker/scripts/run-in-docker.sh ros2-humble pytest tests/

# Execute a Python script in ROS2 Rolling/Jazzy
./docker/scripts/run-in-docker.sh ros2-jazzy python my_script.py
```

## Available Environments

Each ROS version has three service variants:
- Base service (e.g., `ros1`, `ros2-humble`)
- Development environment (e.g., `ros1-dev`, `ros2-humble-dev`)
- Test environment (e.g., `test-ros1`, `test-ros2-humble`)

Supported ROS distributions:
- ROS1 Noetic (EOL: May 2025)
- ROS2 Humble Hawksbill (EOL: May 2027)
- ROS2 Iron Irwini (EOL: Dec 2027)
- ROS2 Jazzy Jalisco (EOL: May 2029)

## Environment Details

| Environment    | Base OS        | Python Version | Status    |
|---------------|----------------|----------------|-----------|
| ROS1 Noetic   | Ubuntu 20.04   | Python 3.8     | Supported |
| ROS2 Humble   | Ubuntu 22.04   | Python 3.10    | Supported |
| ROS2 Iron     | Ubuntu 22.04   | Python 3.10    | Supported |
| ROS2 Rolling  | Ubuntu 24.04   | Python 3.11*   | Supported |

*Note: Rolling/Jazzy uses Python 3.11 from deadsnakes PPA due to Ubuntu 24.04's Python package management changes.

## Special Considerations for Ubuntu 24.04 (Rolling/Jazzy)

The Rolling/Jazzy environment requires special handling due to Ubuntu 24.04's Python package management:

1. Uses Python 3.11 from deadsnakes PPA instead of system Python 3.12
2. Creates a dedicated virtual environment for package installation
3. Automatically activates the virtual environment in the container
4. Uses ROS_DISTRO=rolling (will become jazzy upon official release)

Example Rolling/Jazzy container usage:
```bash
# Development environment
./docker/scripts/run-in-docker.sh ros2-jazzy-dev bash

# The virtual environment is automatically activated
# All pip installations should work without --break-system-packages
pip install some-package
```

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
- **Cross-Platform**: Works on Linux, WSL, macOS, and Windows

### Platform-Specific X11 Configuration

#### Linux
```bash
# Handled automatically by run-in-docker.sh
xhost +local:docker
```

#### Windows (WSL2)
```bash
# Handled automatically by run-in-docker.sh
export DISPLAY=:0
```

#### macOS
```bash
# Install XQuartz first, then:
xhost + 127.0.0.1
```

### Troubleshooting

Common issues and solutions:

1. X11 Display Issues
   ```bash
   # Check X11 socket permissions
   ls -la /tmp/.X11-unix/
   
   # Verify DISPLAY variable
   echo $DISPLAY
   ```

2. Python Package Installation (Ubuntu 24.04/Rolling)
   ```bash
   # Verify virtual environment activation
   which python
   # Should show: /home/ros/.venv/bin/python
   ```

3. ROS Environment
   ```bash
   # Verify ROS distribution
   echo $ROS_DISTRO
   # Should match your target distribution
   ```

## Additional Resources

- [Docker Documentation](https://docs.docker.com/)
- [ROS Docker Official Images](https://hub.docker.com/_/ros)
- [ROS2 Docker Official Images](https://hub.docker.com/_/ros)
- [Ubuntu 24.04 Python Changes (PEP 668)](https://peps.python.org/pep-0668/)

## Testing Infrastructure

### Quick Test Commands

```bash
# Run all tests across all distributions
./scripts/run-tests.sh

# Run specific test file
./scripts/run-tests.sh tests/core/test_ros_detector.py

# Run tests with verbose output
./scripts/run-tests.sh -v
```

### Test Runner Options

The `run-tests.sh` script provides flexible options for running tests:

```bash
Usage: ./scripts/run-tests.sh [OPTIONS] [TEST_PATH]

Options:
  -h, --help     Show this help message
  -v, --verbose  Show verbose output
  -r, --ros1     Test only ROS1
  -2, --ros2     Test only ROS2
  -d, --distro   Test specific distribution (noetic|humble|iron)

Examples:
  ./scripts/run-tests.sh                                    # Run all tests
  ./scripts/run-tests.sh tests/core/test_ros_detector.py   # Run specific test file
  ./scripts/run-tests.sh -d humble                         # Test only on ROS2 Humble
  ./scripts/run-tests.sh -r                                # Test only on ROS1
```

### Test Environments

Tests are run in isolated Docker containers for each ROS distribution:
- ROS1 Noetic: `test-ros1`
- ROS2 Humble: `test-ros2-humble`
- ROS2 Iron: `test-ros2-iron`

The test runner automatically manages container lifecycle and ensures clean test environments for each run.

### Integration Tests

For ROS-specific integration tests, use the `@pytest.mark.integration` decorator:

```python
@pytest.mark.integration
def test_my_ros_feature():
    # This test will run in actual ROS environment
    pass
```
