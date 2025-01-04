# Docker Development Environment

This project uses Docker to provide isolated development environments for both ROS1 and ROS2. The Docker setup consists of:
- A ROS1 (Noetic) environment
- ROS2 environments for supported distributions (Humble, Iron, Jazzy)

## Directory Structure
```
docker/
├── Dockerfile.ros1         # ROS1 Noetic image
├── Dockerfile.ros2-humble  # ROS2 Humble image
├── Dockerfile.ros2-iron    # ROS2 Iron image
└── Dockerfile.ros2-jazzy   # ROS2 Jazzy image
```

## Script Organization
```
scripts/
├── docker-manager.sh     # Docker environment management
├── test-manager.sh       # Test orchestration
└── utils/               # Utility scripts
    ├── analyze_ros_graph.py
    ├── launch_turtlesim.py
    ├── launch-ros1-test-env.sh
    └── launch-ros2-test-env.sh
```

The `docker-manager.sh` script provides centralized management of Docker environments and replaces several legacy scripts:
- Building and cleaning images
- Running commands in containers
- Managing test environments
- Handling platform-specific configurations

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
./scripts/docker-manager.sh run ros1 pytest tests/

# ROS2 Humble
./scripts/docker-manager.sh run ros2-humble pytest tests/

# ROS2 Iron
./scripts/docker-manager.sh run ros2-iron pytest tests/

# ROS2 Jazzy
./scripts/docker-manager.sh run ros2-jazzy pytest tests/
```

## Helper Script Usage

The `docker-manager.sh` script provides a convenient way to manage Docker environments:

```bash
./scripts/docker-manager.sh [command] [options] [service...]

Commands:
  build         Build Docker images
  run           Run a command in a container
  clean         Clean Docker resources
  prune         Remove all project resources

# Available environments:
# - ros1, ros1-dev
# - ros2-humble, ros2-humble-dev
# - ros2-iron, ros2-iron-dev
# - ros2-jazzy, ros2-jazzy-dev
```

Examples:

```bash
# Get an interactive shell in ROS1
./scripts/docker-manager.sh run ros1 bash

# Run pytest in ROS2 Humble
./scripts/docker-manager.sh run ros2-humble pytest tests/

# Execute a Python script in ROS2 Jazzy
./scripts/docker-manager.sh run ros2-jazzy python my_script.py
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

| Environment    | Base OS        | Python Version | DDS Implementation | Status    |
|---------------|----------------|----------------|-------------------|-----------|
| ROS1 Noetic   | Ubuntu 20.04   | Python 3.8     | N/A              | Supported |
| ROS2 Humble   | Ubuntu 22.04   | Python 3.10    | CycloneDDS       | Supported |
| ROS2 Iron     | Ubuntu 22.04   | Python 3.10    | CycloneDDS       | Supported |
| ROS2 Jazzy    | Ubuntu 24.04   | Python 3.11*   | CycloneDDS       | Supported |

*Note: Jazzy uses Python 3.12 but we are using Python 3.11 from deadsnakes PPA due to Ubuntu 24.04's Python package management changes.
This is a temporary workaround until we can figure out how to use Python 3.12 properly.

## ROS2 DDS Configuration

### DDS Implementation Choice

This project uses CycloneDDS as the default DDS implementation for all ROS2 distributions due to:
- More reliable node discovery in Docker environments
- Better handling of multiple network interfaces
- Improved performance for video streaming
- Simpler configuration requirements
- Better compatibility with Docker networking

### Known Issues

- FastDDS can have node discovery issues in Docker when multiple network interfaces are active
- FastDDS may require complex XML configuration for optimal performance in Docker
- Node discovery issues may manifest as "NODE_NAME_UNKNOWN" in graph visualization

## Special Considerations for Ubuntu 24.04 (Jazzy)

### Current Python Version Challenge

There is currently a significant architectural challenge in the Jazzy environment:

- **System State**: 
  - ROS Jazzy uses system Python 3.12
  - Ubuntu 24.04 enforces strict system package management (PEP 668)
  - System packages cannot be modified without `--break-system-packages`

- **Current Workaround**:
  - Using Python 3.11 from deadsnakes PPA for test environment
  - Running tests in isolated virtual environment
  - Maintaining compatibility layer between ROS and test environment

- **Known Issues**:
  - Version mismatch between ROS (3.12) and tests (3.11)
  - Potential ABI compatibility issues with ROS packages
  - Not testing against actual production Python version

- **Future Plans**:
  - Migration to Python 3.12 for test infrastructure
  - Proper isolation techniques for system Python
  - Container-based test isolation approach

This is a temporary solution while we develop a proper approach to handle
Ubuntu 24.04's Python package restrictions. See `.cursornotes` for detailed
context and decision history.

The Jazzy environment requires special handling due to Ubuntu 24.04's Python package management:

1. Uses Python 3.11 from deadsnakes PPA instead of system Python 3.12
2. Creates a dedicated virtual environment for package installation
3. Automatically activates the virtual environment in the container
4. Uses ROS_DISTRO=jazzy

Example Jazzy container usage:
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

2. Python Package Installation (Ubuntu 24.04/Jazzy)
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
