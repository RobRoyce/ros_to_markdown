# Development Guide

## ROS2 Development Notes

### DDS Configuration

The project uses CycloneDDS for all ROS2 distributions. Required environment variables are automatically set in the Docker containers:

```bash
# DDS Implementation
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Domain ID (must be consistent across containers)
ROS_DOMAIN_ID=0

# Logging configuration
RCUTILS_LOGGING_BUFFERED_STREAM=1
RCUTILS_LOGGING_USE_STDOUT=1
RCUTILS_COLORIZED_OUTPUT=1
```

### Known Issues and Solutions

1. Node Discovery Issues
   - If nodes appear as "NODE_NAME_UNKNOWN", verify DDS implementation
   - CycloneDDS is recommended over FastDDS for Docker environments
   - Multiple network interfaces can cause discovery issues with FastDDS

2. Video Streaming Performance
   - CycloneDDS provides better out-of-box performance
   - No additional configuration required for basic usage

3. Docker Networking
   - Use `network_mode: host` in docker-compose.yml
   - Ensure consistent ROS_DOMAIN_ID across containers
   - CycloneDDS handles multiple network interfaces reliably

## Development Environment Setup

### Prerequisites

1. Docker and Docker Compose
2. Python 3.8-3.12 (distribution dependent)
3. Git

### Local Development

1. Clone the repository:
```bash
git clone https://github.com/robroyce/ros_to_markdown.git
cd ros_to_markdown
```

2. Build Docker images:
```bash
docker compose build
```

3. Run tests:
```bash
./scripts/run-tests.sh
```

### Development Containers

Each ROS distribution has a development container variant:
- ros1-dev
- ros2-humble-dev
- ros2-iron-dev
- ros2-jazzy-dev

These containers include:
- All testing dependencies
- Development tools
- Source code mounted at /workspace
- Automatic environment setup

Example usage:
```bash
# Start a development shell
./docker/scripts/run-in-docker.sh ros2-humble-dev bash

# Run tests in development container
./docker/scripts/run-in-docker.sh ros2-humble-dev pytest tests/
```

## Testing

### Test Structure

```
tests/
├── core/              # Core functionality tests
├── integration/       # Tests requiring ROS runtime
└── test_helpers/     # Test utilities and fixtures
```

### Running Tests

```bash
# Run all tests
./scripts/run-tests.sh

# Run specific test file
./scripts/run-tests.sh tests/core/test_ros_detector.py

# Run tests for specific ROS distribution
./scripts/run-tests.sh -d humble
```

### Test Categories

1. Core Tests
   - Unit tests for core functionality
   - No ROS runtime required
   - Fast execution

2. Integration Tests
   - Require ROS runtime
   - Run in Docker containers
   - Test actual ROS communication

3. Distribution-specific Tests
   - Test ROS version-specific features
   - Run in appropriate container
   - Verify cross-version compatibility

## Code Style and Quality

### Code Formatting

We use Ruff for Python code formatting and linting:

```bash
# Format code
ruff format .

# Run linter
ruff check .
```

### Type Hints

All Python code must include type hints:

```python
from typing import List, Optional

def process_nodes(nodes: List[str]) -> Optional[dict]:
    """Process ROS nodes and return results."""
    ...
```

### Documentation

- All functions must have docstrings (PEP 257)
- Update relevant markdown docs when adding features
- Keep README.md current with new functionality

## Debugging

### ROS2 Node Discovery

If experiencing node discovery issues:

1. Check DDS Implementation:
```bash
echo $RMW_IMPLEMENTATION
# Should output: rmw_cyclonedds_cpp
```

2. Verify Domain ID:
```bash
echo $ROS_DOMAIN_ID
# Should output: 0
```

3. Enable Debug Logging:
```bash
export RCUTILS_LOGGING_BUFFERED_STREAM=1
export RCUTILS_LOGGING_USE_STDOUT=1
export RCUTILS_COLORIZED_OUTPUT=1
```

### Container Debugging

1. Interactive Shell:
```bash
./docker/scripts/run-in-docker.sh ros2-humble-dev bash
```

2. Check Environment:
```bash
# Inside container
env | grep ROS
env | grep RMW
```

3. Test Node Discovery:
```bash
# Terminal 1
ros2 run turtlesim turtlesim_node

# Terminal 2
ros2 node list
ros2 topic list
```

## Contributing

1. Fork the repository
2. Create a feature branch
3. Add tests for new functionality
4. Ensure all tests pass
5. Submit a pull request

See CONTRIBUTING.md for detailed guidelines. 