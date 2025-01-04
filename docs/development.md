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
# Build all images
docker compose build

# Build specific services
./scripts/docker-manager.sh build ros2-humble ros2-humble-dev

# Clean and rebuild
./scripts/docker-manager.sh build --clean
```

To clean up project-specific images and containers:
```bash
./scripts/docker-manager.sh clean
```

3. Run tests:
```bash
./scripts/test-manager.sh test
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
./scripts/docker-manager.sh run ros2-humble-dev bash

# Run tests in development container
./scripts/docker-manager.sh run ros2-humble-dev pytest tests/
```

#### Recommended Aliases

Add these to your shell configuration:
```bash
# Docker management
alias rtm-docker='./scripts/docker-manager.sh'
alias rtm-test='./scripts/test-manager.sh'

# Development environments
alias rtm-ros1='./scripts/docker-manager.sh run ros1-dev'
alias rtm-humble='./scripts/docker-manager.sh run ros2-humble-dev'
alias rtm-iron='./scripts/docker-manager.sh run ros2-iron-dev'
alias rtm-jazzy='./scripts/docker-manager.sh run ros2-jazzy-dev'
```

#### ROS2 Jazzy Python Environment

ROS2 Jazzy uses Python 3.12 for system packages but we use Python 3.11 in a virtual environment for compatibility. The PYTHONPATH is configured to handle both:

1. Workspace code (`/workspace/src`)
2. Python 3.11 venv packages (`/home/ros/.venv/lib/python3.11/site-packages`)
3. ROS2 Python 3.12 packages (`/opt/ros/jazzy/lib/python3.12/site-packages`)

Development containers (`ros2-jazzy-dev`) automatically activate the virtual environment.

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
./scripts/test-manager.sh test

# Run specific test file
./scripts/test-manager.sh test tests/core/test_ros_detector.py

# Run tests for specific ROS distribution
./scripts/test-manager.sh test -d humble

# Run integration tests
./scripts/test-manager.sh integration

# Generate coverage report
./scripts/test-manager.sh coverage
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
./scripts/docker-manager.sh run ros2-humble-dev bash
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

## Development Tools

### Quick Start Commands
```bash
# Docker management
./scripts/docker-manager.sh build ros2-humble  # Build specific image
./scripts/docker-manager.sh clean             # Clean Docker resources
./scripts/docker-manager.sh prune             # Deep clean all resources

# Testing
./scripts/test-manager.sh check              # Run all pre-commit checks
./scripts/test-manager.sh test               # Run tests only
./scripts/test-manager.sh coverage           # Generate coverage report
./scripts/test-manager.sh integration        # Run integration tests

# Documentation
./scripts/generate-docs.py                   # Generate all docs
```

### Shell Aliases
```bash
# Docker management
alias rtm-docker='./scripts/docker-manager.sh'
alias rtm-test='./scripts/test-manager.sh'

# Development environments
alias rtm-ros1='./scripts/docker-manager.sh run ros1-dev'
alias rtm-humble='./scripts/docker-manager.sh run ros2-humble-dev'
alias rtm-iron='./scripts/docker-manager.sh run ros2-iron-dev'
alias rtm-jazzy='./scripts/docker-manager.sh run ros2-jazzy-dev'
```

### Project Scripts

The project uses several scripts for development and testing:

```
scripts/
├── check.sh              # Pre-commit checks (ruff + pytest)
├── docker-manager.sh     # Docker environment management
├── test-manager.sh       # Test orchestration
└── utils/               # Utility scripts
    ├── analyze_ros_graph.py
    ├── launch_turtlesim.py
    ├── launch-ros1-test-env.sh
    └── launch-ros2-test-env.sh
```

#### Common Commands
```bash
# Run pre-commit checks
./scripts/check.sh

# Build Docker images
./scripts/docker-manager.sh build --clean

# Run tests
./scripts/test-manager.sh test

# Run integration tests
./scripts/test-manager.sh integration

# Start a development shell
./scripts/docker-manager.sh run ros2-humble-dev bash
```

### Testing

```bash
# Run all tests
./scripts/test-manager.sh test

# Run specific test file
./scripts/test-manager.sh test tests/core/test_ros_detector.py

# Run tests for specific ROS distribution
./scripts/test-manager.sh test -d humble

# Run integration tests
./scripts/test-manager.sh integration

# Generate coverage report
./scripts/test-manager.sh coverage
``` 