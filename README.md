# ros_to_markdown

Convert ROS systems to Markdown for LLM processing.

## Overview

`ros_to_markdown` automatically generates comprehensive Markdown documentation from ROS1 and ROS2 systems. It can analyze:
- ROS workspaces (packages, source code)
- Running ROS systems (nodes, topics, services)
- ROS bag files

## Supported ROS Versions

- ROS1:
  - Noetic (Ubuntu 20.04)

- ROS2:
  - Humble Hawksbill (Ubuntu 22.04, LTS)
  - Iron Irwini (Ubuntu 22.04)
  - Jazzy Jalisco (Ubuntu 24.04)

The generated Markdown files are optimized for Large Language Model (LLM) processing, ensuring detailed and structured documentation of your ROS ecosystem.

## ROS2 Support Status

Currently supported:
- Humble (LTS, EOL May 2027)
- Iron (EOL Dec 2024)

Coming soon:
- Jazzy (Support planned for future release)

Note: ROS2 Jazzy support is temporarily disabled. The Dockerfile and configuration are included but commented out for future use.

## Features

- **Multi-Source Support**:
  - ROS Workspace analysis
  - Runtime system inspection
  - ROS bag file processing

- **Comprehensive Documentation**:
  - Nodes (name, package, description, subscribers, publishers, etc.)
  - Topics (name, type, description, subscribers, frequency, publishers)
  - Services (name, type, description, clients, providers)
  - Actions (name, type, description, clients, providers)
  - Parameters (name, type, description, value)
  - Messages (name, type, description, fields)
  - Launch Files (name, description, nodes, topics, services, actions, parameters)

- **Flexible Configuration**:
  - Automatic ROS1/ROS2 detection
  - Configurable file type inclusion/exclusion
  - Blacklist support for filtering components
  - Docker support for isolated execution

## Development

### Prerequisites

- Docker Engine and Docker Compose (Docker Desktop recommended)
- Python 3.8-3.11
- Git

### Quick Start

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

### Testing

The project includes comprehensive test suites that run across all supported ROS distributions:

```bash
# Run all tests
./scripts/run-tests.sh

# Test specific ROS version
./scripts/run-tests.sh -r  # ROS1 only
./scripts/run-tests.sh -2  # ROS2 only

# Test specific distribution
./scripts/run-tests.sh -d humble

# Run specific test file
./scripts/run-tests.sh tests/core/test_ros_detector.py

# Run with verbose output
./scripts/run-tests.sh -v
```

Tests are automatically run in isolated Docker containers for each ROS distribution:
- ROS1 Noetic (Ubuntu 20.04, Python 3.8)
- ROS2 Humble (Ubuntu 22.04, Python 3.10)
- ROS2 Iron (Ubuntu 22.04, Python 3.10)

### Project Structure

```
ros_to_markdown/
├── src/
│   └── ros_to_markdown/
│       ├── core/           # Core functionality
│       ├── models/         # Data models
│       └── utils/          # Utility functions
├── tests/
│   ├── core/              # Core tests
│   └── test_helpers/      # Test utilities
├── docker/
│   ├── Dockerfile.*       # Docker configurations
│   └── scripts/           # Docker helper scripts
├── docs/                  # Documentation
├── scripts/               # Development scripts
└── pyproject.toml        # Project configuration
```

### Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Run tests: `./scripts/run-tests.sh`
5. Submit a pull request

For more details, see the [Docker documentation](docs/docker.md).
