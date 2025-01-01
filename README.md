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
