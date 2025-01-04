# Testing Guide

This document describes the testing infrastructure and procedures for the ros_to_markdown project.

## Test Structure

+ ## Quick Start
+ 
+ ```bash
+ # Run all tests
+ ./scripts/run-tests.sh
+ 
+ # Run quick sanity check (ROS detection only)
+ ./scripts/run-tests.sh -s
+ 
+ # Run tests for specific ROS distribution
+ ./scripts/run-tests.sh -d humble
+ 
+ # Run tests with verbose output
+ ./scripts/run-tests.sh -v
+ ```
+ 
+ ## Test Categories
+ 
+ ### 1. Sanity Check
+ - Quick verification of ROS detection across all distributions
+ - Run with `-s` or `--sanity` flag
+ - Tests basic environment setup and ROS version detection
+ - Useful for CI/CD pipelines and quick environment validation
+ 
+ ### 2. Core Tests
+ - Located in `tests/core/`
+ - Unit tests for core functionality
+ - No ROS runtime required
+ - Fast execution
+ 
+ ### 3. Integration Tests
+ - Located in `tests/integration/`
+ - Require ROS runtime environment
+ - Run in Docker containers
+ - Test actual ROS communication
+ 
+ ## Test Options
+ 
+ ```bash
+ Options:
+   -h, --help     Show help message
+   -v, --verbose  Show verbose output
+   -r, --ros1     Test only ROS1
+   -2, --ros2     Test only ROS2
+   -d, --distro   Test specific distribution (noetic|humble|iron|jazzy)
+   -s, --sanity   Run quick sanity check (ROS detection tests only)
+ ```
+ 
+ ## Directory Structure
+ 
+ ```
+ tests/
+ ├── core/              # Core functionality tests
+ │   ├── test_ros_detector.py    # ROS detection tests (used by sanity check)
+ │   └── test_ros_components.py  # Component model tests
+ ├── integration/       # Tests requiring ROS runtime
+ └── test_helpers/     # Test utilities and fixtures
+ ```
+ 
+ ## Development Workflow
+ 
+ When developing new features:
+ 
+ 1. Start with sanity check:
+    ```bash
+    ./scripts/run-tests.sh -s
+    ```
+ 
+ 2. Run specific tests for your feature:
+    ```bash
+    ./scripts/run-tests.sh tests/path/to/test.py
+    ```
+ 
+ 3. Run distribution-specific tests if needed:
+    ```bash
+    ./scripts/run-tests.sh -d humble tests/path/to/test.py
+    ```
+ 
+ 4. Finally, run all tests:
+    ```bash
+    ./scripts/run-tests.sh
+    ```
+ 
+ ## Docker Integration
+ 
+ Tests are run in Docker containers to ensure consistent environments:
+ 
+ - ROS1 (Noetic): `ros1` container
+ - ROS2 (Humble): `ros2-humble` container
+ - ROS2 (Iron): `ros2-iron` container
+ - ROS2 (Jazzy): `ros2-jazzy` container
+ 
+ The test runner automatically manages these containers. See `docs/docker.md` for more details. 