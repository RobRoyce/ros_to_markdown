#!/bin/bash

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

# Default values
VERBOSE=0
ROS1_ONLY=0
ROS2_ONLY=0
SPECIFIC_DISTRO=""
TEST_PATH=""
COVERAGE=0

# Function to show usage
show_usage() {
    echo -e "${GREEN}ROS Test Manager${NC}"
    echo "Usage: $0 [command] [options] [test_path]"
    echo
    echo "Commands:"
    echo "  check         Run all checks (lint, type check, test)"
    echo "  test          Run tests only"
    echo "  coverage      Generate and display coverage report"
    echo "  integration   Run integration tests"
    echo
    echo "Options:"
    echo "  -v, --verbose     Show verbose output"
    echo "  -r, --ros1        Test only ROS1"
    echo "  -2, --ros2        Test only ROS2"
    echo "  -d, --distro      Test specific distribution"
    echo "  -s, --sanity      Run sanity checks only"
    echo "  -c, --coverage    Generate coverage report"
    echo "  -h, --help        Show this help message"
}

# Function to run all checks
run_checks() {
    echo -e "${GREEN}Running all checks...${NC}"
    
    # 1. Linting
    echo -e "\n${YELLOW}Running Ruff linter and formatter...${NC}"
    if ! ruff check ${VERBOSE:+--verbose} .; then
        echo -e "${RED}Linting failed${NC}"
        return 1
    fi
    if ! ruff format ${VERBOSE:+--verbose} .; then
        echo -e "${RED}Formatting failed${NC}"
        return 1
    fi
    
    # 2. Type checking
    echo -e "\n${YELLOW}Running type checks...${NC}"
    if ! python -m mypy .; then
        echo -e "${RED}Type checking failed${NC}"
        return 1
    fi
    
    # 3. Tests with optional coverage
    if [ $COVERAGE -eq 1 ]; then
        handle_coverage
    else
        run_tests
    fi
}

# Function to run tests
run_tests() {
    echo -e "\n${YELLOW}Running tests...${NC}"
    
    local pytest_args=()
    [ $VERBOSE -eq 1 ] && pytest_args+=("-v")
    [ -n "$TEST_PATH" ] && pytest_args+=("$TEST_PATH")
    
    if [ $ROS1_ONLY -eq 1 ]; then
        echo "Running ROS1 tests only"
        DOCKER_ARGS="--env ROS_VERSION=1" ./scripts/docker-manager.sh run ros1 pytest "${pytest_args[@]}"
    elif [ $ROS2_ONLY -eq 1 ]; then
        echo "Running ROS2 tests only"
        DOCKER_ARGS="--env ROS_VERSION=2" ./scripts/docker-manager.sh run ros2-humble pytest "${pytest_args[@]}"
    elif [ -n "$SPECIFIC_DISTRO" ]; then
        echo "Running tests for ROS distribution: $SPECIFIC_DISTRO"
        DOCKER_ARGS="--env ROS_DISTRO=$SPECIFIC_DISTRO" ./scripts/docker-manager.sh run ros2-$SPECIFIC_DISTRO pytest "${pytest_args[@]}"
    else
        echo "Running all tests"
        ./scripts/docker-manager.sh run ros1 pytest "${pytest_args[@]}"
        for distro in humble iron jazzy; do
            ./scripts/docker-manager.sh run ros2-$distro pytest "${pytest_args[@]}"
        done
    fi
}

# Function to handle coverage
handle_coverage() {
    echo -e "${YELLOW}Generating coverage report...${NC}"
    
    # Run tests with coverage
    python -m pytest --cov=src/ros_to_markdown \
                    --cov-report=term-missing \
                    --cov-report=html:coverage_html \
                    ${VERBOSE:+"-v"} \
                    ${TEST_PATH}
    
    # Show coverage report
    echo -e "\n${GREEN}Coverage report generated in coverage_html/${NC}"
}

# Function to run integration tests
run_integration() {
    echo -e "${YELLOW}Running integration tests...${NC}"
    TEST_PATH="tests/integration/"
    
    if [ $VERBOSE -eq 1 ]; then
        run_tests -v
    else
        run_tests
    fi
}

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        check|test|coverage)
            COMMAND=$1
            shift
            ;;
        -v|--verbose)
            VERBOSE=1
            shift
            ;;
        -r|--ros1)
            ROS1_ONLY=1
            shift
            ;;
        -2|--ros2)
            ROS2_ONLY=1
            shift
            ;;
        -d|--distro)
            SPECIFIC_DISTRO="$2"
            shift 2
            ;;
        -s|--sanity)
            TEST_PATH="tests/core/test_ros_detector.py"
            VERBOSE=1
            shift
            ;;
        -c|--coverage)
            COVERAGE=1
            shift
            ;;
        -h|--help)
            show_usage
            exit 0
            ;;
        *)
            if [ -z "$TEST_PATH" ]; then
                TEST_PATH="$1"
            else
                echo -e "${RED}Unknown argument: $1${NC}"
                show_usage
                exit 1
            fi
            shift
            ;;
    esac
done

# Execute command
case "$COMMAND" in
    check)
        run_checks
        ;;
    test)
        run_tests
        ;;
    coverage)
        handle_coverage
        ;;
    integration)
        run_integration
        ;;
    *)
        show_usage
        exit 1
        ;;
esac 