#!/usr/bin/env bash

# Exit on error
set -e

# Default values
VERBOSE=0
LINT_ONLY=0
TEST_ONLY=0
TEST_ARGS=""

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        -h|--help)
            echo "Usage: $0 [OPTIONS]"
            echo ""
            echo "Options:"
            echo "  -h, --help     Show this help message"
            echo "  -v, --verbose  Show verbose output"
            echo "  -l, --lint     Run only linting"
            echo "  -t, --test     Run only tests"
            echo "  -- [args]      Pass remaining arguments to test runner"
            exit 0
            ;;
        -v|--verbose)
            VERBOSE=1
            shift
            ;;
        -l|--lint)
            LINT_ONLY=1
            shift
            ;;
        -t|--test)
            TEST_ONLY=1
            shift
            ;;
        --)
            shift
            TEST_ARGS="$@"
            break
            ;;
        *)
            echo "Unknown option: $1"
            exit 1
            ;;
    esac
done

# Function to run linting
run_lint() {
    echo "Running Ruff linter and formatter..."
    if [ $VERBOSE -eq 1 ]; then
        ruff check . --verbose
        ruff format . --verbose
    else
        ruff check .
        ruff format .
    fi
}

# Function to run tests
run_tests() {
    echo "Running tests..."
    if [ -n "$TEST_ARGS" ]; then
        ./scripts/run-tests.sh $TEST_ARGS
    elif [ $VERBOSE -eq 1 ]; then
        ./scripts/run-tests.sh -v
    else
        ./scripts/run-tests.sh
    fi
}

# Main execution
if [ $LINT_ONLY -eq 1 ]; then
    run_lint
elif [ $TEST_ONLY -eq 1 ]; then
    run_tests
else
    run_lint
    echo ""
    run_tests
fi 