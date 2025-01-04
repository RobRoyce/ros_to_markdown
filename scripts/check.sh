#!/usr/bin/env bash

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

# Default values
VERBOSE=0
LINT_ONLY=0
TEST_ONLY=0
TEST_ARGS=""

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        -h|--help)
            echo -e "${GREEN}Pre-commit Check Tool${NC}"
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
            echo -e "${RED}Unknown option: $1${NC}"
            exit 1
            ;;
    esac
done

# Function to run linting
run_lint() {
    echo -e "${YELLOW}Running Ruff linter and formatter...${NC}"
    if [ $VERBOSE -eq 1 ]; then
        ruff check . --fix --verbose
        ruff format . --verbose
    else
        ruff check . --fix
        ruff format .
    fi
}

# Function to run tests
run_tests() {
    echo -e "${YELLOW}Running tests...${NC}"
    if [ -n "$TEST_ARGS" ]; then
        ./scripts/test-manager.sh test $TEST_ARGS
    elif [ $VERBOSE -eq 1 ]; then
        ./scripts/test-manager.sh test -v
    else
        ./scripts/test-manager.sh test
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