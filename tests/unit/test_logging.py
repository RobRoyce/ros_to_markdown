"""Test logging configuration and utilities."""
import logging
import pytest
import structlog

from ros_to_markdown.logging import setup_logging, get_logger


@pytest.fixture(autouse=True)
def reset_logging():
    """Reset logging configuration before each test."""
    logger = logging.getLogger("ros_to_markdown")
    logger.handlers = []  # Clear any existing handlers
    logger.setLevel(logging.NOTSET)
    yield
    logger.handlers = []
    logger.setLevel(logging.NOTSET)


def test_setup_logging():
    """Test logging setup."""
    # Test debug mode
    setup_logging(debug=True)
    logger = logging.getLogger("ros_to_markdown")
    assert logger.getEffectiveLevel() == logging.DEBUG
    
    # Test normal mode
    setup_logging(debug=False)
    assert logger.getEffectiveLevel() == logging.INFO
    
    # Verify structlog configuration
    bound_logger = get_logger("test")
    assert isinstance(bound_logger, structlog.stdlib.BoundLogger)
    assert "rich" in str(bound_logger._processors[-1])  # Check for rich renderer


def test_get_logger():
    """Test logger retrieval and configuration."""
    logger = get_logger("test_module")
    assert logger.name == "test_module"  # Test exact name match
    
    # Test logging at different levels
    logger.debug("Debug message")
    logger.info("Info message")
    logger.warning("Warning message")
    logger.error("Error message")


def test_logger_formatting():
    """Test log message formatting."""
    logger = get_logger("test")
    
    # Test with extra fields
    logger.info("Test message", extra_field="value")
    logger.error("Error message", error_code=500)
    
    # Test with exception
    try:
        raise ValueError("Test error")
    except ValueError as e:
        logger.exception("Caught error") 