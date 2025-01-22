import logging
import sys
from typing import Any, Optional

from rich.console import Console
from rich.text import Text
import structlog

# Create rich console for colored output
console = Console(file=sys.stderr)


def add_rich_renderer(
    _: structlog.types.WrappedLogger,
    __: str,
    event_dict: structlog.types.EventDict,
) -> str:
    """Add color and styling to log output using rich."""
    level = event_dict.pop("level", "info").lower()
    event = event_dict.pop("event")
    timestamp = event_dict.pop("timestamp", "")

    # Color mapping
    level_colors = {
        "debug": "blue",
        "info": "green",
        "warning": "yellow",
        "error": "red",
        "critical": "red",
    }

    # Build the message using Rich's Text class
    msg = Text()
    msg.append(f"{timestamp} ", style="white")
    msg.append(f"{level:>8}", style=level_colors.get(level, "white"))
    msg.append(" │ ", style="white")
    msg.append(event)

    # Add any remaining key-value pairs
    if event_dict:
        msg.append(" ")
        for k, v in event_dict.items():
            msg.append(f"{k}=", style="cyan")
            msg.append(f"{v} ", style="white")

    # Print using Rich's console and return empty string
    # (since we're handling the printing ourselves)
    console.print(msg)
    return ""


def setup_logging(debug: bool = False) -> None:
    """Configure structlog for the application."""
    # Set up stdlib logging
    logging.basicConfig(
        format="%(message)s",
        stream=sys.stderr,
        level=logging.DEBUG if debug else logging.INFO,
    )

    structlog.configure(
        processors=[
            structlog.stdlib.add_log_level,
            structlog.stdlib.filter_by_level,
            structlog.processors.TimeStamper(fmt="%Y-%m-%d %H:%M:%S"),
            structlog.processors.StackInfoRenderer(),
            structlog.processors.format_exc_info,
            structlog.processors.UnicodeDecoder(),
            add_rich_renderer,
        ],
        context_class=dict,
        logger_factory=structlog.stdlib.LoggerFactory(),
        wrapper_class=structlog.stdlib.BoundLogger,
        cache_logger_on_first_use=True,
    )


def get_logger(name: Optional[str] = None) -> structlog.stdlib.BoundLogger:
    """Get a logger instance with optional name binding."""
    logger = structlog.get_logger(name)
    return logger.bind(module=name) if name else logger


class LogContext:
    """Context manager for temporary log context."""
    def __init__(self, **kwargs: Any):
        self.logger = get_logger()
        self.kwargs = kwargs
        self.token: Optional[Any] = None

    def __enter__(self) -> structlog.stdlib.BoundLogger:
        self.token = structlog.contextvars.bind_contextvars(**self.kwargs)
        return self.logger

    def __exit__(self, *args: Any) -> None:
        structlog.contextvars.unbind_contextvars(self.token)
