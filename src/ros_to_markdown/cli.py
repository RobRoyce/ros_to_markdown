#!/usr/bin/env python3
import asyncio
import logging
import os
from pathlib import Path
from typing import Any, Dict, Optional

from ros_to_markdown.analysis.factory import AnalyzerInitError, get_analyzer
from ros_to_markdown.config.environment import get_env_config
from ros_to_markdown.config.schema import Config, RosDistro, RosVersion
from ros_to_markdown.logging import get_logger, setup_logging
from ros_to_markdown.perspectives.engine import PerspectiveEngine
from ros_to_markdown.perspectives.loader import load_perspective

import click

# Configure logging before any ROS imports
logging.getLogger("rosout").setLevel(logging.ERROR)
logging.getLogger("rospy").setLevel(logging.ERROR)
os.environ["ROS_PYTHON_LOG_CONFIG_FILE"] = ""
os.environ["ROSCONSOLE_CONFIG_FILE"] = ""
os.environ["ROSCONSOLE_STDOUT_LINE_BUFFERED"] = "0"

# Conditionally import ROS modules based on ROS_VERSION
ros_version = int(os.getenv("ROS_VERSION", "2"))
if ros_version == 1:
    try:
        import rospy

        rospy.core.configure_logging = lambda *args, **kwargs: None
    except ImportError:
        pass  # ROS1 not installed, that's ok

logger = get_logger(__name__)


def get_config(cli_config: Optional[dict] = None) -> Config:
    """
    Get configuration following precedence: CLI > ENV > defaults.
    """
    logger.debug("Getting configuration", cli_config=cli_config)

    # Convert to enum values
    ros_distro = RosDistro(os.getenv("ROS_DISTRO", "humble"))
    ros_version = RosVersion(int(os.getenv("ROS_VERSION", "2")))

    logger.info("ROS configuration", ros_distro=ros_distro, ros_version=ros_version)

    config = Config(ros_distro=ros_distro, ros_version=ros_version)  # Start with defaults

    # Load from environment
    if env_config := get_env_config():
        logger.debug("Loaded config from environment", config=env_config)
        config = config.model_copy(update=env_config.model_dump(exclude_unset=True))

    # Load from CLI
    if cli_config:
        logger.debug("Loaded config from CLI", config=cli_config)
        config = config.model_copy(update=cli_config)

    logger.debug("Final configuration", config=config)
    return config


@click.group()
@click.option("--output-dir", type=click.Path(), help="Output directory for markdown files")
@click.option("--debug/--no-debug", default=None, help="Enable debug logging")
@click.option("--perspective", type=str, help="Analysis perspective to use")
@click.pass_context
def cli(
    ctx: click.Context,
    output_dir: Optional[str],
    debug: Optional[bool],
    perspective: Optional[str],
) -> None:
    """ROS to Markdown - Generate markdown documentation from ROS systems."""
    # Initialize logging first
    setup_logging(debug=debug if debug is not None else False)
    logger.debug("CLI startup", output_dir=output_dir, debug=debug, perspective=perspective)

    cli_config: Dict[str, Any] = {}
    if output_dir:
        cli_config["output_dir"] = str(output_dir)
    if debug is not None:
        cli_config["debug"] = debug
    if perspective:
        cli_config["perspective"] = perspective

    ctx.obj = get_config(cli_config)
    logger.debug("Starting ROS to Markdown", version="0.1.0", perspective=perspective)


@cli.command()
@click.option("--namespace", help="ROS namespace to analyze")
@click.option("--node-filter", multiple=True, help="Node name patterns to include")
@click.option("--topic-filter", multiple=True, help="Topic name patterns to include")
@click.pass_obj
def runtime(
    config: Config,
    namespace: Optional[str],
    node_filter: tuple,
    topic_filter: tuple,
) -> None:
    """Analyze a running ROS system."""
    logger.debug(
        "Runtime command start",
        namespace=namespace,
        node_filter=node_filter,
        topic_filter=topic_filter,
    )

    if namespace:
        config.runtime.namespace = namespace
    if node_filter:
        config.runtime.node_filter = list(node_filter)
    if topic_filter:
        config.runtime.topic_filter = list(topic_filter)

    logger.info(
        "Starting runtime analysis",
        namespace=config.runtime.namespace,
        node_filter=config.runtime.node_filter,
        topic_filter=config.runtime.topic_filter,
    )

    # Initialize analyzer
    logger.debug("Creating analyzer")
    try:
        analyzer = get_analyzer(config)
        logger.info("Analyzer initialized", type=type(analyzer).__name__)
    except AnalyzerInitError as e:
        logger.error("Failed to initialize analyzer", error=str(e))
        raise click.ClickException(str(e)) from e

    # Load perspective (use basic if none specified)
    perspective_name = config.perspective or "basic"
    try:
        logger.debug("Loading perspective", name=perspective_name)
        perspective = load_perspective(perspective_name)
        logger.debug("Loaded perspective", name=perspective.name)
    except Exception as e:
        logger.error("Failed to load perspective", error=str(e))
        return

    # Create and run perspective engine
    logger.debug("Creating perspective engine")
    engine = PerspectiveEngine(perspective)

    async def run_analysis() -> None:
        try:
            logger.debug("Starting analysis execution")
            # Run perspective pipeline
            result = await engine.execute({"analyzer": analyzer})
            logger.debug("Analysis execution complete")

            # Get markdown output
            if "overview_doc" not in result:
                logger.error("Perspective did not generate expected output")
                return

            # Write output
            output_dir = Path(config.output_dir or ".")
            output_dir.mkdir(parents=True, exist_ok=True)

            output_file = output_dir / f"{perspective.name}.md"
            logger.debug("Writing output", file=str(output_file))
            with open(output_file, "w") as f:
                f.write(result["overview_doc"])

            logger.info("Analysis complete", output_file=str(output_file))

        except Exception as e:
            logger.error("Analysis failed", error=str(e))
            raise RuntimeError("Analysis failed") from e

    logger.debug("Starting analysis run")
    asyncio.run(run_analysis())


# Entry point for the CLI
def main() -> None:
    """Entry point for the ros-to-markdown CLI."""
    cli()


if __name__ == "__main__":
    main()
