import click
import os
from typing import Optional, List
import asyncio

from .config.schema import Config, RosVersion, RosDistro
from .config.environment import get_env_config
from .logging import setup_logging, get_logger
from .analysis.interfaces import SystemAnalyzer
from .analysis.factory import get_analyzer


def get_config(cli_config: Optional[dict] = None) -> Config:
    """
    Get configuration following precedence: CLI > ENV > defaults.
    """
    logger = get_logger(__name__)
    ros_distro = os.getenv("ROS_DISTRO", "humble")
    ros_version = int(os.getenv("ROS_VERSION", "2"))

    # Convert to enum values
    ros_distro = RosDistro(ros_distro)
    ros_version = RosVersion(ros_version)

    config = Config(
        ros_distro=ros_distro,
        ros_version=ros_version
        )  # Start with defaults

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
@click.option(
    "--output-dir", type=click.Path(), help="Output directory for markdown files"
)
@click.option("--debug/--no-debug", default=None, help="Enable debug logging")
@click.option("--perspective", type=str, help="Analysis perspective to use")
@click.pass_context
def cli(
    ctx: click.Context,
    output_dir: Optional[str],
    debug: Optional[bool],
    perspective: Optional[str],
):
    """ROS to Markdown - Generate markdown documentation from ROS systems."""
    # Initialize logging first
    setup_logging(debug=debug if debug is not None else False)
    logger = get_logger(__name__)

    cli_config = {}
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
    config: Config, namespace: Optional[str], node_filter: tuple, topic_filter: tuple
):
    """Analyze a running ROS system."""
    logger = get_logger(__name__)

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
    analyzer = get_analyzer(config)
    
    # Run analysis
    async def run_analysis():
        try:
            snapshot = await analyzer.get_snapshot()
            logger.info("System snapshot captured", 
                       nodes=len(snapshot.nodes),
                       topics=len(snapshot.topics))
            
            # TODO: Analyze frequencies and latencies based on filters
            
        except Exception as e:
            logger.error("Analysis failed", error=str(e))
    
    asyncio.run(run_analysis())


# Entry point for the CLI
def main() -> None:
    """Entry point for the ros-to-markdown CLI."""
    cli()


if __name__ == "__main__":
    main()
