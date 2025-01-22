import os
from typing import Optional

from .schema import Config, RosVersion, RuntimeConfig


def get_env_config() -> Optional[Config]:
    """Load configuration from environment variables."""
    if not any(k.startswith("R2MD_") for k in os.environ):
        return None

    runtime_config = RuntimeConfig(
        namespace=os.getenv("R2MD_NAMESPACE"),
        node_filter=os.getenv("R2MD_NODE_FILTER", "").split(",") if os.getenv("R2MD_NODE_FILTER") else None,
        topic_filter=os.getenv("R2MD_TOPIC_FILTER", "").split(",") if os.getenv("R2MD_TOPIC_FILTER") else None,
    )

    return Config(
        runtime=runtime_config,
        output_dir=os.getenv("R2MD_OUTPUT_DIR", "./docs"),
        debug=os.getenv("R2MD_DEBUG", "false").lower() == "true"
    )
