# ros_to_markdown

Convert ROS systems to Markdown for LLM processing.

## Overview
`ros_to_markdown` is a Python tool that generates comprehensive Markdown documentation from ROS1 and ROS2 systems. It can analyze:
- ROS workspaces (packages, source code)
- Running ROS systems (nodes, topics, services)
- ROS bag files

The generated documentation is optimized for consumption by LLMs and AI agents.

## Installation
```bash
pip install -e .
```


## Supported ROS Distributions
- ROS1: Noetic
- ROS2: Humble (LTS), Iron (LTS), Jazzy

## Project Status
> [!Note]
> This repository started as an experiment to see how good current AI tools are at creating complex projects. The vast majority of code was generated by AI (Cursor with Claude 3.5 Sonnet), with almost no code written directly by the authors. That said, the authors did write the documentation and project structure, and spent a **lot** of time iterating with the AI to make it all work. Lessons learned will follow once the project nears completion.

## License
Apache 2.0

