import argparse


def parse_args():
    parser = argparse.ArgumentParser(description="ROS to Markdown")
    parser.add_argument("--input", type=str, help="Input ROS bag file")
    parser.add_argument("--output", type=str, help="Output Markdown file")
    return parser.parse_args()
