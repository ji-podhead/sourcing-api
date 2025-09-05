#!/usr/bin/env python3

import argparse
import datetime
import pathlib

def create_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Generate a ROS2 bag name for recordings")

    parser.add_argument(
        "directory",
        type=str,
        help="directory where the data is stored")

    parser.add_argument(
        "name",
        type=str,
        help="basename of the file")

    return parser

def parse_program_arguments() -> pathlib.Path:
    """
    Parse program arguments.

    :returns: path of the ROS2 bag directory and the basename
    """
    parser = create_argument_parser()
    args = parser.parse_args()

    return pathlib.Path(args.directory), args.name

def generate_name(
        basename: str,
        date: datetime) -> str:
    """
    # ...rest of file unchanged...
    """