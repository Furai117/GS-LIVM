#!/usr/bin/env python3
"""Utility script to launch GS-LIVM on recorded datasets.

This script wraps ``roslaunch`` to run GS-LIVM with an input ROS bag.
It selects the correct launch file based on the sensor type and forwards
optional parameter overrides to ``roslaunch``.

Example:
    python scripts/run_dataset.py my.bag r3live \\
        --override debug_output:=1 --override output_path:=/tmp/out
"""
import argparse
import shlex
import subprocess
from typing import List

# Mapping between human readable sensor types and roslaunch files
SENSOR_LAUNCH_MAP = {
    "r3live": "livo_r3live_compressed.launch",
    "ntu": "livo_ntu.launch",
    "fastlivo": "livo_fastlivo_compressed.launch",
    "botanic": "livo_botanic_garden.launch",
    "botanic_livox": "livo_botanic_garden_livox.launch",
}


def build_command(bag: str, sensor: str, overrides: List[str]) -> List[str]:
    """Build the roslaunch command.

    Args:
        bag: Path to the rosbag file.
        sensor: Sensor type key from ``SENSOR_LAUNCH_MAP``.
        overrides: A list of parameter override strings ``name:=value``.

    Returns:
        A list representing the roslaunch command and its arguments.
    """
    launch_file = SENSOR_LAUNCH_MAP[sensor]
    cmd = ["roslaunch", "gslivm", launch_file, f"bag_path:={bag}"]
    for ov in overrides:
        if ":=" not in ov:
            # Allow KEY=VAL syntax for convenience
            ov = ov.replace("=", ":=", 1)
        cmd.append(ov)
    return cmd


def main() -> None:
    parser = argparse.ArgumentParser(description="Run GS-LIVM on a dataset")
    parser.add_argument("bag", help="Path to the ROS bag file")
    parser.add_argument("sensor", choices=sorted(SENSOR_LAUNCH_MAP),
                        help="Sensor/dataset type")
    parser.add_argument(
        "-o",
        "--override",
        action="append",
        default=[],
        help=(
            "Parameter override in the form name:=value."
            " Can be specified multiple times."
        ),
    )
    args = parser.parse_args()

    cmd = build_command(args.bag, args.sensor, args.override)
    print("Running:", " ".join(shlex.quote(c) for c in cmd))
    subprocess.run(cmd)


if __name__ == "__main__":
    main()
