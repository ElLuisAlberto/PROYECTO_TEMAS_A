#!/usr/bin/env python3
"""Validate the expanded H2c robot description and controller contract."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import subprocess
from xml.etree import ElementTree

import yaml


EXPECTED_JOINTS = [f"joint_{index}" for index in range(1, 7)]


def main() -> int:
    """Expand Xacro and verify the model without launching Gazebo."""
    parser = argparse.ArgumentParser()
    parser.add_argument("xacro_file", type=Path)
    parser.add_argument("controllers_file", type=Path)
    arguments = parser.parse_args()
    completed = subprocess.run(
        ["xacro", str(arguments.xacro_file)],
        check=True,
        capture_output=True,
        text=True,
    )
    root = ElementTree.fromstring(completed.stdout)
    movable = [
        joint.attrib["name"]
        for joint in root.findall("joint")
        if joint.attrib["type"] != "fixed"
    ]
    if movable != EXPECTED_JOINTS:
        raise AssertionError(f"unexpected movable joints: {movable}")
    links = {link.attrib["name"] for link in root.findall("link")}
    required_links = {"world", "base_link", "tool_link", "tool_frame"}
    if not required_links <= links:
        raise AssertionError(f"missing links: {sorted(required_links - links)}")
    ros2_control = root.find("ros2_control")
    if ros2_control is None:
        raise AssertionError("ros2_control block is missing")
    controlled = [
        joint.attrib["name"] for joint in ros2_control.findall("joint")
    ]
    if controlled != EXPECTED_JOINTS:
        raise AssertionError(f"unexpected controlled joints: {controlled}")
    for link in root.findall("link"):
        inertial = link.find("inertial")
        if inertial is None:
            continue
        mass = float(inertial.find("mass").attrib["value"])
        diagonal = inertial.find("inertia").attrib
        values = [mass] + [
            float(diagonal[key]) for key in ("ixx", "iyy", "izz")
        ]
        if not all(math.isfinite(value) and value > 0.0 for value in values):
            raise AssertionError(f"invalid inertia for {link.attrib['name']}")
    plugins = [
        plugin
        for gazebo in root.findall("gazebo")
        for plugin in gazebo.findall("plugin")
    ]
    if not any(
        plugin.attrib.get("name")
        == "gz_ros2_control::GazeboSimROS2ControlPlugin"
        for plugin in plugins
    ):
        raise AssertionError("gz_ros2_control plugin is missing")
    controller_data = yaml.safe_load(
        arguments.controllers_file.read_text(encoding="utf-8")
    )
    configured = controller_data["arm_controller"]["ros__parameters"]["joints"]
    if configured != EXPECTED_JOINTS:
        raise AssertionError(f"unexpected controller joints: {configured}")
    print(
        json.dumps(
            {
                "robot": root.attrib["name"],
                "movable_joints": movable,
                "controlled_joints": controlled,
                "required_links": sorted(required_links),
                "result": "PASS",
            },
            indent=2,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
