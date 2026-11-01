#!/usr/bin/env python3
"""Run a reproducible two-operator demo without ROS, Gazebo or networking."""

from __future__ import annotations

import json
from pathlib import Path
import sys

REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPOSITORY_ROOT / "src"))

from teleop_cooperativo.arbiter import CooperativeArbiter
from teleop_cooperativo.lease_manager import LeaseManager
from teleop_cooperativo.simulation_runner import SimulationRunner
from teleop_cooperativo.tool_simulator import DeterministicToolSimulator


def main() -> int:
    """Execute motion, input expiry and lease release in deterministic time."""
    arbiter = CooperativeArbiter(
        LeaseManager(input_timeout_sec=0.15, lease_timeout_sec=0.25)
    )
    simulator = DeterministicToolSimulator()
    runner = SimulationRunner(arbiter, simulator)
    runner.start(now=0.0)

    translation = arbiter.claim("operator-a", ["X", "Y", "Z"], now=0.0)
    rotation = arbiter.claim(
        "operator-b", ["ROLL", "PITCH", "YAW"], now=0.0
    )
    arbiter.submit_input(
        "operator-a",
        1,
        {"X": 0.8, "Y": -0.4, "Z": 0.2},
        _tokens(translation),
        now=0.01,
    )
    arbiter.submit_input(
        "operator-b",
        1,
        {"ROLL": 0.2, "PITCH": 0.0, "YAW": 0.6},
        _tokens(rotation),
        now=0.01,
    )

    timeline = []
    for timestamp in (0.05, 0.10, 0.15, 0.20, 0.25, 0.30):
        step = runner.step(now=timestamp)
        timeline.append(
            {
                "time_sec": timestamp,
                "room_state": step.arbiter.state.value,
                "position": step.pose.position,
                "quaternion_xyzw": step.pose.quaternion_xyzw,
                "linear_twist": step.arbiter.twist.linear,
                "angular_twist": step.arbiter.twist.angular,
                "zeroed_axes": [
                    axis.value
                    for axis in step.arbiter.maintenance.zeroed_axes
                ],
                "released_axes": [
                    axis.value
                    for axis in step.arbiter.maintenance.released_axes
                ],
            }
        )

    print(
        json.dumps(
            {
                "demo": "deterministic_two_operator_harness",
                "final_room_state": arbiter.state.value,
                "timeline": timeline,
            },
            indent=2,
        )
    )
    return 0


def _tokens(leases):
    return {lease.axis: (lease.lease_id, lease.generation) for lease in leases}


if __name__ == "__main__":
    raise SystemExit(main())
