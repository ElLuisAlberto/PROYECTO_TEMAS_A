#!/usr/bin/env python3
"""Summarize append-only H3 evidence JSONL without third-party packages.

The output is deliberately JSON so it can be attached to a run sheet or used
later to generate tables.  It accepts current telemetry-only logs and future
gateway event records, preserving unknown event types as counters.
"""

from __future__ import annotations

import argparse
from collections import Counter
import json
import math
from pathlib import Path
from typing import Any, Iterable


def _finite(value: object, label: str) -> float:
    if isinstance(value, bool):
        raise ValueError(f"{label} must be finite")
    try:
        numeric = float(value)
    except (TypeError, ValueError) as error:
        raise ValueError(f"{label} must be finite") from error
    if not math.isfinite(numeric):
        raise ValueError(f"{label} must be finite")
    return numeric


def load_records(path: Path) -> list[dict[str, Any]]:
    """Read and minimally validate one run's ordered JSONL records."""
    records: list[dict[str, Any]] = []
    run_id: str | None = None
    previous_index = 0
    for line_number, raw_line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        if not raw_line.strip():
            continue
        try:
            record = json.loads(raw_line)
        except json.JSONDecodeError as error:
            raise ValueError(f"line {line_number}: invalid JSON") from error
        if not isinstance(record, dict):
            raise ValueError(f"line {line_number}: record must be an object")
        current_run = record.get("run_id")
        if not isinstance(current_run, str) or not current_run:
            raise ValueError(f"line {line_number}: missing run_id")
        if run_id is None:
            run_id = current_run
        elif current_run != run_id:
            raise ValueError(f"line {line_number}: mixed run_id values")
        index = record.get("event_index")
        if not isinstance(index, int) or index != previous_index + 1:
            raise ValueError(f"line {line_number}: event_index is not consecutive")
        previous_index = index
        _finite(record.get("monotonic_sec"), f"line {line_number} monotonic_sec")
        if not isinstance(record.get("event_type"), str):
            raise ValueError(f"line {line_number}: missing event_type")
        records.append(record)
    if not records:
        raise ValueError("log has no records")
    return records


def summarize(records: Iterable[dict[str, Any]]) -> dict[str, Any]:
    """Return reproducible event and command statistics for one evidence log."""
    items = list(records)
    event_counts = Counter(str(item["event_type"]) for item in items)
    times = [_finite(item["monotonic_sec"], "monotonic_sec") for item in items]
    room_states = Counter(
        str(item["room_state"].get("state"))
        for item in items
        if isinstance(item.get("room_state"), dict)
        and item["room_state"].get("state") is not None
    )
    telemetry = [item for item in items if item["event_type"] == "telemetry"]
    linear_norms: list[float] = []
    angular_norms: list[float] = []
    steps: list[float] = []
    zero_transitions: list[float] = []
    previously_moving = False
    for item in telemetry:
        payload = item.get("payload")
        if not isinstance(payload, dict):
            continue
        twist = payload.get("twist")
        if not isinstance(twist, dict):
            continue
        linear = twist.get("linear")
        angular = twist.get("angular")
        if not isinstance(linear, list) or not isinstance(angular, list):
            continue
        if len(linear) != 3 or len(angular) != 3:
            continue
        linear_norm = math.sqrt(sum(_finite(value, "linear twist") ** 2 for value in linear))
        angular_norm = math.sqrt(sum(_finite(value, "angular twist") ** 2 for value in angular))
        linear_norms.append(linear_norm)
        angular_norms.append(angular_norm)
        moving = linear_norm > 1e-12 or angular_norm > 1e-12
        if previously_moving and not moving:
            zero_transitions.append(_finite(item["monotonic_sec"], "monotonic_sec"))
        previously_moving = moving
        if "dt_sec" in payload:
            steps.append(_finite(payload["dt_sec"], "dt_sec"))

    result: dict[str, Any] = {
        "run_id": items[0]["run_id"],
        "records": len(items),
        "monotonic_start_sec": times[0],
        "monotonic_end_sec": times[-1],
        "duration_sec": times[-1] - times[0],
        "event_counts": dict(sorted(event_counts.items())),
        "room_state_counts": dict(sorted(room_states.items())),
        "telemetry": {
            "samples": len(telemetry),
            "max_linear_speed_mps": max(linear_norms, default=0.0),
            "max_angular_speed_radps": max(angular_norms, default=0.0),
            "command_zero_transition_monotonic_sec": zero_transitions,
        },
    }
    if steps:
        result["telemetry"]["dt_sec"] = {
            "min": min(steps),
            "max": max(steps),
            "mean": sum(steps) / len(steps),
        }
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description="summarize a teleop JSONL evidence log")
    parser.add_argument("log", type=Path)
    parser.add_argument("--output", type=Path, help="write JSON summary exclusively")
    arguments = parser.parse_args()
    try:
        summary = summarize(load_records(arguments.log))
    except (OSError, ValueError) as error:
        parser.error(str(error))
    encoded = json.dumps(summary, indent=2, ensure_ascii=False, allow_nan=False)
    if arguments.output is None:
        print(encoded)
    else:
        arguments.output.parent.mkdir(parents=True, exist_ok=True)
        with arguments.output.open("x", encoding="utf-8") as stream:
            stream.write(f"{encoded}\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
