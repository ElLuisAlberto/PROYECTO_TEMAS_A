"""Tests for the standalone JSONL metrics summarizer."""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import unittest


_SCRIPT = Path(__file__).resolve().parents[2] / "scripts" / "summarize_run_log.py"
_SPEC = importlib.util.spec_from_file_location("summarize_run_log", _SCRIPT)
assert _SPEC is not None and _SPEC.loader is not None
summary_script = importlib.util.module_from_spec(_SPEC)
sys.modules[_SPEC.name] = summary_script
_SPEC.loader.exec_module(summary_script)


class RunSummaryTest(unittest.TestCase):
    def test_summarizes_telemetry_and_zero_transition(self) -> None:
        records = [
            _record(1, 1.0, "claim_granted", {"state": "ACTIVE"}, {}),
            _record(
                2,
                1.05,
                "telemetry",
                {"state": "ACTIVE"},
                {"dt_sec": 0.05, "twist": {"linear": [0.05, 0.0, 0.0], "angular": [0.0, 0.0, 0.0]}},
            ),
            _record(
                3,
                1.10,
                "telemetry",
                {"state": "ACTIVE"},
                {"dt_sec": 0.05, "twist": {"linear": [0.0, 0.0, 0.0], "angular": [0.0, 0.0, 0.0]}},
            ),
        ]
        result = summary_script.summarize(records)
        self.assertEqual(result["records"], 3)
        self.assertEqual(result["event_counts"], {"claim_granted": 1, "telemetry": 2})
        self.assertEqual(result["telemetry"]["command_zero_transition_monotonic_sec"], [1.1])
        self.assertAlmostEqual(result["telemetry"]["max_linear_speed_mps"], 0.05)

    def test_rejects_nonconsecutive_records(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "run.jsonl"
            path.write_text(json.dumps(_record(2, 1.0, "event", None, {})) + "\n")
            with self.assertRaises(ValueError):
                summary_script.load_records(path)


def _record(index, monotonic, event_type, room_state, payload):
    return {
        "schema": "teleop_cooperativo.run_event.v1",
        "run_id": "run-001",
        "event_index": index,
        "timestamp_utc": "2026-11-01T00:00:00.000Z",
        "monotonic_sec": monotonic,
        "event_type": event_type,
        "room_state": room_state,
        "payload": payload,
    }


if __name__ == "__main__":
    unittest.main()
