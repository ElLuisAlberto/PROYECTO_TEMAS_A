"""Tests for append-only evidence emitted on the mother laptop clock."""

from __future__ import annotations

import json
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from teleop_cooperativo.command_composer import Twist
from teleop_cooperativo.event_log import RunEventLogger, SCHEMA
from teleop_cooperativo.simulation_port import ToolPose


class RunEventLoggerTest(unittest.TestCase):
    def test_writes_ordered_mother_clock_records_and_telemetry(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "logs" / "run-001.jsonl"
            with RunEventLogger(
                path,
                run_id="run-001",
                wall_clock=lambda: 1_700_000_000.125,
                monotonic_clock=lambda: 42.5,
            ) as logger:
                logger.record(
                    "claim_granted",
                    room_state={"state": "ACTIVE", "revision": 2},
                    payload={"axes": ["X", "Y"]},
                )
                logger.record_telemetry(
                    room_state={"state": "ACTIVE", "revision": 3},
                    twist=Twist((0.05, 0.0, 0.0), (0.0, 0.0, 0.175)),
                    pose=ToolPose((0.01, 0.0, 0.0), (0.0, 0.0, 0.1, 0.99)),
                    dt_sec=0.05,
                )

            records = [json.loads(line) for line in path.read_text().splitlines()]
            self.assertEqual([record["event_index"] for record in records], [1, 2])
            self.assertTrue(all(record["schema"] == SCHEMA for record in records))
            self.assertTrue(all(record["run_id"] == "run-001" for record in records))
            self.assertTrue(all(record["monotonic_sec"] == 42.5 for record in records))
            self.assertTrue(records[0]["timestamp_utc"].endswith("Z"))
            self.assertEqual(records[1]["event_type"], "telemetry")
            self.assertEqual(records[1]["payload"]["twist"]["linear"][0], 0.05)
            self.assertEqual(records[1]["payload"]["pose"]["position"][0], 0.01)

    def test_rejects_non_json_values_without_writing_partial_record(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "run.jsonl"
            with RunEventLogger(path, run_id="run") as logger:
                with self.assertRaises(ValueError):
                    logger.record("bad", payload={"value": float("nan")})
                logger.record("healthy")

            records = [json.loads(line) for line in path.read_text().splitlines()]
            self.assertEqual(len(records), 1)
            self.assertEqual(records[0]["event_index"], 1)
            self.assertEqual(records[0]["event_type"], "healthy")

    def test_refuses_to_overwrite_an_existing_run_log(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "run.jsonl"
            first = RunEventLogger(path, run_id="run")
            first.close()
            with self.assertRaises(FileExistsError):
                RunEventLogger(path, run_id="run")


if __name__ == "__main__":
    unittest.main()
