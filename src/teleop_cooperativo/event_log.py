"""Append-only, mother-clock event records for reproducible local runs."""

from __future__ import annotations

from datetime import datetime, timezone
import json
import math
from pathlib import Path
import time
from typing import Callable, Mapping, TextIO

from .command_composer import Twist
from .simulation_port import ToolPose


SCHEMA = "teleop_cooperativo.run_event.v1"


class RunEventLogger:
    """Write one self-contained JSON object per event without client clocks.

    A path is created exclusively, so a run cannot silently overwrite another
    run's evidence.  Both UTC wall time and monotonic time originate on the
    mother laptop; browser timestamps are deliberately not accepted here.
    """

    def __init__(
        self,
        path: str | Path,
        *,
        run_id: str,
        wall_clock: Callable[[], float] = time.time,
        monotonic_clock: Callable[[], float] = time.monotonic,
    ) -> None:
        if not isinstance(run_id, str) or not run_id.strip():
            raise ValueError("run_id must be a non-empty string")
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._stream: TextIO = self.path.open("x", encoding="utf-8")
        self.run_id = run_id
        self._wall_clock = wall_clock
        self._monotonic_clock = monotonic_clock
        self._event_index = 0

    def __enter__(self) -> "RunEventLogger":
        return self

    def __exit__(self, *_unused: object) -> None:
        self.close()

    def close(self) -> None:
        """Close the log; calling it more than once is harmless."""
        if not self._stream.closed:
            self._stream.close()

    def record(
        self,
        event_type: str,
        *,
        room_state: Mapping[str, object] | None = None,
        payload: Mapping[str, object] | None = None,
        monotonic_sec: float | None = None,
    ) -> dict[str, object]:
        """Persist an event and return the exact record written to disk."""
        if self._stream.closed:
            raise RuntimeError("event logger is closed")
        if not isinstance(event_type, str) or not event_type.strip():
            raise ValueError("event_type must be a non-empty string")
        timestamp = self._finite_time(self._wall_clock(), "wall clock")
        monotonic = self._finite_time(
            self._monotonic_clock() if monotonic_sec is None else monotonic_sec,
            "monotonic_sec",
        )
        self._event_index += 1
        record: dict[str, object] = {
            "schema": SCHEMA,
            "run_id": self.run_id,
            "event_index": self._event_index,
            "timestamp_utc": self._utc(timestamp),
            "monotonic_sec": monotonic,
            "event_type": event_type,
            "room_state": dict(room_state) if room_state is not None else None,
            "payload": dict(payload) if payload is not None else {},
        }
        try:
            serialized = json.dumps(
                record, ensure_ascii=False, separators=(",", ":"), allow_nan=False
            )
        except (TypeError, ValueError) as error:
            self._event_index -= 1
            raise ValueError("event data must be finite JSON-compatible values") from error
        self._stream.write(f"{serialized}\n")
        self._stream.flush()
        return record

    def record_telemetry(
        self,
        *,
        room_state: Mapping[str, object],
        twist: Twist,
        pose: ToolPose,
        dt_sec: float,
        monotonic_sec: float | None = None,
    ) -> dict[str, object]:
        """Record one simulator-facing command and resulting deterministic pose."""
        duration = self._finite_time(dt_sec, "dt_sec")
        return self.record(
            "telemetry",
            room_state=room_state,
            monotonic_sec=monotonic_sec,
            payload={
                "dt_sec": duration,
                "twist": {
                    "linear": list(twist.linear),
                    "angular": list(twist.angular),
                },
                "pose": {
                    "position": list(pose.position),
                    "quaternion_xyzw": list(pose.quaternion_xyzw),
                },
            },
        )

    @staticmethod
    def _finite_time(value: object, name: str) -> float:
        if isinstance(value, bool):
            raise ValueError(f"{name} must be finite")
        try:
            numeric = float(value)
        except (TypeError, ValueError) as error:
            raise ValueError(f"{name} must be finite") from error
        if not math.isfinite(numeric):
            raise ValueError(f"{name} must be finite")
        return numeric

    @staticmethod
    def _utc(timestamp: float) -> str:
        return (
            datetime.fromtimestamp(timestamp, tz=timezone.utc)
            .isoformat(timespec="milliseconds")
            .replace("+00:00", "Z")
        )
