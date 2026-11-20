#!/usr/bin/env python3
"""Exercise H3 through two real local WebSocket clients and save JSON evidence.

Start the gateway first.  Add ``--expect-telemetry`` only when it is started
with ``--local-simulator``; the ROS/Gazebo route deliberately does not yet
publish web telemetry.
"""

from __future__ import annotations

import argparse
import asyncio
from datetime import datetime, timezone
import json
from pathlib import Path
import time
from typing import Any, Callable


class RecordedClient:
    """One JSON WebSocket client with a complete receive-side audit trail."""

    def __init__(self, name: str, socket: Any, records: list[dict[str, object]]) -> None:
        self.name = name
        self.socket = socket
        self.records = records

    async def send(self, message: dict[str, object]) -> None:
        await self.socket.send(json.dumps(message, separators=(",", ":")))
        self._record("sent", message)

    async def expect(
        self,
        predicate: Callable[[dict[str, object]], bool],
        description: str,
        *,
        timeout_sec: float = 2.0,
    ) -> dict[str, object]:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            remaining = deadline - time.monotonic()
            raw = await asyncio.wait_for(self.socket.recv(), timeout=remaining)
            message = json.loads(raw)
            if not isinstance(message, dict):
                raise RuntimeError(f"{self.name} received non-object frame")
            self._record("received", message)
            if predicate(message):
                return message
        raise RuntimeError(f"timeout waiting for {description} on {self.name}")

    def _record(self, direction: str, message: dict[str, object]) -> None:
        evidence_message = json.loads(json.dumps(message))
        if "session_id" in evidence_message:
            evidence_message["session_id"] = "<redacted>"
        leases = evidence_message.get("leases")
        if isinstance(leases, list):
            for lease in leases:
                if isinstance(lease, dict) and "lease_id" in lease:
                    lease["lease_id"] = "<redacted>"
        elif isinstance(leases, dict):
            for lease in leases.values():
                if isinstance(lease, dict) and "lease_id" in lease:
                    lease["lease_id"] = "<redacted>"
        self.records.append(
            {
                "monotonic_sec": time.monotonic(),
                "client": self.name,
                "direction": direction,
                "message": evidence_message,
            }
        )


def _leases(grant: dict[str, object]) -> dict[str, dict[str, object]]:
    raw = grant.get("leases")
    if not isinstance(raw, list):
        raise RuntimeError("claim_granted lacks leases")
    result: dict[str, dict[str, object]] = {}
    for lease in raw:
        if not isinstance(lease, dict) or not isinstance(lease.get("axis"), str):
            raise RuntimeError("claim_granted has malformed lease")
        result[lease["axis"]] = {
            "lease_id": lease["lease_id"],
            "generation": lease["generation"],
        }
    return result


async def run(url: str, expect_telemetry: bool) -> dict[str, object]:
    try:
        import websockets
    except ImportError as error:
        raise RuntimeError("install requirements.txt before running this script") from error

    records: list[dict[str, object]] = []
    started = time.monotonic()
    async with websockets.connect(url) as socket_a, websockets.connect(url) as socket_b:
        a = RecordedClient("operator-a", socket_a, records)
        b = RecordedClient("operator-b", socket_b, records)
        await a.send({"type": "hello", "operator_name": "operator-a", "room_id": "default"})
        await a.expect(lambda message: message.get("type") == "welcome", "welcome A")
        await b.send({"type": "hello", "operator_name": "operator-b", "room_id": "default"})
        await b.expect(lambda message: message.get("type") == "welcome", "welcome B")

        await a.send({"type": "claim", "axes": ["X", "Y", "Z"]})
        grant_a = await a.expect(lambda message: message.get("type") == "claim_granted", "claim A")
        tokens_a = _leases(grant_a)
        await b.send({"type": "claim", "axes": ["ROLL", "PITCH", "YAW"]})
        grant_b = await b.expect(lambda message: message.get("type") == "claim_granted", "claim B")
        _leases(grant_b)

        await b.send({"type": "claim", "axes": ["X"]})
        denied = await b.expect(
            lambda message: message.get("type") == "error" and message.get("code") == "claim_denied",
            "atomic conflicting claim",
        )
        if denied.get("code") != "claim_denied":
            raise RuntimeError("conflicting claim was not denied")

        await a.send(
            {
                "type": "input",
                "sequence": 1,
                "values": {"X": 1.0},
                "leases": {"X": tokens_a["X"]},
            }
        )
        await a.expect(
            lambda message: message.get("type") == "input_accepted",
            "input acceptance",
        )
        if expect_telemetry:
            await a.expect(
                lambda message: message.get("type") == "telemetry"
                and message.get("twist", {}).get("linear", [0.0])[0] > 0.0,
                "non-zero telemetry",
            )
            await a.expect(
                lambda message: message.get("type") == "telemetry"
                and message.get("twist", {}).get("linear") == [0.0, 0.0, 0.0],
                "zero telemetry after input timeout",
                timeout_sec=1.0,
            )
        else:
            await asyncio.sleep(0.18)

        await b.send({"type": "heartbeat"})
        await b.expect(lambda message: message.get("type") == "heartbeat_ack", "heartbeat B")
        await socket_a.close()
        released = await b.expect(
            lambda message: message.get("type") == "room_state"
            and not message["axes"]["X"]["assigned"]
            and message["axes"]["ROLL"]["assigned"],
            "A closure releases only A axes",
        )
        await b.send({"type": "release"})
        await b.expect(lambda message: message.get("type") == "release_confirmed", "release B")
        ready = await b.expect(
            lambda message: message.get("type") == "room_state" and message.get("state") == "READY",
            "room ready after final release",
        )

    return {
        "schema": "teleop_cooperativo.h3_local_two_client.v1",
        "timestamp_utc": datetime.now(timezone.utc).isoformat(timespec="milliseconds").replace("+00:00", "Z"),
        "url": url,
        "expect_telemetry": expect_telemetry,
        "duration_sec": time.monotonic() - started,
        "result": "PASS",
        "final_room_state": ready,
        "records": records,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="Two-client local H3 WebSocket evidence run")
    parser.add_argument("--url", default="ws://127.0.0.1:8765/ws")
    parser.add_argument("--output", required=True, help="new JSON evidence file")
    parser.add_argument("--expect-telemetry", action="store_true")
    arguments = parser.parse_args()
    output = Path(arguments.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    if output.exists():
        parser.error(f"output already exists: {output}")
    try:
        result = asyncio.run(run(arguments.url, arguments.expect_telemetry))
    except Exception as error:
        result = {
            "schema": "teleop_cooperativo.h3_local_two_client.v1",
            "timestamp_utc": datetime.now(timezone.utc).isoformat(timespec="milliseconds").replace("+00:00", "Z"),
            "url": arguments.url,
            "expect_telemetry": arguments.expect_telemetry,
            "result": "FAIL",
            "error": str(error),
        }
        output.open("x", encoding="utf-8").write(json.dumps(result, indent=2) + "\n")
        print(json.dumps(result, indent=2))
        return 1
    output.open("x", encoding="utf-8").write(json.dumps(result, indent=2) + "\n")
    print(json.dumps({key: value for key, value in result.items() if key != "records"}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
