"""Authoritative WebSocket gateway for cooperative Cartesian control.

The protocol controller has no network dependency; the optional adapter at
the end uses ``websockets``.  Thus all authorization rules remain testable
without ROS, Gazebo, or a browser.
"""

from __future__ import annotations

import argparse
import asyncio
from collections import deque
from dataclasses import dataclass
import hmac
import hashlib
import ipaddress
import json
import logging
import math
from pathlib import Path
import time
from typing import Any, Callable, Mapping
from uuid import uuid4

from .arbiter import ArbiterTick, CooperativeArbiter, RoomState
from .domain import AXES, Axis
from .event_log import RunEventLogger
from .command_composer import Twist
from .lease_manager import ClaimDenied, InputRejected, Lease
from .local_simulation_runtime import (
    LocalSimulationRuntime,
    LocalSimulationTick,
    RunnerFault,
)
from .simulation_port import TelemetrySample


PROTOCOL_VERSION = 1
MAX_MESSAGE_BYTES = 16 * 1024


@dataclass(frozen=True)
class GatewayTick:
    """A periodic expiry pass and the public snapshot to broadcast."""

    arbiter_tick: ArbiterTick
    room_state: dict[str, object]
    changed: bool
    telemetry: dict[str, object] | None = None


@dataclass
class _Session:
    session_id: str
    operator_name: str
    role: str = "operator"
    credential_fingerprint: str | None = None


def load_credentials(path: str) -> dict[str, tuple[str, ...]]:
    """Read per-session bearer secrets from an owner-only JSON file."""
    credential_path = Path(path)
    if credential_path.stat().st_mode & 0o077:
        raise ValueError("credential file must be accessible only to its owner (mode 0600)")
    parsed = json.loads(credential_path.read_text(encoding="utf-8"))
    if not isinstance(parsed, dict):
        raise ValueError("credential file must be a JSON object")
    result = {}
    all_tokens = []
    for role in ("operators", "observers", "admins"):
        values = parsed.get(role, [])
        if not isinstance(values, list) or any(not isinstance(v, str) or not v.isascii() or len(v) < 32 for v in values):
            raise ValueError(f"{role} must be an array of tokens of at least 32 characters")
        result[role] = tuple(values)
        all_tokens.extend(values)
    if not result["operators"] or not result["observers"] or not result["admins"]:
        raise ValueError("at least one credential is required for each role")
    if len(set(all_tokens)) != len(all_tokens):
        raise ValueError("credentials must be unique")
    return result


class CooperativeGateway:
    """JSON protocol boundary; the arbiter remains the only control authority."""

    def __init__(
        self,
        arbiter: CooperativeArbiter | None = None,
        *,
        room_id: str = "default",
        heartbeat_interval_ms: int = 100,
        local_runtime: LocalSimulationRuntime | None = None,
        event_logger: RunEventLogger | None = None,
        twist_sink: Callable[[Twist], None] | None = None,
        readiness_probe: Callable[[], bool] | None = None,
        telemetry_source: Callable[[], TelemetrySample | None] | None = None,
        credentials: Mapping[str, tuple[str, ...]] | None = None,
        anonymous_lan: bool = False,
    ) -> None:
        if not isinstance(room_id, str) or not room_id.strip():
            raise ValueError("room_id must be a non-empty string")
        if (
            isinstance(heartbeat_interval_ms, bool)
            or not isinstance(heartbeat_interval_ms, int)
            or heartbeat_interval_ms <= 0
        ):
            raise ValueError("heartbeat_interval_ms must be a positive integer")
        self.arbiter = arbiter if arbiter is not None else CooperativeArbiter()
        if local_runtime is not None and local_runtime.arbiter is not self.arbiter:
            raise ValueError("local_runtime must use the gateway arbiter")
        self.room_id = room_id
        self.heartbeat_interval_ms = heartbeat_interval_ms
        self.local_runtime = local_runtime
        self.event_logger = event_logger
        if twist_sink is not None and not callable(twist_sink):
            raise ValueError("twist_sink must be callable")
        if readiness_probe is not None and not callable(readiness_probe):
            raise ValueError("readiness_probe must be callable")
        if telemetry_source is not None and not callable(telemetry_source):
            raise ValueError("telemetry_source must be callable")
        self.twist_sink = twist_sink
        self.readiness_probe = readiness_probe
        self.telemetry_source = telemetry_source
        self.credentials = credentials
        self.anonymous_lan = anonymous_lan
        self._connections: dict[str, str | None] = {}
        self._remote_control_allowed: dict[str, bool] = {}
        self._sessions: dict[str, _Session] = {}
        self._revision = 0

    def open_connection(self, connection_id: str, *, remote_control_allowed: bool = True) -> None:
        """Register a transport connection.  It has no session until hello."""
        if not isinstance(connection_id, str) or not connection_id:
            raise ValueError("connection_id must be a non-empty string")
        if connection_id in self._connections:
            raise ValueError("connection is already open")
        self._connections[connection_id] = None
        self._remote_control_allowed[connection_id] = remote_control_allowed

    def close_connection(self, connection_id: str) -> bool:
        """Release a browser's axes immediately when its transport closes."""
        session_id = self._connections.pop(connection_id, None)
        self._remote_control_allowed.pop(connection_id, None)
        if session_id is None:
            return False
        self._sessions.pop(session_id, None)
        self._bump_revision()
        try:
            released = self.arbiter.release(session_id)
        except RuntimeError:
            # A FAULT already discarded all authority.
            released = ()
        return True

    def mark_ready(self) -> None:
        """Host-only readiness transition after its simulation/bridge check."""
        if self.readiness_probe is not None and not self.readiness_probe():
            raise RuntimeError("ROS bridge is not ready")
        if self.readiness_probe is not None and self.telemetry_source is not None and self._fresh_telemetry() is None:
            raise RuntimeError("fresh telemetry is required")
        previous = self.arbiter.state
        if self.local_runtime is None:
            self.arbiter.mark_ready()
        else:
            self.local_runtime.start()
        if self.arbiter.state != previous:
            self._bump_revision()

    def fault(self) -> tuple[Axis, ...]:
        """Host-only fail-safe: zero/revoke every axis and block new claims."""
        released = self.arbiter.fault()
        self._bump_revision()
        return released

    def receive(
        self, connection_id: str, raw_message: str | bytes | Mapping[str, object]
    ) -> list[dict[str, object]]:
        """Validate and process one message from exactly one open connection."""
        if connection_id not in self._connections:
            raise ValueError("connection is not open")
        try:
            message = self._decode(raw_message)
        except ValueError as error:
            return self._finish_protocol("invalid", [self._error("invalid_message", str(error))])

        message_type = message.get("type")
        if not isinstance(message_type, str):
            return self._finish_protocol("invalid", [self._error("invalid_message", "type must be a string")])
        if message_type == "hello":
            return self._finish_protocol(message_type, self._hello(connection_id, message))

        session_id = self._connections[connection_id]
        if session_id is None:
            return self._finish_protocol(message_type, [self._error("hello_required", "send hello before requesting control")])
        role = self._sessions[session_id].role
        if message_type in {"claim", "input", "release"} and role != "operator":
            return self._finish_protocol(message_type, [self._error("forbidden", "operator role required")])
        if message_type in {"stop_global", "rearm"} and role != "admin":
            return self._finish_protocol(message_type, [self._error("forbidden", "admin role required")])
        try:
            if message_type == "stop_global":
                self.fault()
                return self._finish_protocol(message_type, [{"type": "stop_confirmed"}, self.room_state()])
            if message_type == "rearm":
                if self.arbiter.state is not RoomState.FAULT:
                    raise RuntimeError("room is not in FAULT")
                if self.telemetry_source is not None and self._fresh_telemetry() is None:
                    raise RuntimeError("fresh telemetry is required before rearming")
                self.mark_ready()
                return self._finish_protocol(message_type, [{"type": "rearm_confirmed"}, self.room_state()])
            if message_type == "claim":
                axes = self._axes(message)
                leases = self.arbiter.claim(session_id, axes)
                self._bump_revision()
                return self._finish_protocol(message_type, [
                    {
                        "type": "claim_granted",
                        "revision": self._revision,
                        "leases": [self._lease_payload(lease) for lease in leases],
                    },
                    self.room_state(),
                ])
            if message_type == "input":
                sequence = self._integer(message.get("sequence"), "sequence")
                values = self._mapping(message.get("values"), "values")
                tokens = self._tokens(message.get("leases"))
                self.arbiter.submit_input(session_id, sequence, values, tokens)
                return self._finish_protocol(message_type, [{"type": "input_accepted", "sequence": sequence}])
            if message_type == "heartbeat":
                self.arbiter.heartbeat(session_id)
                return self._finish_protocol(message_type, [{"type": "heartbeat_ack"}])
            if message_type == "release":
                axes = self._axes(message) if "axes" in message else None
                released = self.arbiter.release(session_id, axes)
                if released:
                    self._bump_revision()
                return self._finish_protocol(message_type, [
                    {"type": "release_confirmed", "axes": [a.value for a in released]},
                    self.room_state(),
                ])
            return self._finish_protocol(message_type, [self._error("unknown_type", f"unsupported message type: {message_type}")])
        except ClaimDenied as error:
            return self._finish_protocol(message_type, [self._error("claim_denied", str(error))])
        except InputRejected as error:
            return self._finish_protocol(message_type, [self._error("input_rejected", str(error))])
        except (TypeError, ValueError) as error:
            return self._finish_protocol(message_type, [self._error("validation_error", str(error))])
        except RuntimeError as error:
            return self._finish_protocol(message_type, [self._error("room_not_operational", str(error))])

    def maintenance_tick(self) -> GatewayTick:
        """Apply server-clock expirations.  Call this at the 20 Hz host tick."""
        before = self.arbiter.state
        telemetry: dict[str, object] | None = None
        runtime_tick: LocalSimulationTick | None = None
        simulator_advanced = False
        sample = self._fresh_telemetry() if self.telemetry_source is not None else None
        if self.readiness_probe is not None and (not self.readiness_probe() or (self.telemetry_source is not None and sample is None)):
            if self.arbiter.state is not RoomState.FAULT:
                self.fault()
            arbiter_tick = self.arbiter.tick()
        elif self.local_runtime is None:
            arbiter_tick = self.arbiter.tick()
        else:
            try:
                runtime_tick = self.local_runtime.tick()
                simulator_advanced = True
            except RunnerFault:
                runtime_tick = self.local_runtime.safe_telemetry()
            arbiter_tick = runtime_tick.arbiter_tick
            telemetry = self._telemetry_payload(runtime_tick)
        if self.twist_sink is not None:
            try:
                self.twist_sink(arbiter_tick.twist)
            except Exception:
                self.fault()
                arbiter_tick = self.arbiter.tick()
        changed = bool(
            arbiter_tick.maintenance.zeroed_axes
            or arbiter_tick.maintenance.released_axes
            or arbiter_tick.state != before
        )
        if changed:
            self._bump_revision()
        room_state = self.room_state()
        if (
            simulator_advanced
            and runtime_tick is not None
            and self.event_logger is not None
        ):
            self.event_logger.record_telemetry(
                room_state=room_state,
                twist=runtime_tick.twist,
                pose=runtime_tick.pose,
                dt_sec=runtime_tick.dt_sec,
            )
        elif self.telemetry_source is not None:
            # ROS/Gazebo telemetry is part of the live protocol, independently
            # of whether this particular run also persists an evidence log.
            # Keeping those concerns separate avoids a silent no-telemetry UI
            # during the normal base-station launch (which has no event log).
            telemetry = self._external_telemetry_payload(arbiter_tick, sample)
            if self.event_logger is not None:
                payload = {
                    key: value
                    for key, value in telemetry.items()
                    if key not in {"type", "state"}
                }
                self.event_logger.record(
                    "telemetry", room_state=room_state, payload=payload
                )
        return GatewayTick(arbiter_tick, room_state, changed, telemetry)

    def _fresh_telemetry(self) -> TelemetrySample | None:
        if self.telemetry_source is None:
            return None
        try:
            sample = self.telemetry_source()
            if sample is None or not math.isfinite(sample.age_sec) or not 0 <= sample.age_sec <= 0.30:
                return None
            return sample
        except Exception:
            return None

    @staticmethod
    def _telemetry_payload(runtime_tick: LocalSimulationTick) -> dict[str, object]:
        """Serialize local-only pose and command observations for H3 clients."""
        return {
            "type": "telemetry",
            "state": runtime_tick.arbiter_tick.state.value,
            "dt_sec": runtime_tick.dt_sec,
            "twist": {
                "linear": list(runtime_tick.twist.linear),
                "angular": list(runtime_tick.twist.angular),
            },
            "pose": {
                "position": list(runtime_tick.pose.position),
                "quaternion_xyzw": list(runtime_tick.pose.quaternion_xyzw),
            },
        }

    @staticmethod
    def _external_telemetry_payload(
        arbiter_tick: ArbiterTick, sample: TelemetrySample | None
    ) -> dict[str, object]:
        """Serialize an optional ROS/Gazebo observation for every UI client."""
        payload: dict[str, object] = {
            "type": "telemetry",
            "state": arbiter_tick.state.value,
            "telemetry_available": sample is not None,
            "twist": {
                "linear": list(arbiter_tick.twist.linear),
                "angular": list(arbiter_tick.twist.angular),
            },
        }
        if sample is not None:
            payload.update(
                {
                    "source": sample.source,
                    "pose": {
                        "position": list(sample.pose.position),
                        "quaternion_xyzw": list(sample.pose.quaternion_xyzw),
                    },
                    "joint_positions": list(sample.joint_positions),
                    "pose_age_sec": sample.age_sec,
                }
            )
        return payload

    def room_state(self) -> dict[str, object]:
        """Public state: owners are visible, lease tokens are not."""
        axes: dict[str, dict[str, object]] = {}
        for axis in AXES:
            lease = self.arbiter.leases.lease(axis)
            session = self._sessions.get(lease.session_id) if lease else None
            axes[axis.value] = {
                "assigned": lease is not None,
                "owner": session.operator_name if session else None,
                "generation": lease.generation if lease else None,
            }
        return {
            "type": "room_state",
            "protocol_version": PROTOCOL_VERSION,
            "room_id": self.room_id,
            "state": self.arbiter.state.value,
            "revision": self._revision,
            "axes": axes,
            "modes": {
                "operator": (
                    self.arbiter.state in (RoomState.READY, RoomState.ACTIVE)
                    and sum(s.role == "operator" for s in self._sessions.values()) < 2
                    and any(not axis["assigned"] for axis in axes.values())
                ),
                "observer": True,
                "admin": not any(s.role == "admin" for s in self._sessions.values()),
            },
        }

    def _hello(
        self, connection_id: str, message: Mapping[str, object]
    ) -> list[dict[str, object]]:
        if self._connections[connection_id] is not None:
            return [self._error("already_initialized", "hello has already been accepted")]
        room_id = message.get("room_id")
        if room_id != self.room_id:
            return [self._error("room_not_found", "requested room is unavailable")]
        role = message.get("role", "operator")
        if role not in ("operator", "observer", "admin"):
            return [self._error("authentication_failed", "invalid credentials or role")]
        if self.credentials is not None:
            token = message.get("credential")
            available = self.credentials[{"operator": "operators", "observer": "observers", "admin": "admins"}[role]]
            valid = isinstance(token, str) and token.isascii() and any(hmac.compare_digest(token, item) for item in available)
            anonymous = self.anonymous_lan and not token and self._remote_control_allowed[connection_id]
            if (not valid and not anonymous) or (role != "observer" and not self._remote_control_allowed[connection_id]):
                return [self._error("authentication_failed", "invalid credentials, role or client origin")]
            fingerprint = hashlib.sha256(token.encode("ascii")).hexdigest() if valid else None
            if fingerprint and any(session.credential_fingerprint == fingerprint for session in self._sessions.values()):
                return [self._error("authentication_failed", "credential is already in use")]
        else:
            fingerprint = None
        if self.anonymous_lan and (
            (role == "operator" and not self.room_state()["modes"]["operator"])
            or (role == "admin" and not self.room_state()["modes"]["admin"])
        ):
            return [self._error("mode_unavailable", "El modo no está disponible")]
        raw_name = message.get("operator_name")
        if not isinstance(raw_name, str):
            return [self._error("validation_error", "operator_name must be a string")]
        operator_name = "".join(c for c in raw_name.strip() if c.isprintable())
        if not 1 <= len(operator_name) <= 32:
            return [
                self._error(
                    "validation_error", "operator_name must contain 1 to 32 characters"
                )
            ]
        session_id = str(uuid4())
        self._connections[connection_id] = session_id
        self._sessions[session_id] = _Session(session_id, operator_name, role, fingerprint)
        self._bump_revision()
        return [
            {
                "type": "welcome",
                "protocol_version": PROTOCOL_VERSION,
                "room_id": self.room_id,
                "session_id": session_id,
                "operator_name": operator_name,
                "role": role,
                "state": self.arbiter.state.value,
                "heartbeat_interval_ms": self.heartbeat_interval_ms,
            },
            self.room_state(),
        ]

    @staticmethod
    def _decode(raw_message: str | bytes | Mapping[str, object]) -> Mapping[str, object]:
        if isinstance(raw_message, Mapping):
            return raw_message
        if isinstance(raw_message, bytes):
            raise ValueError("binary messages are not accepted")
        if not isinstance(raw_message, str):
            raise ValueError("message must be a JSON object")
        if len(raw_message.encode("utf-8")) > MAX_MESSAGE_BYTES:
            raise ValueError("message exceeds maximum size")
        try:
            message = json.loads(raw_message)
        except json.JSONDecodeError as error:
            raise ValueError("message is not valid JSON") from error
        if not isinstance(message, Mapping):
            raise ValueError("message must be a JSON object")
        return message

    @staticmethod
    def _mapping(value: object, field: str) -> Mapping[str, object]:
        if not isinstance(value, Mapping):
            raise ValueError(f"{field} must be an object")
        return value

    @staticmethod
    def _integer(value: object, field: str) -> int:
        if isinstance(value, bool) or not isinstance(value, int):
            raise ValueError(f"{field} must be an integer")
        return value

    @staticmethod
    def _axes(message: Mapping[str, object]) -> list[object]:
        axes = message.get("axes")
        if not isinstance(axes, list):
            raise ValueError("axes must be an array")
        return axes

    @classmethod
    def _tokens(cls, value: object) -> dict[str, tuple[str, int]]:
        tokens: dict[str, tuple[str, int]] = {}
        for raw_axis, raw_token in cls._mapping(value, "leases").items():
            if not isinstance(raw_axis, str):
                raise ValueError("lease axis must be a string")
            token = cls._mapping(raw_token, f"lease for {raw_axis}")
            lease_id = token.get("lease_id")
            if not isinstance(lease_id, str) or not lease_id:
                raise ValueError(f"lease_id for {raw_axis} must be a string")
            tokens[raw_axis] = (
                lease_id,
                cls._integer(token.get("generation"), f"generation for {raw_axis}"),
            )
        return tokens

    @staticmethod
    def _lease_payload(lease: Lease) -> dict[str, object]:
        return {
            "axis": lease.axis.value,
            "lease_id": lease.lease_id,
            "generation": lease.generation,
        }

    @staticmethod
    def _error(code: str, detail: str) -> dict[str, object]:
        return {"type": "error", "code": code, "detail": detail}

    def _finish_protocol(
        self, message_type: str, replies: list[dict[str, object]]
    ) -> list[dict[str, object]]:
        """Record outcomes without retaining connection, lease, or session secrets."""
        if self.event_logger is not None:
            first = replies[0] if replies else {}
            payload: dict[str, object] = {
                "message_type": message_type,
                "outcome": first.get("type", "none"),
            }
            if first.get("type") == "error":
                payload["error_code"] = first.get("code", "unknown")
            self.event_logger.record(
                "protocol", room_state=self.room_state(), payload=payload
            )
        return replies

    def _bump_revision(self) -> None:
        self._revision += 1


async def serve_websocket(
    gateway: CooperativeGateway,
    *,
    host: str = "0.0.0.0",
    port: int = 8765,
    tick_interval_sec: float = 0.02,
    allowed_origins: tuple[str, ...] | None = None,
    trusted_proxy: str | None = None,
    base_local_ips: tuple[str, ...] = (),
    allow_local_web: bool = False,
) -> None:
    """Run ``ws://host:port/ws`` (and reject all other WebSocket paths)."""
    try:
        import websockets
        from websockets.exceptions import ConnectionClosed
    except ImportError as error:  # pragma: no cover - deployment dependent
        raise RuntimeError(
            "WebSocket runtime requires websockets>=12,<16; install requirements.txt"
        ) from error
    if tick_interval_sec <= 0:
        raise ValueError("tick_interval_sec must be positive")

    clients: dict[Any, asyncio.Queue[str]] = {}
    connection_count = 0

    def broadcast(message: Mapping[str, object]) -> None:
        payload = json.dumps(message, separators=(",", ":"), allow_nan=False)
        for socket, queue in tuple(clients.items()):
            try:
                queue.put_nowait(payload)
            except asyncio.QueueFull:
                # Never let a slow receiver stall the authoritative tick.
                clients.pop(socket, None)
                asyncio.create_task(socket.close(code=1013, reason="client too slow"))

    async def sender(socket: Any, queue: asyncio.Queue[str]) -> None:
        while True:
            await socket.send(await queue.get())

    async def handler(socket: Any, path: str | None = None) -> None:
        nonlocal connection_count
        # websockets <= 13 passes path separately; newer releases expose it on
        # the request object.  Supporting both keeps the declared range usable.
        request_path = path or getattr(getattr(socket, "request", None), "path", "/")
        if request_path != "/ws":
            await socket.close(code=1008, reason="use /ws")
            return
        if connection_count >= 32:
            await socket.close(code=1013, reason="connection limit")
            return
        connection_count += 1
        headers = getattr(getattr(socket, "request", None), "headers", None)
        if headers is None:
            headers = getattr(socket, "request_headers", {})
        peer = socket.remote_address[0] if socket.remote_address else ""
        origin = headers.get("Origin")
        forwarded = headers.get("X-Forwarded-For")
        remote_control_allowed = gateway.credentials is None
        if gateway.credentials is not None and trusted_proxy and peer == trusted_proxy and origin in (allowed_origins or ()):
            try:
                address = ipaddress.ip_address(forwarded)
                remote_control_allowed = allow_local_web or (not address.is_loopback and str(address) not in base_local_ips)
            except ValueError:
                remote_control_allowed = False
        connection_id = str(uuid4())
        gateway.open_connection(connection_id, remote_control_allowed=remote_control_allowed)
        queue: asyncio.Queue[str] = asyncio.Queue(maxsize=16)
        writer = asyncio.create_task(sender(socket, queue))
        message_times: deque[float] = deque()
        try:
            while True:
                try:
                    session_id = gateway._connections[connection_id]
                    # Release scarce operator/admin modes promptly when a
                    # client disappears without a clean TCP close. Leases
                    # still expire independently on their shorter watchdog.
                    idle_timeout = 5.0 if session_id is None else (
                        2.0 if gateway._sessions[session_id].role in ("operator", "admin") else 30.0
                    )
                    raw_message = await asyncio.wait_for(socket.recv(), timeout=idle_timeout)
                except asyncio.TimeoutError:
                    await socket.close(code=1008, reason="session timeout")
                    break
                except ConnectionClosed:
                    # Normal and abrupt disconnects share the cleanup below;
                    # neither is an unexpected server handler failure.
                    break
                now = time.monotonic()
                message_times.append(now)
                while message_times and now - message_times[0] > 1.0:
                    message_times.popleft()
                if len(message_times) > 200:
                    await socket.close(code=1008, reason="message rate exceeded")
                    break
                replies = gateway.receive(connection_id, raw_message)
                for reply in replies:
                    await socket.send(json.dumps(reply, separators=(",", ":")))
                if replies and replies[0].get("type") == "welcome":
                    clients[socket] = queue
                elif replies and replies[0].get("code") == "authentication_failed":
                    await socket.close(code=1008, reason="authentication failed")
                    break
                if not (len(replies) == 1 and replies[0].get("type") == "error"):
                    broadcast(gateway.room_state())
        finally:
            clients.pop(socket, None)
            writer.cancel()
            connection_count -= 1
            if gateway.close_connection(connection_id):
                broadcast(gateway.room_state())

    async def maintenance() -> None:
        while True:
            await asyncio.sleep(tick_interval_sec)
            tick = gateway.maintenance_tick()
            if tick.changed:
                broadcast(tick.room_state)
            if tick.telemetry is not None:
                broadcast(tick.telemetry)

    origins = [*allowed_origins, None] if allowed_origins is not None else None
    async with websockets.serve(handler, host, port, max_size=MAX_MESSAGE_BYTES, origins=origins):
        await maintenance()


def build_argument_parser() -> argparse.ArgumentParser:
    """Build the CLI parser separately so model-selection remains unit-testable."""
    parser = argparse.ArgumentParser(description="Cooperative teleoperation gateway")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--tick-ms", type=float, default=20.0)
    parser.add_argument("--credentials-file", help="owner-only JSON containing operator, observer and admin credentials")
    parser.add_argument("--allowed-origin", action="append", default=[], help="exact allowed HTTPS origin; may be repeated")
    parser.add_argument("--trusted-proxy", help="loopback address of the TLS proxy; it must overwrite X-Forwarded-For")
    parser.add_argument("--base-local-ip", action="append", default=[], help="IP assigned to this base, to exclude local operator/admin clients; repeat for each address")
    parser.add_argument("--insecure-development", action="store_true", help="disable authentication for local development only")
    parser.add_argument("--anonymous-lan", action="store_true", help="allow remote LAN clients to choose a role without credentials (trusted LAN only)")
    parser.add_argument("--allow-local-web", action="store_true", help="permit the base computer to use the web UI through the trusted proxy for simulation tests")
    parser.add_argument(
        "--room-id",
        default="default",
        help="single room accepted by this gateway (default: default)",
    )
    parser.add_argument(
        "--local-simulator",
        action="store_true",
        help="run the deterministic Cartesian simulator behind the gateway",
    )
    parser.add_argument(
        "--event-log",
        help="new JSONL evidence file; requires --run-id",
    )
    parser.add_argument(
        "--run-id",
        help="identifier recorded in --event-log",
    )
    parser.add_argument(
        "--ros-twist-topic",
        help="publish the sole accepted twist to this ROS 2 topic",
    )
    parser.add_argument(
        "--robot-model",
        choices=("jaco2", "generic"),
        default="generic",
        help=(
            "kinematic model for ROS telemetry; jaco2 is required for the "
            "J2N6S300 simulation, while generic is retained only for tests"
        ),
    )
    parser.add_argument(
        "--ros-ready-timeout-sec",
        type=float,
        default=30.0,
        help="maximum bridge-readiness wait before the gateway refuses to start",
    )
    return parser


def main() -> int:
    """Start the standalone H3 gateway after explicitly marking it ready."""
    parser = build_argument_parser()
    arguments = parser.parse_args()
    if not 1 <= arguments.port <= 65535:
        parser.error("--port must be between 1 and 65535")
    if arguments.tick_ms <= 0:
        parser.error("--tick-ms must be positive")
    if arguments.ros_ready_timeout_sec <= 0:
        parser.error("--ros-ready-timeout-sec must be positive")
    if not arguments.room_id.strip():
        parser.error("--room-id must not be empty")
    if arguments.event_log and not arguments.run_id:
        parser.error("--event-log requires --run-id")
    if arguments.local_simulator and arguments.robot_model != "generic":
        parser.error("--local-simulator is available only with --robot-model generic")
    if arguments.robot_model == "jaco2" and not arguments.ros_twist_topic:
        parser.error("--robot-model jaco2 requires --ros-twist-topic")
    if arguments.insecure_development:
        if arguments.host not in ("127.0.0.1", "::1", "localhost") or arguments.ros_twist_topic:
            parser.error("--insecure-development requires loopback host and cannot connect to ROS")
        if arguments.credentials_file or arguments.allowed_origin or arguments.trusted_proxy or arguments.base_local_ip or arguments.anonymous_lan:
            parser.error("--insecure-development cannot be combined with security options")
        credentials = None
    else:
        if arguments.host not in ("127.0.0.1", "::1", "localhost"):
            parser.error("secure gateway must bind loopback behind the TLS proxy")
        if not arguments.credentials_file or not arguments.allowed_origin or arguments.trusted_proxy not in ("127.0.0.1", "::1") or not arguments.base_local_ip:
            parser.error("secure mode requires --credentials-file, --allowed-origin, --base-local-ip and loopback --trusted-proxy")
        try:
            arguments.base_local_ip = [str(ipaddress.ip_address(address)) for address in arguments.base_local_ip]
        except ValueError:
            parser.error("--base-local-ip must be an IP address")
        if any(not origin.startswith("https://") for origin in arguments.allowed_origin):
            parser.error("allowed origins must use https://")
        try:
            credentials = load_credentials(arguments.credentials_file)
        except (OSError, ValueError, json.JSONDecodeError) as error:
            parser.error(f"invalid credentials file: {error}")
    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(message)s")
    event_logger = (
        RunEventLogger(arguments.event_log, run_id=arguments.run_id)
        if arguments.event_log
        else None
    )
    ros_sink = None
    if arguments.ros_twist_topic:
        from .ros_twist_sink import RosTwistSink

        try:
            ros_sink = RosTwistSink(
                arguments.ros_twist_topic,
                robot_model=arguments.robot_model,
            )
        except (RuntimeError, ValueError) as error:
            logging.error("%s", error)
            return 2
        if not ros_sink.wait_until_ready(arguments.ros_ready_timeout_sec):
            logging.error("ROS bridge did not report readiness before timeout")
            ros_sink.close()
            return 2
        deadline = time.monotonic() + arguments.ros_ready_timeout_sec
        while ros_sink.telemetry() is None and time.monotonic() < deadline:
            time.sleep(0.05)
        if ros_sink.telemetry() is None:
            logging.error("ROS telemetry did not become available before timeout")
            ros_sink.close()
            return 2
    if arguments.local_simulator:
        from .tool_simulator import DeterministicToolSimulator

        arbiter = CooperativeArbiter()
        runtime = LocalSimulationRuntime(arbiter, DeterministicToolSimulator())
        gateway = CooperativeGateway(
            arbiter,
            room_id=arguments.room_id,
            local_runtime=runtime,
            event_logger=event_logger,
            twist_sink=ros_sink,
            readiness_probe=ros_sink.ready if ros_sink else None,
            telemetry_source=ros_sink.telemetry if ros_sink else None,
            credentials=credentials,
            anonymous_lan=arguments.anonymous_lan,
        )
    else:
        gateway = CooperativeGateway(
            room_id=arguments.room_id,
            event_logger=event_logger,
            twist_sink=ros_sink,
            readiness_probe=ros_sink.ready if ros_sink else None,
            telemetry_source=ros_sink.telemetry if ros_sink else None,
            credentials=credentials,
            anonymous_lan=arguments.anonymous_lan,
        )
    gateway.mark_ready()
    try:
        asyncio.run(
            serve_websocket(
                gateway,
                host=arguments.host,
                port=arguments.port,
                tick_interval_sec=arguments.tick_ms / 1000.0,
                allowed_origins=tuple(arguments.allowed_origin) if credentials is not None else None,
                trusted_proxy=arguments.trusted_proxy,
                base_local_ips=tuple(arguments.base_local_ip),
                allow_local_web=arguments.allow_local_web,
            )
        )
    except KeyboardInterrupt:  # pragma: no cover - interactive entry point
        return 0
    except RuntimeError as error:  # pragma: no cover - dependency dependent
        logging.error("%s", error)
        return 2
    finally:
        if event_logger is not None:
            event_logger.close()
        if ros_sink is not None:
            ros_sink.close()
    return 0


if __name__ == "__main__":  # pragma: no cover - module entry point
    raise SystemExit(main())
