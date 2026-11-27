"""Native monitoring console for the base-station computer.

The console observes the gateway without claiming axes. When explicitly
configured for the direct bridge, it also provides a local ROS output switch.
"""

from __future__ import annotations

import argparse
import asyncio
from collections import deque
from dataclasses import dataclass, field
import json
import math
from pathlib import Path
import threading
import time
from typing import Any, Mapping


AXIS_NAMES = ("X", "Y", "Z", "ROLL", "PITCH", "YAW", "PINZA")


def _finite_vector(value: object, size: int) -> tuple[float, ...] | None:
    if not isinstance(value, list) or len(value) != size:
        return None
    if any(isinstance(item, bool) or not isinstance(item, (int, float)) for item in value):
        return None
    vector = tuple(float(item) for item in value)
    return vector if all(math.isfinite(item) for item in vector) else None


def _parse_virtual_task(value: object) -> dict[str, object] | None:
    if not isinstance(value, Mapping) or value.get("total") != 5:
        return None
    objects = value.get("objects")
    if not isinstance(objects, list) or len(objects) != 5:
        return None
    result = []
    seen = set()
    for item in objects:
        if not isinstance(item, Mapping):
            return None
        index = item.get("id")
        position = _finite_vector(item.get("position"), 3)
        orientation = _finite_vector(item.get("quaternion_xyzw"), 4)
        if type(index) is not int or index not in range(5) or index in seen or position is None or orientation is None:
            return None
        if any(abs(x) > 20 for x in position) or not 0.9 <= sum(x*x for x in orientation) <= 1.1:
            return None
        seen.add(index)
        state = item.get("state")
        state = state if state in ("sujeto", "apoyado", "caido", "en_aire", "entregado") else None
        result.append({"id": index, "position": position, "quaternion_xyzw": orientation, "delivered": item.get("delivered") is True, "state": state})
    held = value.get("held")
    if held is not None and (type(held) is not int or held not in range(5)):
        return None
    delivered = value.get("delivered")
    if type(delivered) is not int or not 0 <= delivered <= 5:
        return None
    return {"objects": result, "held": held, "delivered": delivered, "total": 5, "release_reason": "impacto" if value.get("release_reason") == "impacto" else None}


@dataclass(frozen=True)
class DesktopSnapshot:
    """Immutable public state rendered by the Qt main thread."""

    connected: bool = False
    connection_label: str = "Desconectado"
    room_id: str = "default"
    room_state: str = "—"
    revision: int = 0
    owners: Mapping[str, str] = field(
        default_factory=lambda: {axis: "Disponible" for axis in AXIS_NAMES}
    )
    telemetry_source: str = "Sin telemetría"
    telemetry_age_sec: float | None = None
    position: tuple[float, float, float] | None = None
    quaternion: tuple[float, float, float, float] | None = None
    linear: tuple[float, float, float] | None = None
    angular: tuple[float, float, float] | None = None
    joint_positions: tuple[float, ...] | None = None
    finger_positions: tuple[float, float, float] | None = None
    finger_source: str | None = None
    virtual_task: Mapping[str, object] | None = None
    updated_monotonic: float | None = None
    events: tuple[str, ...] = ()
    reserved_count: int | None = None


class DesktopState:
    """Thread-safe reducer for untrusted gateway messages."""

    def __init__(self, room_id: str = "default", *, max_events: int = 150) -> None:
        if not isinstance(room_id, str) or not room_id.strip():
            raise ValueError("room_id must be a non-empty string")
        if max_events <= 0:
            raise ValueError("max_events must be positive")
        self._lock = threading.Lock()
        self._room_id = room_id
        self._connected = False
        self._connection_label = "Desconectado"
        self._room_state = "—"
        self._reserved_count: int | None = None
        self._revision = 0
        self._owners = {axis: "Disponible" for axis in AXIS_NAMES}
        self._telemetry_source = "Sin telemetría"
        self._telemetry_age_sec: float | None = None
        self._position: tuple[float, float, float] | None = None
        self._quaternion: tuple[float, float, float, float] | None = None
        self._linear: tuple[float, float, float] | None = None
        self._angular: tuple[float, float, float] | None = None
        self._joint_positions: tuple[float, ...] | None = None
        self._finger_positions: tuple[float, float, float] | None = None
        self._finger_source: str | None = None
        self._virtual_task: dict[str, object] | None = None
        self._updated_monotonic: float | None = None
        self._events: deque[str] = deque(maxlen=max_events)

    def set_connection(self, connected: bool, label: str) -> None:
        with self._lock:
            changed = self._connected != connected or self._connection_label != label
            self._connected = connected
            self._connection_label = str(label)
            if not connected:
                self._clear_observation_locked()
                self._reserved_count = None
                self._room_state = "—"
                self._owners = {axis: "Desconocido" for axis in AXIS_NAMES}
            if changed:
                self._append_event_locked(label)

    def _clear_observation_locked(self) -> None:
        self._telemetry_source = "Telemetría no disponible"
        self._telemetry_age_sec = None
        self._position = self._quaternion = self._joint_positions = self._finger_positions = None
        self._finger_source = None
        self._virtual_task = None
        self._linear = self._angular = None
        self._updated_monotonic = None

    def apply(self, message: Mapping[str, object]) -> bool:
        """Apply a valid public message; malformed fields are ignored safely."""
        if not isinstance(message, Mapping):
            return False
        message_type = message.get("type")
        if not isinstance(message_type, str):
            return False
        with self._lock:
            if message_type == "welcome":
                self._room_state = self._text(message.get("state"), self._room_state)
                self._append_event_locked("Sesión de observación iniciada")
                return True
            if message_type == "room_state":
                self._apply_room_locked(message)
                return True
            if message_type == "telemetry":
                self._apply_telemetry_locked(message)
                return True
            if message_type == "error":
                detail = self._text(message.get("detail"), "Error sin detalle")
                self._append_event_locked(f"Gateway: {detail}")
                return True
            return False

    def add_event(self, text: str) -> None:
        with self._lock:
            self._append_event_locked(str(text))

    def snapshot(self) -> DesktopSnapshot:
        with self._lock:
            return DesktopSnapshot(
                connected=self._connected,
                connection_label=self._connection_label,
                room_id=self._room_id,
                room_state=self._room_state,
                revision=self._revision,
                owners=dict(self._owners),
                reserved_count=self._reserved_count,
                telemetry_source=self._telemetry_source,
                telemetry_age_sec=self._telemetry_age_sec,
                position=self._position,
                quaternion=self._quaternion,
                linear=self._linear,
                angular=self._angular,
                joint_positions=self._joint_positions,
                finger_positions=self._finger_positions,
                finger_source=self._finger_source,
                virtual_task=self._virtual_task,
                updated_monotonic=self._updated_monotonic,
                events=tuple(self._events),
            )

    def _apply_room_locked(self, message: Mapping[str, object]) -> None:
        self._room_state = self._text(message.get("state"), self._room_state)
        revision = message.get("revision")
        if isinstance(revision, int) and not isinstance(revision, bool) and revision >= 0:
            self._revision = revision
        axes = message.get("axes")
        if not isinstance(axes, Mapping):
            return
        self._reserved_count = None
        if all(isinstance(axes.get(axis), Mapping) and isinstance(axes[axis].get("assigned"), bool) for axis in AXIS_NAMES):
            self._reserved_count = sum(axes[axis]["assigned"] for axis in AXIS_NAMES)
        for axis in AXIS_NAMES:
            item = axes.get(axis)
            if isinstance(item, Mapping) and item.get("available") is False:
                self._owners[axis] = "No disponible"
                continue
            if not isinstance(item, Mapping) or item.get("assigned") is not True:
                self._owners[axis] = "Disponible"
                continue
            self._owners[axis] = self._text(item.get("owner"), "Asignado")

    def _apply_telemetry_locked(self, message: Mapping[str, object]) -> None:
        self._virtual_task = _parse_virtual_task(message.get("virtual_task"))
        self._room_state = self._text(message.get("state"), self._room_state)
        twist = message.get("twist")
        if isinstance(twist, Mapping):
            linear = _finite_vector(twist.get("linear"), 3)
            angular = _finite_vector(twist.get("angular"), 3)
            if linear is not None:
                self._linear = linear  # type: ignore[assignment]
            if angular is not None:
                self._angular = angular  # type: ignore[assignment]

        # An unavailable ROS sample still carries the authoritative commanded
        # twist, but it must invalidate every observation of the physical
        # simulation.  Otherwise the console could present an old pose as live.
        if message.get("telemetry_available") is False:
            self._telemetry_source = "Telemetría no disponible"
            self._telemetry_age_sec = None
            self._position = None
            self._quaternion = None
            self._joint_positions = None
            self._finger_positions = None
            self._finger_source = None
            self._updated_monotonic = None
            return

        pose = message.get("pose")
        position = _finite_vector(pose.get("position"), 3) if isinstance(pose, Mapping) else None
        quaternion = _finite_vector(pose.get("quaternion_xyzw"), 4) if isinstance(pose, Mapping) else None
        joints = _finite_vector(message.get("joint_positions"), 6)
        age = message.get("pose_age_sec", message.get("age_sec"))
        valid_age = isinstance(age, (int, float)) and not isinstance(age, bool) and math.isfinite(age) and age >= 0
        if message.get("source") in ("ros_gazebo_jaco2_fk", "ros_mock_jaco2_fk", "ros_physical_jaco2_fk"):
            if position is None or quaternion is None or joints is None or not valid_age:
                self._clear_observation_locked()
                return
            from .jaco2_kinematics import _validated_joints
            try:
                _validated_joints(joints)
            except (TypeError, ValueError):
                self._clear_observation_locked()
                return
        self._telemetry_source = self._text(message.get("source"), "Gateway")
        if valid_age:
            self._telemetry_age_sec = float(age)
        if isinstance(pose, Mapping):
            position = _finite_vector(pose.get("position"), 3)
            quaternion = _finite_vector(pose.get("quaternion_xyzw"), 4)
            if position is not None:
                self._position = position  # type: ignore[assignment]
            if quaternion is not None:
                self._quaternion = quaternion  # type: ignore[assignment]
        joints = message.get("joint_positions")
        if isinstance(joints, list) and joints:
            parsed = _finite_vector(joints, len(joints))
            if parsed is not None:
                self._joint_positions = parsed
        fingers = _finite_vector(message.get("finger_positions"), 3)
        self._finger_positions = fingers if fingers is not None and all(0 <= v <= math.radians(80) for v in fingers) else None
        self._finger_source = message.get("finger_source") if self._finger_positions is not None and message.get("finger_source") in ("simulated", "physical") else None
        self._updated_monotonic = time.monotonic()

    def _append_event_locked(self, text: str) -> None:
        stamp = time.strftime("%H:%M:%S")
        self._events.appendleft(f"[{stamp}] {text}")

    @staticmethod
    def _text(value: object, fallback: str) -> str:
        return value.strip() if isinstance(value, str) and value.strip() else fallback


class GatewayObserver:
    """Reconnectable WebSocket observer running outside the GUI main thread."""

    def __init__(
        self,
        state: DesktopState,
        *,
        url: str,
        room_id: str = "default",
        operator_name: str = "Consola base",
        reconnect_delay_sec: float = 1.0,
        credential: str | None = None,
    ) -> None:
        if not url.startswith(("ws://", "wss://")):
            raise ValueError("url must use ws:// or wss://")
        if reconnect_delay_sec <= 0:
            raise ValueError("reconnect_delay_sec must be positive")
        self.state = state
        self.url = url
        self.room_id = room_id
        self.operator_name = operator_name
        self.reconnect_delay_sec = reconnect_delay_sec
        self.credential = credential
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    @property
    def running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def start(self) -> None:
        if self.running:
            return
        self._stop.clear()
        self._thread = threading.Thread(
            target=self._thread_main, name="gateway-observer", daemon=True
        )
        self._thread.start()

    def stop(self, timeout: float = 2.0) -> None:
        self._stop.set()
        thread = self._thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=max(0.0, timeout))
        self.state.set_connection(False, "Consola cerrada")

    def _thread_main(self) -> None:
        try:
            asyncio.run(self._run())
        except Exception as error:  # defensive boundary for a GUI background thread
            self.state.set_connection(False, f"Error de observación: {error}")

    async def _run(self) -> None:
        try:
            import websockets
        except ImportError:
            self.state.set_connection(False, "Falta la dependencia websockets")
            return
        while not self._stop.is_set():
            self.state.set_connection(False, "Conectando al gateway…")
            try:
                async with websockets.connect(self.url, max_size=16 * 1024) as socket:
                    await socket.send(
                        json.dumps(
                            {
                                "type": "hello",
                                "operator_name": self.operator_name,
                                "room_id": self.room_id,
                                "role": "observer",
                                **({"credential": self.credential} if self.credential else {}),
                            },
                            separators=(",", ":"),
                        )
                    )
                    self.state.set_connection(True, "Conectado")
                    next_heartbeat = time.monotonic() + 0.5
                    while not self._stop.is_set():
                        if time.monotonic() >= next_heartbeat:
                            await socket.send('{"type":"heartbeat"}')
                            next_heartbeat = time.monotonic() + 0.5
                        try:
                            raw = await asyncio.wait_for(socket.recv(), timeout=max(0.01, next_heartbeat - time.monotonic()))
                        except asyncio.TimeoutError:
                            continue
                        if not isinstance(raw, str):
                            continue
                        try:
                            decoded = json.loads(raw)
                        except json.JSONDecodeError:
                            self.state.add_event("Mensaje JSON inválido ignorado")
                            continue
                        if isinstance(decoded, Mapping):
                            self.state.apply(decoded)
            except Exception as error:
                if not self._stop.is_set():
                    self.state.set_connection(False, f"Desconectado: {error}")
            if not self._stop.is_set():
                await asyncio.sleep(self.reconnect_delay_sec)
        self.state.set_connection(False, "Desconectado")


class DesktopConsole:
    """Lazy Qt frontend, keeping the state reducer usable without GUI libraries."""

    def __new__(cls, state: DesktopState, observer: GatewayObserver, direct_bridge: bool = False, physical: bool = False):
        from .desktop_qt import QtDesktopConsole
        return QtDesktopConsole(state, observer, direct_bridge=direct_bridge, physical=physical)

    @staticmethod
    def _format_vector(value: tuple[float, ...] | None) -> str:
        return "—" if value is None else "[" + ", ".join(f"{item:.4f}" for item in value) + "]"


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Consola nativa de la estación base")
    parser.add_argument("--server-url", default="ws://127.0.0.1:8765/ws")
    parser.add_argument("--room-id", default="default")
    parser.add_argument("--direct-bridge", action="store_true", help="control local de salida del bridge directo")
    parser.add_argument("--physical", action="store_true", help="mostrar reconexión USB física")
    parser.add_argument("--alias", default="Consola base")
    parser.add_argument("--credentials-file", type=Path, help="JSON local con credencial de observador")
    return parser


def main() -> int:
    arguments = build_argument_parser().parse_args()
    state = DesktopState(arguments.room_id)
    credential = None
    if arguments.credentials_file:
        try:
            credentials = json.loads(arguments.credentials_file.read_text(encoding="utf-8"))
            credential = credentials["observers"][0]
            if not isinstance(credential, str) or not credential:
                raise ValueError("credencial de observador inválida")
        except (OSError, ValueError, KeyError, IndexError, TypeError):
            print("No se pudo cargar una credencial de observador desde el archivo indicado.")
            return 2
    observer = GatewayObserver(
        state,
        url=arguments.server_url,
        room_id=arguments.room_id,
        operator_name=arguments.alias,
        credential=credential,
    )
    try:
        console = DesktopConsole(state, observer, direct_bridge=arguments.direct_bridge, physical=arguments.physical)
    except (ImportError, RuntimeError, OSError, ValueError) as error:
        print(f"No se pudo iniciar la interfaz nativa: {error}")
        return 2
    console.run()
    return 0


if __name__ == "__main__":  # pragma: no cover - GUI entry point
    raise SystemExit(main())
