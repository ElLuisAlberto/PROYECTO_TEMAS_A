"""Optional ROS 2 publisher used by the gateway's one authoritative tick."""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from importlib import import_module
from typing import Callable

from .command_composer import Twist
from .simulation_port import TelemetrySample, ToolPose


@dataclass(frozen=True)
class RobotKinematics:
    """The deliberately small FK contract the ROS telemetry adapter needs."""

    name: str
    joint_names: tuple[str, ...]
    joint_limits: tuple[tuple[float, float], ...]
    forward_kinematics: Callable[[tuple[float, ...]], ToolPose]
    telemetry_source: str


def robot_kinematics(robot_model: str, telemetry_source: str | None = None) -> RobotKinematics:
    """Load the JACO2 kinematic model without a silent fallback."""
    if robot_model == "generic":
        raise ValueError("generic ROS kinematics is not included; select jaco2")
    if robot_model == "jaco2":
        try:
            module = import_module(".jaco2_kinematics", __package__)
        except ImportError as error:
            raise RuntimeError(
                "Jaco2 kinematics is unavailable; install the project's "
                "teleop_cooperativo.jaco2_kinematics module before selecting "
                "--robot-model jaco2"
            ) from error
        return _validated_kinematics(
            name="jaco2",
            joint_names=getattr(module, "JOINT_NAMES", None),
            joint_limits=getattr(module, "JOINT_LIMITS", None),
            forward_kinematics=getattr(module, "forward_kinematics", None),
            telemetry_source=telemetry_source or "ros_gazebo_jaco2_fk",
        )
    raise ValueError("robot_model must be 'jaco2' or 'generic'")


def _validated_kinematics(
    *,
    name: str,
    joint_names: object,
    joint_limits: object,
    forward_kinematics: object,
    telemetry_source: str,
) -> RobotKinematics:
    """Reject incomplete model modules before accepting any ROS telemetry."""
    try:
        names = tuple(joint_names)  # type: ignore[arg-type]
        limits = tuple(tuple(limit) for limit in joint_limits)  # type: ignore[arg-type]
    except TypeError as error:
        raise RuntimeError(f"{name} kinematics has an invalid joint contract") from error
    if len(names) != 6 or len(limits) != len(names) or not all(
        isinstance(joint_name, str) and joint_name for joint_name in names
    ):
        raise RuntimeError(f"{name} kinematics must define six named arm joints")
    if not callable(forward_kinematics):
        raise RuntimeError(f"{name} kinematics must provide forward_kinematics")
    normalized_limits: list[tuple[float, float]] = []
    for limit in limits:
        if len(limit) != 2:
            raise RuntimeError(f"{name} kinematics has an invalid joint limit")
        try:
            lower, upper = (float(value) for value in limit)
        except (TypeError, ValueError, OverflowError) as error:
            raise RuntimeError(f"{name} kinematics has an invalid joint limit") from error
        if not (math.isfinite(lower) and math.isfinite(upper) and lower <= upper):
            raise RuntimeError(f"{name} kinematics has an invalid joint limit")
        normalized_limits.append((lower, upper))
    return RobotKinematics(
        name=name,
        joint_names=names,
        joint_limits=tuple(normalized_limits),
        forward_kinematics=forward_kinematics,  # type: ignore[arg-type]
        telemetry_source=telemetry_source,
    )


class RosTwistSink:
    """Publish a validated world-frame Cartesian twist to ROS 2.

    Importing this module is safe without ROS.  Constructing it is explicitly
    opt-in and fails clearly when the ROS environment is not sourced.
    """

    def __init__(
        self,
        topic: str = "/teleop/cartesian_twist",
        *,
        robot_model: str = "generic",
        telemetry_source: str | None = None,
    ) -> None:
        if not isinstance(topic, str) or not topic.startswith("/"):
            raise ValueError("ROS twist topic must be an absolute non-empty name")
        self._kinematics = robot_kinematics(robot_model, telemetry_source=telemetry_source)
        try:
            import rclpy
            from geometry_msgs.msg import Twist as RosTwist
            from sensor_msgs.msg import JointState
            from std_msgs.msg import Bool, Float64MultiArray, Int32
        except ImportError as error:  # pragma: no cover - ROS environment dependent
            raise RuntimeError(
                "ROS 2 geometry_msgs is unavailable; source /opt/ros/humble/setup.bash"
            ) from error
        self._rclpy = rclpy
        self._message_type = RosTwist
        self._bool_type = Bool
        self._owns_context = not rclpy.ok()
        if self._owns_context:
            rclpy.init()
        self._node = rclpy.create_node("cooperative_teleop_gateway")
        self._publisher = self._node.create_publisher(RosTwist, topic, 10)
        self._recovery = False
        self._recovery_at = 0.0
        self._recovery_pub = self._node.create_publisher(Bool, "/kinova_direct/recovery_permit", 10)
        self._web_permit_pub = self._node.create_publisher(Bool, "/kinova_direct/web_permit", 10)
        self._node.create_subscription(Bool, "/kinova_direct/recovery_active", self._on_recovery, 10)
        self._bridge_ready = False
        self._bridge_ready_at: float | None = None
        self._game_reset_requested = False
        self._object_reset_requests = []
        self._node.create_subscription(Bool, "/teleop/game_reset", self._on_game_reset, 10)
        self._node.create_subscription(Int32, "/teleop/game_reset_object", self._on_object_reset, 10)
        self._finger_ready = False
        self._finger_ready_at = 0.0
        self._node.create_subscription(Bool, "/kinova_direct/finger_ready", self._on_finger_ready, 10)
        self._finger_positions: tuple[float, float, float] | None = None
        self._finger_at: float | None = None
        self._node.create_subscription(Float64MultiArray, "/kinova_direct/finger_state", self._on_fingers, 10)
        self._joint_state: tuple[float, ...] | None = None
        self._joint_state_at: float | None = None
        self._node.create_subscription(Bool, "/teleop/bridge_ready", self._on_ready, 10)
        self._node.create_subscription(JointState, "/joint_states", self._on_joint_state, 10)

    def _on_game_reset(self, message: object) -> None:
        if bool(getattr(message, "data", False)):
            self._game_reset_requested = True

    def take_game_reset(self) -> bool:
        requested = self._game_reset_requested
        self._game_reset_requested = False
        return requested

    def _on_object_reset(self, message: object) -> None:
        index = getattr(message, "data", -1)
        if type(index) is int and 0 <= index < 5 and index not in self._object_reset_requests:
            self._object_reset_requests.append(index)

    def take_object_resets(self):
        requests, self._object_reset_requests = self._object_reset_requests, []
        return requests

    def _on_recovery(self, message: object) -> None:
        self._recovery = bool(getattr(message, "data", False))
        self._recovery_at = time.monotonic()

    def recovery_state(self) -> bool | None:
        if not self._rclpy.ok():
            return None
        try:
            for _ in range(4):
                self._rclpy.spin_once(self._node, timeout_sec=0.0)
        except Exception:
            return None
        return self._recovery if time.monotonic() - self._recovery_at < 0.3 else None

    def recovery_permit(self, allowed: bool) -> None:
        message = self._bool_type()
        message.data = bool(allowed)
        self._recovery_pub.publish(message)

    def web_permit(self, allowed: bool) -> None:
        message = self._bool_type()
        message.data = bool(allowed)
        self._web_permit_pub.publish(message)

    def _on_ready(self, message: object) -> None:
        if bool(getattr(message, "data", False)):
            self._bridge_ready = True
            self._bridge_ready_at = time.monotonic()
        else:
            self._bridge_ready = False
            self._bridge_ready_at = None

    def _on_finger_ready(self, message: object) -> None:
        self._finger_ready = bool(getattr(message, "data", False))
        self._finger_ready_at = time.monotonic()

    def _on_fingers(self, message: object) -> None:
        try:
            values = tuple(float(v) for v in getattr(message, "data", ()))
        except (TypeError, ValueError, OverflowError):
            values = ()
        if len(values) == 3 and all(math.isfinite(v) and 0.0 <= v <= math.radians(80.0) for v in values):
            self._finger_positions = values
            self._finger_at = time.monotonic()
        else:
            self._finger_positions = None
            self._finger_at = None

    def _on_joint_state(self, message: object) -> None:
        names = getattr(message, "name", ())
        positions = getattr(message, "position", ())
        indexed = dict(zip(names, positions))
        if not all(name in indexed for name in self._kinematics.joint_names):
            return
        candidate = tuple(float(indexed[name]) for name in self._kinematics.joint_names)
        if all(
            math.isfinite(value) and limits[0] <= value <= limits[1]
            for value, limits in zip(candidate, self._kinematics.joint_limits)
        ):
            self._joint_state = candidate
            self._joint_state_at = time.monotonic()

    def ready(self, *, maximum_age_sec: float = 0.30) -> bool:
        """Return whether the bridge has recently reported a fresh joint state."""
        if not self._rclpy.ok():
            return False
        try:
            self._rclpy.spin_once(self._node, timeout_sec=0.0)
        except Exception:
            return False
        return bool(
            self._bridge_ready
            and self._bridge_ready_at is not None
            and time.monotonic() - self._bridge_ready_at <= maximum_age_sec
        )

    def wait_until_ready(self, timeout_sec: float) -> bool:
        """Wait at startup for the bridge's explicit readiness heartbeat."""
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if self.ready():
                return True
            time.sleep(0.05)
        return self.ready()

    def __call__(self, twist: Twist) -> None:
        message = self._message_type()
        message.linear.x, message.linear.y, message.linear.z = twist.linear
        message.angular.x, message.angular.y, message.angular.z = twist.angular
        self._publisher.publish(message)

    def telemetry(self, *, maximum_age_sec: float = 0.30) -> TelemetrySample | None:
        """Return FK telemetry from fresh Gazebo joint states, if available."""
        if not self._rclpy.ok():
            return None
        try:
            self._rclpy.spin_once(self._node, timeout_sec=0.0)
        except Exception:
            return None
        if self._joint_state is None or self._joint_state_at is None:
            return None
        age = time.monotonic() - self._joint_state_at
        if age > maximum_age_sec:
            return None
        source = self._kinematics.telemetry_source
        if getattr(self, "_physical_intent", False) or getattr(self, "_physical_session", False):
            if getattr(self, "_physical_source", "physical") == "mock":
                source = "ros_mock_jaco2_fk"
            else:
                source = "ros_physical_jaco2_fk"
        return TelemetrySample(
            source=source,
            pose=self._kinematics.forward_kinematics(self._joint_state),
            joint_positions=self._joint_state,
            age_sec=age,
            finger_positions=self._finger_positions if self._finger_ready and time.monotonic() - self._finger_ready_at <= maximum_age_sec and self._finger_at is not None and time.monotonic() - self._finger_at <= maximum_age_sec else None,
        )

    def close(self) -> None:
        """Release resources owned by this optional publisher."""
        # SIGINT may already have shut the global rclpy context down before
        # gateway cleanup runs.  Destroying a node against that context emits
        # a spurious wait-set error and obscures otherwise valid evidence.
        if self._rclpy.ok():
            self._node.destroy_node()
            if self._owns_context:
                self._rclpy.shutdown()
