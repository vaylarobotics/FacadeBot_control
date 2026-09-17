import math
import time

from rcl_interfaces.msg import ParameterDescriptor

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.lifecycle import Node as LifecycleNode
from rclpy.lifecycle import State, TransitionCallbackReturn
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState

from esp32_bridge.transport import Esp32Transport, Esp32TransportError
from facade_msgs.srv import ReadJointPositions
from facadebot_description import robot_model

# Must match SERVO_IDS in esp32_firmware/main.py (one servo per joint, base to end effector)
_EXPECTED_SERVO_COUNT = 4

_JOINT_CMD_TOPIC = "/facade_bot/joint_cmd"
_JOINT_STREAM_TOPIC = "/facade_bot/joint_stream"  # continuous-motion setpoints (see continuous_trajectory_node)
_QUEUE_DEPTH = 10  # small buffer - joint commands are infrequent, not a high-rate stream
_READ_POSITIONS_SERVICE = "/facade_bot/read_joint_positions"

_DEFAULT_ESP32_HOST = "192.168.1.150"  # must match STATIC_IP in esp32_firmware/main.py
_DEFAULT_ESP32_PORT = 5000  # must match TCP_PORT in esp32_firmware/main.py
_DEFAULT_TIMEOUT_SEC = 7.0  # ESP32 now blocks the move ack until the shaped
                            # trajectory finishes (esp32_firmware/main.py's
                            # move_joints_min_jerk): worst case ~120ms pre-move
                            # readback + 5000ms max duration_ms clamp ~= 5.12s.
                            # 7.0s leaves ~1.9s margin for Wi-Fi/TCP jitter.
_DEFAULT_MOVE_DURATION_MS = 1000  # JointState has no timing field; this fills the ESP32's duration_ms
# duration_ms sent with each streamed "servo" setpoint. Should be about one stream step
# (continuous_trajectory_node's stream_period_sec, ~120ms) so the servo interpolates
# smoothly to each setpoint over the gap before the next one arrives.
_DEFAULT_SERVO_MOVE_DURATION_MS = 120

# LX-16A datasheet: the 0-1000 raw position range spans the servo's full 0-240°
# mechanical travel (0.24 deg per unit). This bridge is now the only place that
# knows about degrees - esp32_firmware/main.py only ever sees raw position counts.
_SERVO_ANGLE_MAX_DEG = 240.0
_POSITION_MIN_RAW = 0  # LX-16A raw position floor, must match _POSITION_MIN in esp32_firmware/main.py
_POSITION_MAX_RAW = 1000  # must match _POSITION_MAX in esp32_firmware/main.py

# Measured true center of each joint, base -> end-effector order (radians, as
# the user read them directly off the physical arm - not derived from the
# URDF). This is now the 0° reference for that joint: commanded angles are
# offset from here, not from raw position 0.
# STALE: measured on the v1 arm. The arm has since been rebuilt (URDF_V2), so
# every one of these has to be re-read off the physical joints before the arm is
# commanded. Unlike the limits, these are per-servo calibration rather than a
# property of the model, so they are not in robot_model.yaml.
_JOINT_CENTER_RAD = (2.17, 2.25, 2.05, 2.25)
_JOINT_CENTER_DEG = tuple(math.degrees(r) for r in _JOINT_CENTER_RAD)
_JOINT_CENTER_RAW = tuple(
    round(d * _POSITION_MAX_RAW / _SERVO_ANGLE_MAX_DEG) for d in _JOINT_CENTER_DEG
)

# Small settle margin before reading positions back - the ESP32 now blocks the
# move ack until the full shaped trajectory (including the final step) has been
# sent (esp32_firmware/main.py's move_joints_min_jerk), so this no longer needs
# to cover duration_ms itself, just LX-16A mechanical settle after the last write.
_FAULT_CHECK_MARGIN_MS = 200

# A joint counts as "reached" within this many degrees of the commanded angle.
# Starting estimate pending real-hardware calibration. Converted to raw units since
# the fault check compares raw counts directly (see _check_move_completed).
_POSITION_TOLERANCE_DEG = 5.0
_POSITION_TOLERANCE_RAW = round(_POSITION_TOLERANCE_DEG * _POSITION_MAX_RAW / _SERVO_ANGLE_MAX_DEG)

# read_positions returns null for a servo that misses its UART read window
# (esp32_firmware/main.py); usually one servo per call. Retry and merge to fill
# gaps. 5 is empirical (observed on the arm).
_MAX_READ_RETRIES = 5

# read_joint_positions_fast trades retries for speed: measured on the arm, a
# single attempt already gets all 4 joints about half the time (~170ms), while
# the full _MAX_READ_RETRIES budget above occasionally stretches past 600ms
# waiting for a joint that keeps missing its UART window. Callers that can
# tolerate an occasional missing joint (e.g. a /joint_states publisher, which
# can just hold a joint's last known reading) get a bounded, quick answer
# instead of blocking behind that worst case. _check_move_completed's fault
# check is NOT this - it needs the full retry budget above, since it is
# deciding whether a move actually reached target.
_FAST_READ_ATTEMPTS = 1
_READ_POSITIONS_FAST_SERVICE = "/facade_bot/read_joint_positions_fast"

# Per-joint safe range, degrees, relative to that joint's own _JOINT_CENTER_DEG
# above. Measured on the arm, not derived from the URDF - URDF_V2 declares every
# joint `continuous` with no <limit> at all. The numbers now live in
# facadebot_description's robot_model.yaml so this gate and facade_control's IK
# cannot drift apart; _load_joint_limits() reads them once at startup.
#
# This is the mandatory bounds-check CLAUDE.md requires before any command
# reaches the Hiwonder board - the last-resort gate, since a joint command can
# arrive here directly (e.g. `ros2 topic pub`, bypassing facade_control's IK
# entirely).
_joint_limits_deg: tuple[tuple[float, float], ...] | None = None


def _load_joint_limits() -> robot_model.RobotModel:
    """Read the active arm's limits. Raises RobotModelError rather than guessing."""
    global _joint_limits_deg
    model = robot_model.load_robot_model()
    if len(model.joints) != _EXPECTED_SERVO_COUNT:
        raise robot_model.RobotModelError(
            f"model '{model.name}' has {len(model.joints)} joints but the firmware "
            f"drives {_EXPECTED_SERVO_COUNT} servos")
    _joint_limits_deg = model.joint_limits_deg
    return model


def _check_joint_bounds(angles_deg: list[float]) -> list[str]:
    """Return one message per joint outside its safe range (empty if all OK)."""
    if _joint_limits_deg is None:
        raise robot_model.RobotModelError(
            "joint limits were never loaded; refusing to bounds-check a command")
    violations = []
    for i, (angle_deg, (lower, upper)) in enumerate(zip(angles_deg, _joint_limits_deg)):
        if angle_deg < lower or angle_deg > upper:
            violations.append(
                f"joint {i}: commanded {angle_deg:.1f} deg, outside safe range ({lower:.1f}-{upper:.1f} deg)"
            )
    return violations


def _angle_deg_to_position_raw(angle_deg: float, joint_index: int) -> int:
    position_raw = _JOINT_CENTER_RAW[joint_index] + round(
        angle_deg * _POSITION_MAX_RAW / _SERVO_ANGLE_MAX_DEG
    )
    return max(_POSITION_MIN_RAW, min(_POSITION_MAX_RAW, position_raw))


def _position_raw_to_angle_deg(position_raw: int, joint_index: int) -> float:
    return (position_raw - _JOINT_CENTER_RAW[joint_index]) * _SERVO_ANGLE_MAX_DEG / _POSITION_MAX_RAW


class Esp32BridgeNode(LifecycleNode):
    """Subscribes to a joint-position command topic and forwards each command
    to the ESP32 over Wi-Fi/TCP. This is the only place joint commands cross
    from ROS2 into the ESP32 transport - see esp32_bridge/transport.py for the
    wire protocol itself.

    This is a lifecycle node: it does nothing on construction. A separate
    "configure" step opens the ESP32 connection, and a separate "activate"
    step starts listening for commands - see on_configure/on_activate below.
    """

    def __init__(self) -> None:
        super().__init__("esp32_bridge_node")
        self.declare_parameter(
            "esp32_host", _DEFAULT_ESP32_HOST,
            ParameterDescriptor(description="IP address of the ESP32 TCP server"))
        self.declare_parameter(
            "esp32_port", _DEFAULT_ESP32_PORT,
            ParameterDescriptor(description="TCP port the ESP32 listens on"))
        self.declare_parameter(
            "esp32_timeout_sec", _DEFAULT_TIMEOUT_SEC,
            ParameterDescriptor(description="seconds to wait for a connect/response before giving up"))
        self.declare_parameter(
            "move_duration_ms", _DEFAULT_MOVE_DURATION_MS,
            ParameterDescriptor(description="milliseconds the ESP32 should take to reach each commanded pose"))
        self.declare_parameter(
            "servo_move_duration_ms", _DEFAULT_SERVO_MOVE_DURATION_MS,
            ParameterDescriptor(description="milliseconds per streamed setpoint; keep in step with "
                                            "continuous_trajectory_node's stream_period_sec"))

        self._transport: Esp32Transport | None = None
        self._subscription = None
        self._stream_subscription = None
        self._service = None
        self._fast_service = None

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        host = self.get_parameter("esp32_host").value
        port = self.get_parameter("esp32_port").value
        timeout_sec = self.get_parameter("esp32_timeout_sec").value

        transport = Esp32Transport(host, port, timeout_sec)
        try:
            transport.connect()
        except Esp32TransportError as exc:
            self.get_logger().error(f"failed to connect to ESP32 at {host}:{port}: {exc}")
            return TransitionCallbackReturn.FAILURE

        self._transport = transport
        self.get_logger().info(f"connected to ESP32 at {host}:{port}")
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        # Subscription is created here, not in on_configure, because LifecycleNode
        # does not gate plain subscriptions by state (unlike publishers) - creating
        # it only now guarantees the callback can't fire before we're truly active.
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=_QUEUE_DEPTH,
        )
        self._subscription = self.create_subscription(
            JointState, _JOINT_CMD_TOPIC, self._joint_cmd_callback, qos
        )
        self.get_logger().info(f"listening on {_JOINT_CMD_TOPIC}")

        self._stream_subscription = self.create_subscription(
            JointState, _JOINT_STREAM_TOPIC, self._joint_stream_callback, qos
        )
        self.get_logger().info(f"listening on {_JOINT_STREAM_TOPIC}")

        self._service = self.create_service(
            ReadJointPositions, _READ_POSITIONS_SERVICE, self._handle_read_joint_positions
        )
        self.get_logger().info(f"serving {_READ_POSITIONS_SERVICE}")

        self._fast_service = self.create_service(
            ReadJointPositions, _READ_POSITIONS_FAST_SERVICE, self._handle_read_joint_positions_fast
        )
        self.get_logger().info(f"serving {_READ_POSITIONS_FAST_SERVICE}")
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        if self._subscription is not None:
            self.destroy_subscription(self._subscription)
            self._subscription = None
        if self._stream_subscription is not None:
            self.destroy_subscription(self._stream_subscription)
            self._stream_subscription = None
        if self._service is not None:
            self.destroy_service(self._service)
            self._service = None
        if self._fast_service is not None:
            self.destroy_service(self._fast_service)
            self._fast_service = None
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        if self._transport is not None:
            self._transport.disconnect()
            self._transport = None
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        if self._subscription is not None:
            self.destroy_subscription(self._subscription)
            self._subscription = None
        if self._stream_subscription is not None:
            self.destroy_subscription(self._stream_subscription)
            self._stream_subscription = None
        if self._service is not None:
            self.destroy_service(self._service)
            self._service = None
        if self._fast_service is not None:
            self.destroy_service(self._fast_service)
            self._fast_service = None
        if self._transport is not None:
            self._transport.disconnect()
            self._transport = None
        return TransitionCallbackReturn.SUCCESS

    def _joint_cmd_callback(self, msg: JointState) -> None:
        angles_deg = [math.degrees(p) for p in msg.position]
        if len(angles_deg) != _EXPECTED_SERVO_COUNT:
            self.get_logger().warn(
                f"expected {_EXPECTED_SERVO_COUNT} joint positions, got {len(angles_deg)} - ignoring"
            )
            return
        self._send_move_command(angles_deg)

    def _send_move_command(self, angles_deg: list[float]) -> None:
        violations = _check_joint_bounds(angles_deg)
        if violations:
            self.get_logger().error("move rejected - out of bounds: " + "; ".join(violations))
            return

        duration_ms = self.get_parameter("move_duration_ms").value
        target_positions_raw = [_angle_deg_to_position_raw(a, i) for i, a in enumerate(angles_deg)]
        command = {"cmd": "move", "positions": target_positions_raw, "duration_ms": duration_ms}
        try:
            self._transport.send_command(command)
            response = self._transport.read_response()
        except Esp32TransportError as exc:
            self.get_logger().error(f"ESP32 command failed: {exc}")
            return

        if response.get("status") != "ok":
            self.get_logger().warn(f"ESP32 rejected command: {response}")
        else:
            self.get_logger().info(f"sent {angles_deg} deg, ESP32 ack ok")
            self._check_move_completed(target_positions_raw, angles_deg)

    def _joint_stream_callback(self, msg: JointState) -> None:
        angles_deg = [math.degrees(p) for p in msg.position]
        if len(angles_deg) != _EXPECTED_SERVO_COUNT:
            self.get_logger().warn(
                f"expected {_EXPECTED_SERVO_COUNT} stream positions, got {len(angles_deg)} - ignoring"
            )
            return
        self._send_servo_command(angles_deg)

    def _send_servo_command(self, angles_deg: list[float]) -> None:
        # Streaming setpoint path: the same mandatory bounds-check applies, but this
        # uses the ESP32's non-blocking "servo" command (no min-jerk shaping, no
        # position readback) so setpoints can flow back-to-back for continuous motion.
        # No per-setpoint info logging - at the streaming rate it would flood the log.
        violations = _check_joint_bounds(angles_deg)
        if violations:
            self.get_logger().error("stream setpoint rejected - out of bounds: " + "; ".join(violations))
            return

        duration_ms = self.get_parameter("servo_move_duration_ms").value
        target_positions_raw = [_angle_deg_to_position_raw(a, i) for i, a in enumerate(angles_deg)]
        command = {"cmd": "servo", "positions": target_positions_raw, "duration_ms": duration_ms}
        try:
            self._transport.send_command(command)
            response = self._transport.read_response()
        except Esp32TransportError as exc:
            self.get_logger().warn(f"ESP32 servo command failed: {exc}")
            return

        if response.get("status") != "ok":
            self.get_logger().warn(f"ESP32 rejected servo command: {response}")

    def _read_positions_with_retry(self, max_attempts: int = _MAX_READ_RETRIES) -> list[int | None]:
        merged_positions_raw: list[int | None] = [None] * _EXPECTED_SERVO_COUNT
        for _attempt in range(max_attempts):
            try:
                self._transport.send_command({"cmd": "read_positions"})
                response = self._transport.read_response()
            except Esp32TransportError as exc:
                self.get_logger().warn(f"read_positions failed, aborting readback: {exc}")
                return merged_positions_raw
            positions_raw = response.get("positions")
            if (response.get("status") != "ok"
                    or not isinstance(positions_raw, list)
                    or len(positions_raw) != _EXPECTED_SERVO_COUNT):
                self.get_logger().warn(f"unexpected read_positions reply: {response}")
                continue
            for i, position_raw in enumerate(positions_raw):
                if merged_positions_raw[i] is None and position_raw is not None:
                    merged_positions_raw[i] = position_raw
            if all(p is not None for p in merged_positions_raw):
                break
        return merged_positions_raw

    def _check_move_completed(
        self, target_positions_raw: list[int], commanded_angles_deg: list[float]
    ) -> None:
        time.sleep(_FAULT_CHECK_MARGIN_MS / 1000.0)

        positions_raw = self._read_positions_with_retry()

        # Every joint silent is a dead bus, not four simultaneous stalls: a stalled
        # servo still answers a position read. Reported separately because the
        # per-joint message below reads as "the arm tried and failed", which sends
        # you looking at the mechanism instead of at power and wiring.
        if all(position_raw is None for position_raw in positions_raw):
            self.get_logger().error(
                "servo bus fault - no joint answered a position read. The ESP32 acked the "
                "move, but an ack only means the command reached the ESP32, never that a "
                "servo received it. Check servo power, the ESP32-BusLinker TTL wiring and "
                "its common ground, then run scripts/check_servo_bus.py over USB.")
            return

        # target_positions_raw / commanded_angles_deg / positions_raw are all in
        # SERVO_IDS order (base -> end effector), so index i is the same joint in all three.
        faults: list[str] = []
        for i, target_raw in enumerate(target_positions_raw):
            actual_raw = positions_raw[i]
            if actual_raw is None:
                faults.append(f"joint {i}: commanded {commanded_angles_deg[i]:.1f} deg, no reading")
                continue
            if abs(actual_raw - target_raw) > _POSITION_TOLERANCE_RAW:
                actual_angle_deg = _position_raw_to_angle_deg(actual_raw, i)
                faults.append(
                    f"joint {i}: commanded {commanded_angles_deg[i]:.1f} deg, "
                    f"read {actual_angle_deg:.1f} deg"
                )

        if faults:
            self.get_logger().error("move fault - joint(s) did not reach target: " + "; ".join(faults))

    def _handle_read_joint_positions(
        self, request: ReadJointPositions.Request, response: ReadJointPositions.Response
    ) -> ReadJointPositions.Response:
        positions_raw = self._read_positions_with_retry()

        response.positions_deg = [
            _position_raw_to_angle_deg(p, i) if p is not None else float("nan")
            for i, p in enumerate(positions_raw)
        ]
        response.all_valid = all(p is not None for p in positions_raw)
        return response

    def _handle_read_joint_positions_fast(
        self, request: ReadJointPositions.Request, response: ReadJointPositions.Response
    ) -> ReadJointPositions.Response:
        # Same request/response shape as read_joint_positions - only the retry
        # budget differs (see _FAST_READ_ATTEMPTS above). A missing joint here
        # is expected and normal, not a fault: this is not the safety-critical
        # path, so there is no log line for it - the caller is expected to
        # handle response.all_valid being False as routine, not exceptional.
        positions_raw = self._read_positions_with_retry(max_attempts=_FAST_READ_ATTEMPTS)

        response.positions_deg = [
            _position_raw_to_angle_deg(p, i) if p is not None else float("nan")
            for i, p in enumerate(positions_raw)
        ]
        response.all_valid = all(p is not None for p in positions_raw)
        return response


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    try:
        model = _load_joint_limits()
    except robot_model.RobotModelError as error:
        print(f"esp32_bridge_node: refusing to start: {error}")
        rclpy.shutdown()
        raise SystemExit(1)
    print(robot_model.describe(model))
    node = Esp32BridgeNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
