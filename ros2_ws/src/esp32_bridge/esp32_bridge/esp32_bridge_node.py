import math
import threading
import time

from rcl_interfaces.msg import ParameterDescriptor

import rclpy
import rclpy.logging
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import Node as LifecycleNode
from rclpy.lifecycle import State, TransitionCallbackReturn
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState

from esp32_bridge.transport import Esp32Transport, Esp32TransportError
from facade_msgs.srv import IsMoving, ReadJointPositions
from facadebot_description import robot_model

# Must match SERVO_IDS in esp32_firmware/main.py (one servo per joint, base to end effector)
_EXPECTED_SERVO_COUNT = 4

_JOINT_CMD_TOPIC = "/facade_bot/joint_cmd"
_JOINT_STREAM_TOPIC = "/facade_bot/joint_stream"  # continuous-motion setpoints (see continuous_trajectory_node)
_QUEUE_DEPTH = 10  # small buffer - joint commands are infrequent, not a high-rate stream
_READ_POSITIONS_SERVICE = "/facade_bot/read_joint_positions"
_IS_MOVING_SERVICE = "/facade_bot/is_moving"

_DEFAULT_ESP32_HOST = "192.168.1.150"  # must match STATIC_IP in esp32_firmware/main.py
_DEFAULT_ESP32_PORT = 5000  # must match TCP_PORT in esp32_firmware/main.py
_DEFAULT_TIMEOUT_SEC = 2.0  # Every command the firmware serves now acks promptly:
                            # the slowest is read_positions at ~170ms, and move_async
                            # acks after only its ~120ms pre-move readback. 2.0s is
                            # generous margin for Wi-Fi/TCP jitter on top of that.
                            # NOTE: use_blocking_move (below) puts the bridge back on
                            # the legacy `move`, which does block the ack until the
                            # trajectory finishes - that needs esp32_timeout_sec:=7.0.
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
# Current for the v2 arm, confirmed by the user 2026-09-21. joint_1 was re-measured
# during the v2 rebuild and moved from v1's 2.09 to 2.17 (commit c751abc); joints
# 2-4 were re-checked and came back at their v1 values. Unlike the limits, these are
# per-servo calibration rather than a property of the model, so they are not in
# robot_model.yaml. Re-measure and re-date this line if a servo is swapped or
# re-horned.
_JOINT_CENTER_RAD = (2.17, 2.25, 2.05, 2.25)
_JOINT_CENTER_DEG = tuple(math.degrees(r) for r in _JOINT_CENTER_RAD)
_JOINT_CENTER_RAW = tuple(
    round(d * _POSITION_MAX_RAW / _SERVO_ANGLE_MAX_DEG) for d in _JOINT_CENTER_DEG
)

# Small settle margin before reading positions back. The move ack no longer means
# "finished" - _wait_for_motion_complete polls the firmware's `status` command until
# it reports the shaped trajectory is done, so by the time this runs the last step
# has been written and this only needs to cover LX-16A mechanical settle.
_FAULT_CHECK_MARGIN_MS = 200

# How often to ask the ESP32 whether it is still moving. A status query never
# touches the servo bus, but it is not free: it occupies one firmware main-loop
# iteration and competes for Wi-Fi airtime with the position reads and the
# setpoint stream. 50ms is one third of the firmware's _TRAJECTORY_STEP_MS
# (150ms), so a finished move is noticed well inside one step without adding a
# meaningful share of the traffic. Re-measure if the step interval changes.
_STATUS_POLL_INTERVAL_SEC = 0.05

# The legacy blocking `move` does not ack until its trajectory has finished, so
# use_blocking_move needs the old socket budget: ~120ms pre-move readback + the
# firmware's 5000ms _DURATION_MAX_MS clamp ~= 5.12s. Refusing to configure below
# this is deliberate - the rollback silently timing out on every move, and then
# tearing down the connection to resync, is a worse state than the bug it rolls
# back from.
_BLOCKING_MOVE_MIN_TIMEOUT_SEC = 7.0

# Give up waiting for a move to finish after this long and skip the fault check.
# Derived from the firmware's own worst case: ~120ms pre-move readback + the
# 5000ms _DURATION_MAX_MS clamp, plus margin for steps pushed late by interleaved
# position reads (see _TRAJECTORY_STEP_MS in esp32_firmware/main.py).
_MOVE_COMPLETION_TIMEOUT_SEC = 8.0

# A joint counts as "reached" within this many degrees of the commanded angle.
# Starting estimate pending real-hardware calibration. Converted to raw units since
# the fault check compares raw counts directly (see _check_move_completed).
_POSITION_TOLERANCE_DEG = 5.0
_POSITION_TOLERANCE_RAW = round(_POSITION_TOLERANCE_DEG * _POSITION_MAX_RAW / _SERVO_ANGLE_MAX_DEG)

# read_positions returns null for a servo that misses its UART read window
# (esp32_firmware/main.py); usually one servo per call. Retry and merge to fill
# gaps. 5 is empirical (observed on the arm).
_MAX_READ_RETRIES = 5

# One thread runs a move's status-poll loop; the others stay free so position
# reads and the is_moving service can still be answered while it does. Without
# this the node would serialise them again and /joint_states would go silent for
# the length of every move, which is the bug the non-blocking firmware fixes.
_EXECUTOR_THREAD_COUNT = 4

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
        self.declare_parameter(
            "use_blocking_move", False,
            ParameterDescriptor(description="rollback: send the firmware's legacy blocking `move` "
                                            "instead of `move_async` + status polling. Needs "
                                            "esp32_timeout_sec:=7.0, and stops /joint_states "
                                            "updating during a move"))

        # The ESP32 answers one line per command with no correlation ID, so two
        # threads must never have commands in flight at the same time. Held across
        # each send/read pair (see _transact), never across a sequence of them.
        self._transport_lock = threading.Lock()
        # Reads and status queries are reentrant so one can be answered while a
        # move's status poll is in progress - that is the whole point of the
        # change, and under the default group they would serialise again.
        self._service_callback_group = ReentrantCallbackGroup()
        # The two command topics deliberately do NOT share that. They get one
        # mutually exclusive group between them, so only one command is ever in
        # flight: two joint_cmd messages running at once would race for the
        # transport lock and let the older target win arbitrarily, and a stream
        # setpoint overlapping a move would preempt it in the firmware while this
        # node was still polling for that move to finish.
        self._command_callback_group = MutuallyExclusiveCallbackGroup()

        self._transport: Esp32Transport | None = None
        self._subscription = None
        self._stream_subscription = None
        self._service = None
        self._fast_service = None
        self._is_moving_service = None

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        host = self.get_parameter("esp32_host").value
        port = self.get_parameter("esp32_port").value
        timeout_sec = self.get_parameter("esp32_timeout_sec").value

        if self.get_parameter("use_blocking_move").value and timeout_sec < _BLOCKING_MOVE_MIN_TIMEOUT_SEC:
            self.get_logger().error(
                f"use_blocking_move needs esp32_timeout_sec >= {_BLOCKING_MOVE_MIN_TIMEOUT_SEC} "
                f"(the blocking ack waits out the whole move), got {timeout_sec}. Refusing to "
                "configure rather than time out on every move.")
            return TransitionCallbackReturn.FAILURE

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
            JointState, _JOINT_CMD_TOPIC, self._joint_cmd_callback, qos,
            callback_group=self._command_callback_group
        )
        self.get_logger().info(f"listening on {_JOINT_CMD_TOPIC}")

        self._stream_subscription = self.create_subscription(
            JointState, _JOINT_STREAM_TOPIC, self._joint_stream_callback, qos,
            callback_group=self._command_callback_group
        )
        self.get_logger().info(f"listening on {_JOINT_STREAM_TOPIC}")

        self._service = self.create_service(
            ReadJointPositions, _READ_POSITIONS_SERVICE, self._handle_read_joint_positions,
            callback_group=self._service_callback_group
        )
        self.get_logger().info(f"serving {_READ_POSITIONS_SERVICE}")

        self._fast_service = self.create_service(
            ReadJointPositions, _READ_POSITIONS_FAST_SERVICE, self._handle_read_joint_positions_fast,
            callback_group=self._service_callback_group
        )
        self.get_logger().info(f"serving {_READ_POSITIONS_FAST_SERVICE}")

        self._is_moving_service = self.create_service(
            IsMoving, _IS_MOVING_SERVICE, self._handle_is_moving,
            callback_group=self._service_callback_group
        )
        self.get_logger().info(f"serving {_IS_MOVING_SERVICE}")

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
        if self._is_moving_service is not None:
            self.destroy_service(self._is_moving_service)
            self._is_moving_service = None
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        # Under the lock: a move thread can still be inside _transact, and
        # pulling the transport out from under it would raise AttributeError
        # (not Esp32TransportError) and take down an executor thread.
        with self._transport_lock:
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
        if self._is_moving_service is not None:
            self.destroy_service(self._is_moving_service)
            self._is_moving_service = None
        # Under the lock: a move thread can still be inside _transact, and
        # pulling the transport out from under it would raise AttributeError
        # (not Esp32TransportError) and take down an executor thread.
        with self._transport_lock:
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
        use_blocking_move = self.get_parameter("use_blocking_move").value
        target_positions_raw = [_angle_deg_to_position_raw(a, i) for i, a in enumerate(angles_deg)]
        command = {
            "cmd": "move" if use_blocking_move else "move_async",
            "positions": target_positions_raw,
            "duration_ms": duration_ms,
        }
        try:
            response = self._transact(command)
        except Esp32TransportError as exc:
            self.get_logger().error(f"ESP32 command failed: {exc}")
            return

        status = response.get("status")
        if status == "busy":
            # The firmware refuses a second shaped move rather than changing the
            # arm's direction mid-swing. Nothing queues it: this command is dropped.
            self.get_logger().warn(
                "move rejected - the ESP32 is still executing a previous move. "
                "Commands are not queued; re-send once the arm has stopped.")
            return
        if status != "ok":
            # Includes the firmware refusing to start a move whose start pose it
            # could not read: it never guesses one, so the arm has not moved.
            self.get_logger().error(f"ESP32 refused the move, the arm did not move: {response}")
            return

        self.get_logger().info(f"sent {angles_deg} deg, ESP32 ack ok")
        # The legacy blocking `move` only acks once the trajectory is done, so
        # there is nothing to wait for on that path.
        if not use_blocking_move and not self._wait_for_motion_complete():
            return
        self._check_move_completed(target_positions_raw, angles_deg)

    def _transact(self, command: dict) -> dict:
        """Send one command and read its reply with the transport held exclusively.

        The ESP32 answers one line per command and there is no correlation ID, so
        two threads with commands in flight at once would mis-pair replies. The
        lock covers the send/read pair and nothing wider: anything needing several
        round trips takes it once per round trip, so other callbacks get in between.
        """
        with self._transport_lock:
            if self._transport is None:
                raise Esp32TransportError("transport is not connected")
            try:
                self._transport.send_command(command)
                return self._transport.read_response()
            except Esp32TransportError:
                # A failed exchange breaks the request/reply pairing for good (see
                # Esp32Transport.reconnect), so the socket is rebuilt before anyone
                # else is allowed a turn. Without this, one timeout makes every
                # later command answer the previous one's question - including a
                # `status` poll answering with a stale move ack.
                try:
                    self._transport.reconnect()
                    self.get_logger().warn(
                        "transport error - reconnected to the ESP32. Any move in progress was "
                        "abandoned when the socket closed; the arm holds where it stopped.")
                except Esp32TransportError as reconnect_error:
                    self.get_logger().error(
                        f"transport error, and reconnecting failed: {reconnect_error}")
                raise

    def _wait_for_motion_complete(self) -> bool:
        """Poll the ESP32 until it reports the shaped trajectory has finished.

        This is what replaces the old blocking ack. Releasing the lock between
        polls is the point of the exercise - that gap is when a position read can
        be served, which is what keeps /joint_states alive during a move.

        Returns False if the move could not be confirmed finished, in which case
        the caller skips the fault check rather than judging a moving arm.
        """
        deadline = time.monotonic() + _MOVE_COMPLETION_TIMEOUT_SEC
        while time.monotonic() < deadline:
            try:
                response = self._transact({"cmd": "status"})
            except Esp32TransportError as exc:
                self.get_logger().error(f"status poll failed, skipping move fault check: {exc}")
                return False
            if response.get("status") != "ok" or "moving" not in response:
                # A reply with no `moving` field is "cannot tell", never "stopped":
                # treating it as stopped would run the fault check against a moving
                # arm, which is the failure this poll exists to avoid.
                self.get_logger().warn(f"unexpected status reply, skipping move fault check: {response}")
                return False
            if not response["moving"]:
                self._log_move_timing(response)
                return True
            time.sleep(_STATUS_POLL_INTERVAL_SEC)

        self.get_logger().error(
            f"ESP32 still reports moving after {_MOVE_COMPLETION_TIMEOUT_SEC:.0f}s - skipping "
            "the move fault check. The arm may still be in motion.")
        return False

    def _log_move_timing(self, status_reply: dict) -> None:
        # Diagnostic only (DECISIONS.md D13): whether shaped steps went out late
        # enough for the servos to stop between them. Older firmware and the
        # simulator send no "timing" field; that is not an error.
        timing = status_reply.get("timing")
        if isinstance(timing, dict):
            self.get_logger().info(f"move timing: {timing}")

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
            response = self._transact(command)
        except Esp32TransportError as exc:
            self.get_logger().warn(f"ESP32 servo command failed: {exc}")
            return

        if response.get("status") != "ok":
            self.get_logger().warn(f"ESP32 rejected servo command: {response}")

    def _read_positions_with_retry(self, max_attempts: int = _MAX_READ_RETRIES) -> list[int | None]:
        merged_positions_raw: list[int | None] = [None] * _EXPECTED_SERVO_COUNT
        for _attempt in range(max_attempts):
            try:
                response = self._transact({"cmd": "read_positions"})
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

    def _handle_is_moving(
        self, request: IsMoving.Request, response: IsMoving.Response
    ) -> IsMoving.Response:
        # Asked of the ESP32 on every call rather than answered from a flag kept
        # here: the firmware is the authoritative source for whether the arm is
        # moving, and this node can restart while the arm does not (CLAUDE.md).
        try:
            reply = self._transact({"cmd": "status"})
        except Esp32TransportError as exc:
            self.get_logger().warn(f"is_moving query failed: {exc}")
            response.is_moving = False
            response.valid = False
            return response

        if reply.get("status") != "ok" or "moving" not in reply:
            # valid=False, not is_moving=False - callers treat unknown as moving.
            self.get_logger().warn(f"unexpected status reply: {reply}")
            response.is_moving = False
            response.valid = False
            return response

        response.is_moving = bool(reply["moving"])
        response.valid = True
        return response


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    try:
        model = _load_joint_limits()
    except robot_model.RobotModelError as error:
        rclpy.logging.get_logger("esp32_bridge_node").error(f"refusing to start: {error}")
        rclpy.shutdown()
        raise SystemExit(1)
    node = Esp32BridgeNode()
    # The banner carries the "NOT measured" warning for a placeholder model; at
    # info it would vanish under --log-level warn, which is exactly when it matters.
    if model.limits_source == robot_model._LIMITS_SOURCE_MEASURED:
        node.get_logger().info(robot_model.describe(model))
    else:
        node.get_logger().warn(robot_model.describe(model))
    executor = MultiThreadedExecutor(num_threads=_EXECUTOR_THREAD_COUNT)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
