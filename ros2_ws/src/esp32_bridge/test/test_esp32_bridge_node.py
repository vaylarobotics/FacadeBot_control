# Needs the ROS2 workspace sourced (imports rclpy and facade_msgs), like
# facade_control's node tests. Nothing here touches the ESP32: the node's
# transport is replaced with _FakeTransport, which records what would have
# been sent and plays back scripted replies.

import math

import pytest

import rclpy

from esp32_bridge import esp32_bridge_node as bridge
from esp32_bridge.transport import Esp32TransportError
from facadebot_description import robot_model

_OK = {"status": "ok"}


def _maybe_raise(reply):
    if isinstance(reply, Exception):
        raise reply
    return reply


class _FakeTransport:
    """Stands in for the ESP32, answering by command name rather than from one
    FIFO of replies.

    A move now costs a variable number of `status` round trips (the bridge polls
    until the firmware says it has stopped), so a single queue would make every
    move-path test depend on how many times it happened to poll. `read_positions`
    keeps a strict queue - its call count is deterministic, fixed by the retry
    budget, and the dead-bus test relies on answering every attempt.
    """

    def __init__(self, read_replies=(), move_reply=None, servo_reply=None, moving_polls=0,
                 status_reply=None):
        self.sent = []
        self.reconnects = 0
        self.status_reply = status_reply  # overrides the moving_polls behaviour when set
        self.read_replies = list(read_replies)
        self.move_reply = _OK if move_reply is None else move_reply
        self.servo_reply = _OK if servo_reply is None else servo_reply
        self.moving_polls = moving_polls  # status polls answered "still moving" before "stopped"
        self._pending = None

    def send_command(self, command: dict) -> None:
        self.sent.append(command)
        self._pending = command

    def read_response(self) -> dict:
        name = self._pending["cmd"]
        if name == "status":
            if self.status_reply is not None:
                return _maybe_raise(self.status_reply)
            if self.moving_polls > 0:
                self.moving_polls -= 1
                return {"status": "ok", "moving": True}
            return {"status": "ok", "moving": False}
        if name == "read_positions":
            assert self.read_replies, "the node read positions more times than the test scripted"
            return _maybe_raise(self.read_replies.pop(0))
        if name == "servo":
            return _maybe_raise(self.servo_reply)
        return _maybe_raise(self.move_reply)

    def reconnect(self) -> None:
        self.reconnects += 1

    def status_polls(self) -> int:
        return sum(1 for command in self.sent if command["cmd"] == "status")


class _FakeLogger:
    def __init__(self):
        self.errors = []
        self.warnings = []
        self.infos = []

    def error(self, msg, **kwargs):
        self.errors.append(msg)

    def warn(self, msg, **kwargs):
        self.warnings.append(msg)

    def info(self, msg, **kwargs):
        self.infos.append(msg)


@pytest.fixture(autouse=True)
def loaded_limits():
    """Every test runs against the real active model's limits, exactly as the
    node would at startup."""
    model = bridge._load_joint_limits()
    yield model


@pytest.fixture
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def node(ros_context, monkeypatch):
    monkeypatch.setattr(bridge, "_FAULT_CHECK_MARGIN_MS", 0)
    monkeypatch.setattr(bridge, "_STATUS_POLL_INTERVAL_SEC", 0.0)
    node = bridge.Esp32BridgeNode()
    logger = _FakeLogger()
    monkeypatch.setattr(node, "get_logger", lambda: logger)
    node.fake_logger = logger
    yield node
    node.destroy_node()


def _centers_raw() -> list[int]:
    return list(bridge._JOINT_CENTER_RAW)


# --- bounds gate -------------------------------------------------------------

def test_bounds_gate_refuses_to_run_without_limits(monkeypatch):
    monkeypatch.setattr(bridge, "_joint_limits_deg", None)
    with pytest.raises(robot_model.RobotModelError):
        bridge._check_joint_bounds([0.0, 0.0, 0.0, 0.0])


def test_bounds_gate_passes_zero_and_limits_inclusive(loaded_limits):
    assert bridge._check_joint_bounds([0.0, 0.0, 0.0, 0.0]) == []
    lower = [limit[0] for limit in loaded_limits.joint_limits_deg]
    upper = [limit[1] for limit in loaded_limits.joint_limits_deg]
    assert bridge._check_joint_bounds(lower) == []
    assert bridge._check_joint_bounds(upper) == []


def test_bounds_gate_names_every_violating_joint(loaded_limits):
    upper = [limit[1] for limit in loaded_limits.joint_limits_deg]
    angles_deg = [0.0, upper[1] + 0.1, 0.0, upper[3] + 50.0]
    violations = bridge._check_joint_bounds(angles_deg)
    assert len(violations) == 2
    assert violations[0].startswith("joint 1:")
    assert violations[1].startswith("joint 3:")


# --- degree <-> raw conversion -----------------------------------------------

def test_zero_degrees_is_the_measured_center():
    for joint_index in range(bridge._EXPECTED_SERVO_COUNT):
        assert bridge._angle_deg_to_position_raw(0.0, joint_index) == bridge._JOINT_CENTER_RAW[joint_index]


def test_conversion_round_trips_within_one_servo_count():
    one_count_deg = bridge._SERVO_ANGLE_MAX_DEG / bridge._POSITION_MAX_RAW
    for joint_index in range(bridge._EXPECTED_SERVO_COUNT):
        for angle_deg in (-100.0, -33.3, 0.0, 12.7, 100.0):
            raw = bridge._angle_deg_to_position_raw(angle_deg, joint_index)
            back_deg = bridge._position_raw_to_angle_deg(raw, joint_index)
            assert abs(back_deg - angle_deg) <= one_count_deg / 2 + 1e-9


def test_conversion_clamps_to_servo_mechanical_range():
    assert bridge._angle_deg_to_position_raw(-1000.0, 0) == bridge._POSITION_MIN_RAW
    assert bridge._angle_deg_to_position_raw(1000.0, 0) == bridge._POSITION_MAX_RAW


def test_center_raw_matches_center_radians():
    for joint_index, center_rad in enumerate(bridge._JOINT_CENTER_RAD):
        expected_raw = round(math.degrees(center_rad) * bridge._POSITION_MAX_RAW / bridge._SERVO_ANGLE_MAX_DEG)
        assert bridge._JOINT_CENTER_RAW[joint_index] == expected_raw


# --- move path ---------------------------------------------------------------

def test_move_sends_raw_centers_for_zero_command(node):
    transport = _FakeTransport(read_replies=[{"status": "ok", "positions": _centers_raw()}])
    node._transport = transport

    node._send_move_command([0.0, 0.0, 0.0, 0.0])

    assert transport.sent[0] == {
        "cmd": "move_async",
        "positions": _centers_raw(),
        "duration_ms": bridge._DEFAULT_MOVE_DURATION_MS,
    }
    assert transport.sent[-1] == {"cmd": "read_positions"}
    assert node.fake_logger.errors == []


def test_out_of_bounds_move_sends_nothing(node, loaded_limits):
    transport = _FakeTransport(read_replies=[])
    node._transport = transport
    upper = [limit[1] for limit in loaded_limits.joint_limits_deg]

    node._send_move_command([upper[0] + 1.0, 0.0, 0.0, 0.0])

    assert transport.sent == []
    assert any("out of bounds" in msg for msg in node.fake_logger.errors)


def test_move_reports_fault_when_a_joint_misses_target(node):
    far_raw = _centers_raw()
    far_raw[2] += bridge._POSITION_TOLERANCE_RAW + 1
    transport = _FakeTransport(read_replies=[{"status": "ok", "positions": far_raw}])
    node._transport = transport

    node._send_move_command([0.0, 0.0, 0.0, 0.0])

    assert any("move fault" in msg and "joint 2" in msg for msg in node.fake_logger.errors)


def test_move_reports_dead_bus_when_every_joint_is_silent(node):
    # The retry budget is a default argument, fixed at import time, so the fake
    # has to answer every one of the real attempts.
    silent = {"status": "ok", "positions": [None] * bridge._EXPECTED_SERVO_COUNT}
    transport = _FakeTransport(read_replies=[silent] * bridge._MAX_READ_RETRIES)
    node._transport = transport

    node._send_move_command([0.0, 0.0, 0.0, 0.0])

    assert any("servo bus fault" in msg for msg in node.fake_logger.errors)


def test_transport_error_on_move_is_logged_not_raised(node):
    transport = _FakeTransport(move_reply=Esp32TransportError("link down"))
    node._transport = transport

    node._send_move_command([0.0, 0.0, 0.0, 0.0])

    assert any("link down" in msg for msg in node.fake_logger.errors)


def test_joint_cmd_callback_converts_radians_and_ignores_wrong_length(node):
    transport = _FakeTransport(read_replies=[{"status": "ok", "positions": _centers_raw()}])
    node._transport = transport

    from sensor_msgs.msg import JointState
    msg = JointState()
    msg.position = [0.0, 0.0, 0.0]
    node._joint_cmd_callback(msg)
    assert transport.sent == []

    msg.position = [0.0, 0.0, 0.0, 0.0]
    node._joint_cmd_callback(msg)
    assert transport.sent[0]["cmd"] == "move_async"


# --- stream path -------------------------------------------------------------

def test_stream_setpoint_uses_servo_command_and_stream_duration(node):
    transport = _FakeTransport()
    node._transport = transport

    node._send_servo_command([0.0, 0.0, 0.0, 0.0])

    assert transport.sent == [{
        "cmd": "servo",
        "positions": _centers_raw(),
        "duration_ms": bridge._DEFAULT_SERVO_MOVE_DURATION_MS,
    }]


def test_stream_setpoint_is_bounds_checked_too(node, loaded_limits):
    transport = _FakeTransport(read_replies=[])
    node._transport = transport
    lower = [limit[0] for limit in loaded_limits.joint_limits_deg]

    node._send_servo_command([0.0, lower[1] - 1.0, 0.0, 0.0])

    assert transport.sent == []
    assert any("out of bounds" in msg for msg in node.fake_logger.errors)


# --- position readback -------------------------------------------------------

def test_readback_merges_missing_joints_across_attempts(node):
    transport = _FakeTransport(read_replies=[
        {"status": "ok", "positions": [500, None, 500, 500]},
        {"status": "ok", "positions": [None, 510, None, None]},
    ])
    node._transport = transport

    assert node._read_positions_with_retry() == [500, 510, 500, 500]
    assert len(transport.sent) == 2


def test_readback_keeps_first_reading_when_later_attempts_differ(node):
    transport = _FakeTransport(read_replies=[
        {"status": "ok", "positions": [500, None, 500, 500]},
        {"status": "ok", "positions": [999, 510, 999, 999]},
    ])
    node._transport = transport

    assert node._read_positions_with_retry() == [500, 510, 500, 500]


def test_fast_readback_makes_exactly_one_attempt(node):
    transport = _FakeTransport(read_replies=[{"status": "ok", "positions": [500, None, 500, 500]}])
    node._transport = transport

    request = bridge.ReadJointPositions.Request()
    response = node._handle_read_joint_positions_fast(request, bridge.ReadJointPositions.Response())

    assert len(transport.sent) == 1
    assert response.all_valid is False
    assert math.isnan(response.positions_deg[1])


def test_full_readback_service_converts_to_center_relative_degrees(node):
    transport = _FakeTransport(read_replies=[{"status": "ok", "positions": _centers_raw()}])
    node._transport = transport

    response = node._handle_read_joint_positions(
        bridge.ReadJointPositions.Request(), bridge.ReadJointPositions.Response())

    assert response.all_valid is True
    assert list(response.positions_deg) == pytest.approx([0.0, 0.0, 0.0, 0.0])


# --- non-blocking move: status polling, busy, rollback -----------------------

def test_move_waits_for_status_to_report_stopped_before_checking(node):
    # Two polls say "still moving", the third says stopped. The fault check must
    # not read positions until then - reading a moving arm would report a fault
    # on every move, which is exactly what the blocking ack used to prevent.
    transport = _FakeTransport(
        read_replies=[{"status": "ok", "positions": _centers_raw()}], moving_polls=2)
    node._transport = transport

    node._send_move_command([0.0, 0.0, 0.0, 0.0])

    assert transport.status_polls() == 3
    names = [command["cmd"] for command in transport.sent]
    assert names.index("read_positions") > max(
        index for index, name in enumerate(names) if name == "status")
    assert node.fake_logger.errors == []


def test_move_logs_firmware_step_timing_when_reported(node):
    timing = {"steps": 6, "late_steps": 1, "worst_idle_ms": 230}
    node._transport = _FakeTransport(
        read_replies=[{"status": "ok", "positions": _centers_raw()}],
        status_reply={"status": "ok", "moving": False, "timing": timing})

    node._send_move_command([0.0, 0.0, 0.0, 0.0])

    assert sum("move timing:" in msg for msg in node.fake_logger.infos) == 1
    assert node.fake_logger.errors == []


@pytest.mark.parametrize("status_reply", [
    {"status": "ok", "moving": False},                        # older firmware, simulator
    {"status": "ok", "moving": False, "timing": "garbage"},  # malformed field
])
def test_move_without_usable_timing_logs_nothing_and_still_checks(node, status_reply):
    transport = _FakeTransport(
        read_replies=[{"status": "ok", "positions": _centers_raw()}], status_reply=status_reply)
    node._transport = transport

    node._send_move_command([0.0, 0.0, 0.0, 0.0])

    assert not any("move timing:" in msg for msg in node.fake_logger.infos)
    assert [command["cmd"] for command in transport.sent][-1] == "read_positions"
    assert node.fake_logger.errors == [] and node.fake_logger.warnings == []


def test_move_rejected_as_busy_is_logged_and_not_checked(node):
    transport = _FakeTransport(move_reply={"status": "busy"})
    node._transport = transport

    node._send_move_command([0.0, 0.0, 0.0, 0.0])

    assert [command["cmd"] for command in transport.sent] == ["move_async"]
    assert any("still executing a previous move" in msg for msg in node.fake_logger.warnings)
    assert node.fake_logger.errors == []


def test_move_skips_fault_check_when_the_arm_never_stops(node, monkeypatch):
    monkeypatch.setattr(bridge, "_MOVE_COMPLETION_TIMEOUT_SEC", 0.05)
    transport = _FakeTransport(read_replies=[], moving_polls=10_000)
    node._transport = transport

    node._send_move_command([0.0, 0.0, 0.0, 0.0])

    assert "read_positions" not in [command["cmd"] for command in transport.sent]
    assert any("still reports moving" in msg for msg in node.fake_logger.errors)


def test_blocking_move_rollback_sends_move_and_never_polls_status(node):
    from rclpy.parameter import Parameter
    node.set_parameters([Parameter("use_blocking_move", Parameter.Type.BOOL, True)])
    transport = _FakeTransport(read_replies=[{"status": "ok", "positions": _centers_raw()}])
    node._transport = transport

    node._send_move_command([0.0, 0.0, 0.0, 0.0])

    names = [command["cmd"] for command in transport.sent]
    assert names == ["move", "read_positions"]
    assert transport.status_polls() == 0


# --- is_moving service -------------------------------------------------------

def test_is_moving_reports_the_firmware_answer(node):
    transport = _FakeTransport(moving_polls=1)
    node._transport = transport

    moving = node._handle_is_moving(bridge.IsMoving.Request(), bridge.IsMoving.Response())
    assert moving.is_moving is True
    assert moving.valid is True

    stopped = node._handle_is_moving(bridge.IsMoving.Request(), bridge.IsMoving.Response())
    assert stopped.is_moving is False
    assert stopped.valid is True


def test_is_moving_reports_invalid_when_the_link_is_down(node):
    class _DeadTransport(_FakeTransport):
        def send_command(self, command):
            raise Esp32TransportError("link down")

    node._transport = _DeadTransport()

    response = node._handle_is_moving(bridge.IsMoving.Request(), bridge.IsMoving.Response())

    # valid=False, not is_moving=False: callers must be able to tell "stopped"
    # apart from "no idea", since trajectory_node treats unknown as still moving.
    assert response.valid is False
    assert response.is_moving is False


# --- status replies that cannot be trusted ----------------------------------

def test_status_reply_without_moving_field_skips_the_fault_check(node):
    # "ok" with no `moving` key is "cannot tell", not "stopped". Reading back
    # here would judge a still-moving arm and fault on every move.
    transport = _FakeTransport(read_replies=[], status_reply=_OK)
    node._transport = transport

    node._send_move_command([0.0, 0.0, 0.0, 0.0])

    assert "read_positions" not in [command["cmd"] for command in transport.sent]
    assert any("unexpected status reply" in msg for msg in node.fake_logger.warnings)


def test_is_moving_reports_invalid_when_the_reply_has_no_moving_field(node):
    transport = _FakeTransport(status_reply=_OK)
    node._transport = transport

    response = node._handle_is_moving(bridge.IsMoving.Request(), bridge.IsMoving.Response())

    assert response.valid is False
    assert response.is_moving is False


def test_transport_error_rebuilds_the_connection(node):
    # One failed exchange leaves the ESP32's late reply queued as the answer to
    # the next command. The socket has to be replaced, not reused.
    transport = _FakeTransport(move_reply=Esp32TransportError("link down"))
    node._transport = transport

    node._send_move_command([0.0, 0.0, 0.0, 0.0])

    assert transport.reconnects == 1
    assert any("link down" in msg for msg in node.fake_logger.errors)


def test_command_and_service_callbacks_are_in_different_groups(node):
    # The two command topics must not share the reentrant group the services use:
    # concurrent joint commands would put two sources on the servo bus at once.
    from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

    assert isinstance(node._command_callback_group, MutuallyExclusiveCallbackGroup)
    assert isinstance(node._service_callback_group, ReentrantCallbackGroup)
    assert node._command_callback_group is not node._service_callback_group


def test_move_refused_for_unreadable_start_pose_is_an_error_and_not_waited_on(node):
    node._transport = _FakeTransport(
        move_reply={"status": "error", "msg": "start position unreadable - move refused"})
    node._send_move_command([0.0, 0.0, 0.0, 0.0])
    assert node._transport.status_polls() == 0
    assert not any(c["cmd"] == "read_positions" for c in node._transport.sent)
    assert any("did not move" in e for e in node.fake_logger.errors)
