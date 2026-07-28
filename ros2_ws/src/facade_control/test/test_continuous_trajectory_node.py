# Unlike test_trajectory_planning.py, this file imports rclpy/facade_msgs and so
# needs the ROS2 workspace sourced to run - it can't be run with bare pytest alone.

import threading
import time
from types import SimpleNamespace

import pytest

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState

from facade_control import continuous_trajectory_node, kinematics
from facade_msgs.msg import Waypoint
from facade_msgs.srv import ReadJointPositions

_SEED_ANGLES_DEG = (10.0, -20.0, 15.0, 5.0)


class _FakeGoalHandle:
    """Stands in for rclpy's real action GoalHandle so _execute_callback can be
    tested directly, without a full ActionServer/ActionClient pair."""

    def __init__(self, waypoints, tool_speed_mmps=200.0, corner_blend_m=0.0):
        self.request = SimpleNamespace(
            waypoints=waypoints,
            tool_speed_mmps=tool_speed_mmps,
            corner_blend_m=corner_blend_m,
        )
        self.is_cancel_requested = False
        self.feedback_messages = []
        self.status = None

    def publish_feedback(self, feedback_msg):
        self.feedback_messages.append(feedback_msg)

    def succeed(self):
        self.status = "succeeded"

    def abort(self):
        self.status = "aborted"

    def canceled(self):
        self.status = "canceled"


class _StubNode(Node):
    """Serves read_joint_positions with a test-scripted response and captures every
    setpoint published to the joint-stream topic, standing in for esp32_bridge_node."""

    def __init__(self, read_positions_handler):
        super().__init__("stub_node")
        self._read_positions_handler = read_positions_handler
        self.stream_messages = []
        self.create_service(
            ReadJointPositions,
            continuous_trajectory_node._READ_JOINT_POSITIONS_SERVICE,
            self._handle_read_positions,
        )
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(
            JointState, continuous_trajectory_node._JOINT_STREAM_TOPIC, self._on_stream, qos
        )

    def _handle_read_positions(self, request, response):
        return self._read_positions_handler(request, response)

    def _on_stream(self, msg):
        self.stream_messages.append(msg)


@pytest.fixture
def ros_context(monkeypatch):
    # Faster streaming/feedback timing for the tests - production defaults untouched.
    monkeypatch.setattr(continuous_trajectory_node, "_DEFAULT_STREAM_PERIOD_SEC", 0.02)
    monkeypatch.setattr(continuous_trajectory_node, "_FEEDBACK_PERIOD_SEC", 0.02)
    monkeypatch.setattr(continuous_trajectory_node, "_SERVICE_WAIT_TIMEOUT_SEC", 0.5)
    monkeypatch.setattr(continuous_trajectory_node, "_READ_CALL_TIMEOUT_SEC", 0.5)

    rclpy.init()
    yield
    rclpy.shutdown()


def _waypoint_from_angles(angles_deg) -> Waypoint:
    x_m, y_m, z_m, tool_angle_deg = kinematics.forward_kinematics(*angles_deg)
    waypoint = Waypoint()
    waypoint.x_m = x_m
    waypoint.y_m = y_m
    waypoint.z_m = z_m
    waypoint.tool_angle_deg = tool_angle_deg
    return waypoint


def _valid_read_handler(request, response):
    response.positions_deg = list(_SEED_ANGLES_DEG)
    response.all_valid = True
    return response


def _start_stub(read_positions_handler) -> _StubNode:
    stub_node = _StubNode(read_positions_handler)
    threading.Thread(target=rclpy.spin, args=(stub_node,), daemon=True).start()
    return stub_node


def test_execute_callback_streams_setpoints_and_succeeds(ros_context):
    stub = _start_stub(_valid_read_handler)

    node = continuous_trajectory_node.ContinuousTrajectoryNode()
    try:
        waypoints = [
            _waypoint_from_angles(_SEED_ANGLES_DEG),
            _waypoint_from_angles((15.0, -25.0, 18.0, 8.0)),
        ]
        goal_handle = _FakeGoalHandle(waypoints)
        result = node._execute_callback(goal_handle)

        assert result.success
        assert result.fraction_completed == pytest.approx(1.0)
        assert goal_handle.status == "succeeded"
        assert len(goal_handle.feedback_messages) >= 1

        time.sleep(0.1)  # let the stub's subscription drain the last setpoints
        assert len(stub.stream_messages) >= 2
        assert all(len(m.position) == 4 for m in stub.stream_messages)
    finally:
        node.destroy_node()


def test_execute_callback_aborts_when_current_position_unreadable(ros_context):
    def read_handler(request, response):
        response.positions_deg = [0.0, 0.0, 0.0, 0.0]
        response.all_valid = False  # a joint had no reading
        return response

    stub = _start_stub(read_handler)

    node = continuous_trajectory_node.ContinuousTrajectoryNode()
    try:
        waypoints = [
            _waypoint_from_angles(_SEED_ANGLES_DEG),
            _waypoint_from_angles((15.0, -25.0, 18.0, 8.0)),
        ]
        result = node._execute_callback(_FakeGoalHandle(waypoints))

        assert not result.success
        assert "current joint positions" in result.message
        time.sleep(0.1)
        assert stub.stream_messages == []
    finally:
        node.destroy_node()


def test_execute_callback_aborts_before_moving_when_path_unreachable(ros_context):
    stub = _start_stub(_valid_read_handler)

    node = continuous_trajectory_node.ContinuousTrajectoryNode()
    try:
        reachable = _waypoint_from_angles(_SEED_ANGLES_DEG)
        unreachable = Waypoint()
        unreachable.x_m, unreachable.y_m, unreachable.z_m, unreachable.tool_angle_deg = 10.0, 0.0, 0.2, 0.0

        goal_handle = _FakeGoalHandle([reachable, unreachable])
        result = node._execute_callback(goal_handle)

        assert not result.success
        assert goal_handle.status == "aborted"
        assert "cannot plan" in result.message
        time.sleep(0.1)
        assert stub.stream_messages == []  # nothing streamed - aborted during planning
    finally:
        node.destroy_node()


def test_execute_callback_cancel_stops_the_stream(ros_context):
    stub = _start_stub(_valid_read_handler)

    node = continuous_trajectory_node.ContinuousTrajectoryNode()
    try:
        # A slow speed over a real path makes many setpoints, so a cancel can land mid-stream.
        waypoints = [
            _waypoint_from_angles(_SEED_ANGLES_DEG),
            _waypoint_from_angles((20.0, -30.0, 22.0, 12.0)),
        ]
        goal_handle = _FakeGoalHandle(waypoints, tool_speed_mmps=20.0)

        def cancel_after_first_setpoint():
            while len(stub.stream_messages) < 1:
                time.sleep(0.005)
            goal_handle.is_cancel_requested = True

        canceler = threading.Thread(target=cancel_after_first_setpoint, daemon=True)
        canceler.start()

        result = node._execute_callback(goal_handle)
        canceler.join(timeout=2.0)

        assert not result.success
        assert goal_handle.status == "canceled"
        assert result.fraction_completed < 1.0
    finally:
        node.destroy_node()
