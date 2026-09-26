# Unlike test_kinematics.py, this file imports rclpy/facade_msgs and so needs
# the ROS2 workspace sourced to run - it can't be run with bare pytest alone.

import threading
import time
from types import SimpleNamespace

import pytest

import rclpy
from rclpy.node import Node

from facade_control import trajectory_node
from facade_msgs.msg import Waypoint
from facade_msgs.srv import IsMoving, MoveToPose, ReadJointPositions


def test_within_tolerance_true_when_all_joints_close():
    assert trajectory_node._within_tolerance((1.0, 2.0, 3.0, 4.0), (1.0, 2.0, 3.0, 4.0), tolerance_deg=0.5)


def test_within_tolerance_false_when_one_joint_far():
    assert not trajectory_node._within_tolerance((1.0, 2.0, 3.0, 100.0), (1.0, 2.0, 3.0, 4.0), tolerance_deg=0.5)


def test_within_tolerance_boundary_is_inclusive():
    assert trajectory_node._within_tolerance((5.0,), (0.0,), tolerance_deg=5.0)


class _FakeGoalHandle:
    """Stands in for rclpy's real action GoalHandle so _execute_callback can be
    tested directly, without needing a full ActionServer/ActionClient pair."""

    def __init__(self, waypoints):
        self.request = SimpleNamespace(waypoints=waypoints)
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


def _stopped_handler(request, response):
    """Default is_moving stub: the arm is never moving. Tests that care about
    the arm still being in motion pass their own handler."""
    response.is_moving = False
    response.valid = True
    return response


class _StubServicesNode(Node):
    """Serves move_to_pose/read_joint_positions/is_moving with test-scripted
    responses, standing in for facade_control_node and esp32_bridge_node."""

    def __init__(self, move_to_pose_handler, read_positions_handler, is_moving_handler):
        super().__init__("stub_services_node")
        self._move_to_pose_handler = move_to_pose_handler
        self._read_positions_handler = read_positions_handler
        self._is_moving_handler = is_moving_handler
        self.create_service(MoveToPose, trajectory_node._MOVE_TO_POSE_SERVICE, self._handle_move_to_pose)
        self.create_service(
            ReadJointPositions, trajectory_node._READ_JOINT_POSITIONS_SERVICE, self._handle_read_positions
        )
        self.create_service(IsMoving, trajectory_node._IS_MOVING_SERVICE, self._handle_is_moving)

    def _handle_move_to_pose(self, request, response):
        return self._move_to_pose_handler(request, response)

    def _handle_read_positions(self, request, response):
        return self._read_positions_handler(request, response)

    def _handle_is_moving(self, request, response):
        return self._is_moving_handler(request, response)


@pytest.fixture
def ros_context(monkeypatch):
    # Faster timing for the stall/poll tests below - doesn't touch production defaults.
    monkeypatch.setattr(trajectory_node, "_WAYPOINT_STALL_TIMEOUT_SEC", 1.0)
    monkeypatch.setattr(trajectory_node, "_POLL_CALL_TIMEOUT_SEC", 0.5)
    monkeypatch.setattr(trajectory_node, "_POLL_RETRY_INTERVAL_SEC", 0.05)
    monkeypatch.setattr(trajectory_node, "_SERVICE_WAIT_TIMEOUT_SEC", 0.5)

    rclpy.init()
    yield
    rclpy.shutdown()


def _make_waypoint(x_m=0.2, y_m=0.0, z_m=0.2, tool_angle_deg=0.0) -> Waypoint:
    waypoint = Waypoint()
    waypoint.x_m = x_m
    waypoint.y_m = y_m
    waypoint.z_m = z_m
    waypoint.tool_angle_deg = tool_angle_deg
    return waypoint


def _start_stub(move_to_pose_handler, read_positions_handler,
                is_moving_handler=_stopped_handler) -> _StubServicesNode:
    stub_node = _StubServicesNode(move_to_pose_handler, read_positions_handler, is_moving_handler)
    threading.Thread(target=rclpy.spin, args=(stub_node,), daemon=True).start()
    return stub_node


def _make_node() -> trajectory_node.TrajectoryNode:
    return trajectory_node.TrajectoryNode()


def test_execute_callback_happy_path(ros_context):
    target_angles_deg = (0.0, 10.0, -10.0, 5.0)
    read_call_count = {"n": 0}

    def move_to_pose_handler(request, response):
        response.success = True
        response.message = "ok"
        response.solved_angles_deg = list(target_angles_deg)
        return response

    def read_positions_handler(request, response):
        read_call_count["n"] += 1
        # Not yet arrived for the first two polls, then matches.
        if read_call_count["n"] < 3:
            response.positions_deg = [0.0, 0.0, 0.0, 0.0]
        else:
            response.positions_deg = list(target_angles_deg)
        response.all_valid = True
        return response

    _start_stub(move_to_pose_handler, read_positions_handler)

    node = _make_node()
    try:
        goal_handle = _FakeGoalHandle([_make_waypoint()])
        result = node._execute_callback(goal_handle)

        assert result.success
        assert result.waypoints_completed == 1
        assert goal_handle.status == "succeeded"
        assert len(goal_handle.feedback_messages) == 1
        assert goal_handle.feedback_messages[0].current_waypoint_index == 0
        assert goal_handle.feedback_messages[0].total_waypoints == 1
    finally:
        node.destroy_node()


def test_execute_callback_ik_failure_aborts_without_polling(ros_context):
    read_positions_calls = {"n": 0}

    def move_to_pose_handler(request, response):
        response.success = False
        response.message = "IK failure: position not reachable"
        return response

    def read_positions_handler(request, response):
        read_positions_calls["n"] += 1
        response.positions_deg = [0.0, 0.0, 0.0, 0.0]
        response.all_valid = True
        return response

    _start_stub(move_to_pose_handler, read_positions_handler)

    node = _make_node()
    try:
        goal_handle = _FakeGoalHandle([_make_waypoint()])
        result = node._execute_callback(goal_handle)

        assert not result.success
        assert result.waypoints_completed == 0
        assert goal_handle.status == "aborted"
        assert "not reachable" in result.message
        assert read_positions_calls["n"] == 0
    finally:
        node.destroy_node()


def test_execute_callback_stall_times_out(ros_context):
    def move_to_pose_handler(request, response):
        response.success = True
        response.message = "ok"
        response.solved_angles_deg = [0.0, 10.0, -10.0, 5.0]
        return response

    def read_positions_handler(request, response):
        response.positions_deg = [0.0, 0.0, 0.0, 0.0]  # never matches the target
        response.all_valid = True
        return response

    _start_stub(move_to_pose_handler, read_positions_handler)

    node = _make_node()
    try:
        goal_handle = _FakeGoalHandle([_make_waypoint()])
        result = node._execute_callback(goal_handle)

        assert not result.success
        assert result.waypoints_completed == 0
        assert goal_handle.status == "aborted"
        assert "stall" in result.message
    finally:
        node.destroy_node()


def test_execute_callback_cancel_mid_poll(ros_context):
    read_call_count = {"n": 0}

    def move_to_pose_handler(request, response):
        response.success = True
        response.message = "ok"
        response.solved_angles_deg = [0.0, 10.0, -10.0, 5.0]
        return response

    def read_positions_handler(request, response):
        read_call_count["n"] += 1
        response.positions_deg = [0.0, 0.0, 0.0, 0.0]  # never matches - forces polling to continue
        response.all_valid = True
        return response

    _start_stub(move_to_pose_handler, read_positions_handler)

    node = _make_node()
    try:
        goal_handle = _FakeGoalHandle([_make_waypoint(), _make_waypoint()])

        def cancel_after_first_poll():
            while read_call_count["n"] < 1:
                time.sleep(0.01)
            goal_handle.is_cancel_requested = True

        canceler = threading.Thread(target=cancel_after_first_poll, daemon=True)
        canceler.start()

        result = node._execute_callback(goal_handle)
        canceler.join(timeout=1.0)

        assert not result.success
        assert goal_handle.status == "canceled"
        assert result.waypoints_completed == 0
    finally:
        node.destroy_node()


def test_execute_callback_waits_while_the_arm_is_still_moving(ros_context):
    """Regression: position alone is not enough to call a waypoint reached.

    Since the firmware loop became non-blocking, esp32_bridge answers a position
    read while the arm is still stepping a trajectory. A waypoint whose start
    pose already sits within tolerance of its target therefore looks "arrived"
    the instant it is commanded, and the next waypoint would be sent on top of
    an in-flight move. The arm must be reported stopped as well.
    """
    target_angles_deg = (0.0, 10.0, -10.0, 5.0)
    is_moving_calls = {"n": 0}

    def move_to_pose_handler(request, response):
        response.success = True
        response.message = "ok"
        response.solved_angles_deg = list(target_angles_deg)
        return response

    def read_positions_handler(request, response):
        # Always already at target - only the is_moving answer can hold the node back.
        response.positions_deg = list(target_angles_deg)
        response.all_valid = True
        return response

    def is_moving_handler(request, response):
        is_moving_calls["n"] += 1
        response.is_moving = is_moving_calls["n"] < 3
        response.valid = True
        return response

    _start_stub(move_to_pose_handler, read_positions_handler, is_moving_handler)

    node = _make_node()
    try:
        goal_handle = _FakeGoalHandle([_make_waypoint()])
        result = node._execute_callback(goal_handle)

        assert result.success
        assert goal_handle.status == "succeeded"
        assert is_moving_calls["n"] >= 3  # did not accept the first two "still moving" answers
    finally:
        node.destroy_node()


def test_execute_callback_treats_unknown_motion_state_as_still_moving(ros_context):
    """An is_moving query that comes back invalid must not read as "stopped" -
    advancing on an unanswerable query is the failure the check exists to stop.
    The waypoint stalls out instead."""
    target_angles_deg = (0.0, 10.0, -10.0, 5.0)

    def move_to_pose_handler(request, response):
        response.success = True
        response.message = "ok"
        response.solved_angles_deg = list(target_angles_deg)
        return response

    def read_positions_handler(request, response):
        response.positions_deg = list(target_angles_deg)
        response.all_valid = True
        return response

    def is_moving_handler(request, response):
        response.is_moving = False
        response.valid = False  # bridge could not reach the ESP32
        return response

    _start_stub(move_to_pose_handler, read_positions_handler, is_moving_handler)

    node = _make_node()
    try:
        goal_handle = _FakeGoalHandle([_make_waypoint()])
        result = node._execute_callback(goal_handle)

        assert not result.success
        assert goal_handle.status == "aborted"
        assert "possible stall" in result.message
    finally:
        node.destroy_node()
