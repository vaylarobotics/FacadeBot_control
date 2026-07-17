import time
from collections.abc import Sequence

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.node import Node

from facade_msgs.action import FollowTrajectory
from facade_msgs.msg import Waypoint
from facade_msgs.srv import MoveToPose, ReadJointPositions

_FOLLOW_TRAJECTORY_ACTION = "/facade_bot/follow_trajectory"
_MOVE_TO_POSE_SERVICE = "/facade_bot/move_to_pose"  # must match facade_control_node's own constant
_READ_JOINT_POSITIONS_SERVICE = "/facade_bot/read_joint_positions"  # must match esp32_bridge_node's own constant

# One thread runs the multi-waypoint execute callback; the other stays free so a
# cancel request can actually be accepted while that callback is still running -
# see the TrajectoryNode docstring for why a single-threaded executor won't do.
_EXECUTOR_THREAD_COUNT = 2

# How long to wait for a service to be up at all - same figure facade_control_node
# uses for the same "is it even there" check.
_SERVICE_WAIT_TIMEOUT_SEC = 2.0

# esp32_bridge is single-threaded, so a read_joint_positions call issued while a
# move is in flight simply queues behind it until that move's ack arrives - up to
# ~120ms pre-move readback + 5000ms max move_duration_ms clamp (esp32_firmware's
# _DURATION_MAX_MS). This is the same worst case esp32_bridge_node.py's own 7.0s
# timeout already covers, reused here rather than re-derived.
_POLL_CALL_TIMEOUT_SEC = 7.0

# Overall per-waypoint budget from "move_to_pose returned" to "confirmed arrived"
# before treating it as a stall. Sized for at least two full _POLL_CALL_TIMEOUT_SEC
# attempts, so one slow-but-legitimate poll doesn't itself cause a false abort.
_WAYPOINT_STALL_TIMEOUT_SEC = 15.0

# Sleep between poll attempts once a call returns but doesn't yet match target -
# not shorter than esp32_bridge_node's own post-move settle margin (200ms), since
# polling faster than the hardware's own settle window buys nothing.
_POLL_RETRY_INTERVAL_SEC = 0.3

# Matches esp32_bridge_node.py's own _POSITION_TOLERANCE_DEG exactly - same
# servos, same physical slack.
_WAYPOINT_POSITION_TOLERANCE_DEG = 5.0

_ARRIVED = "arrived"
_STALLED = "stalled"
_CANCELED = "canceled"


def _within_tolerance(actual_deg: Sequence[float], target_deg: Sequence[float], tolerance_deg: float) -> bool:
    return all(abs(a - t) <= tolerance_deg for a, t in zip(actual_deg, target_deg))


class TrajectoryNode(Node):
    """Moves the arm through an ordered list of Cartesian waypoints, one at a time.

    Exposes this as a ROS2 action (not a service) because following a
    trajectory takes multiple seconds and the caller needs progress feedback
    and the ability to cancel partway through - things a single request/
    response service can't express.

    A callback group is how ROS2 decides which of a node's callbacks are
    allowed to run at the same time; by default every callback on a node
    shares one group and runs one-at-a-time, which would mean a cancel
    request couldn't even be looked at until the whole multi-waypoint
    sequence finished on its own. This node gives its action server a
    ReentrantCallbackGroup and is spun with a MultiThreadedExecutor so a
    cancel request can be accepted while a goal is still executing.

    V1 stops fully at each waypoint (reusing /facade_bot/move_to_pose and
    confirming arrival via /facade_bot/read_joint_positions) rather than
    blending continuously through them - see facade_control/README.md.
    _wait_for_waypoint_arrival is the one piece expected to be replaced when
    blending is added later; goal handling, cancellation, feedback, and
    result construction below it should not need to change.

    Never sends anything to the ESP32 directly - only calls facade_control's
    own move_to_pose service and esp32_bridge's read_joint_positions service,
    same as facade_control_node.

    Safety note: canceling a goal means "don't command any further
    waypoints," not "stop the arm instantly." There is no firmware/protocol
    primitive to abort an in-flight physical move, and emergency-stop logic
    is still not implemented (see CLAUDE.md). If a cancel arrives while
    waiting for a waypoint, the arm still physically finishes whatever move
    was already commanded.
    """

    def __init__(self) -> None:
        super().__init__("trajectory_node")

        self._callback_group = ReentrantCallbackGroup()
        self._action_server = ActionServer(
            self,
            FollowTrajectory,
            _FOLLOW_TRAJECTORY_ACTION,
            execute_callback=self._execute_callback,
            goal_callback=self._handle_goal,
            cancel_callback=self._handle_cancel,
            callback_group=self._callback_group,
        )

        # Calling a service and blocking for the reply from inside this node's
        # own action-execution callback can't spin this node's own executor -
        # it's already spinning that callback. A separate helper node with its
        # own dedicated executor, used only for these two outgoing calls,
        # avoids that re-entrancy problem - same pattern facade_control_node
        # uses for its read_tool_pose service, replicated here as its own
        # separate instance.
        self._client_node = rclpy.create_node("trajectory_node_service_client")
        self._client_executor = SingleThreadedExecutor()
        self._client_executor.add_node(self._client_node)
        self._move_to_pose_client = self._client_node.create_client(MoveToPose, _MOVE_TO_POSE_SERVICE)
        self._read_positions_client = self._client_node.create_client(
            ReadJointPositions, _READ_JOINT_POSITIONS_SERVICE
        )

        self.get_logger().info(f"serving {_FOLLOW_TRAJECTORY_ACTION}")

    def _handle_goal(self, goal_request: FollowTrajectory.Goal) -> GoalResponse:
        if not goal_request.waypoints:
            self.get_logger().warn("rejecting follow_trajectory goal: empty waypoint list")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _handle_cancel(self, goal_handle) -> CancelResponse:
        self.get_logger().info("follow_trajectory cancel requested")
        return CancelResponse.ACCEPT

    def _execute_callback(self, goal_handle) -> FollowTrajectory.Result:
        waypoints = goal_handle.request.waypoints
        total_waypoints = len(waypoints)
        result = FollowTrajectory.Result()

        for index, waypoint in enumerate(waypoints):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success = False
                result.message = f"canceled before waypoint {index}"
                result.waypoints_completed = index
                return result

            self.get_logger().info(f"waypoint {index}/{total_waypoints}: solving")
            move_response = self._call_move_to_pose(waypoint)
            if move_response is None:
                goal_handle.abort()
                result.success = False
                result.message = f"waypoint {index}: move_to_pose service call failed or timed out"
                result.waypoints_completed = index
                return result
            if not move_response.success:
                goal_handle.abort()
                result.success = False
                result.message = f"waypoint {index}: {move_response.message}"
                result.waypoints_completed = index
                return result

            arrival = self._wait_for_waypoint_arrival(move_response.solved_angles_deg, goal_handle)
            if arrival == _CANCELED:
                goal_handle.canceled()
                result.success = False
                result.message = f"canceled while waiting for waypoint {index}"
                result.waypoints_completed = index
                return result
            if arrival == _STALLED:
                goal_handle.abort()
                result.success = False
                result.message = (
                    f"waypoint {index}: arm did not reach target within "
                    f"{_WAYPOINT_STALL_TIMEOUT_SEC:.0f}s - possible stall"
                )
                result.waypoints_completed = index
                return result

            self.get_logger().info(f"waypoint {index}/{total_waypoints}: arrived")
            feedback_msg = FollowTrajectory.Feedback()
            feedback_msg.current_waypoint_index = index
            feedback_msg.total_waypoints = total_waypoints
            goal_handle.publish_feedback(feedback_msg)

        goal_handle.succeed()
        result.success = True
        result.message = "ok"
        result.waypoints_completed = total_waypoints
        return result

    def _wait_for_waypoint_arrival(self, target_angles_deg: Sequence[float], goal_handle) -> str:
        deadline = time.monotonic() + _WAYPOINT_STALL_TIMEOUT_SEC
        while time.monotonic() < deadline:
            if goal_handle.is_cancel_requested:
                return _CANCELED

            actual_angles_deg = self._read_joint_positions()
            if actual_angles_deg is not None and _within_tolerance(
                actual_angles_deg, target_angles_deg, _WAYPOINT_POSITION_TOLERANCE_DEG
            ):
                return _ARRIVED

            time.sleep(_POLL_RETRY_INTERVAL_SEC)

        return _STALLED

    def _call_move_to_pose(self, waypoint: Waypoint) -> MoveToPose.Response | None:
        if not self._move_to_pose_client.wait_for_service(timeout_sec=_SERVICE_WAIT_TIMEOUT_SEC):
            return None

        request = MoveToPose.Request()
        request.x_m = waypoint.x_m
        request.y_m = waypoint.y_m
        request.z_m = waypoint.z_m
        request.tool_angle_deg = waypoint.tool_angle_deg

        future = self._move_to_pose_client.call_async(request)
        self._client_executor.spin_until_future_complete(future, timeout_sec=_POLL_CALL_TIMEOUT_SEC)
        return future.result()

    def _read_joint_positions(self) -> tuple[float, ...] | None:
        if not self._read_positions_client.wait_for_service(timeout_sec=_SERVICE_WAIT_TIMEOUT_SEC):
            return None

        future = self._read_positions_client.call_async(ReadJointPositions.Request())
        self._client_executor.spin_until_future_complete(future, timeout_sec=_POLL_CALL_TIMEOUT_SEC)

        result = future.result()
        if result is None or not result.all_valid:
            return None
        return tuple(result.positions_deg)

    def destroy_node(self) -> bool:
        self._client_executor.shutdown()
        self._client_node.destroy_node()
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = TrajectoryNode()
    executor = MultiThreadedExecutor(num_threads=_EXECUTOR_THREAD_COUNT)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
