import math
import time

from rcl_interfaces.msg import ParameterDescriptor

import rclpy
import rclpy.logging
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState

from facade_control import kinematics, trajectory_planning
from facadebot_description import robot_model
from facade_msgs.action import FollowTrajectoryContinuous
from facade_msgs.srv import ReadJointPositions

_FOLLOW_TRAJECTORY_CONTINUOUS_ACTION = "/facade_bot/follow_trajectory_continuous"
_JOINT_STREAM_TOPIC = "/facade_bot/joint_stream"  # must match esp32_bridge_node's own constant
_READ_JOINT_POSITIONS_SERVICE = "/facade_bot/read_joint_positions"  # must match esp32_bridge_node's own constant

_QUEUE_DEPTH = 10  # small buffer - the node paces its own stream, so this shouldn't back up

# One thread runs the streaming execute callback; the other stays free so a cancel
# request can be accepted while that callback is still sleeping between setpoints -
# same reasoning as trajectory_node.
_EXECUTOR_THREAD_COUNT = 2

# Default gap between streamed setpoints. Kept conservative so the node never streams
# faster than esp32_bridge + the servo bus can forward (~80ms to write all 4 servos at
# the current inter-servo delay); outrunning them would back up the RELIABLE queue and
# make the arm lag the plan. Exposed as a parameter so it can be tuned on hardware, and
# must be kept in step with esp32_bridge_node's servo_move_duration_ms.
_DEFAULT_STREAM_PERIOD_SEC = 0.12

# How often to publish progress feedback while streaming (much slower than the setpoint
# rate - feedback is for a human watching, not for control).
_FEEDBACK_PERIOD_SEC = 0.5

# Same "is the service even there / has it answered" figures trajectory_node uses.
# The read timeout no longer has to cover a whole move queued ahead of it: the
# firmware answers a read while it is still stepping a trajectory, and esp32_bridge
# serves reads on their own executor thread. It covers the bridge's full-retry read
# budget (measured past 600ms when a joint keeps missing its UART window) and jitter.
_SERVICE_WAIT_TIMEOUT_SEC = 2.0
_READ_CALL_TIMEOUT_SEC = 2.0


class ContinuousTrajectoryNode(Node):
    """Sweeps the arm's tool tip through a list of Cartesian waypoints at a constant
    speed, rounding corners rather than stopping at each waypoint.

    How it differs from trajectory_node (which stops fully at every waypoint): this
    node plans the entire motion up front - it builds a corner-rounded Cartesian path,
    samples it into a dense stream of points at the requested constant speed, and solves
    each to joint angles (see facade_control/trajectory_planning.py). It validates that
    whole joint path is reachable and in-range BEFORE commanding any motion, then streams
    the setpoints one per period to /facade_bot/joint_stream, which esp32_bridge forwards
    to the ESP32's non-blocking "servo" command. Never touches the ESP32 directly.

    Concurrency: like trajectory_node, the action server uses a ReentrantCallbackGroup and
    a MultiThreadedExecutor so a cancel can be accepted while the execute callback is still
    streaming. A separate helper node + its own executor handles the one outgoing
    read_joint_positions call (an executor can't spin itself re-entrantly from inside its
    own callback).

    Safety note: as with trajectory_node, canceling stops further setpoints from being
    sent - it does not stop the arm mid-move. There is no e-stop primitive; the arm coasts
    to the last setpoint already commanded.
    """

    def __init__(self) -> None:
        super().__init__("continuous_trajectory_node")

        self.declare_parameter(
            "stream_period_sec", _DEFAULT_STREAM_PERIOD_SEC,
            ParameterDescriptor(
                description="seconds between streamed joint setpoints; keep in step with "
                            "esp32_bridge_node's servo_move_duration_ms"))

        self._callback_group = ReentrantCallbackGroup()
        self._action_server = ActionServer(
            self,
            FollowTrajectoryContinuous,
            _FOLLOW_TRAJECTORY_CONTINUOUS_ACTION,
            execute_callback=self._execute_callback,
            goal_callback=self._handle_goal,
            cancel_callback=self._handle_cancel,
            callback_group=self._callback_group,
        )

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=_QUEUE_DEPTH,
        )
        self._stream_publisher = self.create_publisher(JointState, _JOINT_STREAM_TOPIC, qos)

        # Same helper-node + dedicated-executor pattern trajectory_node uses, for the one
        # read_joint_positions call that seeds the IK from the arm's real position.
        self._client_node = rclpy.create_node("continuous_trajectory_node_service_client")
        self._client_executor = SingleThreadedExecutor()
        self._client_executor.add_node(self._client_node)
        self._read_positions_client = self._client_node.create_client(
            ReadJointPositions, _READ_JOINT_POSITIONS_SERVICE
        )

        self.get_logger().info(f"serving {_FOLLOW_TRAJECTORY_CONTINUOUS_ACTION}")

    def _handle_goal(self, goal_request: FollowTrajectoryContinuous.Goal) -> GoalResponse:
        if len(goal_request.waypoints) < 2:
            self.get_logger().warn("rejecting continuous goal: need at least 2 waypoints")
            return GoalResponse.REJECT
        if goal_request.tool_speed_mmps <= 0.0:
            self.get_logger().warn("rejecting continuous goal: tool_speed_mmps must be positive")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _handle_cancel(self, goal_handle) -> CancelResponse:
        self.get_logger().info("continuous trajectory cancel requested")
        return CancelResponse.ACCEPT

    def _execute_callback(self, goal_handle) -> FollowTrajectoryContinuous.Result:
        result = FollowTrajectoryContinuous.Result()

        current_angles_deg, read_message = self._read_current_joint_positions()
        if current_angles_deg is None:
            return self._abort(goal_handle, result,
                               f"can't confirm current joint positions, refusing to move: {read_message}")

        setpoints = self._plan_setpoints(goal_handle, result, current_angles_deg)
        if setpoints is None:
            return result  # _plan_setpoints already aborted the goal and filled result

        return self._stream_setpoints(goal_handle, result, setpoints)

    def _plan_setpoints(self, goal_handle, result, current_angles_deg):
        goal = goal_handle.request
        waypoints = [(w.x_m, w.y_m, w.z_m, w.tool_angle_deg) for w in goal.waypoints]
        stream_period_sec = self.get_parameter("stream_period_sec").value
        try:
            setpoints = trajectory_planning.plan_joint_setpoints(
                waypoints,
                goal.tool_speed_mmps,
                goal.corner_blend_m,
                stream_period_sec,
                current_angles_deg,
            )
        except (ValueError, kinematics.NotReachableError) as exc:
            # Whole path is validated here, before any motion - a bad point aborts cleanly.
            self._abort(goal_handle, result, f"cannot plan continuous trajectory: {exc}")
            return None
        self.get_logger().info(
            f"planned {len(setpoints)} setpoints at {goal.tool_speed_mmps:.1f} mm/s, "
            f"corner blend {goal.corner_blend_m * 1000:.0f} mm"
        )
        return setpoints

    def _stream_setpoints(self, goal_handle, result, setpoints) -> FollowTrajectoryContinuous.Result:
        total = len(setpoints)
        stream_period_sec = self.get_parameter("stream_period_sec").value
        start_time = time.monotonic()
        last_feedback_time = 0.0

        for index, angles_deg in enumerate(setpoints):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success = False
                result.message = f"canceled after {index}/{total} setpoints"
                result.fraction_completed = index / total
                return result

            # Pace to a fixed wall-clock cadence so the tool speed stays constant even if
            # publishing itself takes a variable amount of time.
            target_time = start_time + index * stream_period_sec
            sleep_sec = target_time - time.monotonic()
            if sleep_sec > 0.0:
                time.sleep(sleep_sec)

            self._publish_setpoint(angles_deg)

            now = time.monotonic()
            if now - last_feedback_time >= _FEEDBACK_PERIOD_SEC:
                feedback_msg = FollowTrajectoryContinuous.Feedback()
                feedback_msg.fraction_complete = (index + 1) / total
                goal_handle.publish_feedback(feedback_msg)
                last_feedback_time = now

        goal_handle.succeed()
        result.success = True
        result.message = "ok"
        result.fraction_completed = 1.0
        return result

    def _abort(self, goal_handle, result, message: str) -> FollowTrajectoryContinuous.Result:
        self.get_logger().warn(message)
        goal_handle.abort()
        result.success = False
        result.message = message
        result.fraction_completed = 0.0
        return result

    def _read_current_joint_positions(self):
        """Reads the arm's actual joint angles from esp32_bridge. Returns
        (positions_deg_tuple, "ok") or (None, reason)."""
        if not self._read_positions_client.wait_for_service(timeout_sec=_SERVICE_WAIT_TIMEOUT_SEC):
            return None, f"{_READ_JOINT_POSITIONS_SERVICE} is not available - is esp32_bridge running and active?"

        future = self._read_positions_client.call_async(ReadJointPositions.Request())
        self._client_executor.spin_until_future_complete(future, timeout_sec=_READ_CALL_TIMEOUT_SEC)

        response = future.result()
        if response is None:
            return None, f"{_READ_JOINT_POSITIONS_SERVICE} call timed out"
        if not response.all_valid:
            return None, "one or more joints had no position reading"
        return tuple(response.positions_deg), "ok"

    def _publish_setpoint(self, angles_deg) -> None:
        msg = JointState()
        msg.position = [math.radians(a) for a in angles_deg]
        self._stream_publisher.publish(msg)

    def destroy_node(self) -> bool:
        self._client_executor.shutdown()
        self._client_node.destroy_node()
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    try:
        model = kinematics.load_active_model()
    except robot_model.RobotModelError as error:
        rclpy.logging.get_logger("continuous_trajectory_node").error(f"refusing to start: {error}")
        rclpy.shutdown()
        raise SystemExit(1)
    node = ContinuousTrajectoryNode()
    node.get_logger().info(robot_model.describe(model))
    executor = MultiThreadedExecutor(num_threads=_EXECUTOR_THREAD_COUNT)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
