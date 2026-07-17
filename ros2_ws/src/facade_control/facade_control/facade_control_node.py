import math

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState

from facade_control import kinematics
from facade_msgs.srv import MoveToPose, ReadJointPositions, ReadToolPose

_JOINT_CMD_TOPIC = "/facade_bot/joint_cmd"
_QUEUE_DEPTH = 10  # matches esp32_bridge_node's subscription - joint commands are infrequent
_MOVE_TO_POSE_SERVICE = "/facade_bot/move_to_pose"
_READ_TOOL_POSE_SERVICE = "/facade_bot/read_tool_pose"
_READ_JOINT_POSITIONS_SERVICE = "/facade_bot/read_joint_positions"  # must match esp32_bridge_node's own constant

# How long to wait for esp32_bridge to be up / to answer a read_joint_positions
# call before giving up - esp32_bridge's own retry loop is quick (a handful of
# UART reads), so this just needs margin for that plus normal ROS2 service overhead.
_SERVICE_WAIT_TIMEOUT_SEC = 2.0
_READ_JOINT_POSITIONS_TIMEOUT_SEC = 5.0


class FacadeControlNode(Node):
    """Solves a Cartesian tool-tip target into joint angles and publishes them
    to /facade_bot/joint_cmd. Never sends anything to the ESP32 directly -
    esp32_bridge owns that transport and re-checks every joint command
    against the same limits before it reaches the hardware. It does read
    from esp32_bridge, via its /facade_bot/read_joint_positions service, to
    answer read_tool_pose and to know the arm's current configuration before
    solving a new move_to_pose target - that's a read of already-published
    state, not a new hardware access path.
    """

    def __init__(self) -> None:
        super().__init__("facade_control_node")

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=_QUEUE_DEPTH,
        )
        self._joint_cmd_publisher = self.create_publisher(JointState, _JOINT_CMD_TOPIC, qos)

        # read_tool_pose needs to call another node's service (esp32_bridge's
        # read_joint_positions) and wait for the reply from inside this node's
        # own service callback. That wait can't run on this node's own executor -
        # main() is already spinning it, and an executor can't spin itself
        # re-entrantly. A separate node + its own dedicated executor, used only
        # for this one outgoing call, avoids that without disturbing this
        # node's normal callback handling.
        self._read_positions_node = rclpy.create_node("facade_control_read_positions_client")
        self._read_positions_executor = SingleThreadedExecutor()
        self._read_positions_executor.add_node(self._read_positions_node)
        self._read_positions_client = self._read_positions_node.create_client(
            ReadJointPositions, _READ_JOINT_POSITIONS_SERVICE
        )

        self._move_to_pose_service = self.create_service(
            MoveToPose, _MOVE_TO_POSE_SERVICE, self._handle_move_to_pose
        )
        self._read_tool_pose_service = self.create_service(
            ReadToolPose, _READ_TOOL_POSE_SERVICE, self._handle_read_tool_pose
        )
        self.get_logger().info(
            f"serving {_MOVE_TO_POSE_SERVICE} and {_READ_TOOL_POSE_SERVICE}, "
            f"publishing to {_JOINT_CMD_TOPIC}"
        )

    def _handle_move_to_pose(
        self, request: MoveToPose.Request, response: MoveToPose.Response
    ) -> MoveToPose.Response:
        # The IK solver needs to know where the arm actually is right now to
        # pick the candidate solution closest to it (avoids the elbow flipping
        # between up/down across a sequence of nearby targets) - so this reads
        # current joint positions first and refuses to move if that read fails,
        # rather than guessing a configuration.
        current_angles_deg, read_message = self._read_current_joint_positions()
        if current_angles_deg is None:
            response.success = False
            response.message = f"can't confirm current joint positions, refusing to move: {read_message}"
            return response
        self.get_logger().info(
            f"current joints {[f'{a:.1f}' for a in current_angles_deg]} deg before solving"
        )

        try:
            angles_deg = kinematics.inverse_kinematics(
                request.x_m, request.y_m, request.z_m, request.tool_angle_deg,
                current_angles_deg=current_angles_deg,
            )
        except kinematics.NotReachableError as exc:
            self.get_logger().warn(str(exc))
            response.success = False
            response.message = str(exc)
            return response

        self._publish_joint_cmd(angles_deg)
        self.get_logger().info(
            f"solved ({request.x_m:.3f}, {request.y_m:.3f}, {request.z_m:.3f}) m, "
            f"tool {request.tool_angle_deg:.1f} deg -> joints {[f'{a:.1f}' for a in angles_deg]} deg"
        )
        response.success = True
        response.message = "ok"
        response.solved_angles_deg = list(angles_deg)
        return response

    def _handle_read_tool_pose(
        self, request: ReadToolPose.Request, response: ReadToolPose.Response
    ) -> ReadToolPose.Response:
        # Reads the arm's actual joint angles (not a commanded target) and runs
        # them through forward kinematics, so this can be checked against a
        # physically measured tool-tip position. Pick a pose away from full
        # extension/retraction for that check - those are singularities where
        # forward_kinematics can't reveal an elbow-branch or offset error.
        positions_deg, message = self._read_current_joint_positions()
        if positions_deg is None:
            response.success = False
            response.message = message
            return response

        x_m, y_m, z_m, tool_angle_deg = kinematics.forward_kinematics(*positions_deg)
        response.success = True
        response.message = "ok"
        response.x_m = x_m
        response.y_m = y_m
        response.z_m = z_m
        response.tool_angle_deg = tool_angle_deg
        self.get_logger().info(
            f"read joints {[f'{a:.1f}' for a in positions_deg]} deg -> "
            f"tool pose ({x_m:.3f}, {y_m:.3f}, {z_m:.3f}) m, {tool_angle_deg:.1f} deg"
        )
        return response

    def _read_current_joint_positions(self) -> tuple[tuple[float, float, float, float] | None, str]:
        """Reads the arm's actual joint angles from esp32_bridge.

        Returns (positions_deg, "ok") on success, or (None, reason) if
        esp32_bridge is unreachable, the call times out, or a joint had no
        reading (esp32_bridge already retries internally before giving up -
        see its _read_positions_with_retry).
        """
        if not self._read_positions_client.wait_for_service(timeout_sec=_SERVICE_WAIT_TIMEOUT_SEC):
            return None, f"{_READ_JOINT_POSITIONS_SERVICE} is not available - is esp32_bridge running and active?"

        future = self._read_positions_client.call_async(ReadJointPositions.Request())
        self._read_positions_executor.spin_until_future_complete(
            future, timeout_sec=_READ_JOINT_POSITIONS_TIMEOUT_SEC
        )

        positions_result = future.result()
        if positions_result is None:
            return None, f"{_READ_JOINT_POSITIONS_SERVICE} call timed out"
        if not positions_result.all_valid:
            return None, "one or more joints had no position reading"

        return tuple(positions_result.positions_deg), "ok"

    def _publish_joint_cmd(self, angles_deg: tuple[float, float, float, float]) -> None:
        msg = JointState()
        msg.position = [math.radians(a) for a in angles_deg]
        self._joint_cmd_publisher.publish(msg)

    def destroy_node(self) -> bool:
        self._read_positions_executor.shutdown()
        self._read_positions_node.destroy_node()
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = FacadeControlNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
