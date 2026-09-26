import math

from rcl_interfaces.msg import ParameterDescriptor

import rclpy
import rclpy.logging
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState

from facade_msgs.srv import ReadJointPositions
from facadebot_description import robot_model

# Fixed by robot_state_publisher itself, not a project naming choice - it
# subscribes to this exact name (sensor_msgs/msg/JointState) by default, so
# this is the one topic in the project that does NOT follow the
# /facade_bot/... namespacing convention: remapping it would just mean
# re-remapping it back in every launch file that starts robot_state_publisher.
_JOINT_STATES_TOPIC = "/joint_states"

# esp32_bridge_node's own constant - see its README for why this exists as a
# separate service from /facade_bot/read_joint_positions.
_READ_JOINT_POSITIONS_FAST_SERVICE = "/facade_bot/read_joint_positions_fast"

_QUEUE_DEPTH = 10  # matches this project's other topics

# How long to wait for the service to be up at all - same figure
# facade_control_node uses for the same "is it even there" check.
_SERVICE_WAIT_TIMEOUT_SEC = 2.0

# read_joint_positions_fast measures ~170ms typical on the arm (see its
# docstring in esp32_bridge_node.py); this is a generous multiple of that so a
# single slow call doesn't itself look like a dropped connection.
_READ_CALL_TIMEOUT_SEC = 1.0

_DEFAULT_POLL_RATE_HZ = 5.0  # matches the ~170-220ms single-attempt read time measured on the arm


class JointStatePublisherNode(Node):
    """Polls esp32_bridge's fast (best-effort, no-retry) joint read on a timer
    and republishes it as /joint_states for robot_state_publisher.

    Deliberately does not use /facade_bot/read_joint_positions (the
    full-retry version esp32_bridge_node's own move-fault check uses) -
    that one occasionally blocks past 600ms waiting for one slow joint,
    which is fine for a safety check but would make this feed lag behind
    the arm's actual motion. The fast service can come back missing a
    joint about half the time on this arm; when that happens this node
    just republishes that joint's last known angle rather than blocking
    or leaving a gap, so the feed stays smooth at the cost of that one
    joint occasionally being one poll cycle (~200ms) stale - see
    esp32_bridge's README for the measurements behind that tradeoff.
    """

    def __init__(self) -> None:
        super().__init__("joint_state_publisher_node")

        model = robot_model.load_robot_model()
        self._joint_names = [joint.name for joint in model.joints]

        self.declare_parameter(
            "poll_rate_hz", _DEFAULT_POLL_RATE_HZ,
            ParameterDescriptor(description="how often to poll and publish /joint_states"))

        # PLACEHOLDER: commanded 0 deg (what read_joint_positions_fast reports)
        # is each joint's measured physical center (esp32_bridge_node's
        # _JOINT_CENTER_RAD), not necessarily wherever URDF_V2.urdf calls that
        # joint's zero angle - nobody has checked whether those two zeros
        # line up on the rebuilt v2 arm. Left at 0.0 (i.e. assumed equal)
        # until that's actually checked: command [0,0,0,0], compare the real
        # arm's pose to what RViz renders for the URDF at all-zero, and fill
        # in whatever per-joint difference is seen.
        # dynamic_typing: an override written as [0, 0, 5, 0] arrives as an
        # integer array, which a fixed double-array declaration rejects outright.
        # Accept either and convert below, so a whole-degree offset in a params
        # file or on the command line is not a startup crash.
        self.declare_parameter(
            "urdf_zero_offset_deg", [0.0] * len(self._joint_names),
            ParameterDescriptor(description="per-joint offset from commanded-zero to the URDF's zero pose "
                                             "(base to tip order) - unverified placeholder, see comment above",
                                dynamic_typing=True))
        raw_offset = self.get_parameter("urdf_zero_offset_deg").value
        try:
            if any(isinstance(value, bool) for value in raw_offset):
                raise TypeError("booleans are not angles")
            offset_deg = [float(value) for value in raw_offset]
        except (TypeError, ValueError) as error:
            raise robot_model.RobotModelError(
                f"urdf_zero_offset_deg must be a list of numbers, got {raw_offset!r}") from error
        if len(offset_deg) != len(self._joint_names):
            raise robot_model.RobotModelError(
                f"urdf_zero_offset_deg has {len(offset_deg)} entries but the model has "
                f"{len(self._joint_names)} joints")
        self._offset_deg = offset_deg
        self.get_logger().warn(
            "urdf_zero_offset_deg is unverified (default all 0.0 - assumes commanded-zero already "
            "matches the URDF's zero pose). Do not trust RViz's rendered pose against the real arm "
            "until this has actually been checked.")

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=_QUEUE_DEPTH,
        )
        self._joint_state_publisher = self.create_publisher(JointState, _JOINT_STATES_TOPIC, qos)

        # Same re-entrancy problem as facade_control_node's read_tool_pose:
        # this node's own timer callback can't block-wait on a service reply
        # using this node's own executor, since that executor is what's
        # running the timer callback in the first place. Separate helper node
        # + dedicated executor, used only for this one outgoing call.
        self._client_node = rclpy.create_node("joint_state_publisher_service_client")
        self._client_executor = SingleThreadedExecutor()
        self._client_executor.add_node(self._client_node)
        self._read_positions_client = self._client_node.create_client(
            ReadJointPositions, _READ_JOINT_POSITIONS_FAST_SERVICE
        )

        # None until that joint's had at least one successful reading ever;
        # holds the last known angle (already offset-corrected, in radians)
        # from then on, even through a poll that missed it.
        self._last_known_rad: list[float | None] = [None] * len(self._joint_names)

        poll_rate_hz = self.get_parameter("poll_rate_hz").value
        self._timer = self.create_timer(1.0 / poll_rate_hz, self._poll_and_publish)

        self.get_logger().info(f"publishing {_JOINT_STATES_TOPIC} at {poll_rate_hz:.1f} Hz")

    def _poll_and_publish(self) -> None:
        # A failed poll republishes the last known pose rather than returning
        # empty-handed. Dropping the message instead makes /joint_states go
        # silent, which downstream reads as "no data" rather than "unchanged" -
        # robot_state_publisher simply stops updating /tf and RViz freezes the
        # arm mid-pose with nothing on the graph saying why.
        if not self._read_positions_client.wait_for_service(timeout_sec=_SERVICE_WAIT_TIMEOUT_SEC):
            self.get_logger().warn(
                f"{_READ_JOINT_POSITIONS_FAST_SERVICE} not available - is esp32_bridge active?")
            self._publish_last_known()
            return

        future = self._read_positions_client.call_async(ReadJointPositions.Request())
        self._client_executor.spin_until_future_complete(future, timeout_sec=_READ_CALL_TIMEOUT_SEC)

        result = future.result()
        if result is None:
            self.get_logger().warn(f"{_READ_JOINT_POSITIONS_FAST_SERVICE} call timed out")
            self._publish_last_known()
            return

        for i, angle_deg in enumerate(result.positions_deg):
            if not math.isnan(angle_deg):
                self._last_known_rad[i] = math.radians(angle_deg + self._offset_deg[i])

        self._publish_last_known()

    def _publish_last_known(self) -> None:
        if any(angle_rad is None for angle_rad in self._last_known_rad):
            return  # haven't had a first successful reading for every joint yet

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self._joint_names
        msg.position = list(self._last_known_rad)
        self._joint_state_publisher.publish(msg)

    def destroy_node(self) -> bool:
        self._client_executor.shutdown()
        self._client_node.destroy_node()
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    try:
        node = JointStatePublisherNode()
    except robot_model.RobotModelError as error:
        rclpy.logging.get_logger("joint_state_publisher_node").error(f"refusing to start: {error}")
        rclpy.shutdown()
        raise SystemExit(1)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
