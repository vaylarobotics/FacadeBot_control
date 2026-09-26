"""Bring up the FacadeBot control stack: esp32_bridge_node, facade_control_node,
trajectory_node, and joint_state_publisher_node.

esp32_bridge_node is a lifecycle node so its startup normally needs two manual
steps after launch (`ros2 lifecycle set /esp32_bridge_node configure` then
`activate` - see LAUNCH.md steps 5-6). This launch file drives those same two
transitions automatically, one after the other becomes possible, so the arm is
live as soon as `ros2 launch` finishes. There is still no e-stop, so the arm can
move as soon as this file returns control - do not run it with anyone near the
arm's reach envelope.
"""

from launch import LaunchDescription
from launch.actions import EmitEvent, RegisterEventHandler
from launch.events import matches_action
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description() -> LaunchDescription:
    esp32_bridge_node = LifecycleNode(
        package="esp32_bridge",
        executable="esp32_bridge_node",
        name="esp32_bridge_node",
        namespace="",
    )

    # No `name=` here: all three nodes' own code creates a second, internal
    # helper node (a plain rclpy.create_node(...) used for a separate
    # service-call context) with its own hardcoded name. Launch's `name=`
    # works by injecting a process-wide `-r __node:=...` remap, which renames
    # every node created in that process - so setting it here would silently
    # rename the helper node too, and both would collide under the one name
    # (visible as "Publisher already registered for node name" and a
    # duplicate entry in `ros2 node list`). The nodes already name themselves
    # correctly in code; leave it to them.
    facade_control_node = Node(
        package="facade_control",
        executable="facade_control_node",
    )

    trajectory_node = Node(
        package="facade_control",
        executable="trajectory_node",
    )

    # Polls esp32_bridge's fast (best-effort, no-retry) joint read on a timer
    # and republishes /joint_states - see the node's own docstring for why it
    # uses that service rather than the full-retry read_joint_positions.
    joint_state_publisher_node = Node(
        package="facade_control",
        executable="joint_state_publisher_node",
    )

    # unconfigured -> inactive, as soon as the node exists
    configure_on_start = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(esp32_bridge_node),
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )

    # inactive -> active, once configure above has actually completed.
    # start_state matters: a lifecycle node lands in "inactive" from three
    # places - the end of configure (from "configuring"), a manual deactivate
    # (from "deactivating"), and a failed activate (from "activating").
    # Matching goal_state alone re-activated the bridge after every one of
    # them, which made `ros2 lifecycle set ... deactivate` impossible to use as
    # a stop while this launch was running.
    activate_after_configure = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=esp32_bridge_node,
            start_state="configuring",
            goal_state="inactive",
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(esp32_bridge_node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                )
            ],
        )
    )

    return LaunchDescription([
        esp32_bridge_node,
        facade_control_node,
        trajectory_node,
        joint_state_publisher_node,
        activate_after_configure,
        configure_on_start,
    ])
