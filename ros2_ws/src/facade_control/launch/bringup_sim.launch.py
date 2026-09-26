"""Bring up the FacadeBot control stack against the simulated ESP32
(scripts/fake_esp32_server.py) instead of the real hardware, for exercising the
ROS2 control loop - bounds gate, IK, trajectory sequencing, /joint_states - with
no arm, ESP32, or servos attached at all. See LAUNCH.md step 2b.

Starts the fake server as its own process, then esp32_bridge_node hardcoded to
talk to it (esp32_host=127.0.0.1 - this file must never point anywhere else;
LAUNCH.md's bringup.launch.py is the one for real hardware), facade_control_node,
trajectory_node, and joint_state_publisher_node, driving the bridge straight
through configure -> activate exactly like bringup.launch.py does.
"""

from launch import LaunchDescription
from launch.actions import EmitEvent, ExecuteProcess, RegisterEventHandler, TimerAction
from launch.events import matches_action
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition

# Repo-root dev tool, not part of any ROS2 package (see scripts/CLAUDE.md), so it
# can't be found relative to this file once colcon installs it - hardcoded like
# mpremote's path elsewhere in this project (see root CLAUDE.md).
_FAKE_ESP32_SERVER_PATH = "/home/harthik/FacadeBot_control/scripts/fake_esp32_server.py"
_SIM_ESP32_HOST = "127.0.0.1"

# Empirical margin for fake_esp32_server.py's socket to finish binding before
# esp32_bridge_node's on_configure tries to connect to it - Esp32Transport.connect()
# has no retry, so a too-short delay here shows up as a one-time "failed to connect"
# and the node stuck unconfigured (just rerun `ros2 lifecycle set ... configure`).
_SERVER_STARTUP_DELAY_SEC = 1.0


def generate_launch_description() -> LaunchDescription:
    fake_esp32_server = ExecuteProcess(
        cmd=["python3", _FAKE_ESP32_SERVER_PATH, "--host", _SIM_ESP32_HOST],
        name="fake_esp32_server",
        output="screen",
    )

    esp32_bridge_node = LifecycleNode(
        package="esp32_bridge",
        executable="esp32_bridge_node",
        name="esp32_bridge_node",
        namespace="",
        parameters=[{"esp32_host": _SIM_ESP32_HOST}],
    )

    # No `name=` here - see bringup.launch.py / facade_control/CLAUDE.md's "Traps
    # already hit": these three nodes each create their own internal helper node,
    # and launch's `name=` would rename it too via a process-wide remap.
    facade_control_node = Node(
        package="facade_control",
        executable="facade_control_node",
    )

    trajectory_node = Node(
        package="facade_control",
        executable="trajectory_node",
    )

    joint_state_publisher_node = Node(
        package="facade_control",
        executable="joint_state_publisher_node",
    )

    # Delayed, not immediate like bringup.launch.py, so the fake server's socket
    # is listening before the bridge's on_configure tries to connect to it.
    configure_after_server_starts = TimerAction(
        period=_SERVER_STARTUP_DELAY_SEC,
        actions=[
            EmitEvent(
                event=ChangeState(
                    lifecycle_node_matcher=matches_action(esp32_bridge_node),
                    transition_id=Transition.TRANSITION_CONFIGURE,
                )
            )
        ],
    )

    # inactive -> active, once configure above has actually completed. Matches
    # start_state="configuring" specifically, not just goal_state="inactive" - see
    # facade_control/CLAUDE.md's "Traps already hit" for why (a manual deactivate
    # or a failed activate also land in "inactive").
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
        fake_esp32_server,
        esp32_bridge_node,
        facade_control_node,
        trajectory_node,
        joint_state_publisher_node,
        activate_after_configure,
        configure_after_server_starts,
    ])
