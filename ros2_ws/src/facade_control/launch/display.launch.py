"""Visualization-only launch: robot_state_publisher (+ /tf) and RViz for the
active arm model.

Deliberately separate from bringup.launch.py (see CLAUDE.md's Project Status
table) - this assumes a desktop session and reads the raw URDF XML, which is
fine for a visualization path but must never happen inside a control node.
Run it alongside bringup.launch.py, not instead of it: this publishes
/robot_description and /tf from whatever /joint_states is already on the
graph (bringup.launch.py's joint_state_publisher_node) - it does not move the
arm or read positions itself.

Which URDF file gets loaded follows robot_model.yaml's active_model, same as
every control node - there is no second place a model gets selected.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

from facadebot_description.robot_model import load_robot_model


def generate_launch_description() -> LaunchDescription:
    model = load_robot_model()
    urdf_path = os.path.join(
        get_package_share_directory("facadebot_description"), model.source_urdf)
    with open(urdf_path) as urdf_file:
        robot_description = urdf_file.read()

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
    )

    rviz_config_path = os.path.join(
        get_package_share_directory("facade_control"), "rviz", "facadebot.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_path],
    )

    return LaunchDescription([
        robot_state_publisher_node,
        rviz_node,
    ])
