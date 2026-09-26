# facade_msgs — CLAUDE.md

Every custom message, service and action definition, and nothing else. This is an
`ament_cmake` package; after editing anything here, rebuild it **before** the
packages that use it:
```bash
cd ros2_ws && source /opt/ros/jazzy/setup.bash
colcon build --packages-select facade_msgs && source install/setup.bash
```

## What exists

| Definition | Used by |
|------------|---------|
| `srv/ReadJointPositions.srv` | `esp32_bridge` serves it (two services, same type); every planner and the joint-state publisher call it |
| `srv/MoveToPose.srv` | `facade_control_node` serves; `trajectory_node` calls |
| `srv/ReadToolPose.srv` | `facade_control_node` serves |
| `msg/Waypoint.msg` | both trajectory actions |
| `action/FollowTrajectory.action` | `trajectory_node` |
| `action/FollowTrajectoryContinuous.action` | `continuous_trajectory_node` |

Joint commands themselves use `sensor_msgs/JointState` (radians, order assumed),
not a custom type. Moving to `trajectory_msgs/JointTrajectory` so a move can carry
its own duration is an open decision (structural finding 7); do not add a custom
command message without the user choosing.

## Rules

- Field names carry units: `x_m`, `tool_angle_deg`, `tool_speed_mmps`.
- Comments in `.srv`/`.msg` files are the contract callers read. Keep them true.
  `tool_angle_deg` means joint_4's own angle (tilt out of the arm plane), and the
  comments say so as of 2026-09-18.
- Adding a field is a breaking change for every caller. List the callers from the
  table above and update them in the same change.
- Never put a message definition in another package.
