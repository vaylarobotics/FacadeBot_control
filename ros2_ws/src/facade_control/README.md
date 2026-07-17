# facade_control

Turns a Cartesian tool-tip target into joint angles and publishes them to
`/facade_bot/joint_cmd`, the same topic `esp32_bridge` already subscribes
to. Never opens a connection to the ESP32 itself - it only publishes an
existing message type using existing units/QoS, so `esp32_bridge` needs no
changes to receive these commands.

## Arm geometry

The arm is base yaw (`joint_1`) plus three parallel-axis pitch joints
(`joint_2` shoulder, `joint_3` elbow, `joint_4` wrist) that all rotate
within a single vertical plane whose azimuth is set by `joint_1`. This
follows directly from `ros2_ws/src/facadebot_description/urdf/URDF_Test.urdf`:
`joint_2`'s `<origin rpy="1.5708 -1.5708 0">` re-orients its rotation axis
from the URDF's local `z` to the physical horizontal pitch axis; `joint_3`
and `joint_4` have identity-rotation origins, so they inherit the same axis
direction and stay parallel to `joint_2` regardless of angle.

Link lengths (`facade_control/kinematics.py`), taken straight from the
URDF's joint origins:

| Segment | Length | Source |
|---|---|---|
| Base → shoulder pivot height | 0.10061 m | `joint_1` origin `z` |
| Shoulder → elbow | 0.12511 m | `joint_2`→`joint_3` origin `x` |
| Elbow → wrist | 0.16511 m | `joint_3`→`joint_4` origin `x` |
| Wrist → tool tip | 0.05 m | measured on hardware - not in the URDF |

Non-obvious wrinkle: at all joint angles = 0 (the URDF's neutral pose), the
arm points **straight up**, not horizontally outward. This was verified by
composing the actual URDF origin rotations, not assumed - `kinematics.py`'s
`forward_kinematics` is a literal transcription of those transforms (not a
hand-simplified formula) specifically to avoid a wrong "textbook" zero
reference.

Joint limits, degrees, relative to each joint's own measured true center
(see `esp32_bridge_node.py`'s `_JOINT_CENTER_RAD` for the centers
themselves - no longer derived from the URDF). Must match
`esp32_bridge_node.py`'s `_JOINT_LIMITS_DEG` - that copy is the hardware's
last-resort gate, this one decides IK reachability before a command gets
that far:

| Joint | Range (relative to that joint's center) |
|---|---|
| `joint_1` | -110.0° – +110.0° |
| `joint_2` | -110.0° – +110.0° |
| `joint_3` | -110.0° – +110.0° |
| `joint_4` | -100.0° – +100.0° |

## `tool_angle_deg` convention

The tool's pointing direction, in degrees from horizontal, measured in the
vertical plane that contains the target point (i.e. relative to the
direction from the base straight toward `(x_m, y_m)`). `0°` = pointing
level, away from the base; `+90°` = pointing straight up; negative =
tilted down. This reference is anchored to the target's own azimuth
(`atan2(y_m, x_m)`), not to any particular joint solution, since the same
tool-tip pose can be reached two different ways (see below) that disagree
on which way the base is actually facing.

## Solving

Two joint-angle families can reach the same tool-tip pose: turn the base
to face the target and reach outward, or turn the base 180° away and fold
the shoulder/elbow/wrist back over the top. `inverse_kinematics` tries
both (and both elbow-bend directions within each), keeps every candidate
that lands inside every joint's safe range and - before trusting it -
reproduces the requested target when run back through `forward_kinematics`.
A target is rejected (`NotReachableError`) if it's geometrically out of
reach, or if every candidate that reaches it needs a joint outside its
safe range.

When more than one candidate is valid, `move_to_pose` picks the one with
the least total joint travel (summed across all 4 joints) from the arm's
*current* joint positions - read fresh from `esp32_bridge` before solving,
the same way `read_tool_pose` does. This keeps the elbow from flipping
between up/down configurations across a sequence of nearby targets. If
that position read fails (see below), `move_to_pose` also fails rather
than guessing a configuration.

## ROS2 interface

| | |
|---|---|
| Service | `/facade_bot/move_to_pose` |
| Service type | `facade_msgs/srv/MoveToPose` |
| Publishes | `/facade_bot/joint_cmd` (`sensor_msgs/msg/JointState`, radians, `ReliabilityPolicy.RELIABLE`) - only on a successful solve |

```bash
ros2 service call /facade_bot/move_to_pose facade_msgs/srv/MoveToPose \
  "{x_m: 0.2, y_m: 0.0, z_m: 0.2, tool_angle_deg: 0.0}"
```

Response is `success` (bool), `message` (`"ok"`, or the specific reason
the target was rejected), and `solved_angles_deg` (the joint angles this
target was solved to, base→end-effector order - only meaningful when
`success` is true; `trajectory_node` uses this to know exactly which
target to confirm arrival at, see below). Because solving now needs the
arm's current joint positions first, this also fails if `esp32_bridge` is
unreachable or a joint read times out/comes back invalid - same failure
modes as `read_tool_pose` below.

## Reading the arm's actual tool-tip pose

`move_to_pose` reports where a *commanded* target was solved to go.
`read_tool_pose` instead reads the arm's real joint angles back from
`esp32_bridge` (via its `/facade_bot/read_joint_positions` service) and runs
them through `forward_kinematics`, so the result reflects where the tool tip
actually is - useful for checking real-world accuracy against a physically
measured position. Doesn't publish or move anything.

| | |
|---|---|
| Service | `/facade_bot/read_tool_pose` |
| Service type | `facade_msgs/srv/ReadToolPose` |

```bash
ros2 service call /facade_bot/read_tool_pose facade_msgs/srv/ReadToolPose {}
```

Response is `success` (bool), `message` (`"ok"`, or why not - `esp32_bridge`
unreachable, the read timed out, or a joint reading was unavailable), and
`x_m`/`y_m`/`z_m`/`tool_angle_deg`.

**Pick a pose away from full extension or full retraction when using this to
check accuracy.** Both are kinematic singularities for this arm (the elbow
angle is exactly 0° or ±180°), so `forward_kinematics` can't reveal an
elbow-branch or per-joint offset error at those poses even if one exists.

Implementation note: answering this service means calling *another* node's
service (`esp32_bridge`'s `read_joint_positions`) and waiting for the reply
from inside this node's own service callback. A ROS2 node can't wait for
that reply on its own main executor - that executor is already busy running
the callback that's doing the waiting, and it can't spin itself
re-entrantly (`rclpy` raises `RuntimeError: Executor is already spinning` if
you try). `facade_control_node` works around this with a second, internal
helper node + its own dedicated executor, used only for this one outgoing
call - see `_read_positions_node`/`_read_positions_executor` in
`facade_control_node.py`.

## Following a trajectory

`trajectory_node` (a second node in this package, separate from
`facade_control_node`) moves the arm through an ordered list of Cartesian
waypoints, one at a time. It's a ROS2 **action** rather than a service,
since following a trajectory takes multiple seconds and callers need
progress feedback and the ability to cancel partway through.

| | |
|---|---|
| Action | `/facade_bot/follow_trajectory` |
| Action type | `facade_msgs/action/FollowTrajectory` |

```bash
ros2 run facade_control trajectory_node
```

```bash
ros2 action send_goal /facade_bot/follow_trajectory facade_msgs/action/FollowTrajectory \
  "{waypoints: [{x_m: 0.2, y_m: 0.0, z_m: 0.2, tool_angle_deg: 0.0}, {x_m: 0.2, y_m: 0.05, z_m: 0.2, tool_angle_deg: 0.0}]}" \
  --feedback
```

For each waypoint in order: calls this node's own `move_to_pose` service
(reusing its IK-solving and least-effort configuration selection as-is),
then confirms the arm actually reached the solved joint angles by polling
`esp32_bridge`'s `read_joint_positions` within a tolerance, before moving on
to the next waypoint. Feedback reports `current_waypoint_index`/
`total_waypoints` after each confirmed arrival. The result reports
`success`, `message`, and `waypoints_completed` (how many waypoints were
actually confirmed-reached, useful for telling how far a failed/canceled
run got).

**V1 stops fully at each waypoint** - there's no blending/continuous motion
between points yet. The user's stated goal is eventually sweeping a
rectangular area, which will need continuous blending through a dense
waypoint sequence rather than stop-and-go; that needs new ESP32
firmware/protocol support (today's `move_joints_min_jerk` is a single
blocking point-to-point primitive with no notion of a queued next segment),
not just a ROS2-side change, and is tracked as future work. The action's
Goal/Feedback/Result shape here is designed to not need to change when that
lands - only the internal "wait for this waypoint to be confirmed
physically reached" step is expected to be replaced.

**Cancellation caveat**: canceling a goal stops `trajectory_node` from
commanding any *further* waypoints - it does not stop the arm mid-move.
There's no firmware/protocol primitive to abort an in-flight physical move,
and emergency-stop logic is still not implemented (see the top-level
`CLAUDE.md`). If a cancel arrives while waiting on a waypoint, the arm
still physically finishes whatever move was already commanded.

## Known limitations

- **Hardware zero vs. geometric zero are not confirmed to match.** This
  file's geometry treats joint angle `0°` as the URDF-neutral pose (arm
  pointing straight up, per the wrinkle noted above). `esp32_bridge_node.py`
  now treats `0°` as each joint's separately measured true center. Whether
  those two zeros are actually the same physical pose has not been
  confirmed - if they're not, IK output will still be off by a per-joint
  constant even with correct joint-limit bounds-checking. This is the
  project's current tracked IK-accuracy blocker.
- **No caller-specified configuration choice.** `move_to_pose` always picks
  the valid candidate closest (by total joint travel) to the arm's current
  position - there's no way to ask for a specific configuration instead
  (e.g. "prefer elbow up regardless of current position"). Also, this
  least-effort choice depends on the current-position read succeeding; it
  can't fall back to elbow-up/down preference on its own.
- **The 50 mm tool offset is a straight-line measurement** along the arm's
  reach direction - it doesn't account for any real nozzle geometry that
  might offset the tool tip sideways or add its own bend.
- **This node never touches hardware directly** - it has no host/port
  parameters and doesn't validate that `esp32_bridge` is even running.
  `esp32_bridge`'s own bounds-check is still the thing that actually gates
  what reaches the servos.
