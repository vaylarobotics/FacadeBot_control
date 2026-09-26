# facade_control

Turns a Cartesian tool-tip target into joint angles and publishes them to
`/facade_bot/joint_cmd`, the same topic `esp32_bridge` already subscribes
to. Never opens a connection to the ESP32 itself - it only publishes an
existing message type using existing units/QoS, so `esp32_bridge` needs no
changes to receive these commands.

## Arm geometry

Nothing about the arm's shape is written into this package. `kinematics.py` is
configured at startup from the model that `facadebot_description`'s
`robot_model.yaml` names (`active_model`, currently `v2`): link offsets, joint
origins and axes come from the generated `geometry_v2.yaml`, the joint limits
and the joint_4-to-tool-tip offset from the hand-measured entries in
`robot_model.yaml`. Every control node prints those numbers at startup; that
banner is the geometry in force.

The closed-form solver handles exactly this shape and refuses anything else
(`ModelStructureError`): a base yaw (`joint_1`), a shoulder and elbow rotating
about parallel axes so they sweep one plane (`joint_2`, `joint_3`), and a wrist
(`joint_4`) whose axis leaves that plane. On v2, joint_4's rotation tips the
tool sideways out of the arm plane rather than pitching it within the plane as
it did on v1; the solver reduces to the v1 case when the wrist axis happens to
be parallel to the elbow's, so the retired model still solves.

At all joint angles = 0 the arm points straight up (the URDF's neutral pose).
`esp32_bridge` maps that 0 to each joint's separately measured centre
(`_JOINT_CENTER_RAD`), and joint limits are measured relative to those centres.
Whether the two zeros coincide on the physical v2 arm is still unverified (see
`joint_state_publisher_node`'s `urdf_zero_offset_deg`).

## `tool_angle_deg` convention

`tool_angle_deg` is **joint_4's own angle**, in degrees: how far the wrist tips
the tool out of the shoulder/elbow plane. `0` keeps the tool in that plane.
It is not the tool's pitch from horizontal; that was the v1 meaning, and the
`.srv`/`.msg` comments now say so. `read_tool_pose` returns the same quantity,
which is what makes forward and inverse kinematics exact inverses.

## Solving

Several joint configurations can reach the same tool-tip pose. With joint_4
fixed by `tool_angle_deg`, the wrist and tool collapse into one rigid vector
off the elbow; the base angle then has at most two solutions (the two
directions that put the target the right distance out of the arm plane), and
each of those has two elbow bends. A target on the base axis, such as the
straight-up home pose, is reachable at any base angle, and the solver keeps the
current one. `inverse_kinematics` tries every combination, keeps every candidate
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
then confirms the arm actually reached the solved joint angles before moving
on to the next waypoint. Arrival means **both** that `esp32_bridge`'s
`is_moving` reports the arm has stopped **and** that `read_joint_positions` is
within tolerance of the solved angles. Position alone is not sufficient: since
the ESP32 firmware loop became non-blocking, a position read is answered while
the arm is still moving, so a waypoint whose start pose already sits within
tolerance of its target would otherwise report "arrived" before the arm had
moved at all — and the next waypoint would be commanded on top of an in-flight
move. An `is_moving` query that cannot be answered counts as *still moving*, so
a dead bridge ends the waypoint as a stall rather than as an early arrival. Feedback reports `current_waypoint_index`/
`total_waypoints` after each confirmed arrival. The result reports
`success`, `message`, and `waypoints_completed` (how many waypoints were
actually confirmed-reached, useful for telling how far a failed/canceled
run got).

**`trajectory_node` stops fully at each waypoint** - it's for precise
point-to-point positioning. For continuous motion that flows through the
waypoints at a steady speed (e.g. sweeping a surface), use
`continuous_trajectory_node` below instead; the two are separate nodes with
separate actions and both remain available.

**Cancellation caveat**: canceling a goal stops `trajectory_node` from
commanding any *further* waypoints - it does not stop the arm mid-move. The
firmware can now *report* whether it is moving (`/facade_bot/is_moving`), but
there is still no primitive to abort an in-flight physical move, and
emergency-stop logic is still not implemented (see the top-level `CLAUDE.md`;
a software stop is deferred pending `DECISIONS.md` D1/D2). If a cancel arrives
while waiting on a waypoint, the arm still physically finishes whatever move
was already commanded.

## Following a trajectory continuously

`continuous_trajectory_node` (a third node in this package) sweeps the tool
tip through the waypoints at a **constant Cartesian speed**, rounding corners
rather than stopping at each point - for even coverage when painting or
cleaning a surface.

| | |
|---|---|
| Action | `/facade_bot/follow_trajectory_continuous` |
| Action type | `facade_msgs/action/FollowTrajectoryContinuous` |

```bash
ros2 run facade_control continuous_trajectory_node
```

```bash
ros2 action send_goal /facade_bot/follow_trajectory_continuous facade_msgs/action/FollowTrajectoryContinuous \
  "{waypoints: [{x_m: 0.20, y_m: -0.05, z_m: 0.20, tool_angle_deg: 0.0}, {x_m: 0.20, y_m: 0.0, z_m: 0.20, tool_angle_deg: 0.0}, {x_m: 0.20, y_m: 0.05, z_m: 0.20, tool_angle_deg: 0.0}], tool_speed_mmps: 30.0, corner_blend_m: 0.02}" \
  --feedback
```

The Goal adds `tool_speed_mmps` (constant tool-tip speed) and `corner_blend_m`
(how far from each interior waypoint the path is allowed to curve; `0` means
sharp corners). The whole motion is planned and validated **before any
movement**: it builds a corner-rounded Cartesian path
(`trajectory_planning.build_blended_path`), samples it at the requested speed
with a smooth speed-up/slow-down at the ends
(`sample_at_constant_speed`), and solves every sample to joint angles with
chained IK so the elbow configuration stays continuous
(`solve_path_to_setpoints`). If any point on the path is unreachable or
out-of-range, the goal is aborted before the arm moves. It then streams the
joint setpoints to `/facade_bot/joint_stream` (which `esp32_bridge` forwards
as the non-blocking `servo` command), paced by the `stream_period_sec`
parameter. Feedback reports `fraction_complete` (0.0-1.0); the result reports
`success`, `message`, and `fraction_completed`.

**Why round corners:** true constant speed and sharp corners are physically
incompatible (an instant direction change needs infinite acceleration), so the
path curves within `corner_blend_m` of each interior waypoint instead of
passing exactly through it. Straight/collinear runs stay straight, so a
raster's parallel passes are unaffected and only the U-turns get rounded.

**Constraints and tuning:**
- `stream_period_sec` (default 0.12 s) must stay `>=` the rate `esp32_bridge` +
  the servo bus can actually forward (~80 ms to write all 4 servos at the
  current inter-servo delay, ~12 Hz), and should match `esp32_bridge`'s
  `servo_move_duration_ms`. Streaming faster backs up the topic queue and makes
  the arm lag the plan.
- Wi-Fi jitter on the streamed setpoints causes small velocity ripple.
- Corner acceleration is roughly `speed^2 / corner_blend_m`; very tight corners
  at high speed may need a lower speed. Curvature-based corner speed-limiting is
  future work.

**Cancellation caveat** is the same as `trajectory_node`'s: canceling stops
further setpoints, but the arm coasts to the last one already commanded. There is
no stop control in software or firmware (the bench stop switch was removed
2026-09-26); the only stop is the servo power plug.

**Streaming and shaped moves do not mix politely.** At the firmware level a
`servo` stream setpoint *preempts* a shaped `move_async` still in flight - both
drive the same servos over the same UART, so the stepper is cancelled rather
than left to fight the stream - and a shaped move arriving mid-motion is refused
as `busy`.

`esp32_bridge` sits in front of both and puts `joint_cmd` and `joint_stream` in
one mutually exclusive callback group, so in practice commands from this package
**serialise** and neither case is normally reached. What that does *not* give you
is arbitration between goals: two action goals can still interleave their
setpoints in the queue, the arm just executes the interleaving one command at a
time. Do not run `trajectory_node` and `continuous_trajectory_node` against the
arm at the same time (`DECISIONS.md` D3; one motion owner is still the design).

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
