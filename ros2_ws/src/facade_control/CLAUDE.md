# facade_control — CLAUDE.md

Everything between "a Cartesian target" and "joint angles on a topic". Four nodes,
two pure-maths modules, two launch files. Never opens a connection to the ESP32.
Detailed interface docs are in `README.md`; this file is the invariants and traps.

## Layout

| File | Role | ROS interface |
|------|------|---------------|
| `kinematics.py` | Closed-form FK/IK for the loaded model. Pure, no ROS | — |
| `trajectory_planning.py` | Corner-rounded path → constant-speed samples → chained IK. Pure, no ROS | — |
| `facade_control_node.py` | Cartesian target → IK → publish joint command | `/facade_bot/move_to_pose`, `/facade_bot/read_tool_pose` |
| `trajectory_node.py` | Stop-at-each-waypoint following. Verified on hardware, but **re-verify after the D4 arrival-check change** | action `/facade_bot/follow_trajectory` |
| `continuous_trajectory_node.py` | Constant-speed streaming. **Not verified on hardware** | action `/facade_bot/follow_trajectory_continuous` |
| `joint_state_publisher_node.py` | Polls the fast read service, publishes `/joint_states` | `/joint_states` |
| `launch/bringup.launch.py` | Bridge + control + trajectory + joint states, auto-activates the bridge | — |
| `launch/bringup_sim.launch.py` | Same four nodes plus `scripts/fake_esp32_server.py` as a subprocess, bridge hardcoded to `esp32_host=127.0.0.1`. No real hardware involved; kept separate from `bringup.launch.py` on purpose (user's call, 2026-09-19) so the real bring-up path has no parameter to mistype | — |
| `launch/display.launch.py` | `robot_state_publisher` + RViz. Visualization only, own launch file by the user's call | — |

## Invariants

- No geometry is hardcoded. `kinematics.configure()` takes a `RobotModel` and
  validates that the arm has the shape the closed form handles (base yaw, two
  parallel-axis joints, wrist axis out of plane). A different shape is refused, not
  approximated.
- `tool_angle_deg` is **joint_4's own angle**, the tool's tilt out of the arm plane.
  It is not tool pitch (that was v1). The `.srv`/`.msg` comments and README say so.
- IK collects every valid candidate and picks by least total joint travel from the
  arm's current position. The current position is always read live from
  `esp32_bridge` (never cached). The metric itself is under review (structural
  finding 6); changing it is the user's call.
- A move is rejected, not attempted, if the current-position read fails, the target
  is unreachable, or any joint would leave its safe range.
- Continuous trajectories validate the whole joint path before any setpoint is sent.
- Every node that must block on another node's service does it through a separate
  helper node and executor. Four copies exist. If you touch one, consider extracting
  the helper (finding 13) rather than adding a fifth copy.

## Traps already hit

- Giving a launch `Node` an explicit `name=` injects a process-wide remap that also
  renames the helper node, producing "Publisher already registered" and a duplicate
  in `ros2 node list`. Leave `name=` unset for the three nodes that create helpers.
- `bringup.launch.py`'s activate handler once fired on **any** transition into
  `inactive`, undoing a manual `deactivate`. It now matches
  `start_state="configuring"` only. A lifecycle node reaches `inactive` from
  configure, deactivate, and a failed activate; match the start state, not just the
  goal.
- IK used to reject the all-zero home pose (tool on the base axis, base angle
  undefined) and ~1% of full-extension poses through floating-point noise. Fixed
  2026-09-18: on-axis targets keep the current base angle; the elbow cosine has a
  1e-9 slack. Both have regression tests. Not yet re-run on the arm.
- The ±5° arrival tolerance is ~29 mm at full reach. Whether that is acceptable per
  task is the user's decision (finding 8).
- Arrival is `not moving AND within tolerance`, never position alone. Since the
  firmware loop became non-blocking (D4) a read is answered mid-move, so a waypoint
  whose start pose already sits within tolerance would otherwise report "arrived"
  before the arm moved, and the next waypoint would land on an in-flight move.
  `_is_arm_moving()` treats an unanswerable query as *moving* on purpose.
- There is no stop topic (the bench stop switch was removed 2026-09-26, user's
  decision). Cancelling a goal stops further commands only; the arm finishes the
  move in flight. The servo power plug is the only stop.
- `%360` wrap-around once broke the bounds check; `_normalize_deg` folds to
  (−180, 180] on purpose.

## Tests

38 tests, all pure-maths and node-against-stubs. `conftest.py` builds the model from
the real generated geometry with stand-in limits.
```bash
cd ros2_ws && source /opt/ros/jazzy/setup.bash && source install/setup.bash
python3 -m pytest src/facade_control/test -q -k 'not flake8 and not pep257 and not copyright' -p no:cacheprovider
```
Run this package's tests on their own; `esp32_bridge` shares test file names.
Rebuild before running if code changed: `colcon build --packages-select facade_control`.
