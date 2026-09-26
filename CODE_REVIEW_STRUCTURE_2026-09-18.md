# Structural Review — 2026-09-18

Scope: the whole control stack at HEAD (`c751abc`) plus the uncommitted working
tree — ESP32 firmware, transport, `esp32_bridge`, `facade_control` (all four
nodes), `facadebot_description`, `facade_msgs`, launch files, and `scripts/`.
Review only — no code was modified.

This sits one level above `CODE_REVIEW_2026-09-18.md` (which is a bug-level
pass and is not repeated here). The question asked was: *where will the
current structure fall behind or fail as the project grows, and which of those
are robotics decisions rather than coding decisions?*

Each finding says what the problem is, why it matters for the arm, what the
options are, and what I would do. Where the choice depends on how the robot is
meant to behave, that is marked **Your call** — the code can go either way,
and the person who understands the hardware should pick.

Verified this session: 33 `facade_control` unit tests pass. `esp32_bridge`
has no unit tests (only lint). Nothing was run against the arm.

---

## Summary, ranked

| # | Finding | Kind | Severity |
|---|---------|------|----------|
| 1 | The firmware cannot be interrupted, so no e-stop design can work until it changes | robotics + firmware | **High** |
| 2 | Two unarbitrated command paths into one arm (`joint_cmd` vs `joint_stream`) | architecture | **High** |
| 3 | `esp32_bridge` is one thread on one blocking socket: every move freezes the whole node | architecture | **High** |
| 4 | `scripts/test_trajectory.py` bypasses every safety gate and is stale | safety | **High** |
| 5 | The safety-critical node has zero tests; the maths node has 33 | process | **High** |
| 6 | IK candidate selection: least-total-travel picks a 184° single-joint swing over a 114° one | robotics | Medium |
| 7 | Speed is a node parameter, not a property of a move | robotics | Medium |
| 8 | The "arrived" tolerance is ±5° per joint, which is ~29 mm at the tool | robotics | Medium |
| 9 | 4-DOF task space: the raster generator will have to invent `tool_angle_deg` per point | robotics | Medium |
| 10 | 16 "must match" constants, and the docs have already drifted (IP `.100` vs `.150`) | maintenance | Medium |
| 11 | A dead bridge probably leaves the ESP32 wedged on a half-open socket | operations | Medium (unverified) |
| 12 | Wi-Fi credentials committed in `esp32_firmware/main.py` | hygiene | Low |
| 13 | Four copies of the helper-node-per-node pattern for one blocking service call | maintenance | Low |
| 14 | `/joint_states` republishes held values with no way for a consumer to tell | observability | Low |

Findings 1, 3 and 11 share one root cause and one fix (see "The one change
that unlocks three findings" after finding 3).

---

## 1. The firmware cannot be interrupted, so no e-stop design can work yet

**Where:** `esp32_firmware/main.py` — `handle_client` (line 171),
`move_joints_min_jerk` (line 127), `main` (line 219).

CLAUDE.md says e-stop is "not started". The bigger point is that it *cannot*
be started on the RPi side, because the ESP32 has no way to hear it:

- The firmware is a single loop. While `move_joints_min_jerk` runs (up to 5 s
  per `move`), it is not reading the socket at all. A `stop` command sent
  during a move would sit in the TCP buffer until the move finished on its
  own.
- `server.listen(1)` and one `handle_client` at a time: a second "emergency"
  connection from another node cannot even be accepted.
- There is no GPIO input, no watchdog, and no heartbeat. If the RPi dies mid
  `move`, the ESP32 finishes the move. If it dies mid-stream, the arm holds
  the last setpoint (which is fine — servos hold position — unless it is
  pressing a tool against a wall).

The LX-16A bus already has the two primitives an e-stop needs. Neither is in
the firmware today:

| Servo command | Effect | Good for |
|---|---|---|
| `SERVO_MOVE_STOP` (cmd 12) | freeze at current position, keep torque | stopping a runaway move but staying up against gravity |
| `SERVO_LOAD_OR_UNLOAD_WRITE` (cmd 31, param 0) | torque off — arm goes limp | guaranteed no force on anything, but the arm will fall |

**Your call (robotics):** what should "emergency stop" physically mean for an
arm hanging on a building facade?

- *Freeze and hold* is safer against gravity and keeps the tool where it is,
  but a stalled servo keeps pushing against whatever it stalled on.
- *Go limp* removes all force instantly but the arm drops under gravity, and
  the tool with it.
- A common compromise: freeze first, then torque-off after a short hold if
  the stop is still asserted. That is a firmware state machine, not much
  code, but the behaviour has to be chosen before it is written.

**Your call (hardware):** where does the stop signal enter?

- A physical pushbutton on an ESP32 GPIO works even if Wi-Fi and the RPi
  are both dead. This is the only option that does not depend on the link.
- A software `stop` over the existing TCP link works only after finding 3 is
  fixed (the firmware has to poll the socket between trajectory steps).
- Both is the normal answer for anything that moves near people.

**What I would do:** GPIO button first (ISR sets a flag; the main loop acts
on it, per the CLAUDE.md ISR rule), plus a software `stop` once the firmware
loop is non-blocking. Have the bridge expose one `/facade_bot/stop` service
and one `/facade_bot/estop_asserted` state topic so every higher node can
refuse to command while it is set.

## 2. Two unarbitrated command paths into one arm

**Where:** `esp32_bridge_node.py` — `_joint_cmd_callback` (line 259) and
`_joint_stream_callback` (line 290); `facade_control_node`, `trajectory_node`
and `continuous_trajectory_node` all publish to one or the other.

Nothing in the graph says who owns the arm. Concretely:

- `trajectory_node` and `continuous_trajectory_node` can both have an active
  goal at once. Both will happily command the same servos.
- A `ros2 topic pub` to `joint_cmd` during a continuous sweep is accepted.
- The failure mode I would expect first on hardware: a `move` arrives while a
  stream is running. The bridge blocks for up to 5 s inside the `move`.
  Stream setpoints pile up in the RELIABLE queue (depth 10), the rest are
  dropped, and when the `move` returns the ten queued setpoints fire back to
  back at full rate — a jump, then the sweep resumes from wherever the plan
  thinks it is, not where the arm is.
- The streaming path has no readback and no fault check (`servo` acks
  immediately). A stall mid-sweep is invisible. `continuous_trajectory_node`
  streams the plan open-loop from start to finish.

**Options:**

1. *Bridge-level busy flag* (cheap, do now): the bridge tracks "a `move` is in
   flight" and "a stream is active" and rejects the other path with an error
   log. Stops the burst-after-move scenario. Does not stop two streaming
   goals.
2. *One motion owner*: a single action interface (one node) that accepts at
   most one goal at a time and preempts or rejects the next. `trajectory_node`
   and `continuous_trajectory_node` become two execution modes of it, and
   `joint_cmd` / `joint_stream` become internal. This is what ros2_control's
   controller manager does for you in the standard stack.
3. *Leave it, rely on the operator.* Fine on a bench with one person typing
   commands. Not fine once the raster generator or vision issues goals.

**Your call:** whether the arm should ever be driven by more than one source
at once (I cannot think of a reason it should). If not, option 2 is the
design, and option 1 is the stopgap that makes the current code safe to keep
testing on.

## 3. `esp32_bridge` is one thread on one blocking socket

**Where:** `esp32_bridge_node.py` — `SingleThreadedExecutor` (line 421),
`_send_move_command` (line 268), `_check_move_completed` (line 344).

Every bridge callback does a synchronous TCP round trip and returns only when
the ESP32 answers. A `move` therefore holds the node for: ESP32 pre-move
readback (~120 ms) + up to 5000 ms shaped move + 200 ms settle + up to five
readback retries. While that runs:

- `/joint_states` stops updating (the 5 Hz publisher's calls time out).
- `read_joint_positions` calls from `trajectory_node` queue behind it —
  which is why that node's poll timeout is 7 s and its stall timeout is
  15 s. Those numbers are a symptom: the whole stack's timeouts are sized
  around the bridge being unresponsive during motion.
- Stream setpoints queue (see finding 2).

This is acceptable for point-to-point bench moves. It will fall behind at
the first thing that needs feedback *during* motion: mid-move stall detection,
a `/joint_states` feed that RViz or vision can trust, or e-stop.

**Options:**

1. *RPi-side worker thread*: run the move on a background thread, protect the
   transport with a lock, let services answer between the ESP32's replies.
   Helps `/joint_states` a little; the ESP32 is still deaf during a move, so
   readbacks still wait.
2. *Non-blocking firmware*: the ESP32 main loop becomes "poll socket, then
   advance the current trajectory by one step". Commands (`read_positions`,
   `stop`, a new `servo` setpoint) are answered between steps, ~150 ms
   worst-case latency. The `move` ack changes from "done" to "accepted", and
   completion becomes a state the bridge can query or is told about.
3. *Both.*

### The one change that unlocks three findings

Option 2 above — a non-blocking firmware loop — is the single highest-leverage
change in the stack. It is what makes a software `stop` possible (finding 1),
lets the bridge see stalls during a move and query state during a stream
(finding 3), and gives the firmware a place to notice a dead client (finding
11). It is roughly 60 lines of MicroPython restructuring in one file.
Everything above it can stay as is initially, because the `move` command's
shape does not have to change — only what the ack means.

**Your call:** this changes the `move` ack semantics that `trajectory_node`
currently relies on (ack = physically done). I would keep a `move` blocking
ack as a compatibility mode and add a `move_async` + `status` pair, so the
verified stop-at-each-waypoint path keeps working while the new one is bench
tested.

## 4. `scripts/test_trajectory.py` bypasses every safety gate and is stale

**Where:** `scripts/test_trajectory.py`.

This script opens its own socket to the ESP32 and sends `move` commands
directly. Nothing about it goes through `_check_joint_bounds`. Worse:

- It sends *absolute* servo angles (0–240° servo frame), not centre-relative
  degrees. The demo drives joint 4 to 20° absolute and 220° absolute (lines
  27–28). With joint 4's centre at 2.25 rad (129°), that is −109° and +91°
  relative — inside the ±110° limit only by chance, and only for joint 4.
- It targets `192.168.1.100`. The firmware is at `.150`. So today it simply
  fails to connect, which is the only reason it is not dangerous.

The firmware's own clamp (`build_move_packet`, raw 0–1000) is the servo's
full mechanical range. It protects the servo, not the arm. Anything that talks
to port 5000 can drive any joint anywhere.

**Options:**

1. Delete the script. The ROS path plus `ros2 topic pub` covers the same
   need with the gate in place.
2. Rewrite it to publish to `/facade_bot/joint_cmd` instead of opening a
   socket.
3. Additionally give the firmware a second fence: the bridge pushes per-servo
   raw min/max to the ESP32 at configure time (`set_limits` command), derived
   from `robot_model.yaml`. The single source of limits stays where CLAUDE.md
   wants it; there are just two enforcers. This is defence in depth for
   exactly the "someone runs a script" case.

**What I would do:** 1 now, 3 when the firmware loop is being reworked
anyway. `scripts/check_servo_bus.py` is fine — it is read-only and runs over
USB.

## 5. The safety-critical node has zero tests; the maths node has 33

**Where:** `ros2_ws/src/esp32_bridge/test/` contains only the three ament
lint tests (which already fail at HEAD for style reasons — see memory).

`esp32_bridge_node.py` holds the only mandatory gate
(`_check_joint_bounds`), the centre-offset conversion
(`_angle_deg_to_position_raw`, which silently clamps), the readback merge,
and the fault check. None of it is exercised by a test. `kinematics.py` and
`trajectory_planning.py`, which cannot physically hurt anything on their own,
have 33.

This matters more for how you intend to work than for the code as it stands.
If the heavy coding is going to be generated, the place a wrong generation
does physical harm is this file, and today nothing would catch it before the
arm moves. The three conversion and gate functions are pure and can be tested
without ROS in an afternoon; the node itself can be tested against a fake
`Esp32Transport` the same way `test_trajectory_node.py` already stubs
services.

**What I would do:** treat a test file for the bridge as the first task,
before any further feature. Also a "hardware-touching change" checklist in
CLAUDE.md: any edit to the bridge, the firmware, or `robot_model.yaml` runs
the bridge tests and a `[0,0,0,0]` move on the bench before anything else.

## 6. IK candidate selection: least-total-travel vs least-peak-joint-travel

**Where:** `kinematics.py` — `travel_deg` inside `inverse_kinematics`
(line 428).

You flagged this on 2026-09-17. I measured it this session against the v2
geometry: 5000 random (current pose, target) pairs.

| | |
|---|---|
| Targets with more than one valid candidate | 4139 of 5000 |
| Cases where least-total (L1) and least-peak (L∞) disagree | 1231 of 4139 (~30%) |
| Worst extra single-joint swing from choosing by L1 | 70° |

One concrete disagreement, current pose `[-37, -90, -43, 42]`:

| Choice | Joint angles | Peak joint travel | Total travel |
|---|---|---|---|
| L1 (current code) | `[-42, 93, -71, -9]` | **184°** | 268° |
| L∞ | `[-42, 8, 71, -9]` | **114°** | 268° |

Both candidates have identical total travel, so the current metric is a tie
and the code picks whichever came first in the list — effectively random.
Because every move gets the same `move_duration_ms` (finding 7), peak joint
travel *is* peak joint velocity, so the L1 choice moves joint 2 60% faster
than necessary for no benefit.

Caveat that matters: for the continuous path, IK is chained one small step at
a time, so the metric almost never decides anything there. It bites on
`move_to_pose` from far away, and on the first point of a sweep.

**Options (your call, robotics):**

1. *L∞* — minimise the largest single-joint move. Minimises peak joint
   velocity for a fixed duration. Tie-break with L1.
2. *Weighted L∞* — divide each joint's travel by that joint's allowed speed
   (or by a hand-picked weight so the base, which swings the whole arm and
   tool, counts more). Minimises actual move time once per-joint speeds
   exist. Better, but needs numbers you do not have yet.
3. *Branch hysteresis* — keep the current elbow branch and base branch unless
   they are out of range, regardless of travel. Most predictable behaviour
   for an operator; can pick a longer move.
4. *Caller chooses* — a `preferred_configuration` field on `MoveToPose`
   (elbow up/down, base front/back). The README already lists this as a
   known limitation.

**What I would do:** 1 now (a two-line change plus a test that reproduces the
table above), and 4 when the raster generator needs to guarantee elbow-up
along a wall. 2 becomes possible after finding 7.

## 7. Speed is a node parameter, not a property of a move

**Where:** `esp32_bridge_node.py` `move_duration_ms` param (line 160);
`sensor_msgs/JointState` as the command type on `/facade_bot/joint_cmd`.

Every `move` takes exactly `move_duration_ms` (default 1 s) regardless of how
far it goes. A 2° correction and a 180° swing take the same second, so joint
speed varies by nearly 100× between moves. The 5° fault tolerance and 200 ms
settle margin are tuned around that. The memory note "velocity control
deferred" records this as intentional for now; the structural cost is that the
command message has no place to put timing when you want it.

`JointState` is a *sensor* message. Using it as a command works but it has no
time field, ignores its own `name` field (order is assumed), and will not be
recognised by any standard tool as a command.

**Options (your call, architecture):**

1. *Custom command message* in `facade_msgs` with `positions_deg[4]` and
   `duration_ms`. Minimal, self-explanatory, stays custom.
2. *`trajectory_msgs/JointTrajectory`* — the standard ROS2 joint command,
   with `time_from_start` per point and joint names. Used by ros2_control and
   MoveIt. A one-point trajectory is a `move`; a many-point one is a stream.
   Adopting it does not force the rest of the standard stack on you, but it
   keeps that door open.
3. *Bridge computes the duration* from the largest joint travel and a
   per-joint max speed in `robot_model.yaml`. Can be combined with either of
   the above as the default when no time is given.

**What I would do:** 2 + 3. This is the "custom stack vs standard stack"
decision in miniature — see the closing section.

## 8. The "arrived" tolerance is ±5° per joint, which is ~29 mm at the tool

**Where:** `esp32_bridge_node.py` `_POSITION_TOLERANCE_DEG` (line 68);
`trajectory_node.py` `_WAYPOINT_POSITION_TOLERANCE_DEG` (line 46).

The arm's reach is roughly 335 mm (135 + 140 + 60). A 5° error in the base
or shoulder at full reach moves the tool ~29 mm. `trajectory_node` advances to
the next waypoint as soon as every joint is within 5°, so a waypoint can be
called "arrived" with the tool 2–3 cm off. That is also why the 100×50 mm
rectangle closing "within ~2 mm" was a good result: the check is far looser
than the arm actually achieved.

For inspection this is fine. For painting or cleaning a facade, "close
enough" is a Cartesian question, not a per-joint one.

**Your call:** what tool-tip error is acceptable per task. Then the arrival
check should run the read-back joints through forward kinematics and compare
in millimetres, which `read_tool_pose` already does. The joint-degree check
can stay as a cheap first pass.

## 9. 4-DOF task space: the raster generator will have to invent `tool_angle_deg`

**Where:** `MoveToPose.srv`, `Waypoint.msg`, `kinematics.py` docstring.

The task space is `(x, y, z, joint_4 angle)`. With four joints that is the
correct count of numbers, but the fourth one is a *joint* angle, not a tool
property. The tool's orientation relative to the wall is whatever falls out of
the other three joints plus that value. For a raster over a wall, the caller
must compute, per point, which joint_4 value keeps the tool roughly normal to
the surface — and that depends on where the base is relative to the wall.

This is the next feature in your stated order (raster generator), so the
decision is imminent.

**Options (your call, robotics):**

1. Keep `(x, y, z, joint_4)` and make the raster generator do the per-point
   joint_4 calculation. Simple, but every future caller repeats it.
2. Define the task in a *wall frame*: the generator takes a wall plane (or
   just "the wall is the plane x = X_wall") and a rectangle in it, and the
   planner chooses joint_4 to keep the tool as normal as the 4 DOF allow.
   Puts the geometry in one place.
3. Add a 5th joint. Not a software question.

**What I would do:** 2, as a function in `trajectory_planning.py`, before
writing the raster generator against the current API.

## 10. Sixteen "must match" constants, and the docs have already drifted

**Where:** `grep -rn "must match\|keep in step" ros2_ws/src esp32_firmware`
finds 16 sites. Examples: servo count, raw range, IP, port, topic and service
names, tolerance values, stream period vs servo duration.

The drift has already happened:

- Firmware and bridge use `192.168.1.150`. `LAUNCH.md` (three places), the
  bridge README (two places) and `scripts/test_trajectory.py` say `.100`.
  The troubleshooting step "ping 192.168.1.100" will fail on a healthy
  system.
- The bridge README's first paragraph still says "No bounds-checking and no
  e-stop yet". Bounds-checking has existed for months.
- `MoveToPose.srv` / `Waypoint.msg` comments still describe `tool_angle_deg`
  as pitch (already in the bug-level review).

None of these break anything today. The pattern is what fails: a
protocol-shape change on one side of the Wi-Fi link is not detected until a
move misbehaves (LAUNCH.md's own troubleshooting entry admits this).

**What I would do:**

1. A protocol version in the firmware's `{"status": "ready"}` greeting,
   checked by the transport at `connect()`. A mismatched flash then fails at
   `configure`, not mid-move. Ten lines total.
2. Topic and service names in one small module (say
   `facade_msgs`-adjacent `facade_names.py`) that every node imports. The
   seven string copies disappear.
3. Host and port in a launch-file parameter file, not defaults in two
   Python files.

## 11. A dead bridge probably leaves the ESP32 wedged (unverified)

**Where:** `esp32_firmware/main.py` `main` (line 229) and `handle_client`.

If the bridge process dies without a clean TCP close (kill -9, RPi power
loss, Wi-Fi drop), the ESP32 stays inside `handle_client` blocked on
`f.readline()` for a connection that no longer exists. It never sends, so it
never learns the peer is gone. A restarted bridge connects, lands in the
`listen(1)` backlog, never receives the greeting, and times out after 7 s
with "failed to connect". The only recovery is an ESP32 power cycle.

I have not reproduced this on hardware; it follows from the code and from how
TCP half-open connections behave. It is worth a five-minute bench test
(`kill -9` the bridge, restart it) because it will otherwise show up as an
unexplained "can't connect" during a session.

**Fix:** a socket timeout on the ESP32 side plus a periodic heartbeat from
the bridge, or simply TCP keepalive on both ends. Falls out naturally from
the non-blocking loop in finding 3.

## 12. Wi-Fi credentials committed in the firmware

`esp32_firmware/main.py` lines 9–10 contain the SSID and password in a
public-looking repo. Move them to a `secrets.py` that is gitignored and
imported by `main.py`. Also note for later: the ESP32 accepts commands from
anything on the LAN with no authentication. Fine on a bench network; not fine
on a job site's Wi-Fi.

## 13. Four copies of the helper-node-per-node pattern

`facade_control_node`, `trajectory_node`, `continuous_trajectory_node` and
`joint_state_publisher_node` each create a second hidden node and executor
to make one blocking service call. It works, and the launch-name bug it
already caused is documented. It is a maintenance trap for generated code:
the next node will copy it, possibly wrong. Either extract one
`BlockingServiceCaller` helper or move to async callbacks with a
`ReentrantCallbackGroup` (which `trajectory_node` already uses). Low
priority; do it when one of these nodes is next touched.

## 14. `/joint_states` republishes held values with no way to tell

`joint_state_publisher_node` holds a joint's last angle through a missed
read (which happens about half the time). The published message carries no
indication of which joints are fresh. RViz does not care. Vision or a
stall detector will. Cheap fix: publish a parallel `/facade_bot/joint_states_age`
or put the per-joint miss count in the `effort` field with a note. Or fix the
bus flakiness the memory notes already flag (`check_servo_bus.py` pass).

---

## The bigger decision: custom stack or standard stack

Several findings (2, 3, 7, 13) are things the standard ROS2 motion stack —
ros2_control for the hardware interface and controller arbitration, a
`JointTrajectoryController`, optionally MoveIt for planning — already solves.
It is fair to ask whether FacadeBot should move onto it rather than keep
building its own.

The honest assessment for *this* arm:

- ros2_control hardware interfaces are C++ and would replace the Python
  bridge. Doable, but it moves the hardest code to the language you are
  least able to read.
- MoveIt on a 4-DOF arm works with a position-only IK plugin but is fiddly,
  and its planners are overkill for rasters on a plane.
- What you have is small, readable, and verified on hardware. That is worth
  a lot for someone who wants to reason about the code rather than trust it.

**My recommendation:** stay custom, but adopt the standard *interfaces* where
they cost nothing — `trajectory_msgs/JointTrajectory` for commands (finding
7), `JointState` with names on `/joint_states` (already done), a single
motion-owner action (finding 2). That keeps the door to ros2_control open
without walking through it, and it makes the custom code look familiar to
anyone who joins later.

## Suggested order

1. Bridge unit tests (finding 5) — before anything else moves.
2. Bridge busy flag (finding 2, option 1) and delete the direct script
   (finding 4) — an afternoon, removes the two most likely accidents.
3. Decide e-stop semantics and signal path (finding 1) — a conversation, not
   code.
4. Non-blocking firmware loop with protocol version + `stop` + keepalive
   (findings 1, 3, 10, 11) — one focused firmware change, bench tested with
   the existing `move` path unchanged.
5. IK metric to L∞ (finding 6) — small, testable, closes your open question.
6. Wall-frame task definition (finding 9) before the raster generator.
7. `JointTrajectory` command message with duration (finding 7) when the
   raster generator needs speed control anyway.

---

## Appendix: bug-level pass (high effort, same day)

A separate bug-level review was run over the same tree. Ten findings came
back. Five repeat `CODE_REVIEW_2026-09-18.md` (joint-centre provenance,
stale `tool_angle_deg` docs, `print()` in node startup, the unverified
geometry hash, the weakened elbow-flip test) and are not repeated. The five
new ones, with what I re-verified myself:

| # | Finding | Verified |
|---|---------|----------|
| B1 | **`bringup.launch.py` re-activates the bridge on any transition into `inactive`.** The handler at line 66 filters on `goal_state="inactive"` only. A manual `ros2 lifecycle set /esp32_bridge_node deactivate` — currently the only "stop accepting commands" primitive the project has — is undone within milliseconds while bringup is running. A failed `on_activate` loops forever. Fix: add `start_state="unconfigured"` or `transition="configure"` to the matcher, or use a one-shot handler. | Confirmed from launch_ros source: the matcher tests only the fields given (`on_state_transition.py` lines 75–82). |
| B2 | **The all-zero home pose is unreachable to IK.** `inverse_kinematics(*forward_kinematics(0,0,0,0))` raises "outside the arm's reach" on v2, because the tool tip sits on joint_1's axis and `_base_angle_candidates_rad` (line 332) returns nothing when the out-of-plane distance is ~0. Any base angle is valid there; the current one should be returned. Practical effect: `move_to_pose` cannot send the arm to straight-up, and a sweep crossing the base axis is rejected before motion with a misleading message. | Confirmed this session. |
| B3 | **Full-extension poses are rejected by floating-point noise.** `_elbow_solutions_rad` (line 348) rejects `cos_interior > 1.0` exactly; a fully extended target computed by FK lands at 1.0 + 1e-16 about 1% of the time. Clamp with a small epsilon before `acos`. | Confirmed: 4 of 500 random theta3 = 0 poses fail the round trip. |
| B4 | **`urdf_zero_offset_deg` rejects integer overrides.** Declared from a float list, so `-p urdf_zero_offset_deg:="[0,0,5,0]"` is an integer array and rclpy raises at declare time; only `RobotModelError` is caught in `main()`, so the node dies with a traceback. | Not re-run; consistent with rclpy parameter typing. |
| B5 | **Timed-out service futures are never cancelled** in any of the four helper-node copies. With the bridge stalled (finding 3), `joint_state_publisher_node` adds one orphaned pending request per tick. Strengthens finding 13: extract one helper and fix it once. | By reading; not load-tested. |

B1 belongs with finding 1: until e-stop exists, `deactivate` is the closest
thing to a stop, and bringup silently defeats it. B2 and B3 belong with the
"IK not yet re-verified on hardware" item in CLAUDE.md's Current blocker —
both would show up as unexplained "unreachable" rejections during that check.
