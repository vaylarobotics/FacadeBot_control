# Open Decisions

Robotics and architecture choices that the code can go either way on and that
the user, not Claude, decides. Each one lists the options as they were presented,
Claude's recommendation, and a blank answer line. Fill in the answer whenever you
get to it; a session that touches the affected area reads this file first and
treats a filled-in answer as settled.

Source for most of these: `CODE_REVIEW_STRUCTURE_2026-09-18.md` (finding numbers
in brackets). Dates are when the question was raised.

---

## D1. What does emergency stop physically do? [finding 1] — 2026-09-18

The LX-16A bus supports two primitives, neither in the firmware yet.

| Option | Effect | Tradeoff |
|--------|--------|----------|
| Freeze and hold (`SERVO_MOVE_STOP`, cmd 12) | Servos stop at their current position and keep torque | Arm stays up against gravity and tool stays put, but a stalled servo keeps pushing against whatever it stalled on |
| Torque off (`SERVO_LOAD_OR_UNLOAD_WRITE`, cmd 31, param 0) | Arm goes limp | No force on anything, but the arm and tool drop under gravity |
| Freeze, then torque off after a short hold if stop is still asserted | Both, in sequence | A small firmware state machine; behaviour has to be chosen before it is written |

Recommendation: freeze first, torque-off after a hold, but this depends on what
the tool is and what is below the arm on a facade.

Both primitives are confirmed native LX-16A bus commands (2026-09-21): `SERVO_MOVE_STOP`
is `55 55 ID 03 0C CHK`, `SERVO_LOAD_OR_UNLOAD_WRITE` is `55 55 ID 04 1F P CHK`
(P=0 unload, 1 load); neither replies, and both accept broadcast ID `0xFE` so one
packet covers all four joints. Note that `SERVO_MOVE_STOP` only aborts the servo's own
interpolation — a firmware stop must also kill the trajectory stepper and refuse
incoming motion, or the next setpoint resumes the move within ~120 ms.

**Removed 2026-09-26 (user):** "get rid of the stop switch, actually feels pretty useless
now." The GPIO 27 bench switch and everything built on it (firmware stop, bridge polling
and `/facade_bot/estop_active`, trajectory-node aborts, simulator switch, tests) was
deleted. There is no stop in firmware or software; the servo power plug is the only stop.
The history below is kept for the record. D1/D2 are open again for whenever a stop is
wanted.

**Answer:** **Deferred, not decided** (2026-09-21, user): "let's keep the estop for later,
honestly it's pretty low priority." Do not raise D1/D2 as the blocking next step, and do
not build e-stop unprompted. (Superseded for the bench on 2026-09-26: **freeze and hold**,
`SERVO_MOVE_STOP` broadcast plus killing the stepper and refusing motion, triggered by
the GPIO switch in D2. Torque-off is not used.) The consequence is that root `CLAUDE.md`'s rule — no code
that moves the arm counts as complete without e-stop — keeps every motion feature at
"bench-verified" rather than "done", and bench runs stay bounded by the standing rule
that nobody is in the arm's reach. Revisit before anything runs unattended or on a real
facade.

## D2. Where does the stop signal enter? [finding 1] — 2026-09-18

| Option | Works when | Depends on |
|--------|-----------|------------|
| Physical pushbutton on an ESP32 GPIO | Always, even with Wi-Fi and the Pi dead | Firmware ISR + main-loop handling |
| Software `stop` over the existing TCP link | Only after the firmware loop is non-blocking (D4) | Wi-Fi up, Pi up, bridge up |
| Both | | The usual answer for anything moving near people |

Recommendation: both, button first.

**Answer:** **Deferred with D1** (2026-09-21) — same reasoning, same revisit trigger.
The GPIO switch below was built 2026-09-26 and removed the same day (see D1).

**Superseded 2026-09-26 (user) — bench interim stop:** a plain toggle switch between
**GPIO 27 and GND**, internal pull-up, closed = run, open = stop (so a broken wire also
stops). An interrupt only sets a flag; the main loop acts on it. **Resume is the switch
alone** — flipping it back re-enables motion, no `reset` command — but everything that was
queued or in progress when the stop hit is scrapped, so nothing old reaches the servos
after release. Motion commands are refused with `{"status": "estopped"}` while stopped;
reads keep working, so the software keeps running. Ships in **the same reflash as the
non-blocking firmware (D4)**. Limits accepted for the bench: depends on the firmware being
alive (servo power plug stays the backup), ~150–200 ms worst-case reaction. The software
`stop` over TCP is still not built. Design of how "scrap the queue" reaches upstream nodes
was confirmed with the user and implemented the same day (unit tests and simulator
only). Follow-up decided in the safety review (user, 2026-09-26): a shaped move
never starts from a guessed pose. The firmware re-reads a joint whose start
position missed (up to 5 attempts) and refuses the move if it still cannot be read.
This also answers the 2026-09-20 open item on seeding the min-jerk start pose.

## D3. Should more than one source ever command the arm at once? [finding 2] — 2026-09-18

Today `joint_cmd` and `joint_stream` are unarbitrated, and `trajectory_node` and
`continuous_trajectory_node` can both hold an active goal.

| Option | What changes |
|--------|--------------|
| No: one motion owner | One action interface holds at most one goal; the two trajectory nodes become modes of it; the raw topics become internal |
| Stopgap only | A busy flag in the bridge rejects a stream setpoint during a move and vice versa; two streaming goals are still possible |
| Leave it | Operator discipline only. Fine on a bench, not once vision or a raster generator issues goals |

Recommendation: the stopgap now, one motion owner as the design.

**Answer:** (2026-09-20) The stopgap, implemented **in the firmware** rather than as
a bridge flag: a `move`/`move_async` arriving while a shaped motion is active is
refused with `{"status": "busy"}` and dropped — not queued, not retried. The bridge
logs a warning. `servo` deliberately keeps preempting, since retargeting is the whole
point of the streaming path and the stepper would otherwise fight the stream on the
same UART. Two streaming goals are still possible; one motion owner remains the
design, unbuilt.

**Amendment, same day, after the safety review:** the bridge also had to put its two
command topics in one `MutuallyExclusiveCallbackGroup` (the multi-threading otherwise
let two joint commands race for the transport). The side effect is that on the normal
ROS2 path commands now **serialise rather than being refused** — a second `joint_cmd`
waits its turn and then runs. Verified in the simulator: two overlapping moves both
executed, in order, and no `busy` was seen. The firmware refusal is now the backstop
below that, reached mainly when the bridge's `status` poll times out. This satisfies
the intent ("never change direction mid-swing") more completely than dropping did, but
it is not literally what was chosen here, and it introduces a new question: a queued
command is never stale-checked, so a burst of ten executes in full however old each
target has become. Flag for the user — capping or stale-dropping the queue is a
separate decision.

## D4. Make the firmware loop non-blocking? [finding 3] — 2026-09-18

The ESP32 does not read its socket during a move (up to 5 s). Making the main
loop "poll socket, advance trajectory one step" unlocks a software stop (D2),
mid-move stall detection, and dead-client detection, in one firmware change.
Cost: the `move` ack changes meaning from "physically done" to "accepted", which
`trajectory_node` currently relies on.

| Option | |
|--------|--|
| Yes, keep a blocking `move` for compatibility and add `move_async` + `status` | Verified path untouched while the new one is bench tested |
| Yes, change `move` in place | Smaller protocol, but re-verifies `trajectory_node` on hardware |
| Not now | E-stop stays button-only; streaming stays blind |

Recommendation: the first.

**Answer:** (2026-09-20) The first — implemented. `move_async` (acks on accept) and
`status` (`{"moving": bool}`) added; the blocking `move` is untouched and reachable
via the bridge's new `use_blocking_move` parameter, so the hardware-verified path can
be restored without a reflash. The bridge polls `status` in place of the blocking ack
and gained a `MultiThreadedExecutor`, a transport lock, and `/facade_bot/is_moving`.
`trajectory_node` no longer relies on the ack: arrival is now "not moving AND within
tolerance". Verified in tests and against the simulator; **not yet run on the arm**.
The software stop this unblocks is still deferred pending D1/D2.

## D5. Which IK solution to prefer when several are valid? [finding 6] — 2026-09-17

Measured on v2 geometry over 5000 random moves: least-total-travel (current)
disagrees with least-peak-joint-travel about 30% of the time when there is a
choice, and in ties it picks by list order. Worst case seen: a 184° single-joint
swing chosen over a 114° one with identical total travel.

| Option | Minimises | Needs |
|--------|-----------|-------|
| Least peak joint travel (L∞), tie-break by total | Peak joint velocity for a fixed move duration | Nothing new; two lines plus a test |
| Weighted by per-joint max speed | Actual move time | Per-joint speed numbers you do not have yet |
| Branch hysteresis: keep the current elbow/base branch unless impossible | Surprise for an operator | Nothing new; may pick longer moves |
| Caller chooses via a `preferred_configuration` field | Whatever the caller wants | A message change |

Recommendation: L∞ now; caller-chooses when the raster generator needs to hold
elbow-up along a wall.

**Answer:** (2026-09-21) **Option 1 — least peak joint travel (L∞), tie-break by total.**
Decided, **not yet written**; no code has changed. The change is in
`facade_control/kinematics.py`'s `inverse_kinematics`, which today scores candidates with
an unweighted L1 sum (`travel_deg`, ~line 451) and selects `min(in_range, key=travel_deg)`:

- Score each candidate by the **largest** single-joint change from `current_angles_deg`,
  falling back to the L1 sum only to break a tie. This also removes the current arbitrary
  tiebreak, where `min()` keeps whichever candidate the generators happened to emit first
  (base `phase+offset` before `phase-offset`, elbow `+interior` before `-interior`).
- joint_4 is identical across all candidates (it is pinned to the caller's
  `tool_angle_deg`), so it contributes nothing to the ranking under either metric —
  in effect this ranks joints 1–3.
- The `current_angles_deg is None` path still returns `in_range[0]` by list order; this
  decision does not change that, and it is worth deciding separately whether that
  fallback should exist at all.
- Rationale: every move runs for a fixed `move_duration_ms` regardless of distance
  (see D6), so peak joint velocity is set by the largest joint travel, not the sum.
  If D6 later makes duration proportional to travel, revisit — the two interact.

Also write the `TESTS.md` **Wanted** row "IK prefers the smaller peak-joint swing",
which was blocked on this answer and is now unblocked. Closes Current-blocker item 2 in
`STATUS.md` as a *decision*; the blocker stays open until the code and test land.

## D6. Speed as a property of a move, and which message carries it [finding 7] — 2026-09-18

Every move takes `move_duration_ms` (1 s) regardless of distance, so joint speed
varies ~100× between moves. `JointState` has no time field.

| Option | |
|--------|--|
| Custom command message in `facade_msgs` with positions and `duration_ms` | Minimal, stays custom |
| `trajectory_msgs/JointTrajectory` (standard; `time_from_start` per point) | Keeps the door to ros2_control and MoveIt open; one-point = move, many = stream |
| Bridge computes duration from largest joint travel and a per-joint max speed in `robot_model.yaml` | Can combine with either of the above as the default |

Recommendation: `JointTrajectory` plus the computed default.

**Answer:** (2026-09-26) **Computed from a per-joint speed in YAML; no time field on the
command.** `/facade_bot/joint_cmd` stays `JointState`. Each joint gets a
**peak** speed (deg/s) in `robot_model.yaml`, next to its limits, and the bridge converts
it to the `duration_ms` the servos need:

    duration = max over joints of (travel_deg / peak_speed_deg_per_s) * 1.875

- **Joints finish together** (coordinated move, one duration for all four): the joint with
  the longest travel/speed ratio runs at its cap, the others slower. No firmware change
  needed for this — the min-jerk stepper already uses one duration.
- **The number is a peak, not an average**: 1.875 is min-jerk's peak-to-average velocity
  ratio, so the joint never exceeds the YAML value mid-move.
- Scope: point-to-point moves only (`move_to_pose`, `trajectory_node`, raw `joint_cmd`).
  The streamed `servo` path keeps its timing from `tool_speed_mmps`.
- A per-move time can still be added later (e.g. `JointTrajectory`, consistent with D10)
  if a caller needs one; not built now.
- D5 still holds: with duration proportional to peak travel/speed, L∞ minimises move time.
- **Speed values:** 60 deg/s peak on all four joints (user, 2026-09-26), a bench starting
  point, not a measured limit.
- **Moves longer than the firmware's 5000 ms `duration_ms` clamp: the bridge splits them**
  (user, 2026-09-26, "so it's easier to debug"). Firmware and its clamp stay unchanged.
  The bridge cuts the move into equal pieces of at most 5000 ms along the straight
  joint-space line, sends each as its own shaped move, and waits for `status` to report
  stopped before sending the next. Each piece still obeys the peak cap. Accepted
  consequence: the arm eases to a stop at each piece boundary (at 60 deg/s that is any
  move with a single-joint travel over ~160 deg). Never rely on the clamp: the bridge must
  not send a `duration_ms` above 5000.
- Decided, **not yet implemented**; nothing in the tree has changed for D6.

## D7. What tool-tip error counts as "arrived"? [finding 8] — 2026-09-18

The check is ±5° per joint, which is ~29 mm at full reach (335 mm). Painting and
cleaning probably need a millimetre answer; inspection may not. Once a number
exists, the arrival check should run the read-back joints through forward
kinematics and compare in millimetres.

**Answer (mm, per task if they differ):** (2026-09-26) **5 mm, one number for every
task for now.** `trajectory_node` counts a waypoint as arrived when the bridge reports
not moving AND the tool tip computed from the read-back joints is within 5 mm (straight-line
distance) of the waypoint's Cartesian target.
- `trajectory_node` runs `kinematics.forward_kinematics` on the read-back angles itself
  (same import pattern as `continuous_trajectory_node`), rather than calling
  `/facade_bot/read_tool_pose`, to avoid a second service hop per poll.
- Replaces `_WAYPOINT_POSITION_TOLERANCE_DEG = 5.0` with a mm constant. The bridge's own
  `_POSITION_TOLERANCE_DEG` (log-only stall/fault check) stays 5 deg in joint space — the
  bridge must not depend on the planner's FK — so the "matches exactly" link between the
  two constants is removed from both comments.
- Caveat: this measures where the servos report the tool is, not where it physically
  is; flex, backlash and calibration error are invisible to it (the IK-vs-ruler check
  covers that). Below ~3 mm, servo resolution would make arrival flaky; the 2026-09-12
  rectangle run closed to ~2 mm.
- The 15 s `_WAYPOINT_STALL_TIMEOUT_SEC` still covers the slowest D6 move.
- Decided, **not yet implemented**.

## D8. How is a raster over a wall specified? [finding 9] — 2026-09-18

The task space is `(x, y, z, joint_4 angle)`. The fourth number is a joint angle,
not a tool property, so a caller must compute per point what keeps the tool
normal to the wall.

| Option | |
|--------|--|
| Keep the current API; the raster generator computes joint_4 per point | Simple; every future caller repeats it |
| Define the task in a wall frame (plane + rectangle) and let the planner choose joint_4 | Geometry in one place; needed before writing the generator |
| A fifth joint | Hardware, not software |

Recommendation: the wall frame, as a function in `trajectory_planning.py`, before
the raster generator is written.

**Answer:** **Deferred** (2026-09-26, user): revisit when the raster generator is next.
Also open for then: whether the tool must be exactly normal to the wall or tolerates
tilt, which sets how much wall a 4-joint arm can cover.

## D9. Second fence in the firmware? [finding 4] — 2026-09-18

The firmware clamps only to the servo's full 0–1000 range. Option: the bridge
pushes per-servo raw min/max to the ESP32 at configure time, derived from
`robot_model.yaml`, so limits stay defined once but are enforced twice. This
protects against any tool that talks to port 5000 directly.

Recommendation: yes, when the firmware loop is reworked anyway (D4).

**Answer:**

## D10. Custom stack or standard stack? [closing section of the structural review] — 2026-09-18

Stay custom but adopt standard interfaces (`JointTrajectory`, named
`JointState`, one motion-owner action), or move the hardware interface to
ros2_control (C++) and planning toward MoveIt.

Recommendation: stay custom, adopt the interfaces.

**Answer:** (2026-09-26) **Stay custom, adopt standard interfaces** (`JointTrajectory`,
named `JointState`, one motion-owner action) as and when each is needed. No ros2_control
or MoveIt migration.

## D11. What should IK do with a target that is almost, but not exactly, on the base axis? — 2026-09-18

A target within 0.1 mm of the base axis now keeps the current base angle (the
home pose case). A target a few millimetres off the axis is solved normally, and
because the direction from the axis to that point is nearly arbitrary, the base
can be commanded to swing up to 180° for a tool-tip motion smaller than one
servo count. Pre-existing behaviour, found by the safety review; the limits and
the forward-kinematics check still apply, so it is a surprise, not a hazard.

| Option | |
|--------|--|
| Widen the on-axis band to about one servo count at the tool radius (~1 mm) and keep the current base angle inside it | Simple; the base holds still whenever it would not matter |
| Refuse near-axis Cartesian targets with a message telling the user to use the joint-space home move | Explicit; nothing surprising ever happens near the singularity |
| Leave it | Operator learns to send home in joint space |

Recommendation: the first, with the band as a named constant tied to the servo
resolution.

**Answer:** (2026-09-26) **Refuse near-axis Cartesian targets, within 10 mm, including
exactly on the axis.** In `kinematics.py`, a target whose horizontal distance from
joint_1's axis (`magnitude` in `_base_angle_candidates_rad`) is under 10 mm raises
`NotReachableError` with a message pointing to a joint-space move instead. The band is a
named constant (`_BASE_AXIS_EXCLUSION_RADIUS_M = 0.010`). Reason for 10 mm rather than
one servo count: a sideways tool move *d* at radius *r* swings the base about *d/r* rad
(1 mm at 10 mm ≈ 6°), so the refused zone covers large-swing moves too, and a 20 mm column
above the base is not facade workspace.
- This **removes the 2026-09-18 tier-1 on-axis special case** (keep the current base
  angle when exactly on the axis) and with it `fallback_base_rad`. The all-zero home is
  no longer reachable through `move_to_pose`; homing is a joint-space move
  (`LAUNCH.md` step 7a), which the user wants unrestricted.
- Joint-space commands (`/facade_bot/joint_cmd`) are not affected. They remain subject
  to the bridge's `_check_joint_bounds`, as every command must be.
- Continuous paths go through the same IK, so a sweep that passes through the zone is
  refused in the pre-motion check.
- The tier-1 on-axis test in `test_kinematics.py` changes meaning (accepts → refuses);
  its `TESTS.md` row changes with it.
- Decided, **not yet implemented**.

## D12. Joint speed along a continuous path near a singularity — 2026-09-26

Raised while discussing D11. There is no Jacobian or joint-velocity check anywhere in the
code. `trajectory_planning.solve_path_to_setpoints` checks each sample is reachable and
in limits, but never how far a joint moves between consecutive setpoints. Near a
singularity (base axis, full extension/retraction) a modest `tool_speed_mmps` can need a
large joint jump in one 120 ms stream step, and nothing clamps it: the streamed `servo`
command is sent with a fixed `servo_move_duration_ms`, so the servo goes as fast as it
physically can (~375 deg/s) to meet it. Applies to `continuous_trajectory_node` only;
point-to-point moves are capped by construction once D6 lands.

| Option | |
|--------|--|
| Refuse the path if any joint would exceed its peak speed | Pre-motion check, no Jacobian; reuses D6's number |
| Scale the whole path's tool speed down uniformly until it fits, with a warning | Runs, but slower than asked |
| Warn only | Offers no protection: nothing downstream clamps joint speed |
| Jacobian-based local slowdown | Gets through more paths; tool speed no longer constant |

**Answer:** (2026-09-26) **Refuse the path.** During the existing whole-path validation,
before any motion, compute each joint's change between consecutive setpoints divided by
`stream_period_sec` (including the first step, from the arm's actual current angles to
the first setpoint). If any joint exceeds its peak speed from `robot_model.yaml` — the
same per-joint number D6 introduces, 60 deg/s, not a second copy — refuse the goal with
nothing sent. At the default 0.12 s period that is 7.2 deg per step. The error names the
joint, the path point, and the highest `tool_speed_mmps` that would have passed.
A Jacobian (for slowing down rather than refusing) is left for the raster generator.
Decided, **not yet implemented**.


## D13. Staged motion on the non-blocking firmware path — 2026-09-26

On the arm, `move_async` moves (the bridge default since D4) visibly happen in about
three stages, at 1 s and 3 s, even with `/joint_states` polling stopped. The same
moves on the legacy blocking `move` are smooth. Suspected, not measured: during a
move the bridge polls `status` at 20 Hz (plus 5 Hz for the stop switch, since removed), and a reply's
blocking `sendall` stalls on a Wi-Fi latency spike, delaying the next 150 ms step, so
the servos stop and wait.

| Option | |
|--------|--|
| Measure, then fix the loop | Firmware records the worst step lateness and reports it in `status`; measure on the arm, then fix what it shows (step-before-reply ordering, slower `status` polling, ...). Keeps live `/joint_states` and a prompt stop topic. Needs a reflash and a bench session |
| Make blocking the default | Flip bridge defaults to `use_blocking_move: true`, `esp32_timeout_sec: 7.0`. Smooth today. `/joint_states` freezes during each move; `/facade_bot/estop_active` (since removed) only updates after a move ends |
| Both | Blocking default now, measure-and-fix next |

Recommendation: measure, then fix, (option 1) — or option 3 if bench work must continue first.

**Answer:** Measure, then fix (user, 2026-09-26). Code reading the same day made the
`sendall` stall theory less likely (a ~70-byte reply only fills the TCP send buffer)
and added two suspects: MicroPython garbage-collection pauses triggered by the
per-command allocations, and zero slack in the stepper (each step's servo move time
equals the step period, so any lateness at all is a visible stop). Instrumentation:
`timing` field in `status`, logged by the bridge per move, procedure in `LAUNCH.md`
step 9d. The fix is chosen from the numbers.

**Measured on the arm, 2026-09-26:** bridge alone, 0 late steps (1 s and 3 s,
smooth by eye). Under `bringup.launch.py`, 2-3 of 6 steps late at 1 s and 10 of 20
at 3 s, servos idle up to 70 ms (staged by eye). Cause: a mid-move `read_positions`
took ~93 ms of bus time against ~70 ms of step slack. Neither `sendall` nor the
stop-switch polling contributed. **Fix (user chose option 1, shorten the read):**
5 ms gap after each read instead of 20 ms (`_INTER_READ_DELAY_MS`, 0 misses at any
gap down to 0 ms over USB). After it, under bringup: reads 33 ms, 1 s moves 1 late
step then 0 (smooth by eye), 3 s moves 0 of 20 late. Step overlap (option 2) not
done; not needed at these numbers.

## Still-open follow-ups raised earlier (not yet asked as their own entries)
- D3 amendment: should queued `joint_cmd` messages be capped or stale-dropped in
  normal running (outside a stop)? Today a burst still executes in full.
- D5: should the `current_angles_deg is None` fallback (first candidate by list
  order) exist at all?
- D9 (firmware limit fence): still unanswered; options were in the pending reflash
  (now past), a later reflash, or no.
