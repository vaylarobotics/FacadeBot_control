# Test Catalogue

Every automated test in the project, in plain language: what it proves, and what
would slip through if it were missing. This is the file to review and mark up.
The Python in each package's `test/` folder follows this file, not the other way
round.

## How to give input

Every test row has a **Status** column. Change it and run `/sync-tests` (or just
say "sync the tests"); Claude reads the column, makes the Python match, runs the
suites, and resets the column. Nothing happens to the code until you ask for the
sync.

| Status | Meaning | What the sync does |
|--------|---------|--------------------|
| `keep` | Default. Test stays as described | Nothing |
| `drop` | This test is not worth having | Deletes the test function and removes the row |
| `change: <what>` | Keep the test but alter it, e.g. `change: also cover joint 4` or `change: tolerance should be 2 deg` | Rewrites the test to match, updates "Proves" and "Would catch" |
| `wanted` | A test that should exist (rows under **Wanted**) | Writes it, if its "Blocked on" decision is answered; otherwise asks first |
| `question: <text>` | You want to understand or challenge the test before deciding | Answers in chat, leaves the code alone |

You can also edit the "Proves" or "Would catch" text directly. If the new text
no longer describes what the Python does, the sync treats it as a `change`.
Add a new row anywhere with status `wanted` and a one-line "Proves" for a test
you want that is not listed.

None of these touch the arm, the ESP32, or the Pi. They run on the dev box in a
few seconds:

```bash
cd ros2_ws && source /opt/ros/jazzy/setup.bash && source install/setup.bash
python3 -m pytest src/facadebot_description/test -q -p no:cacheprovider
python3 -m pytest src/esp32_bridge/test -q -k 'not flake8 and not pep257 and not copyright' -p no:cacheprovider
python3 -m pytest src/facade_control/test -q -k 'not flake8 and not pep257 and not copyright' -p no:cacheprovider
```

Each package runs on its own because two of them share test file names. The
`flake8`/`pep257`/`copyright` tests are ament boilerplate that fails at HEAD by
design (style conflicts) and are skipped.

**What passing means.** The logic matches what the tests describe. It does not
mean the arm agrees: the bench check in `LAUNCH.md` step 7a is still the only
proof of that, and `STATUS.md` records which features have had it.

---

## facadebot_description — model loader (4 tests)

The loader is fail-closed: a node that cannot prove which arm it is driving must
not start. These tests corrupt one thing at a time in a throwaway copy of the
installed package.

| Test | Proves | Would catch | Status |
|------|--------|-------------|--------|
| installed model loads | The installed `robot_model.yaml` + `geometry_v2.yaml` + URDF load together as a 4-joint model named `joint_1..joint_4` | A broken install or config that only fails on the Pi | keep |
| edited URDF without regeneration is refused | Appending one comment to the URDF makes every node refuse to start with a "regenerate" message | A SolidWorks re-export silently driving old link lengths (RViz would show the new arm, the controller would solve the old one) | keep |
| missing URDF is refused | No URDF on disk means no load, rather than "trust the geometry file" | A partial install | keep |
| duplicate joint names are refused | Two joints with one name is rejected by name | Limits looked up by name silently applying to the wrong joint | keep |

## esp32_bridge — the hardware gate (33 tests)

The bridge is the only path to the servos and holds the mandatory bounds gate.
These run the real node with the ESP32 replaced by a fake that records every
command it would have sent and plays back scripted replies **per command name**
(a move now costs a variable number of `status` polls, so a single reply queue
would couple every move test to the poll count). Limits come from the real active
model, exactly as at startup.

### Bounds gate

| Test | Proves | Would catch | Status |
|------|--------|-------------|--------|
| refuses to run without limits | If limits were never loaded, the gate raises instead of passing everything | A refactor that lets a command through before startup finished | keep |
| passes zero and the exact limits | 0° and the measured limit values themselves are accepted (inclusive) | An off-by-one that rejects the edge of the safe range | keep |
| names every violating joint | Two joints out of range produce two messages, each naming its joint index | A gate that stops at the first violation and hides the second | keep |

### Degrees to servo counts

| Test | Proves | Would catch | Status |
|------|--------|-------------|--------|
| zero degrees is the measured centre | Commanded 0° on each joint becomes that joint's calibrated raw centre | A regression that sends 0° to raw position 0 (the servo's end stop) | keep |
| round-trips within one servo count | Degrees → raw → degrees lands within half a count (0.12°) for five angles per joint | A scale or offset error in either direction | keep |
| clamps to the servo mechanical range | Absurd angles clamp to raw 0 and 1000 rather than wrapping or overflowing | A negative raw count reaching the wire | keep |
| centre raw matches centre radians | The raw centre table is derived from the radian table, not typed separately | Someone editing one table and not the other | keep |

### Move path (`move_async` command)

| Test | Proves | Would catch | Status |
|------|--------|-------------|--------|
| zero move sends raw centres | `[0,0,0,0]` becomes a `move_async` with the four centre counts and the default duration, followed by a readback | A field renamed or reordered in the wire message | keep |
| out-of-bounds move sends nothing | One joint over its limit means no bytes to the ESP32 and an "out of bounds" error log | The gate being checked after the send instead of before | keep |
| fault when a joint misses target | Readback more than the tolerance away logs "move fault" naming the joint | A fault check that compares the wrong units or index | keep |
| dead bus when every joint is silent | All four joints `null` after every retry logs the "servo bus fault" message, not four stalls | The bus-dead case being misreported as a mechanism problem | keep |
| transport error is logged, not raised | A dropped link during a move produces an error log and the node keeps running | The bridge crashing on the first Wi-Fi hiccup | keep |
| callback converts radians and ignores wrong length | A 3-element message is ignored; a 4-element one in radians reaches the ESP32 as a `move_async` | A degrees/radians mix-up at the topic boundary | keep |
| waits for `status` to report stopped | Two "still moving" polls are not accepted; the readback only happens after the third poll says stopped | The fault check running against a moving arm and reporting a fault on every single move | keep |
| move rejected as busy is logged, not checked | A `{"status":"busy"}` reply logs a warning, sends nothing further, and logs no error | A refused move being silently treated as accepted | keep |
| logs the firmware's step timing | When the final `status` carries a `timing` object, one `move timing:` line is logged | The D13 measurement silently never appearing on the bench | keep |
| no usable timing: silent, fault check still runs (2 cases) | A `status` with no `timing` field (older firmware, simulator) or a non-object one logs nothing extra, no warning, and the readback still happens | The diagnostic breaking or blocking the move-fault check | keep |
| move refused for an unreadable start pose is an error and not waited on | The firmware's "start position unreadable" refusal is logged as an error saying the arm did not move, with no `status` polling and no readback | A refused move being waited on or fault-checked as if it had run | keep |
| skips the fault check when the arm never stops | If `status` never reports stopped, the timeout logs an error and no readback happens | Waiting forever, or judging a still-moving arm | keep |
| blocking-move rollback sends `move` and never polls | With `use_blocking_move` set, the wire sees `move` then the readback, and zero `status` polls | The rollback path silently still using the async protocol | keep |

### Motion state (`is_moving` service)

| Test | Proves | Would catch | Status |
|------|--------|-------------|--------|
| reports the firmware's answer | The service returns what `status` said, call by call, with `valid` true | The service answering from a cached flag rather than asking the ESP32 | keep |
| reports invalid when the link is down | A transport error gives `valid: false`, not `is_moving: false` | "Cannot tell" being indistinguishable from "stopped" — which would let `trajectory_node` advance onto a moving arm | keep |

### Replies that cannot be trusted (added after the 2026-09-20 safety review)

| Test | Proves | Would catch | Status |
|------|--------|-------------|--------|
| a `status` reply with no `moving` field skips the fault check | `{"status":"ok"}` with the field absent is treated as *unknown*, not as stopped, and no readback happens | The fail-open case: a mis-paired or truncated reply reading as "the arm has stopped", faulting on every move and letting `trajectory_node` advance onto a moving arm | keep |
| `is_moving` reports invalid when the reply has no `moving` field | Same reply gives `valid: false` rather than `is_moving: false` | The same fail-open reaching the service callers | keep |
| a transport error rebuilds the connection | A failed exchange reconnects exactly once and still logs the error | A late reply answering the *next* command, permanently shifting every request/reply pair by one | keep |
| command and service callbacks are in different groups | The two command topics are in a mutually exclusive group, the services in a reentrant one | A "simplification" to one group, which would let two joint commands race for the transport and put two sources on the servo bus | keep |

### Stream path (`servo` command)

| Test | Proves | Would catch | Status |
|------|--------|-------------|--------|
| stream setpoint uses `servo` and the stream duration | A streamed setpoint goes out as `servo` with the short per-step duration, not as a `move` | Continuous motion accidentally getting min-jerk shaping and stopping at every point | keep |
| stream setpoint is bounds-checked too | The gate applies to streamed setpoints, not just moves | A path around the gate through the faster topic | keep |

### Position readback

| Test | Proves | Would catch | Status |
|------|--------|-------------|--------|
| readback merges missing joints across attempts | Joint 2 missing on attempt 1 and present on attempt 2 gives a full set after two sends | The retry loop giving up or not merging | keep |
| keeps the first reading when later attempts differ | Once a joint has a value, a later differing value does not overwrite it | Jitter between attempts producing a mixed reading | keep |
| fast readback makes exactly one attempt | The fast service sends once and reports the missing joint as NaN with `all_valid` false | The fast service silently growing a retry loop and stalling `/joint_states` | keep |
| full readback converts to centre-relative degrees | Raw centre counts come back as 0.0° on every joint with `all_valid` true | The read service and the command path disagreeing on what zero means | keep |

## facade_control — kinematics (14 tests)

Pure maths, no ROS. The arm model is the real generated v2 geometry with
stand-in limits (±110° on every joint) so the tests exercise the solver, not the
calibration.

| Test | Proves | Would catch | Status |
|------|--------|-------------|--------|
| zero pose points straight up | All joints at 0 puts the tool tip on the base axis at the summed link height | A wrong joint origin or rotation order in forward kinematics | keep |
| joint 4 tilts the tool out of the plane | Changing only joint 4 moves the tip sideways out of the shoulder/elbow plane, and `tool_angle_deg` comes back unchanged | The v1 meaning of joint 4 (pitch in-plane) creeping back | keep |
| round-trips reachable targets (4 poses) | Forward then inverse kinematics reproduces the tip within 0.1 mm | A branch or sign error in the closed form | keep |
| unreachable target raises | A point 10 m away is refused with `NotReachableError` | A solver returning a plausible-looking answer for the impossible | keep |
| out-of-range joint raises | A reachable point that needs a joint past its limit is refused, with a "safe range" message | IK handing the bridge something the gate will reject, or worse, the gate missing it | keep |
| least-travel solution is chosen | Seeding from the exact answer returns that answer | Total breakage of the seeding logic | keep |
| current position decides between elbow configurations | The same target, seeded from near elbow-up, returns elbow-up; seeded from near elbow-down, returns elbow-down; the two differ | The elbow-flip regression this feature exists for (`current_angles_deg` being ignored) | keep |
| home pose on the base axis solves and keeps the current base angle | The all-zero home pose is solvable; seeded with the base at 35°, the answer keeps the base at 35° and still lands on the target | The home pose being "unreachable" (was the case until 2026-09-18) | keep |
| fully extended poses survive floating-point noise (500 poses) | Straight-elbow poses, where the maths sits exactly on a boundary, all solve | The 1% floating-point rejections found in review | keep |
| random reachable targets round-trip (200 poses) | Random in-range poses all solve back to within 1 mm | Anything the hand-picked poses missed | keep |
| previous arm model still solves | The retired v1 geometry still round-trips | The solver losing the in-plane-wrist special case | keep |
| model with a wrong shape is rejected | An elbow origin with a roll is refused at configure time | A regenerated URDF with a different structure solving nonsense quietly | keep |

## facade_control — trajectory planning (10 tests)

Pure geometry and timing for the continuous sweep, no ROS.

| Test | Proves | Would catch | Status |
|------|--------|-------------|--------|
| straight path length matches endpoint distance | A two-point path measures the straight-line distance and its ends are the waypoints | Arc-length bookkeeping drift | keep |
| constant-speed samples are evenly spaced in cruise | At 50 mm/s and 0.1 s per step, no step exceeds 5 mm and the middle of the path hits exactly 5 mm | The speed profile not actually being constant | keep |
| corner is rounded, not passed through | The path near a 90° corner comes within the blend radius of the vertex but never reaches it | The tool driving into a sharp corner at full speed | keep |
| collinear waypoints stay straight | Three points on a line produce a straight path of the right length | Rounding being applied where there is no corner (raster passes would wobble) | keep |
| tool angle interpolates along the path | Tool angle goes 0 → 15 → 30 across a path from 0 to 30 | Joint 4 jumping at a waypoint instead of blending | keep |
| final sample lands exactly on the path end | The last streamed point is the last waypoint, not one step short | The arm stopping short of the end of every sweep | keep |
| planned setpoints stay in limits | A real two-waypoint plan gives 4-angle setpoints all inside the limits, and the first one recovers the seed | The plan containing something the bridge would reject mid-sweep | keep |
| plan raises when a point on the path is unreachable | One unreachable waypoint fails the whole plan before any setpoint exists | Motion starting and then aborting halfway | keep |
| blended path needs two waypoints | A one-point path is rejected | A degenerate goal reaching the streamer | keep |
| sample rejects non-positive speed | Speed 0 is rejected | A divide-by-zero in the timing | keep |

## facade_control — trajectory node, stop-at-each-waypoint (9 tests)

The node runs for real, with `move_to_pose`, `read_joint_positions` and
`is_moving` served by stub nodes whose replies the test scripts. The action goal handle is a fake
so cancellation can be triggered from the test.

| Test | Proves | Would catch | Status |
|------|--------|-------------|--------|
| within-tolerance: all close / one far / boundary inclusive (3 tests) | The arrival check passes when every joint is inside tolerance, fails when one is out, and treats exactly-on-tolerance as arrived | An off-by-one at the tolerance edge | keep |
| happy path | One waypoint: solve, poll twice without arrival, arrive on the third poll, succeed with one feedback message | The advance-to-next logic or feedback being broken | keep |
| IK failure aborts without polling | A rejected `move_to_pose` aborts the goal with the reason and never polls position | Polling for arrival at a pose the arm was never sent to | keep |
| stall times out | A position that never matches aborts with "stall" after the timeout | A stalled arm hanging the action forever | keep |
| cancel mid-poll | A cancel arriving while polling ends the goal as canceled with zero waypoints completed | Cancel being ignored until the sequence finishes | keep |
| waits while the arm is still moving | With the arm already at target from the first poll, two "still moving" answers are not accepted as arrival | The false arrival the non-blocking firmware makes possible: commanding the next waypoint on top of an in-flight move | keep |
| unknown motion state counts as moving | An `is_moving` reply with `valid: false` stalls the waypoint rather than passing it | An unanswerable query reading as "stopped" and advancing early | keep |

## facade_control — continuous trajectory node (4 tests)

Same stub pattern; the stub also subscribes to the stream topic to count what
was sent.

| Test | Proves | Would catch | Status |
|------|--------|-------------|--------|
| streams setpoints and succeeds | Two reachable waypoints produce a stream of 4-angle setpoints, feedback, and success | The streamer not actually publishing | keep |
| aborts when current position is unreadable | A read with a missing joint aborts before anything is streamed | Planning from a guessed start pose | keep |
| aborts before moving when the path is unreachable | An unreachable waypoint aborts during planning with nothing streamed | Motion starting on a plan that fails partway | keep |
| cancel stops the stream | A cancel during a slow sweep ends the goal as canceled with less than the full path completed | Cancel being ignored mid-sweep | keep |

---

## Not tested by anything automated

Review this list as carefully as the tables above.

| Area | Why not | How it is checked today |
|------|---------|-------------------------|
| `esp32_firmware/main.py` | MicroPython on the board; no test harness | Flash, ping, home move, readback (`LAUNCH.md` steps 1–7a). `scripts/fake_esp32_server.py` mirrors the protocol but is plain CPython — it does not exercise `select.poll`, the `ticks_ms` step scheduler, or UART contention |
| `esp32_bridge/transport.py` | Real sockets only | Covered indirectly: the bridge tests prove what would be sent; the wire format itself is only proven on the arm |
| `facade_control_node.py` | Thin wrapper over kinematics + one service call | Kinematics tests plus the hardware `move_to_pose` runs recorded in `STATUS.md` |
| `joint_state_publisher_node.py` | Timer + one service call; nothing safety-critical | RViz shows the arm; the D4 sim run measured a steady 5 Hz with 16 distinct poses through a 3 s move |
| Both launch files | Would need a running graph | Bringup verified by use; `display.launch.py` not yet run on a real display |
| The IK selection metric being the *right* one | A design question, not a correctness one | `DECISIONS.md` D5 |
| Timing: Wi-Fi jitter, servo-bus rate cap, corner acceleration | Only measurable on the arm | The continuous-sweep bench pass, still to do |
| D13 step-timing counters (firmware `timing` field, bridge `move timing:` log line) | Diagnostic only; a log line changes no behaviour | The counter arithmetic was run once in CPython with stubs on 2026-09-26: a clean 1 s move gave 0 late steps, and a simulated 300 ms stall gave 1 late step with 230 ms idle. The bridge's log line is covered by the 3 tests in the esp32_bridge table. Real numbers come only from the arm (`LAUNCH.md` step 9d) |

## Wanted

Tests that should exist and do not. Add rows here when you spot a gap.

| Test | Why | Blocked on | Status |
|------|-----|-----------|--------|
| Bridge rejects a stream setpoint while a `move` is in flight | The burst-after-move failure in the structural review | ~~D3~~ answered 2026-09-20, but the answer was the opposite: `servo` **preempts** a shaped move by design. What is now wanted is a test that it preempts cleanly rather than interleaving on the bus — and that is firmware behaviour, so it needs the arm | wanted |
| Arrival check in millimetres at the tool tip | ±5° per joint is ~29 mm at full reach | D7 (tolerance) | wanted |
| IK prefers the smaller peak-joint swing | The 184° vs 114° case measured this session | ~~D5~~ answered 2026-09-21: L∞ with a total-travel tie-break. Unblocked; write it alongside the `kinematics.py` change, not before | wanted |
| Firmware protocol version rejected at `configure` when mismatched | Flash/bridge drift is currently found mid-move. D4 landed without a version field (deliberately out of scope); a bridge on new firmware and old firmware on a new bridge both fail safe today — `move_async` comes back "unknown command" — but noisily and late | unblocked; not done | wanted |
