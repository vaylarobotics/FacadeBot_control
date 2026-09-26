# esp32_bridge

Bridges the `/facade_bot/joint_cmd` ROS2 topic to the ESP32's TCP command
server (`esp32_firmware/main.py`). Subscribes to joint positions in radians,
converts to degrees and then to raw servo position counts, and forwards them
to the arm as a `move_async` command over Wi-Fi. It then polls the ESP32's
`status` command until the arm reports it has stopped, reads the servo
positions back once, and logs an error if a joint didn't reach its commanded
angle. Every command is bounds-checked against the limits in
`facadebot_description`'s `robot_model.yaml` before it is sent (the mandatory
gate, `_check_joint_bounds`). There is no stop control in this node or the
firmware; see "Known limitations" below.

## Wire protocol (ESP32 side)

Transcribed from `esp32_firmware/main.py` — re-check against that file if the
firmware changes, since this bridge must match it exactly.

- Transport: TCP, static IP `192.168.1.150`, port `5000` (`main.py:11,16`).
- Framing: newline-delimited JSON — one JSON object per line.
- Handshake: on connect, the ESP32 immediately sends `{"status": "ready"}`
  before the client sends anything (`handle_client` in `main.py`). The
  transport consumes this during `connect()`.
- Concurrency: the firmware answers exactly one line per command and there is
  no correlation ID, so the bridge holds a lock across each send/read pair
  (`_transact`). Since the bridge became multi-threaded this is what stops two
  callbacks mis-pairing each other's replies. A failed exchange is unrecoverable
  for the same reason — the reply may still arrive and answer the *next*
  command — so `_transact` tears the connection down and rebuilds it rather than
  carrying on. Note that dropping the socket makes the ESP32 abandon any move in
  progress, so the arm holds wherever its last trajectory step put it.
- `move_async` command (the default move path):
  ```json
  {"cmd": "move_async", "positions": [p0, p1, p2, p3], "duration_ms": 1000}
  ```
  `positions` is exactly 4 integers, raw LX-16A position counts (0-1000,
  0.24°/unit), ordered base → end-effector (servo IDs `[1, 2, 3, 4]`,
  `SERVO_IDS` in `main.py`). This node converts commanded degrees to raw
  counts before sending — the firmware no longer knows about degrees at all.
  `duration_ms` is an integer, clamped server-side to 100–5000 ms.
  `positions` values are clamped server-side to 0–1000 (inside
  `build_move_packet`) as a hardware safety net — this is a raw-range clamp
  only, not application-level joint-limit bounds-checking, which lives
  entirely on this side in `_check_joint_bounds`. The firmware doesn't move
  directly to `positions` over `duration_ms` — it shapes a minimum-jerk
  trajectory from the servos' current position to the target and steps
  through it one step per main-loop iteration (`start_motion` /
  `advance_motion_step`).

  **The ack means "accepted", not "finished."** It returns as soon as the
  start pose has been read and the trajectory armed. Use `status` (below) to
  find out when the arm has actually stopped.
- Response: `{"status": "ok"}`, `{"status": "busy"}` if a shaped move is
  already running, or `{"status": "error", "msg": "..."}`. One
  error is new (2026-09-26): `"start position unreadable - move refused"`. The
  firmware reads every joint before shaping a move, re-reading a missed joint up
  to 5 attempts in all, and refuses the move rather than guess a start pose. The
  arm has not moved; the bridge logs it as an error and does not re-send.
- `status` command:
  ```json
  {"cmd": "status"}
  ```
  Response: `{"status": "ok", "moving": true|false, "timing": {...}}` — whether
  a shaped trajectory is still being stepped. Since 2026-09-26 the reply also carries a
  `"timing"` object: step-timing counters for the most recent shaped move
  (diagnostic only, DECISIONS.md D13; fields in `LAUNCH.md` step 9d). The bridge
  logs it when a `move_async` finishes and treats it as optional: older firmware
  and `scripts/fake_esp32_server.py` do not send it. A `servo` setpoint does not count as
  moving: the firmware's stepper is not running for it, the servo is just
  interpolating on its own. This is a TCP round trip only and never touches
  the servo bus, so polling it is cheap.
- `move` command (legacy, rollback only):
  ```json
  {"cmd": "move", "positions": [p0, p1, p2, p3], "duration_ms": 1000}
  ```
  Identical fields and identical shaping to `move_async`, but it runs the whole
  trajectory before acking, so the ack does mean "physically finished" and the
  firmware's socket is deaf for the duration (up to ~5.1s). Kept so the
  previously hardware-verified behaviour can be restored without a reflash —
  set the `use_blocking_move` parameter. `/joint_states` goes silent during
  every move on this path; that is the bug the async path exists to fix.
- `servo` command (continuous-motion streaming):
  ```json
  {"cmd": "servo", "positions": [p0, p1, p2, p3], "duration_ms": 120}
  ```
  Same `positions` format as `move` (4 raw counts, base → end-effector), but the
  firmware moves **straight** toward the target with a single servo command and
  acks **immediately** — no minimum-jerk shaping and no position readback
  (`_handle_move_like` in `main.py`, reusing the plain `move_joints`). A `servo`
  setpoint **preempts** any shaped move still running: both write to the same
  servos over the same UART, so the stepper is cancelled rather than left to
  fight the stream. This is
  the primitive continuous motion streams onto: the RPi sends `servo` setpoints
  back-to-back (one per stream step) so each servo retargets before the previous
  step finishes, giving motion that flows through waypoints instead of stopping.
  `duration_ms` should be about one stream step so the servo's own interpolation
  covers the gap. Response: `{"status": "ok"}` or `{"status": "error", "msg": "..."}`.
- `read_positions` command:
  ```json
  {"cmd": "read_positions"}
  ```
  Response: `{"status": "ok", "positions": [p0, p1, p2, p3]}` — one raw
  LX-16A position per servo in `SERVO_IDS` order, each an integer 0–1000 or
  `null` if that servo gave no valid reply within 10 ms
  (`_READ_REPLY_DEADLINE_MS` in `main.py`), or its reply failed the checksum or
  read outside 0–1000. This node issues it once after each move for fault detection
  (see below); it is not published as a joint-state topic. Since the firmware
  loop became non-blocking this is answerable **during** a move. It costs ~30 ms
  of servo-bus time (since 2026-09-26; it was ~90 ms and made moves stage, D13),
  inside a step's slack; a step that does run late is caught up on, never burst.

## Move fault detection

The `move_async` ack only means the command was accepted, so this node first
polls `status` every `_STATUS_POLL_INTERVAL_SEC` (50 ms) until the firmware
reports it has stopped — `_wait_for_motion_complete`, bounded by
`_MOVE_COMPLETION_TIMEOUT_SEC` (8.0 s), after which the check is **skipped**
with an error rather than run against a moving arm. The transport lock is
released between polls on purpose: that gap is when position reads get served,
which is what keeps `/joint_states` alive during a move. Once the arm has
stopped, a short settle margin — `_FAULT_CHECK_MARGIN_MS` (200 ms) — precedes
reading positions back once to confirm. Because a single servo's read often
comes back `null`, the read is retried up to 5 times, merging results so each
servo's value is kept as soon as any attempt returns it. A joint is "reached"
if its read-back position is within ±5° (converted to raw units) of the
commanded position; otherwise — or if still `null` after all retries — the
node logs a `get_logger().error(...)` naming the joint(s), commanded angle,
and read-back angle. It takes no other action: no stop, no lifecycle
change, no blocking of future commands (deferred). The ±5° tolerance, the
200 ms settle margin, and the 5-retry count are empirical starting values
pending real-hardware calibration.

## ROS2 interface

| | |
|---|---|
| Topic | `/facade_bot/joint_cmd` |
| Message type | `sensor_msgs/msg/JointState` (only `position`, in radians, is used) |
| QoS | `ReliabilityPolicy.RELIABLE` (hardware command topic) |

Forwarded as a `move_async` command (min-jerk shaping, then a `status` poll
until the arm stops, then the fault readback).

Both command topics share one **mutually exclusive** callback group, so only one
command is ever in flight: a second `joint_cmd` message waits in the topic queue
until the first move has finished and is then sent normally. It is not dropped.
The firmware's `{"status": "busy"}` refusal is the backstop beneath that, and
fires only when the bridge's own serialisation has been defeated — most
realistically when the `status` poll times out and the bridge moves on while the
arm is genuinely still moving. When it does fire the command **is** dropped, with
a warning; nothing re-sends it.

| | |
|---|---|
| Topic | `/facade_bot/joint_stream` |
| Message type | `sensor_msgs/msg/JointState` (only `position`, in radians, is used) |
| QoS | `ReliabilityPolicy.RELIABLE` (hardware command topic) |

Continuous-motion setpoints from `continuous_trajectory_node`, forwarded as the
non-blocking `servo` command. Bounds-checked exactly like `joint_cmd` (the
mandatory gate applies to streamed setpoints too), but with **no** post-move
fault readback and no per-setpoint logging — at the streaming rate that would
flood the log and stall the stream.

| | |
|---|---|
| Service | `/facade_bot/read_joint_positions` |
| Service type | `facade_msgs/srv/ReadJointPositions` (empty request) |

Reads the servo positions back on request (reuses the same retry-and-merge
readback used for move-fault detection — see below). Doesn't require a
separate connection to the ESP32: the firmware only accepts one TCP client
at a time, so this exists precisely so a position check doesn't have to
compete with the bridge node's own open connection.

```bash
ros2 service call /facade_bot/read_joint_positions facade_msgs/srv/ReadJointPositions {}
```

Response is `positions_deg` (4 floats, base → end-effector, `NaN` for any
joint with no reading) and `all_valid` (`false` if any joint's reading was
unavailable after retries).

| | |
|---|---|
| Service | `/facade_bot/read_joint_positions_fast` |
| Service type | `facade_msgs/srv/ReadJointPositions` (same message as above) |

Same request/response shape as `read_joint_positions`, but a single attempt
with no retries — measured on the arm at ~170ms rather than the plain
service's occasional 600ms+ (waiting out the full retry budget for one slow
joint). About half of single attempts are missing one joint (`all_valid:
false`, that joint `NaN`); this is normal for this service, not a fault, and
callers should treat a miss as "use the last known value," not an error.
**Not a substitute for `read_joint_positions`** in anything safety-relevant —
`esp32_bridge_node`'s own move-fault check still uses the full-retry version.
Intended for a caller that polls continuously and can tolerate one joint
being one poll cycle stale (e.g. a `/joint_states` publisher for RViz).

```bash
ros2 service call /facade_bot/read_joint_positions_fast facade_msgs/srv/ReadJointPositions {}
```

| | |
|---|---|
| Service | `/facade_bot/is_moving` |
| Service type | `facade_msgs/srv/IsMoving` (empty request) |

Whether the arm is still executing a shaped move. Queried from the ESP32's
`status` command on every call rather than answered from a flag kept in this
node — the firmware is the authoritative source, and this node can restart
while the arm does not.

```bash
ros2 service call /facade_bot/is_moving facade_msgs/srv/IsMoving {}
```

Response is `is_moving` and `valid`. **`valid: false` means "could not tell",
not "stopped"** — the query to the ESP32 itself failed, and `is_moving` is
meaningless. Callers must treat unknown as still moving:
`trajectory_node` does, because advancing to the next waypoint while the arm
is mid-swing is exactly what this service exists to prevent. Note a `servo`
stream setpoint does not register as moving (see the `status` command above).

Parameters (all overridable at launch, defaults match the ESP32 firmware's
own constants):

| Parameter | Default | Meaning |
|---|---|---|
| `esp32_host` | `192.168.1.150` | ESP32 TCP server IP |
| `esp32_port` | `5000` | ESP32 TCP server port |
| `esp32_timeout_sec` | `2.0` | Socket connect/response timeout. Every command now acks promptly (the slowest is `read_positions` at ~170ms), so this only covers Wi-Fi/TCP jitter. **Set to `7.0` if you set `use_blocking_move`** |
| `move_duration_ms` | `1000` | Duration sent with every move command (`JointState` has no timing field) |
| `servo_move_duration_ms` | `120` | Duration sent with every streamed `servo` setpoint; keep in step with `continuous_trajectory_node`'s `stream_period_sec` |
| `use_blocking_move` | `false` | Rollback: send the firmware's legacy blocking `move` instead of `move_async` + `status` polling. Needs `esp32_timeout_sec:=7.0`, and stops `/joint_states` updating during a move |

## Running

This is a lifecycle node — it does nothing until explicitly configured and
activated:

```bash
ros2 run esp32_bridge esp32_bridge_node
# in another terminal:
ros2 lifecycle set /esp32_bridge_node configure
ros2 lifecycle set /esp32_bridge_node activate

ros2 topic pub --once /facade_bot/joint_cmd sensor_msgs/msg/JointState \
  "{position: [0.0, 1.5708, 3.14159, 0.7854]}"
```

To stop listening without tearing down the ESP32 connection:
`ros2 lifecycle set /esp32_bridge_node deactivate`.

## Homing / joint centers

Each joint's `0°` no longer means raw position `0` — it means that joint's
measured true center, recorded in `_JOINT_CENTER_RAD` (radians, as read
directly off the physical arm). `_angle_deg_to_position_raw` and
`_position_raw_to_angle_deg` both convert relative to that joint's own
center, not a shared absolute scale. Commanding `[0.0, 0.0, 0.0, 0.0]` moves
every joint to its measured center.

## Bounds-checking

Every command is checked against `_JOINT_LIMITS_DEG` (per-joint safe range,
now expressed relative to each joint's own measured center above — joints
1-3 get ±110°, joint_4/wrist gets a narrower ±100° — rather than an absolute
range converted from `URDF_Test.urdf`) in `_check_joint_bounds`, before any
angle→raw conversion or contact with the ESP32 — this is the single function
CLAUDE.md's safety rule requires, and the last-resort gate regardless of
whether a command came from `facade_control`'s IK or straight from
`ros2 topic pub`. A command with any joint out of range is rejected
outright: `get_logger().error(...)` names the offending joint(s) and values,
and nothing is sent to the hardware. The existing raw 0–1000 clamp inside
`_angle_deg_to_position_raw` stays as a final wire-level safety net, but in
normal operation this check rejects out-of-range commands well before that
point is reached.

## Known limitations / deferred work

- **No stop control.** The bench stop switch (GPIO 27) was removed on
  2026-09-26 by the user's decision; there is no stop in the firmware, this node,
  or over TCP. The only way to stop the arm mid-move is to pull the servo power
  plug.
- **Joint states are published by another node.** `facade_control`'s
  `joint_state_publisher_node` polls `/facade_bot/read_joint_positions_fast`
  and publishes `/joint_states`; this package only serves the read.
- **Fault action is log-only.** A joint outside tolerance is logged as an
  error; no recovery or command-blocking follows (deferred).
- **Queued commands are not stale-checked.** Because the command topics
  serialise, a burst of `joint_cmd` messages executes one after another (topic
  queue depth 10) rather than the later ones being refused. A target published
  during a long move is therefore still executed when its turn comes, however
  old it has become. `trajectory_node` never does this — it waits for
  `/facade_bot/is_moving` before its next waypoint — but a script publishing
  faster than the arm moves will queue up behind itself.
- **A move refused as `busy` is dropped, not queued.** That is the firmware-level
  backstop described above; the bridge logs a warning and nothing re-sends it.
- **Feedback during a move is bounded by the servo bus, not by ROS2.** A
  4-joint readback is ~170 ms of UART, so `/joint_states` tops out near
  5–6 Hz while the arm is moving, and each read pushes a trajectory step late.
  Smooth rendering would need commanded setpoints echoed as well; that has not
  been built.
