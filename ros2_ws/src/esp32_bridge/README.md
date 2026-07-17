# esp32_bridge

Bridges the `/facade_bot/joint_cmd` ROS2 topic to the ESP32's TCP command
server (`esp32_firmware/main.py`). Subscribes to joint positions in radians,
converts to degrees and then to raw servo position counts, and forwards them
to the arm as a `move` command over Wi-Fi. After each move it reads the servo
positions back once and logs an error if a joint didn't reach its commanded
angle. No bounds-checking and no e-stop yet. See "Known limitations" below.

## Wire protocol (ESP32 side)

Transcribed from `esp32_firmware/main.py` — re-check against that file if the
firmware changes, since this bridge must match it exactly.

- Transport: TCP, static IP `192.168.1.100`, port `5000` (`main.py:11,16`).
- Framing: newline-delimited JSON — one JSON object per line.
- Handshake: on connect, the ESP32 immediately sends `{"status": "ready"}`
  before the client sends anything (`main.py:132`). The transport consumes
  this during `connect()`.
- `move` command:
  ```json
  {"cmd": "move", "positions": [p0, p1, p2, p3], "duration_ms": 1000}
  ```
  `positions` is exactly 4 integers, raw LX-16A position counts (0-1000,
  0.24°/unit), ordered base → end-effector (servo IDs `[1, 2, 3, 4]`,
  `main.py:21`). This node converts commanded degrees to raw counts before
  sending — the firmware no longer knows about degrees at all. `duration_ms`
  is an integer, clamped server-side to 100–5000 ms (`main.py:33-34`).
  `positions` values are clamped server-side to 0–1000 (`main.py:57`, inside
  `build_move_packet`) as a hardware safety net — this is a raw-range clamp
  only, not application-level joint-limit bounds-checking, which is not
  implemented anywhere yet. The firmware doesn't move directly to `positions`
  over `duration_ms` — it shapes a minimum-jerk trajectory from the servos'
  current position to the target internally (`move_joints_min_jerk` in
  `main.py`) and only acks once that shaped move has physically finished, so
  the response now doubles as a "move complete" signal (see below).
- Response: `{"status": "ok"}` or `{"status": "error", "msg": "..."}`.
- `read_positions` command:
  ```json
  {"cmd": "read_positions"}
  ```
  Response: `{"status": "ok", "positions": [p0, p1, p2, p3]}` — one raw
  LX-16A position per servo in `SERVO_IDS` order, each an integer 0–1000 or
  `null` if that servo missed its 10 ms UART read window (`main.py:49`). This
  node issues it once after each move for fault detection (see below); it is
  not published as a joint-state topic.

## Move fault detection

The `move` command's `{"status": "ok"}` ack now means the shaped trajectory
has physically finished (the ESP32 blocks internally until the last step is
sent — see `move_joints_min_jerk` in `esp32_firmware/main.py`), so this node
only needs a short settle margin — `_FAULT_CHECK_MARGIN_MS` (200 ms) — before
reading positions back once to confirm. Because a single servo's read often
comes back `null`, the read is retried up to 5 times, merging results so each
servo's value is kept as soon as any attempt returns it. A joint is "reached"
if its read-back position is within ±5° (converted to raw units) of the
commanded position; otherwise — or if still `null` after all retries — the
node logs a `get_logger().error(...)` naming the joint(s), commanded angle,
and read-back angle. It takes no other action: no e-stop, no lifecycle
change, no blocking of future commands (deferred). The ±5° tolerance, the
200 ms settle margin, and the 5-retry count are empirical starting values
pending real-hardware calibration.

## ROS2 interface

| | |
|---|---|
| Topic | `/facade_bot/joint_cmd` |
| Message type | `sensor_msgs/msg/JointState` (only `position`, in radians, is used) |
| QoS | `ReliabilityPolicy.RELIABLE` (hardware command topic) |

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

Parameters (all overridable at launch, defaults match the ESP32 firmware's
own constants):

| Parameter | Default | Meaning |
|---|---|---|
| `esp32_host` | `192.168.1.100` | ESP32 TCP server IP |
| `esp32_port` | `5000` | ESP32 TCP server port |
| `esp32_timeout_sec` | `7.0` | Socket connect/response timeout (covers the ESP32 blocking a `move` ack until its shaped trajectory finishes, worst case ~5.12s) |
| `move_duration_ms` | `1000` | Duration sent with every `move` command (`JointState` has no timing field) |

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

- **No emergency-stop logic.** Tracked as "Not started" in the top-level
  CLAUDE.md status table.
- **No continuously published joint states.** There's an on-request way to
  check position now (`/facade_bot/read_joint_positions`, see above), but
  nothing publishes a live `/facade_bot/joint_states` topic yet.
- **Fault action is log-only.** A joint outside tolerance is logged as an
  error; there is no e-stop, recovery, or command-blocking (deferred).
