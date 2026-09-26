# esp32_bridge — CLAUDE.md

The one place ROS2 crosses into the ESP32 transport, and the one place the
mandatory joint bounds gate lives. This is the most safety-critical file in the
project. Use the `hardware-change` skill for any edit here.

## What it owns

- `transport.py`: the TCP socket and the JSON wire protocol. No ROS imports. Nothing
  else in the project may open a socket to the ESP32.
- `esp32_bridge_node.py`: a lifecycle node. `configure` opens the connection,
  `activate` creates the subscriptions and services. Converts commanded degrees
  (relative to each joint's measured centre) to raw servo counts, forwards
  `move_async` and `servo` commands, polls `status` until the arm stops, reads
  positions back, logs move faults. Also serves `/facade_bot/is_moving`.

## Invariants

- `_check_joint_bounds` is the last-resort gate. Every path that sends a `move` or
  `servo` command calls it first. It raises if limits were never loaded. Never add a
  path around it, and never add a second copy of the limits anywhere.
- Limits come from `facadebot_description`'s `robot_model.yaml` via
  `_load_joint_limits()` at startup. The node refuses to start if they cannot be
  loaded. Keep it fail-closed.
- `_JOINT_CENTER_RAD` is per-servo calibration measured on the physical arm. It is
  deliberately in code, not in `robot_model.yaml`. Changing it changes what `0°`
  means for every node and shifts the bounds window: only change it after a
  measurement, and record the date next to the number. The 2026-09-18 disagreement
  between its comment and STATUS.md (`CODE_REVIEW_2026-09-18.md` finding 1) is
  **resolved**: the user confirmed the values current on 2026-09-21, and the comment
  now records that joint_1 did move (2.09 -> 2.17) during the v2 rebuild while
  joints 2-4 re-measured to their v1 values.
- `JointState.position` on `/facade_bot/joint_cmd` and `/facade_bot/joint_stream`
  is in **radians**; the node converts to degrees. The `name` field is ignored and
  order is assumed base to tip.
- Two read services share one message: `read_joint_positions` (full retry, used by
  the move-fault check and every planner) and `read_joint_positions_fast` (single
  attempt, used only by `joint_state_publisher_node`). Do not swap which callers use
  which.

## Known structural limits (do not paper over)

- Multi-threaded since 2026-09-20 (D4), with **two** callback groups, and the split
  is load-bearing:
  - the three read/status services share a `ReentrantCallbackGroup`, so a position
    read is answered while a move's `status` poll is running — the entire point of
    the change;
  - `joint_cmd` and `joint_stream` share one `MutuallyExclusiveCallbackGroup`, so
    only one command is ever in flight. Putting the commands in the reentrant group
    would let two joint commands race for the transport lock (arbitrary winner) and
    let a stream setpoint preempt a move in the firmware while this node was still
    polling for that move to finish. Do not "simplify" this to one group.
- **The lock is held across each send/read pair only** (`_transact`) — never across a
  sequence of round trips. `_wait_for_motion_complete` releasing it between `status`
  polls is what keeps `/joint_states` alive; holding it across that loop would restore
  the original bug exactly.
- `transport.py` is not thread-safe and has no correlation ID: one line of reply per
  command. The lock is the only thing keeping two callbacks from mis-pairing replies,
  and `_transact` reconnects after any failed exchange because a late reply would
  otherwise answer the next command forever. Do not add a transport caller that
  bypasses `_transact`.
- Because the commands serialise, a second `joint_cmd` **queues and then executes**;
  it is not refused. The firmware's `busy` reply (D3) is the backstop under that, and
  in practice only fires when the `status` poll has timed out. A queued command is
  never stale-checked — a burst executes in full, however old each target has become.
  `servo` still preempts a shaped move in the firmware.
- The stream path has no readback and no fault detection.
- There is no stop control (the bench stop switch was removed 2026-09-26, user's
  decision). Nothing in this node drops commands or reports a stop state; do not
  re-add one piecemeal. The servo power plug is the only stop.

## Constants that must match the firmware

`_EXPECTED_SERVO_COUNT`, `_POSITION_MIN_RAW`/`_POSITION_MAX_RAW`, default host and
port, the command names (`move_async`, `status`, `move`, `servo`,
`read_positions`) and the `status` reply's `moving` field all mirror `esp32_firmware/main.py`. `scripts/fake_esp32_server.py`
is a third copy of the protocol and must move with them. Change every side in one
edit and reflash.

## Tests

`test/test_esp32_bridge_node.py` (33 tests) covers the bounds gate, the degree↔raw
conversion, the move and stream paths, the `status`-poll wait, `busy` rejection, the
`use_blocking_move` rollback, `is_moving`, a status reply with no `moving` field
(which must read as *unknown*, never as stopped), reconnect-after-transport-error,
the callback-group split, fault and dead-bus reporting, and the readback merge. `_FakeTransport` answers **by command name**, not from one FIFO: a
move costs a variable number of `status` round trips, so a queue would couple every
move test to the poll count. They run the real node against a fake
transport, so nothing touches the ESP32. Run them after **every** change here:
```bash
cd ros2_ws && source /opt/ros/jazzy/setup.bash && source install/setup.bash
python3 -m pytest src/esp32_bridge/test -q -k 'not flake8 and not pep257 and not copyright' -p no:cacheprovider
```
Run each package's tests separately; the two packages share test file names.
Rebuild first if code changed: `colcon build --packages-select esp32_bridge`.
Passing tests mean the logic matches the fake; the bench `[0,0,0,0]` move in
`LAUNCH.md` step 7a is still the only proof the arm agrees.
