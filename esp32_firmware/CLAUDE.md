# esp32_firmware — CLAUDE.md

`main.py` is the only code in the project that talks to servos. It runs as
MicroPython on the ESP32, joins Wi-Fi with a static IP, and serves one TCP client
at a time on port 5000 with newline-delimited JSON. Use the `esp32-firmware-change`
skill for any edit here: it covers the bridge-side update, flashing, and the bench
check.

## Invariants

- The firmware knows nothing about degrees, joint centres or limits. It receives raw
  LX-16A position counts (0–1000) and clamps only to that mechanical range. All
  safety lives on the Pi side, in `esp32_bridge`. Do not add a second set of limits
  here without the user deciding it (see finding 4 in
  `CODE_REVIEW_STRUCTURE_2026-09-18.md`).
- Five commands exist: `move_async` (min-jerk shaped, acks when **accepted**, the
  default path), `status` (`{"moving": bool, "timing": {...}}`), `move` (identical shaping but acks
  when physically done — the legacy blocking path, kept only as the bridge's
  `use_blocking_move` rollback), `servo` (straight to setpoint, acks immediately, no
  readback, and **preempts** a shaped move), `read_positions`. The bridge's
  `transport.py` and README mirror this exactly, and so does
  `scripts/fake_esp32_server.py`. A change to a command's shape or ack meaning is a
  protocol change: update firmware, `esp32_bridge_node.py`, `esp32_bridge/README.md`,
  `scripts/fake_esp32_server.py`, `TESTS.md` and `LAUNCH.md` together.
- A shaped move never starts from a guessed pose (user, 2026-09-26).
  `_resolve_start_positions` re-reads missed joints up to `_START_READ_ATTEMPTS`
  and `start_motion` returns False, arming nothing, if any joint still missed; the
  command is answered with an error. The old fallbacks (last completed target, or
  the new target) snapped a joint in the first step and were removed. Cost: up to
  ~0.3 s before the ack when reads keep missing (~30 ms normally, since 2026-09-26).
- The greeting `{"status": "ready"}` is sent on connect. The transport consumes it.
- No `print()` in the firmware. The REPL shows a banner and then nothing when
  `main.py` is healthy; silence is normal, not a fault.
- The main loop is non-blocking (2026-09-20, D4): `handle_client` polls the socket
  with `select.poll()`, dispatches any complete lines, then calls
  `advance_motion_step()` once. A command is therefore answered *during* a shaped
  move.
- **No stop switch** (removed 2026-09-26, user's decision): no GPIO input, no
  `SERVO_MOVE_STOP`, no `estopped` reply. The servo power plug is the only stop.
  `main()` blocks in `accept()`; an accept timeout was tried and broke the accepted
  socket on MicroPython v1.28 (2026-09-26), so do not retry it without testing on
  the board.
- A second shaped move arriving mid-motion is refused with `{"status": "busy"}` and
  dropped, not queued (D3). In normal operation the bridge serialises its commands so
  this never fires; it is the backstop for when that serialisation has been defeated.
  `servo` is the exception and preempts on purpose.
- The connection socket stays **blocking**; `select.poll()` is only a readiness test.
  MicroPython leaves `sendall()` on a non-blocking socket undefined, and a truncated
  reply on a line protocol with no correlation ID desyncs every later reply.
- Trajectory state lives in module-level preallocated globals and is stepped on an
  absolute `ticks_ms` schedule, so a step delayed by an interleaved `read_positions`
  is caught up on instead of stretching the whole trajectory — **but the catch-up is
  clamped**. If the next step is already more than one step-duration overdue the
  schedule resyncs to now. Firing a backlog back to back would drive the arm through
  that stretch faster than commanded; a trajectory that runs long is safe, one that
  runs fast is not.

- Step-timing counters (`_timing_*`, D13, 2026-09-26) are measurement only. They
  are reset in `start_motion`, updated in `advance_motion_step` and the main loop,
  and reported as `"timing"` in `status`. `_record_step_timing` runs on every step:
  keep it allocation-free (int globals only). Remove or keep once D13 is fixed; the
  user decides.

## Traps already hit

- `del some_bytearray[a:b]` raises `TypeError: 'bytearray' object doesn't support
  item deletion` on MicroPython. The D4 firmware (2026-09-20) used it in
  `handle_client` and so dropped every connection on first contact; it was never
  on the board until 2026-09-26. Use slice assignment (`buf[:n] = b""`), tested on
  the board.
- MicroPython's `socket.readinto(buf)` on a blocking socket waits until the whole
  buffer is full, unlike CPython's `recv_into`. With a 256-byte buffer a 20-byte
  command was never answered (no error, just silence). `handle_client` uses
  `recv(n)`, which returns what has arrived. Found on the board 2026-09-26.

- 2026-09-26: a module-level constant referred to one defined further down the
  file. It parsed fine but raised `NameError` at boot, before Wi-Fi, so the board
  simply stopped answering ping. A syntax check is not enough: before flashing,
  execute the file's module level in CPython with `machine`/`network` stubbed and
  the final `main()` call removed. That still cannot catch MicroPython-only API
  differences; only the board can. The same day `irq(..., hard=True)` passed the
  CPython check and failed on the board with `TypeError: extra keyword arguments given`.

- Wi-Fi power save caused 33% packet loss and 1 s ping spikes. `PM_NONE` is set on
  purpose; keep it.
- A `move` ack only proves the command reached the ESP32, never that a servo got it.
  When the arm is silent, the fault is below this layer: use
  `scripts/check_servo_bus.py` over USB, not Wi-Fi.
- Single-attempt position reads used to miss one servo about half the time, and
  from 2026-09-26 servo 3 missed nearly every read. **Cause found 2026-09-26: the
  UART driver's `timeout` did not hold.** In the running firmware `uart.read(14)`
  returned empty 1-2 ms after the request, while every servo's full reply takes
  1.86-1.94 ms (measured over USB, 0 misses in 60 with an explicit wait). The servos
  and cabling were fine; the bus probe saw all four ALIVE. `read_servo_position`
  (and the probe) now wait on `ticks_ms` for a valid reply up to
  `_READ_REPLY_DEADLINE_MS` (10 ms) and only read what `any()` reports. A reply
  with a bad checksum or a position outside 0-1000 counts as a miss (user,
  2026-09-26), so a corrupt or below-zero reading refuses a move instead of
  becoming its start pose. Do not go
  back to relying on `UART(timeout=...)` for the reply wait. The bridge's retry and
  merge stay as a backstop.
- `_TRAJECTORY_STEP_MS` must stay well above the UART write cost for 4 servos
  (~80 ms) or trajectory steps queue faster than the bus executes them. That margin
  is **shared with interleaved reads**. At the old 20 ms gap after each read, a
  mid-move `read_positions` cost ~90 ms, overran the ~70 ms slack and made moves
  visibly staged under `bringup.launch.py` (D13, measured 2026-09-26: 2-3 of 6 steps
  late, servos idle up to 70 ms; 0 late with the bridge alone). Reads now use
  `_INTER_READ_DELAY_MS` (5 ms), ~30 ms per read. Re-measure on the arm (`LAUNCH.md`
  step 9d) before touching either constant.
- Wi-Fi credentials are committed in this file. Moving them to a gitignored
  `secrets.py` is an open item.

## Hardware facts

| Item | Value | Source |
|------|-------|--------|
| Servo bus | UART2, GPIO17 TX → BusLinker RX, GPIO16 RX ← BusLinker TX | wiring |
| Baud | 115200 | BusLinker / LX-16A |
| Servo IDs | 1–4, base to tool | `SERVO_IDS`; must match `_EXPECTED_SERVO_COUNT` in the bridge |
| Raw position range | 0–1000 = 0–240° | LX-16A datasheet |
| Move duration clamp | 100–5000 ms | firmware constants |

## Verify after a change

No unit tests exist for the firmware. Verification is: flash, `ping 192.168.1.150`,
bring up the bridge, run the `[0,0,0,0]` home move from `LAUNCH.md` step 7a, and
check `read_joint_positions` returns all four joints. Say explicitly which of these
you did and which the user still has to do.
