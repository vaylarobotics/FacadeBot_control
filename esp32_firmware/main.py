import json
import network
import select
import socket
import time
from machine import UART, Pin

# ── Wi-Fi and TCP configuration ───────────────────────────────────────────────
# Fill in your network credentials and router gateway before flashing.
WIFI_SSID        = "Airtel_hart_5833"       # replace with your network name
WIFI_PASSWORD    = "16288935"   # replace with your network password
STATIC_IP        = "192.168.1.150"        # fixed IP the ESP32 will claim — pick one
                                           # not already in use on your network
SUBNET_MASK      = "255.255.255.0"
GATEWAY          = "192.168.1.1"          # your router's IP — run `ip route` on RPi to confirm
DNS              = "8.8.8.8"
TCP_PORT         = 5000                   # port the ESP32 listens on for commands

# ── Hardware configuration ────────────────────────────────────────────────────
# Confirm GPIO pins match yur pour physical wiring before flashing.
# UART2 (GPIO16/17) is wired to the BusLinker V2.5 TTL header.
SERVO_IDS        = [1, 2, 3, 4]    # one servo per joint, base to end effector
BAUD_BUSLINKER   = 115200
TX2_PIN          = 17              # ESP32 GPIO17 → BusLinker TTL RX
RX2_PIN          = 16              # ESP32 GPIO16 ← BusLinker TTL TX

# ── Servo protocol constants ──────────────────────────────────────────────────
# LX-16A packet: 0x55 0x55 ID LEN CMD [PARAMS...] CHECKSUM
# LEN = num_params + 3 (covers CMD + PARAMS + CHECKSUM)
_CMD_MOVE_TIME_WRITE  = 1
_CMD_POS_READ         = 0x1C  # LX-16A position read command
_POSITION_MIN         = 0
_POSITION_MAX         = 1000
_DURATION_MIN_MS      = 100
_DURATION_MAX_MS      = 5000
_INTER_SERVO_DELAY_MS = 20    # gap between back-to-back sends on the bus
# Gap after a completed position read, before the next servo's request. Shorter
# than the write gap because a read now returns only once the whole reply is in,
# so the bus is already idle. Measured 2026-09-26 over USB, 200 reads per setting:
# 0 misses at 20/10/5/2/1/0 ms; a 4-servo read took 89/49/29/17/13/9 ms. 5 ms keeps
# margin over the 0 ms that also worked. At 20 ms a read_positions arriving
# mid-move (~90 ms) overran a step's ~70 ms slack and the servos stopped between
# steps (DECISIONS.md D13, measured on the arm the same day).
_INTER_READ_DELAY_MS  = 5

# ── Read response constants ───────────────────────────────────────────────────
# A position read response is 8 bytes: 0x55 0x55 ID 5 CMD POS_LO POS_HI CHECKSUM
# Some BusLinker variants echo the 6-byte request back on RX before the response.
# _READ_MAX_BYTES covers both cases so we can scan for the valid response pattern.
_READ_RESPONSE_BYTES  = 8     # bytes in a valid position read response
_READ_MAX_BYTES       = 14    # _READ_RESPONSE_BYTES + 6-byte echo if BusLinker echoes TX
_READ_TIMEOUT_MS      = 10    # UART driver timeout; now only bounds the stale-byte drain. Reply waits use _READ_REPLY_DEADLINE_MS
_READ_TIMEOUT_CHAR_MS = 5     # ms allowed between successive RX bytes
# Deadline for a whole position reply, timed by read_servo_position itself on
# ticks_ms rather than by the UART driver. Measured 2026-09-26 over USB: every
# servo's full 8-byte reply lands 1.86-1.94 ms after the request, 0 misses in 60.
# The driver's own `timeout` above did not hold: in the running firmware
# uart.read() returned empty after 1-2 ms and servo 3 missed nearly every read
# (suspected cause: the timeout rounds to the 10 ms RTOS tick, so the real wait
# depends on where in the tick the read starts). ~5x the measured reply time.
_READ_REPLY_DEADLINE_MS = 10

uart_buslinker = UART(2, baudrate=BAUD_BUSLINKER, tx=Pin(TX2_PIN), rx=Pin(RX2_PIN),
                      timeout=_READ_TIMEOUT_MS, timeout_char=_READ_TIMEOUT_CHAR_MS)

# ── Minimum-jerk trajectory shaping ───────────────────────────────────────────
# Interval between trajectory steps. move_joints() blocks ~_TRAJECTORY_WRITE_MS
# per call (UART write cost for 4 servos) - this must stay well above that or
# steps queue up faster than the bus can execute them. ~1.9x margin, empirical.
# NOTE: since the loop became non-blocking, a read_positions arriving mid-move
# shares this margin: ~30 ms of UART since 2026-09-26 (_INTER_READ_DELAY_MS), up
# to ~60 ms if every servo misses, against ~70 ms of slack. At the old ~90 ms it
# pushed steps late and the servos stopped between them (D13). Re-measure on the
# arm before changing either constant.
_TRAJECTORY_STEP_MS = 150
_TRAJECTORY_WRITE_MS = _INTER_SERVO_DELAY_MS * len(SERVO_IDS)  # ~80ms

# ── Socket polling ────────────────────────────────────────────────────────────
# The main loop polls the socket instead of blocking on it, so a command
# (read_positions, status) is answered while a shaped trajectory is running.
# Poll briefly while moving so a due step is written close to its scheduled
# time; wait longer when idle so an idle board is not spinning at full tilt.
_SOCKET_POLL_ACTIVE_MS = 5
_SOCKET_POLL_IDLE_MS   = 50

# One command line is ~90 bytes. A line longer than the cap is a protocol fault,
# not a command, and is dropped rather than allowed to grow the buffer without
# bound. recv() allocates one small bytes object per chunk received - per command,
# like the JSON decode - because MicroPython's socket.readinto() on a blocking
# socket waits to fill the WHOLE buffer, so a 20-byte command was never answered
# (found on the board, v1.28, 2026-09-26).
_RECV_CHUNK_BYTES    = 256
_RX_BUFFER_MAX_BYTES = 512
_rx_buffer = bytearray()

# A shaped move starts from where every joint actually is, never from a guess
# (user, 2026-09-26). A missed joint is re-read on its own, up to this many
# attempts in all; 5 matches the bridge's _MAX_READ_RETRIES. If a joint still
# cannot be read the move is refused. The old fallbacks (the last completed move's
# target, or the new target itself) both snapped the joint in the first ~150 ms step.
_START_READ_ATTEMPTS = 5
_start_read_ok = [False] * len(SERVO_IDS)  # preallocated, rewritten per move

# ── Trajectory state ──────────────────────────────────────────────────────────
# A shaped move is advanced one step per main-loop iteration instead of running
# to completion inside the command handler. All of this is preallocated and
# written in place, so stepping a trajectory allocates nothing.
# Honest caveat: the main loop is not allocation-free overall - poll() returns a
# fresh list each iteration, and decoding a command allocates a line and a dict.
# Those are per-command and per-poll, not per-step; keeping the trajectory state
# static is what stops a move from fragmenting the heap mid-swing.
_motion_active             = False
_motion_start_raw          = [0] * len(SERVO_IDS)
_motion_target_raw         = [0] * len(SERVO_IDS)
_motion_step_positions_raw = [0] * len(SERVO_IDS)  # rewritten in place each step
_motion_step_index         = 0
_motion_step_count         = 0
_motion_step_duration_ms   = 0
_motion_next_step_ms       = 0
_motion_last_step_ms       = 0

# ── Step timing diagnostics (DECISIONS.md D13) ────────────────────────────────
# Measurement only, no effect on motion. Each shaped step tells the servos to
# arrive in exactly one step duration, so if the next step is written later than
# that, the servos sit still for the difference: the "staged" motion seen on the
# arm 2026-09-26. These counters record how long, and what the loop was doing
# instead, so the cause is measured rather than guessed. Reset at the start of
# every shaped move, kept after it ends, reported in `status` as "timing".
# A gap below this is not counted as late: ticks_ms jitter and one 5 ms socket
# poll are expected. Visible stalls on the arm were hundreds of ms.
_LATE_STEP_THRESHOLD_MS = 20
_timing_steps          = 0  # steps written this move
_timing_late_steps     = 0  # steps whose servos sat idle > _LATE_STEP_THRESHOLD_MS
_timing_worst_idle_ms  = 0  # longest time servos sat at a reached step
_timing_worst_write_ms = 0  # longest move_joints() call (4 UART writes + gaps)
_timing_worst_cmd_ms   = 0  # longest handle_command() while a move was active
_timing_cmds           = 0  # commands handled while a move was active
_timing_worst_poll_ms  = 0  # longest poller.poll() while a move was active


def build_move_packet(servo_id: int, position: int, duration_ms: int) -> bytes:
    position    = max(_POSITION_MIN, min(_POSITION_MAX, position))
    duration_ms = max(_DURATION_MIN_MS, min(_DURATION_MAX_MS, duration_ms))

    pos_lo  = position    & 0xFF
    pos_hi  = (position    >> 8) & 0xFF
    time_lo = duration_ms & 0xFF
    time_hi = (duration_ms >> 8) & 0xFF

    length   = 7  # 4 params + 3
    checksum = (~(servo_id + length + _CMD_MOVE_TIME_WRITE
                  + pos_lo + pos_hi + time_lo + time_hi)) & 0xFF

    return bytes([0x55, 0x55, servo_id, length,
                  _CMD_MOVE_TIME_WRITE,
                  pos_lo, pos_hi, time_lo, time_hi,
                  checksum])


def build_read_packet(servo_id: int) -> bytes:
    length   = 3  # 0 params + 3
    checksum = (~(servo_id + length + _CMD_POS_READ)) & 0xFF
    return bytes([0x55, 0x55, servo_id, length, _CMD_POS_READ, checksum])


def move_joints(positions: list, duration_ms: int) -> None:
    for servo_id, position in zip(SERVO_IDS, positions):
        uart_buslinker.write(build_move_packet(servo_id, position, duration_ms))
        time.sleep_ms(_INTER_SERVO_DELAY_MS)


def read_servo_position(servo_id: int) -> int | None:
    if uart_buslinker.any():
        uart_buslinker.read()  # drain stale bytes left over from a late servo response
    uart_buslinker.write(build_read_packet(servo_id))
    # Collect bytes until a valid reply is found or the deadline passes. Reading
    # only what any() reports never blocks, so the deadline is the only wait.
    raw = b""
    deadline_ms = time.ticks_add(time.ticks_ms(), _READ_REPLY_DEADLINE_MS)
    while len(raw) < _READ_MAX_BYTES:
        available = uart_buslinker.any()
        if available:
            chunk = uart_buslinker.read(available)
            if chunk:
                raw += chunk
            position = _find_position_reply(raw, servo_id)
            if position is not None:
                return position
        elif time.ticks_diff(deadline_ms, time.ticks_ms()) <= 0:
            break
    return _find_position_reply(raw, servo_id)


def _find_position_reply(raw: bytes, servo_id: int) -> int | None:
    # Scan for valid response pattern regardless of whether TX bytes were echoed back.
    # Response: 0x55 0x55 ID 5 _CMD_POS_READ POS_LO POS_HI CHECKSUM
    # A reply that fails its checksum, or reads outside 0-1000, counts as a miss
    # (user, 2026-09-26). The LX-16A reports a signed 16-bit position, so a joint
    # just below raw 0 reads ~65530; taken as a move's start pose, the first step
    # would drive it to the far end of travel. A miss refuses the move instead.
    for i in range(len(raw) - 7):
        if (raw[i]   == 0x55        and
                raw[i+1] == 0x55        and
                raw[i+2] == servo_id    and
                raw[i+3] == 5           and
                raw[i+4] == _CMD_POS_READ):
            checksum = (~(servo_id + 5 + _CMD_POS_READ + raw[i+5] + raw[i+6])) & 0xFF
            if raw[i+7] != checksum:
                return None
            position = (raw[i+6] << 8) | raw[i+5]
            if position < _POSITION_MIN or position > _POSITION_MAX:
                return None
            return position
    return None


def read_all_positions() -> list:
    positions = []
    for servo_id in SERVO_IDS:
        positions.append(read_servo_position(servo_id))
        time.sleep_ms(_INTER_READ_DELAY_MS)
    return positions


def _min_jerk_position(start: int, end: int, tau: float) -> int:
    # Flash & Hogan minimum-jerk model: closed-form quintic with zero
    # velocity/acceleration at both endpoints. tau in [0,1].
    scale = 10 * tau**3 - 15 * tau**4 + 6 * tau**5
    return round(start + (end - start) * scale)


def _resolve_start_positions() -> bool:
    """Fill _motion_start_raw with where every joint is now. Re-reads only the
    joints that missed. Returns False if any joint never answered."""
    for i in range(len(SERVO_IDS)):
        _start_read_ok[i] = False
    for _attempt in range(_START_READ_ATTEMPTS):
        all_read = True
        for i in range(len(SERVO_IDS)):
            if _start_read_ok[i]:
                continue
            position = read_servo_position(SERVO_IDS[i])
            time.sleep_ms(_INTER_READ_DELAY_MS)
            if position is None:
                all_read = False
                continue
            _motion_start_raw[i] = position
            _start_read_ok[i] = True
        if all_read:
            return True
    return False


def start_motion(positions: list, duration_ms: int) -> bool:
    """Arm a shaped trajectory for the main loop to step through. Returns as soon
    as the start pose has been read - the motion itself runs in advance_motion_step.
    Returns False, arming nothing, if the start pose could not be read."""
    global _motion_active, _motion_step_index, _motion_step_count
    global _motion_step_duration_ms, _motion_next_step_ms

    duration_ms_clamped = max(_DURATION_MIN_MS, min(_DURATION_MAX_MS, duration_ms))

    if not _resolve_start_positions():
        return False
    for i in range(len(SERVO_IDS)):
        _motion_target_raw[i] = positions[i]

    _motion_step_count       = max(1, duration_ms_clamped // _TRAJECTORY_STEP_MS)
    _motion_step_duration_ms = duration_ms_clamped // _motion_step_count
    _motion_step_index       = 0
    _motion_next_step_ms     = time.ticks_ms()  # first step is due immediately
    _reset_timing()
    _motion_active           = True
    return True


def _reset_timing() -> None:
    global _timing_steps, _timing_late_steps, _timing_worst_idle_ms
    global _timing_worst_write_ms, _timing_worst_cmd_ms, _timing_cmds, _timing_worst_poll_ms
    _timing_steps = 0
    _timing_late_steps = 0
    _timing_worst_idle_ms = 0
    _timing_worst_write_ms = 0
    _timing_worst_cmd_ms = 0
    _timing_cmds = 0
    _timing_worst_poll_ms = 0


def _record_step_timing(step_start_ms: int, write_ms: int) -> None:
    """Called once per written step. The first step has no predecessor to idle after."""
    global _timing_steps, _timing_late_steps, _timing_worst_idle_ms, _timing_worst_write_ms
    if _timing_steps > 0:
        # Every servo got the previous step at (previous start + its slot) and got
        # this one at (this start + the same slot), so its idle time is the same for
        # all four: interval between step starts minus the time it was given.
        idle_ms = time.ticks_diff(step_start_ms, _motion_last_step_ms) - _motion_step_duration_ms
        if idle_ms > _timing_worst_idle_ms:
            _timing_worst_idle_ms = idle_ms
        if idle_ms > _LATE_STEP_THRESHOLD_MS:
            _timing_late_steps += 1
    if write_ms > _timing_worst_write_ms:
        _timing_worst_write_ms = write_ms
    _timing_steps += 1


def advance_motion_step() -> None:
    """Write the next trajectory step, if one is due. One step per call."""
    global _motion_active, _motion_step_index, _motion_next_step_ms, _motion_last_step_ms

    if not _motion_active:
        return
    if time.ticks_diff(time.ticks_ms(), _motion_next_step_ms) < 0:
        return

    _motion_step_index += 1
    tau = _motion_step_index / _motion_step_count
    for i in range(len(SERVO_IDS)):
        _motion_step_positions_raw[i] = _min_jerk_position(
            _motion_start_raw[i], _motion_target_raw[i], tau)
    step_start_ms = time.ticks_ms()
    move_joints(_motion_step_positions_raw, _motion_step_duration_ms)
    _record_step_timing(step_start_ms, time.ticks_diff(time.ticks_ms(), step_start_ms))
    _motion_last_step_ms = step_start_ms

    if _motion_step_index >= _motion_step_count:
        _motion_active = False
        return

    # Schedule the next step from the previous step's due time, not from "now":
    # a step pushed late by an interleaved read_positions is then caught up on,
    # instead of every delay permanently stretching the trajectory.
    _motion_next_step_ms = time.ticks_add(_motion_next_step_ms, _motion_step_duration_ms)
    # ...but never let a backlog build up and then fire back to back. Steps can
    # still fall behind (a read that misses on every servo, a slow command), so
    # the clamp stays.
    # Bursting through the backlog would drive the arm through that part of the
    # trajectory faster than commanded, which is the one direction this must not
    # fail in: a trajectory that takes longer than duration_ms is safe, one that
    # moves faster than commanded is not. Give up the lost time instead.
    if time.ticks_diff(time.ticks_ms(), _motion_next_step_ms) > _motion_step_duration_ms:
        # One full period from now, not "now" - resyncing to now would leave the
        # next step already due and it would fire on the very next iteration,
        # which is the burst this clamp exists to prevent.
        _motion_next_step_ms = time.ticks_add(time.ticks_ms(), _motion_step_duration_ms)


def cancel_motion() -> None:
    """Stop stepping the shaped trajectory. The servos hold wherever the last
    step put them - this is not a stop primitive, it only stops new setpoints."""
    global _motion_active
    _motion_active = False


def move_joints_min_jerk(positions: list, duration_ms: int) -> bool:
    """Blocking shaped move: runs the whole trajectory before returning, so the
    socket is deaf for its duration. Kept as the rollback path for the bridge's
    use_blocking_move parameter - start_motion/advance_motion_step is the
    default. Both share the same shaping, step count and step duration.
    Returns False, without moving, if the start pose could not be read."""
    global _motion_active

    if not start_motion(positions, duration_ms):
        return False
    while _motion_active:
        advance_motion_step()
        time.sleep_ms(_SOCKET_POLL_ACTIVE_MS)
    return True


def connect_wifi() -> network.WLAN:
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    # Default power-save parks the radio between AP beacons, measured 2026-09-12 as
    # 33% packet loss and 115/622/1129 ms min/avg/max ping. This link carries ~12 Hz
    # setpoint streams (continuous_trajectory_node), so the radio has to stay awake.
    wlan.config(pm=network.WLAN.PM_NONE)
    # Set static IP before connecting so the address is predictable
    wlan.ifconfig((STATIC_IP, SUBNET_MASK, GATEWAY, DNS))
    wlan.connect(WIFI_SSID, WIFI_PASSWORD)
    while not wlan.isconnected():
        time.sleep_ms(200)
    return wlan


def _handle_move_like(conn: socket.socket, name: str, cmd: dict) -> None:
    """Shared validation for move / move_async / servo - identical field checks."""
    positions = cmd.get("positions")
    duration_ms = cmd.get("duration_ms")
    if not isinstance(positions, list) or len(positions) != len(SERVO_IDS):
        conn.sendall(b'{"status": "error", "msg": "positions must be a list with one value per servo"}\n')
        return
    if not isinstance(duration_ms, int):
        conn.sendall(b'{"status": "error", "msg": "duration_ms must be an integer"}\n')
        return

    if name == "servo":
        # Streaming setpoint preempts a shaped move on purpose: both write to the
        # same servos over the same UART, so leaving the stepper running would have
        # the two fight each other. Retargeting is the whole point of this path.
        cancel_motion()
        move_joints(positions, duration_ms)
        conn.sendall(b'{"status": "ok"}\n')
        return

    # A second shaped move arriving mid-motion is refused rather than allowed to
    # silently change the arm's direction partway through a swing.
    if _motion_active:
        conn.sendall(b'{"status": "busy"}\n')
        return

    if name == "move_async":
        started = start_motion(positions, duration_ms)
    else:
        started = move_joints_min_jerk(positions, duration_ms)
    if not started:
        conn.sendall(b'{"status": "error", "msg": "start position unreadable - move refused"}\n')
        return
    conn.sendall(b'{"status": "ok"}\n')


def handle_command(conn: socket.socket, line: bytes) -> None:
    """Act on one newline-delimited JSON command and send its reply."""
    try:
        cmd = json.loads(line)
    except ValueError:
        conn.sendall(b'{"status": "error", "msg": "invalid json"}\n')
        return

    name = cmd.get("cmd")

    if name == "status":
        conn.sendall(json.dumps(
            {"status": "ok", "moving": _motion_active,
             "timing": {"steps": _timing_steps, "late_steps": _timing_late_steps,
                        "worst_idle_ms": _timing_worst_idle_ms,
                        "worst_write_ms": _timing_worst_write_ms,
                        "worst_cmd_ms": _timing_worst_cmd_ms, "cmds": _timing_cmds,
                        "worst_poll_ms": _timing_worst_poll_ms}}).encode() + b'\n')
    elif name == "read_positions":
        positions = read_all_positions()
        conn.sendall(json.dumps({"status": "ok", "positions": positions}).encode() + b'\n')
    elif name == "move" or name == "move_async" or name == "servo":
        _handle_move_like(conn, name, cmd)
    else:
        conn.sendall(b'{"status": "error", "msg": "unknown command"}\n')


def _handle_command_timed(conn: socket.socket, line: bytes) -> None:
    """handle_command, timed when it runs in the middle of a shaped move."""
    global _timing_worst_cmd_ms, _timing_cmds
    if not _motion_active:
        handle_command(conn, line)
        return
    cmd_start_ms = time.ticks_ms()
    handle_command(conn, line)
    cmd_ms = time.ticks_diff(time.ticks_ms(), cmd_start_ms)
    _timing_cmds += 1
    if cmd_ms > _timing_worst_cmd_ms:
        _timing_worst_cmd_ms = cmd_ms


def handle_client(conn: socket.socket) -> None:
    """Serve one connected client until it disconnects.

    The loop polls the socket rather than blocking on a read, so a command is
    answered while a shaped trajectory is still running - that is what lets the
    Pi read joint positions mid-move. Each iteration: drain whatever arrived,
    act on any complete lines, then advance the trajectory by at most one step.
    """
    global _timing_worst_poll_ms

    conn.sendall(b'{"status": "ready"}\n')
    # The socket deliberately stays BLOCKING. poll() is used purely as a
    # readiness test, so a read only happens when a byte is already there and
    # never blocks in practice. MicroPython leaves sendall() on a non-blocking
    # socket undefined - it can truncate a reply, which on a line protocol with
    # no correlation ID desyncs every later request/reply pair.
    poller = select.poll()
    poller.register(conn, select.POLLIN)
    # Slice assignment, not `del`: MicroPython's bytearray has no item deletion
    # (TypeError on the board, v1.28, 2026-09-26).
    _rx_buffer[:] = b""

    while True:
        poll_start_ms = time.ticks_ms()
        events = poller.poll(_SOCKET_POLL_ACTIVE_MS if _motion_active else _SOCKET_POLL_IDLE_MS)
        if _motion_active:
            poll_ms = time.ticks_diff(time.ticks_ms(), poll_start_ms)
            if poll_ms > _timing_worst_poll_ms:
                _timing_worst_poll_ms = poll_ms
        if events:
            flags = events[0][1]
            # Read before acting on a hangup: POLLIN and POLLHUP can be set
            # together, and the last command is still worth answering.
            if flags & select.POLLIN:
                chunk = conn.recv(_RECV_CHUNK_BYTES)
                if not chunk:
                    break  # client closed the connection
                _rx_buffer.extend(chunk)

                while True:
                    newline = _rx_buffer.find(b"\n")
                    if newline < 0:
                        break
                    line = bytes(_rx_buffer[:newline])
                    _rx_buffer[:newline + 1] = b""
                    _handle_command_timed(conn, line)

                # Dropped silently, with no reply: an unsolicited line here would
                # answer a command nobody sent and desync every reply after it.
                if len(_rx_buffer) > _RX_BUFFER_MAX_BYTES:
                    _rx_buffer[:] = b""
            elif flags & (select.POLLHUP | select.POLLERR):
                break

        advance_motion_step()


def main() -> None:
    connect_wifi()

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    # SO_REUSEADDR lets the ESP32 rebind immediately after a reset without
    # waiting for the OS to release the port (avoids "address already in use")
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(('', TCP_PORT))
    server.listen(1)

    while True:
        # accept() blocks. A timeout here was tried on 2026-09-26 and broke the
        # accepted connection on this MicroPython build (no greeting, or closed on
        # the first command), so it stays blocking.
        conn, addr = server.accept()
        try:
            handle_client(conn)
        except OSError:
            pass  # network errors during a session — just accept the next client
        finally:
            # Nothing advances the trajectory while we are blocked in accept(), so
            # a half-finished motion would resume unpredictably on the next connect.
            # Drop it: the servos hold at the last step written.
            cancel_motion()
            conn.close()


main()
