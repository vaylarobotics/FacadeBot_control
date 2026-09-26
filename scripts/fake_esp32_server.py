"""
Stand-in ESP32 TCP server, for testing the ROS2 control loop with no ESP32,
Hiwonder board, or servos attached at all.

This is NOT a client that commands motion (the thing scripts/CLAUDE.md forbids)
- it is the other end of the socket. It plays the role esp32_firmware/main.py
normally plays: it accepts the one TCP connection esp32_bridge_node opens and
answers its commands, so the bridge, facade_control_node, trajectory_node,
continuous_trajectory_node and joint_state_publisher_node can all run for real,
unmodified, talking real ROS2 messages to a real (loopback) socket - only the
servos are fake. Useful for exercising the bounds gate, IK, trajectory
sequencing/timing, and RViz end to end without the arm powered.

Wire protocol mirrors esp32_firmware/main.py and
ros2_ws/src/esp32_bridge/README.md exactly - keep this in step if either
changes:
  - greeting `{"status": "ready"}` sent immediately on connect
  - `move_async`: arms a shaped motion and acks immediately; the simulated
    position then ramps to target in the background. This is the path the
    bridge uses by default.
  - `move`: the legacy blocking path - blocks until `duration_ms` (clamped
    100-5000ms) has elapsed, then acks. Kept because the bridge can still be
    put back on it with `use_blocking_move:=true`.
  - `status`: `{"status": "ok", "moving": true|false}` - whether a shaped
    motion is still running. The bridge polls this to decide a move has
    finished. The firmware's diagnostic `timing` field is not simulated.
  - `move`/`move_async` arriving while a shaped motion is already running are
    refused with `{"status": "busy"}`, same as the firmware.
  - `servo`: acks immediately and preempts any shaped motion in progress; the
    simulated position ramps to target over `duration_ms`, same as a real
    LX-16A interpolating a MOVE_TIME_WRITE command after the ESP32 has already
    moved on
  - `read_positions`: returns each joint's current simulated position, and is
    answerable mid-move exactly as the non-blocking firmware now is

Simplifications, deliberate:
  - Position ramps linearly, not the firmware's true minimum-jerk quintic -
    close enough for testing arrival timing, not for validating the min-jerk
    math itself (that's real-hardware/firmware territory).
  - No simulated read dropouts. The real bus misses one servo's reading on
    roughly half of single-attempt reads (see the esp32_bridge README) - this
    server never does, so passing against it is not proof of tolerance to
    that. It only exercises the ROS2-level control loop.
  - No simulated UART contention. On the real arm a read_positions arriving
    mid-move costs ~120ms of bus time and pushes a trajectory step late; here
    reads are free, so this cannot tell you what the real feedback rate during
    a move will be.
  - All four joints start at raw position 500 (mid-travel) - there is no
    physical arm here to already be sitting at its measured center.

Usage:
    python3 scripts/fake_esp32_server.py
    # in another terminal, point the bridge at it instead of the real ESP32:
    ros2 run esp32_bridge esp32_bridge_node --ros-args -p esp32_host:=127.0.0.1

No ROS2 needed to run this script itself - stdlib only, works from a plain
checkout with no colcon build.
"""
import argparse
import json
import socket
import time

# Must match SERVO_IDS / _EXPECTED_SERVO_COUNT in esp32_firmware/main.py and
# esp32_bridge_node.py.
_SERVO_COUNT = 4
_POSITION_MIN_RAW = 0
_POSITION_MAX_RAW = 1000
_DURATION_MIN_MS = 100
_DURATION_MAX_MS = 5000
_INITIAL_POSITION_RAW = 500


def _clamp(value: int, lo: int, hi: int) -> int:
    return max(lo, min(hi, value))


def _initial_motion(now: float) -> dict:
    return {
        "start_raw": _INITIAL_POSITION_RAW,
        "target_raw": _INITIAL_POSITION_RAW,
        "start_time": now,
        "duration_sec": 0.0,
    }


def _position_now(motion: dict, now: float) -> int:
    if motion["duration_sec"] <= 0:
        return motion["target_raw"]
    tau = (now - motion["start_time"]) / motion["duration_sec"]
    if tau >= 1.0:
        return motion["target_raw"]
    return round(motion["start_raw"] + (motion["target_raw"] - motion["start_raw"]) * tau)


def _current_positions_raw(sim_state: list) -> list:
    now = time.monotonic()
    return [_position_now(motion, now) for motion in sim_state]


def _apply_motion(sim_state: list, positions_raw: list, duration_ms: int) -> int:
    duration_ms = _clamp(duration_ms, _DURATION_MIN_MS, _DURATION_MAX_MS)
    now = time.monotonic()
    current = _current_positions_raw(sim_state)
    for i in range(_SERVO_COUNT):
        target = _clamp(positions_raw[i], _POSITION_MIN_RAW, _POSITION_MAX_RAW)
        sim_state[i] = {
            "start_raw": current[i],
            "target_raw": target,
            "start_time": now,
            "duration_sec": duration_ms / 1000.0,
        }
    return duration_ms


def _is_moving(shaped: dict) -> bool:
    """Whether a shaped (move/move_async) motion is still running. A `servo`
    setpoint does not count as moving - the firmware's stepper is not running
    for it, the servo is just interpolating on its own."""
    if not shaped["active"]:
        return False
    if time.monotonic() >= shaped["end_time"]:
        shaped["active"] = False
    return shaped["active"]


def _validate_move_like(cmd: dict) -> tuple:
    """Mirrors the exact checks handle_client() in main.py applies to `move`,
    `move_async` and `servo`. Returns (positions, duration_ms, error_msg)."""
    positions = cmd.get("positions")
    duration_ms = cmd.get("duration_ms")
    if not isinstance(positions, list) or len(positions) != _SERVO_COUNT:
        return None, None, "positions must be a list with one value per servo"
    if not isinstance(duration_ms, int):
        return None, None, "duration_ms must be an integer"
    return positions, duration_ms, None


def _send(conn: socket.socket, obj: dict) -> None:
    conn.sendall((json.dumps(obj) + "\n").encode())


def _handle_client(conn: socket.socket, sim_state: list, shaped: dict, verbose: bool) -> None:
    _send(conn, {"status": "ready"})
    f = conn.makefile("rb")
    while True:
        line = f.readline()
        if not line:
            break

        try:
            cmd = json.loads(line.strip())
        except ValueError:
            _send(conn, {"status": "error", "msg": "invalid json"})
            continue

        name = cmd.get("cmd")
        if name == "status":
            moving = _is_moving(shaped)
            if verbose:
                print(f"status <- moving={moving}")
            _send(conn, {"status": "ok", "moving": moving})

        elif name in ("move", "move_async"):
            positions, duration_ms, error_msg = _validate_move_like(cmd)
            if error_msg is not None:
                _send(conn, {"status": "error", "msg": error_msg})
                continue
            if _is_moving(shaped):
                if verbose:
                    print(f"{name:6s} -> refused, already moving")
                _send(conn, {"status": "busy"})
                continue
            actual_duration_ms = _apply_motion(sim_state, positions, duration_ms)
            shaped["active"] = True
            shaped["end_time"] = time.monotonic() + actual_duration_ms / 1000.0
            if name == "move":
                if verbose:
                    print(f"move   -> {positions} over {actual_duration_ms}ms (blocking)")
                time.sleep(actual_duration_ms / 1000.0)
                shaped["active"] = False
            elif verbose:
                print(f"move_async -> {positions} over {actual_duration_ms}ms (non-blocking)")
            _send(conn, {"status": "ok"})

        elif name == "servo":
            positions, duration_ms, error_msg = _validate_move_like(cmd)
            if error_msg is not None:
                _send(conn, {"status": "error", "msg": error_msg})
                continue
            shaped["active"] = False  # a stream setpoint preempts a shaped move
            actual_duration_ms = _apply_motion(sim_state, positions, duration_ms)
            if verbose:
                print(f"servo  -> {positions} over {actual_duration_ms}ms (non-blocking)")
            _send(conn, {"status": "ok"})

        elif name == "read_positions":
            positions = _current_positions_raw(sim_state)
            if verbose:
                print(f"read   <- {positions}")
            _send(conn, {"status": "ok", "positions": positions})

        else:
            _send(conn, {"status": "error", "msg": "unknown command"})


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--host", default="127.0.0.1", help="address to listen on (default: 127.0.0.1)")
    parser.add_argument("--port", type=int, default=5000, help="TCP port to listen on (default: 5000)")
    parser.add_argument("--quiet", action="store_true", help="suppress per-command logging")
    args = parser.parse_args()

    sim_state = [_initial_motion(time.monotonic()) for _ in range(_SERVO_COUNT)]
    shaped = {"active": False, "end_time": 0.0}

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((args.host, args.port))
    server.listen(1)
    print(f"fake ESP32 server listening on {args.host}:{args.port} (Ctrl-C to stop)")
    print(f"all {_SERVO_COUNT} joints start at raw position {_INITIAL_POSITION_RAW}")

    try:
        while True:
            conn, addr = server.accept()
            print(f"client connected: {addr}")
            try:
                _handle_client(conn, sim_state, shaped, verbose=not args.quiet)
            except OSError:
                pass  # client dropped mid-session - just accept the next one
            finally:
                shaped["active"] = False
                conn.close()
                print("client disconnected")
    except KeyboardInterrupt:
        pass
    finally:
        server.close()


if __name__ == "__main__":
    main()
