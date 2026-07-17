"""
Stream a joint trajectory to the FacadeBot ESP32 over Wi-Fi/TCP.

Each waypoint is sent as a move command and acknowledged before the next
waypoint is sent. The ESP32 shapes each move as a minimum-jerk trajectory
internally and only acks once the servos have physically reached the target
(see move_joints_min_jerk in esp32_firmware/main.py) - no extra wait is
needed here before sending the next waypoint.

Usage:
    python3 test_trajectory.py
    python3 test_trajectory.py --ip 192.168.1.100 --port 5000
"""
import argparse
import json
import socket
import sys

ESP32_IP             = "192.168.1.100"  # must match STATIC_IP in esp32_firmware/main.py
TCP_PORT             = 5000             # must match TCP_PORT in esp32_firmware/main.py
RESPONSE_TIMEOUT_SEC = 10

# Demonstration trajectory: slow sweep from center (120°) toward a target pose and back.
# Each row is [servo1_angle_deg, servo2_angle_deg, servo3_angle_deg, servo4_angle_deg, duration_ms].
# Servo range is 0-240 degrees (LX-16A datasheet). Adjust these values to match
# your arm's safe range before running on hardware.
DEMO_TRAJECTORY = [
    [120, 120, 120, 120, 1000],  # center
    [120, 120, 120, 20, 1000],
    [120, 120, 120, 220, 1000],
]

_SERVO_ANGLE_MIN_DEG = 0.0  # must match esp32_bridge_node.py's copy of this constant
_SERVO_ANGLE_MAX_DEG = 240.0
_POSITION_MAX_RAW    = 1000  # must match _POSITION_MAX in esp32_firmware/main.py


def _angle_deg_to_position_raw(angle_deg: float) -> int:
    angle_deg_clamped = max(_SERVO_ANGLE_MIN_DEG, min(_SERVO_ANGLE_MAX_DEG, angle_deg))
    return round(angle_deg_clamped * _POSITION_MAX_RAW / _SERVO_ANGLE_MAX_DEG)


def send_command(f: "socket file", sock: socket.socket, payload: dict) -> dict:
    sock.sendall((json.dumps(payload) + "\n").encode())
    raw = f.readline()
    return json.loads(raw.strip())


def main() -> None:
    parser = argparse.ArgumentParser(description="Stream a joint trajectory to the FacadeBot arm.")
    parser.add_argument("--ip",   default=ESP32_IP,
                        help=f"ESP32 IP address (default: {ESP32_IP})")
    parser.add_argument("--port", type=int, default=TCP_PORT,
                        help=f"ESP32 TCP port (default: {TCP_PORT})")
    args = parser.parse_args()

    print(f"Connecting to ESP32 at {args.ip}:{args.port}...")
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(RESPONSE_TIMEOUT_SEC)
        sock.connect((args.ip, args.port))
    except OSError as e:
        print(f"ERROR: Could not connect — {e}")
        sys.exit(1)

    f = sock.makefile("rb")

    ready_raw = f.readline()
    try:
        ready = json.loads(ready_raw.strip())
        if ready.get("status") != "ready":
            print(f"WARNING: unexpected greeting: {ready}")
    except (ValueError, UnicodeDecodeError):
        print(f"WARNING: could not parse greeting: {ready_raw}")

    print(f"Sending trajectory ({len(DEMO_TRAJECTORY)} waypoints)...")

    for i, point in enumerate(DEMO_TRAJECTORY):
        angles_deg    = point[:4]
        duration_ms   = point[4]
        positions_raw = [_angle_deg_to_position_raw(a) for a in angles_deg]

        response = send_command(f, sock, {
            "cmd":         "move",
            "positions":   positions_raw,
            "duration_ms": duration_ms,
        })

        if response.get("status") != "ok":
            print(f"ERROR at waypoint {i}: {response}")
            sock.close()
            sys.exit(1)

        print(f"  waypoint {i+1}/{len(DEMO_TRAJECTORY)}: {angles_deg} deg in {duration_ms}ms — ok")

    sock.close()
    print("Trajectory complete.")


if __name__ == "__main__":
    main()
