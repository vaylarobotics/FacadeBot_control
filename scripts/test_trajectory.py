"""
Stream a joint trajectory to the FacadeBot ESP32 over Wi-Fi/TCP.

Each waypoint is sent as a move command and acknowledged before the next
waypoint is sent. The script sleeps for duration_ms after each ack so the
servos have time to reach the target before the next command arrives.

Usage:
    python3 test_trajectory.py
    python3 test_trajectory.py --ip 192.168.1.100 --port 5000
"""
import argparse
import json
import socket
import sys
import time

ESP32_IP             = "192.168.1.100"  # must match STATIC_IP in esp32_firmware/main.py
TCP_PORT             = 5000             # must match TCP_PORT in esp32_firmware/main.py
RESPONSE_TIMEOUT_SEC = 10

# Demonstration trajectory: slow sweep from home (500) toward a target pose and back.
# Each row is [servo1, servo2, servo3, servo4, duration_ms].
# Adjust these values to match your arm's safe range before running on hardware.
DEMO_TRAJECTORY = [
    [500, 500, 500, 500, 1000],  # home
    [520, 510, 490, 510,  400],
    [540, 520, 480, 520,  400],
    [560, 530, 470, 530,  400],
    [580, 540, 460, 540,  400],
    [600, 550, 450, 550,  400],
    [580, 540, 460, 540,  400],
    [560, 530, 470, 530,  400],
    [540, 520, 480, 520,  400],
    [520, 510, 490, 510,  400],
    [500, 500, 500, 500, 1000],  # back to home
]


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
        positions   = point[:4]
        duration_ms = point[4]

        response = send_command(f, sock, {
            "cmd":         "move",
            "positions":   positions,
            "duration_ms": duration_ms,
        })

        if response.get("status") != "ok":
            print(f"ERROR at waypoint {i}: {response}")
            sock.close()
            sys.exit(1)

        print(f"  waypoint {i+1}/{len(DEMO_TRAJECTORY)}: {positions} in {duration_ms}ms — ok")
        time.sleep(duration_ms / 1000.0)

    sock.close()
    print("Trajectory complete.")


if __name__ == "__main__":
    main()
