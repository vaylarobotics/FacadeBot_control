"""
Send all FacadeBot arm servos to a named position over Wi-Fi/TCP.

Usage:
    python3 test_home_position.py
    python3 test_home_position.py --ip 192.168.1.100 --pose position_b
"""
import argparse
import json
import socket
import sys

ESP32_IP             = "192.168.1.100"  # must match STATIC_IP in esp32_firmware/main.py
TCP_PORT             = 5000             # must match TCP_PORT in esp32_firmware/main.py
RESPONSE_TIMEOUT_SEC = 10

def main() -> None:
    parser = argparse.ArgumentParser(description="Move FacadeBot arm to a named position.")
    parser.add_argument("--ip",   default=ESP32_IP,
                        help=f"ESP32 IP address (default: {ESP32_IP})")
    parser.add_argument("--port", type=int, default=TCP_PORT,
                        help=f"ESP32 TCP port (default: {TCP_PORT})")
    parser.add_argument("--pose", default="home", choices=["home", "position_b"],
                        help="Named position to move to (default: home)")
    args = parser.parse_args()

    print(f"Connecting to ESP32 at {args.ip}:{args.port}...")
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(RESPONSE_TIMEOUT_SEC)
        sock.connect((args.ip, args.port))
    except OSError as e:
        print(f"ERROR: Could not connect — {e}")
        print("Tip: check that the ESP32 is powered, connected to Wi-Fi, and the IP is correct.")
        sys.exit(1)

    f = sock.makefile('rb')

    # Read the ready message the ESP32 sends on connect
    ready_raw = f.readline()
    try:
        ready = json.loads(ready_raw.strip())
        if ready.get("status") != "ready":
            print(f"WARNING: unexpected greeting from ESP32: {ready}")
    except (ValueError, UnicodeDecodeError):
        print(f"WARNING: could not parse ESP32 greeting: {ready_raw}")

    print(f"Sending {args.pose} command...")
    sock.sendall(f'{{"cmd": "{args.pose}"}}\n'.encode())

    response_raw = f.readline()
    sock.close()

    try:
        response = json.loads(response_raw.strip())
    except (ValueError, UnicodeDecodeError):
        print(f"ERROR: could not parse response: {response_raw}")
        sys.exit(1)

    if response.get("status") == "ok":
        print(f"All servos moved to {args.pose}.")
        sys.exit(0)
    else:
        print(f"ERROR: ESP32 returned: {response}")
        sys.exit(1)


if __name__ == "__main__":
    main()
