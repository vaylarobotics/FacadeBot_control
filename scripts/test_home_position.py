"""
Send all FacadeBot arm servos to their home (center) position.

Usage:
    python3 test_home_position.py
    python3 test_home_position.py --port /dev/ttyACM0
"""
import argparse
import json
import sys
import time

import serial

DEFAULT_PORT          = "/dev/ttyUSB0"
DEFAULT_BAUD          = 115200
BOOT_WAIT_SEC         = 2    # time for ESP32 to finish booting after USB connection opens
RESPONSE_TIMEOUT_SEC  = 10   # how long to wait for ESP32 to confirm the move


def main():
    parser = argparse.ArgumentParser(description="Move FacadeBot arm to home position.")
    parser.add_argument("--port", default=DEFAULT_PORT,
                        help=f"Serial port for the ESP32 (default: {DEFAULT_PORT})")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD,
                        help=f"Baud rate (default: {DEFAULT_BAUD})")
    args = parser.parse_args()

    print(f"Connecting to ESP32 on {args.port} at {args.baud} baud...")
    try:
        port = serial.Serial(args.port, args.baud, timeout=1)
    except serial.SerialException as e:
        print(f"ERROR: Could not open port — {e}")
        print("Tip: run 'ls /dev/ttyUSB* /dev/ttyACM*' to find the correct port.")
        sys.exit(1)

    # Wait for ESP32 to boot, then discard any startup messages
    time.sleep(BOOT_WAIT_SEC)
    port.reset_input_buffer()

    print("Sending home command...")
    port.write(b'{"cmd": "home"}\n')

    deadline = time.monotonic() + RESPONSE_TIMEOUT_SEC
    while time.monotonic() < deadline:
        raw = port.readline()
        if not raw:
            continue
        try:
            response = json.loads(raw.strip())
        except (ValueError, UnicodeDecodeError):
            continue

        if response.get("status") == "ok":
            print("All servos moved to home position.")
            port.close()
            sys.exit(0)
        else:
            print(f"ERROR: Unexpected response from ESP32: {response}")
            port.close()
            sys.exit(1)

    print("ERROR: No response from ESP32 within timeout. Check wiring and firmware.")
    port.close()
    sys.exit(1)


if __name__ == "__main__":
    main()
