# scripts — CLAUDE.md

Bench and design-time tools. None of these run on the Pi as part of the control
stack.

| Script | Runs where | Touches hardware? |
|--------|-----------|-------------------|
| `check_servo_bus.py` | On the ESP32 over USB via `mpremote run` | Read-only position probes. Cannot move the arm. Interrupts `main.py`, so the bridge must be down; reset the board after |
| `generate_geometry.py` | Dev box | No. Bakes a URDF into `config/geometry_<model>.yaml`. Rerun after any URDF re-export |
| `test_trajectory.py` | Dev box | **Yes, and it bypasses the bounds gate.** It opens its own socket and sends absolute servo angles. Flagged for deletion (structural review finding 4). Do not run it and do not copy its pattern. **Also now protocol-stale**: it relies on a `move` ack meaning "physically finished" and sends waypoints back to back with no delay. It was left unmodified in the D4 change pending the user's yes/no on deleting it |
| `setup_camera.sh` | Pi | No. Builds `~/camera_ws` |
| `fake_esp32_server.py` | Dev box | No arm involved at all — it's a stand-in TCP server, not a client. Speaks `esp32_firmware/main.py`'s wire protocol on localhost with simulated positions, so `esp32_bridge` and everything above it can run for real against it. **It is a second implementation of the protocol: any firmware command change lands here in the same edit.** Point `esp32_host` at `127.0.0.1` only for this; never the reverse |

## Rules for anything new here

- A script may read from the ESP32 over USB. It may not command motion over any
  link. Motion goes through ROS2 and `esp32_bridge` so the bounds gate applies.
- Anything that talks to the ESP32 uses `192.168.1.150`, not the older `.100`.
- `mpremote` is at `/home/harthik/.firmware/bin/mpremote`, not on PATH.
- Every new script gets a numbered step in `LAUNCH.md` in the same turn.
