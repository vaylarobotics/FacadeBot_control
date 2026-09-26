---
name: esp32-firmware-change
description: How to change, flash, and verify esp32_firmware/main.py on the ESP32, keeping the bridge and docs in step. Use for any edit on the ESP32 side.
---

# ESP32 firmware change

The firmware is `esp32_firmware/main.py`, MicroPython, flashed over USB with
`mpremote`. Wi-Fi is not available until the new code is running, so flashing is
always a USB step. The `hardware-change` checklist applies on top of this one.

## Before writing

1. Read `esp32_firmware/CLAUDE.md` for the invariants and the traps already hit.
2. If the change alters a command's fields or what its ack means, it is a protocol
   change. List every file that must move together: `main.py`,
   `esp32_bridge/transport.py`, `esp32_bridge_node.py`, `esp32_bridge/README.md`
   (wire protocol section), `LAUNCH.md`. Tell the user before starting.
3. If the change alters timing (`_TRAJECTORY_STEP_MS`, `_INTER_SERVO_DELAY_MS`,
   UART timeouts), state the measured value it was tuned against and what the new
   value assumes. These were set empirically on the arm.

## While writing

- No `print()`; the REPL is silent by design.
- No heap allocation after setup. Static buffers.
- ISR work is a flag only; the main loop acts on it.
- Pin, baud and protocol constants stay at the top with their source.
- Raw position counts only. Degrees, centres and limits belong to the bridge.

## Flashing

The board must be on USB. The bridge must be down (it would just drop anyway).
```bash
/home/harthik/.firmware/bin/mpremote connect /dev/ttyUSB0 fs cp esp32_firmware/main.py :main.py
/home/harthik/.firmware/bin/mpremote connect /dev/ttyUSB0 reset
```
The `reset` is required; `fs cp` alone leaves the old code running. If
`/dev/ttyUSB0` is missing, `ls /dev/ttyUSB* /dev/ttyACM*`. If permission is denied,
the user's shell predates their `dialout` membership; tell them, do not `sudo`.
`mpremote` is not on PATH and is the only flashing tool to suggest.

Claude cannot see the board. Give these commands to the user or run them only if
the user has said the board is on USB at the dev box.

## Verifying

Silence on the REPL does not confirm Wi-Fi joined. Verify over the network:
1. `ping 192.168.1.150`.
2. Bring up the bridge (`LAUNCH.md` steps 4–6, or 5a) and confirm `configure`
   succeeds; that proves the greeting and protocol version still parse.
3. `ros2 service call /facade_bot/read_joint_positions facade_msgs/srv/ReadJointPositions {}`
   and confirm four numbers, no `nan`.
4. The `[0,0,0,0]` home move, `LAUNCH.md` step 7a, with nobody in reach.
5. If the arm does not move: `scripts/check_servo_bus.py` over USB per
   `LAUNCH.md` step 2a, then reset the board.

Report which steps ran and which the user still needs to do. A change to the
`servo` streaming path additionally needs the continuous-sweep bench pass
described in `STATUS.md` before it counts as verified.

## After

- `LAUNCH.md` step 1 is the flashing reference; update it if the procedure changed.
- `STATUS.md`: note the firmware change and its verification state.
