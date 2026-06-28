# FacadeBot Control — CLAUDE.md

## Project Overview

FacadeBot is a robotic arm designed for building facade work (exterior painting, cleaning, and inspection). This repo contains the full control stack: high-level motion planning and vision on the Raspberry Pi, motor driver firmware on the ESP32/Hiwonder board, and ROS2 middleware tying them together.

## Hardware Architecture

| Component | Role |
|-----------|------|
| Raspberry Pi | Main controller: runs ROS2 nodes, vision processing, high-level logic |
| Hiwonder board | Servo/motor driver board connected to the ESP32 |
| ESP32 | Low-level driver: interfaces with the Hiwonder board, exposes a control interface to the RPi |

Communication between the RPi and ESP32 is **under evaluation** — both Wi-Fi/TCP and Serial/UART are being tested. Code that depends on the transport should be written behind an abstraction so the underlying link can be swapped without touching higher-level logic.

## Project Status

> Keep this section current. Update it at the end of any session that changes hardware state, installs something, or completes a milestone. This is the single source of truth so the user doesn't have to re-explain what's done each session.

### Hardware
| Item | Status |
|------|--------|
| Physical wiring: RPi ↔ ESP32 (Serial/UART) | ✅ Done |
| Physical wiring: ESP32 ↔ Hiwonder board | ✅ Done |
| Servos physically connected to Hiwonder board | ✅ Done |
| Servos moving on command | ✅ Done |

### Software
| Item | Status |
|------|--------|
| ROS2 installed and working on RPi | ✅ Done |
| ESP32 firmware: MicroPython script flashed | ✅ Done |
| Transport chosen: Serial/UART (RPi ↔ ESP32) | ✅ Decided |
| ESP32 receiving and acting on commands | ✅ Done |
| Transport abstraction layer (send_command / read_response) | ⬜ Not started |
| `esp32_bridge` ROS2 node | ⬜ Not started |
| `facade_control` ROS2 node | ⬜ Not started |
| `facade_vision` pipeline | ⬜ Not started |
| `facade_msgs` custom message package | ⬜ Not started |
| Bounds-checking for joint commands | ⬜ Not started |
| Emergency-stop logic | ⬜ Not started |

### Current blocker
None — full chain is working end to end. Root cause of previous servo non-response: TX/RX wires between ESP32 UART2 (GPIO16/17) and BusLinker were not crossed over (TX was wired to TX, RX to RX). Fixed by swapping to TX→RX and RX→TX.

## Software Stack

- **Python** — ROS2 nodes, vision pipeline, high-level control logic (runs on RPi)
- **C/C++** — Performance-critical ROS2 nodes or RPi code where Python is too slow
- **ROS2** — Middleware for inter-process communication, node lifecycle, and tooling
- **MicroPython / C/C++** — ESP32 firmware (Hiwonder board driver, transport layer)

## Repo Structure (expected layout as the project grows)

```
FacadeBot_control/
├── ros2_ws/              # ROS2 workspace
│   └── src/
│       ├── facade_control/   # High-level motion & task planning (Python)
│       ├── facade_vision/    # Vision pipeline (Python / C++)
│       └── esp32_bridge/     # ROS2 ↔ ESP32 transport node
├── esp32_firmware/       # MicroPython or C++ firmware for the ESP32
└── scripts/              # Utility scripts (deployment, calibration, testing)
```

## Working Style

**Always ask clarifying questions before writing a new node, module, or piece of hardware-interfacing code.** The user understands the system at a hardware and architecture level but is not an experienced software developer. This means:

- Explain *what* the code will do and *why* it is structured that way before writing it — in plain terms, not jargon.
- When there is more than one reasonable approach, present the options and their tradeoffs briefly, then ask which direction to take.
- Prefer simple, readable implementations over clever or terse ones — the user needs to be able to read and reason about this code.
- When introducing a new concept (e.g. a ROS2 lifecycle state, a callback pattern, a serial framing scheme), explain it with one sentence in plain English the first time it appears.
- Never silently make architecture decisions (which node owns what, how messages are structured, what the topic graph looks like). Surface these and confirm before implementing.

## Coding Conventions

### General
- Write no comments unless the WHY is non-obvious (hidden hardware constraint, non-obvious timing requirement, workaround for a specific board bug). Describe *why*, never *what*.
- Explicit over clever — this code drives a physical arm. A future reader (including the user) must be able to follow the logic without deep Python/C++ knowledge.
- No speculative abstractions. Build exactly what the current task needs; do not design for hypothetical future features.
- All physical quantities must include their units in the variable name: `angle_deg`, `speed_rpm`, `distance_mm`, `timeout_sec`.
- Magic numbers are never allowed. Every hardware limit, pin number, baud rate, or threshold must be a named constant defined at the top of the file with a comment explaining where the value comes from (datasheet, calibration, empirical test).

### Python (ROS2 nodes)
- Python 3.10+. Use type hints on all function signatures.
- One file per ROS2 node. One responsibility per node.
- All node classes inherit from `rclpy.node.Node` (or the lifecycle equivalent).
- Use `self.get_logger().info/warn/error()` — never `print()`.
- Parameters must be declared with `self.declare_parameter()` in `__init__`, with a sensible default and a description string.
- Topic and service callbacks must be short. If the logic is more than ~10 lines, extract it into a private method.
- Imports: standard library first, then third-party, then ROS2, then local — one blank line between each group.

### C/C++ (firmware & performance nodes)
- C++17 for ROS2 nodes. C or MicroPython for ESP32 firmware.
- No heap allocation (no `new`/`malloc`) on the ESP32 after the setup phase — use static or stack-allocated buffers.
- Keep interrupt service routines (ISRs) under ~10 instructions. Set a flag and handle the work in the main loop or a task.
- All hardware register writes must cite the datasheet section or page number in a comment next to the constant definition.

### Communication abstraction (RPi ↔ ESP32)
- Never call `serial`, `socket`, or any transport API directly from business logic. All ESP32 communication must go through a transport class/module with a stable interface (e.g. `send_command(cmd)`, `read_response()`).
- When the transport is finalized, document the wire protocol (message format, framing, baud rate or IP port) in the relevant package README before merging.

### ROS2 specifics
- Use lifecycle nodes (`rclpy.lifecycle.Node`) for any node that directly controls hardware — this enables clean startup and shutdown sequencing.
- Topic names: `snake_case`, namespaced under `/facade_bot/` (e.g. `/facade_bot/joint_states`, `/facade_bot/arm_cmd`).
- Custom message and service definitions go in a dedicated `facade_msgs` package.
- QoS: use `ReliabilityPolicy.RELIABLE` for all hardware command topics; `BEST_EFFORT` is only acceptable for high-rate sensor streams where dropping a frame is safe.

### Safety rules (non-negotiable)
- Every joint command **must** pass through a bounds-check before being sent to the Hiwonder board. The bounds-check function must be the single place where limits are defined.
- Any function that moves a motor must be clearly named (e.g. `move_joint`, `send_motor_cmd`) — never disguise a motion command inside a generic utility function.
- Emergency-stop logic must be implemented before any code that moves the arm is considered complete.

## Development Notes

- **Target platform**: Raspberry Pi (aarch64). Test on hardware or a Pi-compatible environment — do not assume x86 behavior.
- **Vision pipeline**: Runs on the RPi alongside the control stack — be mindful of CPU budget. Prefer lightweight models or hardware-accelerated inference (Pi Camera + picamera2).
