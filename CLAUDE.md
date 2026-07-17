# FacadeBot Control — CLAUDE.md

## Project Overview

FacadeBot is a robotic arm designed for building facade work (exterior painting, cleaning, and inspection). This repo contains the full control stack: high-level motion planning and vision on the Raspberry Pi, motor driver firmware on the ESP32/Hiwonder board, and ROS2 middleware tying them together.

## Hardware Architecture

| Component | Role |
|-----------|------|
| Raspberry Pi | Main controller: runs ROS2 nodes, vision processing, high-level logic |
| Hiwonder board | Servo/motor driver board connected to the ESP32 |
| ESP32 | Low-level driver: interfaces with the Hiwonder board, exposes a control interface to the RPi |

Communication between the RPi and ESP32 uses **Wi-Fi/TCP** (`esp32_firmware/main.py` connects to Wi-Fi and listens on a TCP socket for JSON commands). Code that depends on the transport should still be written behind an abstraction so the underlying link can be swapped without touching higher-level logic.

## Project Status

> Keep this section current. Update it at the end of any session that changes hardware state, installs something, or completes a milestone. This is the single source of truth so the user doesn't have to re-explain what's done each session.

### Hardware
| Item | Status |
|------|--------|
| RPi ↔ ESP32 link (Wi-Fi, no physical wiring) | ✅ Done |
| Physical wiring: ESP32 ↔ Hiwonder board | ✅ Done |
| Servos physically connected to Hiwonder board | ✅ Done |
| Servos moving on command | ✅ Done |

### Software
| Item | Status |
|------|--------|
| ROS2 installed and working on RPi | ✅ Done | 
| ESP32 firmware: MicroPython script flashed | ✅ Done |
| Transport chosen: Wi-Fi/TCP (RPi ↔ ESP32) | ✅ Decided |
| ESP32 receiving and acting on commands | ✅ Done |
| Servo position readback (CMD_POS_READ) | ✅ Done |
| Stall detection (compare commanded vs. read-back position) | ✅ Done — log-only fault check after each move (`esp32_bridge` node); e-stop/recovery deferred |
| Minimum-jerk trajectory shaping for moves | ✅ Done — ESP32-side (`move_joints_min_jerk` in `esp32_firmware/main.py`); `move` command shape unchanged, ack now signals physical completion |
| Transport abstraction layer (send_command / read_response) | ✅ Done (`ros2_ws/src/esp32_bridge/esp32_bridge/transport.py`) |
| `esp32_bridge` ROS2 node | ✅ Done — subscribes to `/facade_bot/joint_cmd`, converts commanded degrees to raw servo positions and forwards `move` commands over Wi-Fi, reads positions back after each move and logs a fault if a joint misses target; serves `/facade_bot/read_joint_positions` for on-request position checks; per-joint bounds-check now in place (see below), no e-stop yet |
| `facade_control` ROS2 node | ✅ Done — first capability is Cartesian move commands: `/facade_bot/move_to_pose` service (`facade_msgs/srv/MoveToPose`) takes a tool-tip `(x_m, y_m, z_m, tool_angle_deg)` target, reads the arm's current joint positions from `esp32_bridge` first, then solves the target with the inverse kinematics in `facade_control/kinematics.py` — picking whichever valid candidate (azimuth branch × elbow-up/down) needs the least total joint travel from that current position, so the elbow no longer flips between up/down configurations across nearby targets — and publishes the result to `/facade_bot/joint_cmd`. Rejects (no publish, error message) anything geometrically unreachable, that would need a joint outside its safe range, or where the current-position read itself fails (`esp32_bridge` unreachable/timeout/invalid reading). Also serves `/facade_bot/read_tool_pose` (`facade_msgs/srv/ReadToolPose`): reads the arm's actual joint angles from `esp32_bridge` and runs them through forward kinematics, for checking real-world accuracy against a physically measured tool-tip position (avoid full-extension/retraction poses — those are singularities). Caveats: the 50 mm wrist→tool-tip offset is a measured constant (straight-line along reach direction, not a full 3D offset), never sends anything to the ESP32 directly, `move_to_pose` now needs `esp32_bridge` active just to solve (not only to move) — see `ros2_ws/src/facade_control/README.md` |
| `trajectory_node` (second node in `facade_control`) | ✅ Done — `/facade_bot/follow_trajectory` action (`facade_msgs/action/FollowTrajectory`) moves through an ordered list of Cartesian waypoints (`facade_msgs/msg/Waypoint`), one at a time: calls `facade_control_node`'s own `move_to_pose` per waypoint (reusing its IK/least-effort logic as-is), then confirms physical arrival by polling `esp32_bridge`'s `read_joint_positions` within a tolerance before advancing (there's no "move complete" signal anywhere else on the ROS2 graph, so this had to be built). Reports feedback (`current_waypoint_index`/`total_waypoints`) and supports cancellation (stops commanding further waypoints only — no e-stop primitive exists to abort an in-flight move). V1 stops fully at each waypoint; blending is tracked future work requiring new ESP32 firmware/protocol support, not just a ROS2-side change — see `ros2_ws/src/facade_control/README.md`'s "Following a trajectory" section. Not yet verified on the real arm (built/tested against stub services only so far). |
| `facade_vision` pipeline | ⬜ Not started |
| `facade_msgs` custom message package | ✅ Done — `ReadJointPositions.srv`, `ReadToolPose.srv`, and `MoveToPose.srv` (now also returns `solved_angles_deg`) (used by `esp32_bridge`/`facade_control`); `msg/Waypoint.msg` and `action/FollowTrajectory.action` (used by `trajectory_node`) |
| Joint homing / zero calibration | ✅ Done — `esp32_bridge_node.py`'s `_JOINT_CENTER_RAD` now holds each joint's separately measured true center (radians, read directly off the arm); commanded `0°` means that center, not raw position 0. Replaces the old shared, uncalibrated 0–240° mapping |
| Bounds-checking for joint commands | ✅ Done — `esp32_bridge_node.py`'s `_check_joint_bounds`, checked against per-joint limits now relative to each joint's measured center (±110° for joints 1–3, ±100° for joint_4/wrist) instead of the old absolute range converted from `URDF_Test.urdf`; out-of-range commands are rejected outright (logged, nothing sent to the ESP32). This is the single, mandatory, last-resort gate — it applies regardless of whether a command came from `facade_control`'s IK or straight from `ros2 topic pub` |
| Emergency-stop logic | ⬜ Not started |

### Current blocker
IK accuracy: ✅ resolved — user confirmed IK "works now" (2026-07-14) after the joint-homing recalibration and a `kinematics.py` normalization bug fix (the `%360` candidate-angle wrap-around broke bounds-checking once joint limits became center-relative).

Elbow configuration continuity: ✅ resolved (2026-07-15) — `inverse_kinematics` now collects every valid candidate instead of returning the first, and `facade_control_node` reads the arm's actual joint positions from `esp32_bridge` before each `move_to_pose` call and picks the candidate with the least total joint travel from there (see `facade_control/kinematics.py`'s `current_angles_deg` param and `facade_control_node.py`'s `_read_current_joint_positions`). This adds a UART round-trip to every move and a new failure mode (move rejected if the position read fails) — not yet verified on hardware for added latency.

Trajectory following (V1, stop-at-each-waypoint): ✅ built (2026-07-15) — see `trajectory_node` row above. Not yet run against the real arm.

Next task: verify `trajectory_node` on hardware (real multi-waypoint run, confirm poll/timeout constants are generous enough against real Wi-Fi/servo jitter, test cancellation mid-run). After that, the user's stated next step is a raster/pattern generator for sweeping a rectangular area, followed eventually by continuous blending between waypoints (needs new ESP32 firmware/protocol support — out of scope for the ROS2-side work done so far).

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
│       ├── facade_msgs/      # Custom ROS2 message/service definitions
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
