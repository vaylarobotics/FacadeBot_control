# FacadeBot Control — CLAUDE.md

## Project overview

FacadeBot is a robotic arm for building facade work (exterior painting, cleaning,
inspection). This repo is the full control stack: motion planning and vision on a
Raspberry Pi, servo-driver firmware on an ESP32 talking to a Hiwonder BusLinker,
and ROS2 tying them together.

| Component | Role |
|-----------|------|
| Raspberry Pi (Ubuntu Server, aarch64) | Runs ROS2 nodes, vision, high-level logic |
| ESP32 (MicroPython) | Low-level driver: TCP server on Wi-Fi, LX-16A servo bus on UART2 |
| Hiwonder BusLinker + 4× LX-16A servos | The arm's joints, base to tool tip |

RPi ↔ ESP32 is **Wi-Fi/TCP**, newline-delimited JSON. ESP32 is static at
`192.168.1.150:5000`. The Pi is `harthik@192.168.1.17` (passwordless SSH from
the dev box; `sudo` there needs a password, so hand root-level steps to the user).

## Where things live

| Path | Owns | Details |
|------|------|---------|
| `esp32_firmware/` | The only code that touches servos | `esp32_firmware/CLAUDE.md` |
| `ros2_ws/src/esp32_bridge/` | ROS2 ↔ ESP32 transport, the mandatory joint bounds gate, joint centre calibration | `ros2_ws/src/esp32_bridge/CLAUDE.md` |
| `ros2_ws/src/facade_control/` | Kinematics, Cartesian moves, trajectory following, `/joint_states`, launch files | `ros2_ws/src/facade_control/CLAUDE.md` |
| `ros2_ws/src/facade_msgs/` | Every custom message, service and action | `ros2_ws/src/facade_msgs/CLAUDE.md` |
| `ros2_ws/src/facadebot_description/` | URDFs, generated geometry, measured limits, model loader | `ros2_ws/src/facadebot_description/CLAUDE.md` |
| `ros2_ws/src/facade_vision/` | Not started. Camera driver lives in `~/camera_ws` on the Pi | — |
| `scripts/` | Bench and design-time tools (servo bus probe, geometry generator) | `scripts/CLAUDE.md` |
| `STATUS.md` | What is done, verified, blocked. Imported below, so it loads every session | — |
| `DECISIONS.md` | Open robotics/architecture questions with options; an answered one is settled | — |
| `TESTS.md` | Plain-language catalogue of every test and of what is untested; the user reviews tests here, not in Python | — |
| `LAUNCH.md` | Every run instruction, powered-off to every capability | — |

Each package `CLAUDE.md` loads automatically when you work on files under it. It
holds that package's invariants, traps already hit, and which tests to run.

## Start of a task

1. The project status is already in context (imported at the end of this file).
   Before touching an area with an open question, check `DECISIONS.md` for it.
2. If the task is one of the recurring workflows below, invoke the skill.
3. Before a change to the bridge, firmware, limits or kinematics is tested on the
   arm, have the `safety-reviewer` agent review the diff.

## Working style

The user understands the system at a hardware and architecture level and makes the
robotics decisions. Claude carries the heavy implementation. That means:

- **Ask before writing a new node, module, or hardware-interfacing code.** Explain
  what it will do and why it is structured that way, in plain terms, first.
- When more than one approach is reasonable, present the options and tradeoffs
  briefly, then ask. Never silently make an architecture decision (node ownership,
  message shape, topic graph, safety behaviour).
- Robotics choices (tolerances, e-stop semantics, what "arrived" means, which IK
  solution to prefer, task-frame definitions) are the user's call. Lay out the
  options with consequences; do not pick one alone.
- Introduce a new concept (lifecycle state, callback group, framing scheme) with one
  plain-English sentence the first time it appears.
- Prefer simple, readable code over clever code. The user must be able to read it.
- Report outcomes plainly: what was verified in tests, what was verified on the arm,
  what was not verified at all. Never describe unverified code as working.

## Coding conventions

### General
- Comments only where the WHY is non-obvious (hardware constraint, timing, board
  bug workaround). Describe why, never what.
- Explicit over clever. No speculative abstractions. Build what the current task
  needs.
- Physical quantities carry units in the name: `angle_deg`, `speed_rpm`,
  `distance_mm`, `timeout_sec`.
- No magic numbers. Every limit, pin, baud rate or threshold is a named constant at
  the top of the file with a comment saying where the value comes from (datasheet,
  calibration, empirical test).
- When a constant must match another file (firmware ↔ bridge, node ↔ node), say so
  in the comment and update both sides in the same change.

### Python (ROS2 nodes)
- Python 3.10+, type hints on every function signature.
- One file per node, one responsibility per node. Node classes inherit from
  `rclpy.node.Node` or the lifecycle equivalent.
- `self.get_logger().info/warn/error()`, never `print()`. For a failure before the
  node exists, use `rclpy.logging.get_logger(...)`.
- Parameters declared in `__init__` with `self.declare_parameter()`, a sensible
  default and a description.
- Callbacks stay short. More than ~10 lines goes in a private method.
- Imports: standard library, third-party, ROS2, local. One blank line between groups.
- Match the surrounding style (double quotes, this import order), not flake8. The
  ament flake8/pep257 tests fail at HEAD by design; do not "fix" them and do not
  report them as a regression. Do not add new long lines or docstring errors.

### C/C++ and MicroPython (firmware)
- C++17 for ROS2 nodes. MicroPython (or C) on the ESP32.
- No heap allocation on the ESP32 after setup. Static or stack buffers.
- ISRs under ~10 instructions: set a flag, do the work in the main loop.
- Every hardware register write cites the datasheet section next to the constant.

### Communication (RPi ↔ ESP32)
- Business logic never calls `socket` or `serial`. All ESP32 traffic goes through
  `esp32_bridge/transport.py` (`send_command`, `read_response`).
- The wire protocol is documented in `ros2_ws/src/esp32_bridge/README.md`. A
  protocol change updates the firmware, the transport, and that README together.

### ROS2
- Lifecycle nodes for anything that directly controls hardware.
- Topic names `snake_case` under `/facade_bot/`. The one exception is
  `/joint_states`, which `robot_state_publisher` expects by that exact name.
- Custom messages live in `facade_msgs` only.
- QoS `RELIABLE` for every hardware command topic. `BEST_EFFORT` only for high-rate
  sensor streams where a dropped frame is safe.
- A node that needs the arm's current state queries the authoritative source
  (`esp32_bridge`'s read service) every time. Never cache "last commanded" in a
  node as a proxy for physical state: nodes restart, the arm does not.

## Safety rules (non-negotiable)

- Every joint command passes through `esp32_bridge_node.py`'s `_check_joint_bounds`
  before it reaches the servo bus. Limits are defined once, in
  `facadebot_description/config/robot_model.yaml`; nothing else may hold a copy.
- Any function that moves a motor is named for it (`move_joints`,
  `send_motor_cmd`). Never hide motion inside a generic utility.
- No script or tool may open its own connection to the ESP32 to command motion.
  Motion goes through ROS2 and the bridge. Read-only USB probes are fine.
- There is no e-stop of any kind (the GPIO 27 bench switch was removed 2026-09-26);
  the servo power plug is the only stop. Anything that starts the bridge makes the
  arm live. Say so in run instructions and
  never run a launch file with someone in the arm's reach.

## Git

**Never run `git commit` or `git push`, and do not offer to.** The user gates
`main` on hardware verification and commits themselves. Leave all work in the
working tree; a large uncommitted diff is normal here. Reading git state is fine.
A hook in `.claude/settings.json` enforces this.

## Documentation duties

- Every new node, topic, service, action, script or CLI command gets a numbered
  step in `LAUNCH.md`, in the same turn, matching its style. Do not wait to be asked.
- Every session that changes hardware state, installs something, or completes a
  milestone updates `STATUS.md` (the `end-session` skill covers this).
- A package README documents interfaces and protocol; a package `CLAUDE.md`
  documents invariants and traps. Keep them in sync when either changes.
- Every test added, removed, or changed in meaning gets its row in `TESTS.md`
  updated in the same turn. `TESTS.md` is the reviewable statement of what the
  tests check; the Python follows it.

## Skills and agent

| Name | Use when |
|------|----------|
| `hardware-change` | Any edit to `esp32_bridge`, `robot_model.yaml`, joint centres, or kinematics limits |
| `esp32-firmware-change` | Any edit to `esp32_firmware/main.py`, including flashing and verifying |
| `new-node` | Adding a ROS2 node, service, action or topic |
| `end-session` | Closing out: STATUS.md, LAUNCH.md, what was and was not verified |
| `sync-tests` | The user has edited the Status column in `TESTS.md`; make the Python match, run suites, reset the column |
| `safety-reviewer` (agent) | Reviewing a diff that touches the hardware path before it is tested on the arm. Read-only, local tree only |

## Environment facts

- Dev box has ROS2 Jazzy and a built `ros2_ws`; unit tests run locally without the arm.
- `mpremote` is at `/home/harthik/.firmware/bin/mpremote` (not on PATH). It is the
  chosen flashing tool; do not suggest others.
- The Pi is Ubuntu Server, not Raspberry Pi OS. `harthikpi.local` does not resolve;
  use the IP.
- Camera stack lives in a separate `~/camera_ws` on the Pi; its third-party source
  is not vendored here (`scripts/setup_camera.sh`, `camera_ws.repos`).

## Project status (imported)

@STATUS.md
