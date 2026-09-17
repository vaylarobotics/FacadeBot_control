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
| Non-blocking `servo` streaming command (firmware) | ✅ Done — `esp32_firmware/main.py` `handle_client` gains a `servo` command that moves straight toward a setpoint via the existing plain `move_joints` and acks immediately (no min-jerk, no readback). This is the primitive continuous motion streams onto (see `continuous_trajectory_node` row) |
| Transport abstraction layer (send_command / read_response) | ✅ Done (`ros2_ws/src/esp32_bridge/esp32_bridge/transport.py`) |
| `esp32_bridge` ROS2 node | ✅ Done — subscribes to `/facade_bot/joint_cmd`, converts commanded degrees to raw servo positions and forwards `move` commands over Wi-Fi, reads positions back after each move and logs a fault if a joint misses target; serves `/facade_bot/read_joint_positions` for on-request position checks; per-joint bounds-check now in place (see below), no e-stop yet. Also serves `/facade_bot/read_joint_positions_fast` (same `ReadJointPositions` message, single attempt/no retry, ~170ms measured vs. the full-retry service's occasional 600ms+) for a caller that polls continuously and can tolerate an occasional missing joint (e.g. `joint_state_publisher_node`, below) — the safety-critical move-fault check keeps using the full-retry version unchanged |
| `facade_control` ROS2 node | ✅ Done, verified on real hardware (2026-09-12) — first capability is Cartesian move commands: `/facade_bot/move_to_pose` service (`facade_msgs/srv/MoveToPose`) takes a tool-tip `(x_m, y_m, z_m, tool_angle_deg)` target, reads the arm's current joint positions from `esp32_bridge` first, then solves the target with the inverse kinematics in `facade_control/kinematics.py` — picking whichever valid candidate (azimuth branch × elbow-up/down) needs the least total joint travel from that current position, so the elbow no longer flips between up/down configurations across nearby targets — and publishes the result to `/facade_bot/joint_cmd`. Rejects (no publish, error message) anything geometrically unreachable, that would need a joint outside its safe range, or where the current-position read itself fails (`esp32_bridge` unreachable/timeout/invalid reading). Also serves `/facade_bot/read_tool_pose` (`facade_msgs/srv/ReadToolPose`): reads the arm's actual joint angles from `esp32_bridge` and runs them through forward kinematics, for checking real-world accuracy against a physically measured tool-tip position (avoid full-extension/retraction poses — those are singularities). Caveats: the 50 mm wrist→tool-tip offset is a measured constant (straight-line along reach direction, not a full 3D offset), never sends anything to the ESP32 directly, `move_to_pose` now needs `esp32_bridge` active just to solve (not only to move) — see `ros2_ws/src/facade_control/README.md` |
| `trajectory_node` (second node in `facade_control`) | ✅ Done, **verified on real hardware** (2026-09-12) — `/facade_bot/follow_trajectory` action (`facade_msgs/action/FollowTrajectory`) moves through an ordered list of Cartesian waypoints (`facade_msgs/msg/Waypoint`), one at a time: calls `facade_control_node`'s own `move_to_pose` per waypoint (reusing its IK/least-effort logic as-is), then confirms physical arrival by polling `esp32_bridge`'s `read_joint_positions` within a tolerance before advancing (there's no "move complete" signal anywhere else on the ROS2 graph, so this had to be built). Reports feedback (`current_waypoint_index`/`total_waypoints`) and supports cancellation (stops commanding further waypoints only — no e-stop primitive exists to abort an in-flight move). This node is for precise point-to-point positioning (stops fully at each waypoint); continuous constant-speed motion is now a separate node (`continuous_trajectory_node`, below). Ran a real 30 mm straight-line sweep and a real 100×50 mm rectangular perimeter (13 waypoints) on the physical arm, both closing to within ~2 mm — see Previously resolved. |
| Continuous constant-velocity trajectory (`continuous_trajectory_node`) | ✅ Done — `/facade_bot/follow_trajectory_continuous` action (`facade_msgs/action/FollowTrajectoryContinuous`, adds `tool_speed_mmps` + `corner_blend_m` to the waypoint list) sweeps the tool tip through the waypoints at a constant Cartesian speed, rounding corners rather than stopping. Pure planning math lives in `facade_control/trajectory_planning.py` (corner-rounded Bézier path → constant-speed sampling with start/end ramps → chained IK); the whole path is validated for reachability **before** any motion, then joint setpoints are streamed to `/facade_bot/joint_stream`, which `esp32_bridge` forwards as the non-blocking `servo` command. Reuses `kinematics.py` and `trajectory_node`'s action-server/executor/helper-node patterns. Cancellation caveat same as `trajectory_node`. Unit-tested (planning + node against stubs); **not yet verified on real hardware** (Wi-Fi jitter, servo-bus rate cap ~12 Hz, corner accel) — see `ros2_ws/src/facade_control/README.md`'s "Following a trajectory continuously" section |
| Single-command bringup (`bringup.launch.py`) | ✅ Done (2026-09-12) — `ros2 launch facade_control bringup.launch.py` starts `esp32_bridge_node`, `facade_control_node`, `trajectory_node`, and `joint_state_publisher_node` together, and drives the bridge's lifecycle straight through `configure` → `activate` automatically (the standard ROS2 event-handler pattern for this), so the arm is live the moment the launch returns — no manual `ros2 lifecycle set` needed. `continuous_trajectory_node` is deliberately left out (run by hand — it's the one node not yet verified on real hardware). Lives in `ros2_ws/src/facade_control/launch/`; see `LAUNCH.md` step 5a. One trap hit and fixed while building this: giving a launch `Node` action an explicit `name=` injects a process-wide node-name remap, which also silently renamed the internal helper client node that `facade_control_node`/`trajectory_node`/`joint_state_publisher_node` each create for their own outgoing service calls — visible as a "Publisher already registered" warning and a duplicate entry in `ros2 node list`. Fixed by leaving `name=` unset for those three (they already name themselves in code); `esp32_bridge_node` has no such helper, so it keeps its explicit name (needed for the lifecycle event matching). |
| `/joint_states` publisher (`joint_state_publisher_node`, third node in `facade_control`) | ✅ Done (2026-09-12) — polls `esp32_bridge`'s `/facade_bot/read_joint_positions_fast` on a timer (`poll_rate_hz` param, default 5 Hz) and publishes `sensor_msgs/JointState` on `/joint_states` — deliberately the one topic in the project that does *not* get the `/facade_bot/...` namespace, since `robot_state_publisher` subscribes to this exact name by default. Holds each joint's last known angle across a miss rather than blocking or leaving a gap (see the fast-read note on the `esp32_bridge` row above); refuses to publish at all until every joint has had at least one real reading. Joint names come from the loaded model config, not hardcoded. Ships a placeholder `urdf_zero_offset_deg` parameter (all 0.0, startup warning) for the still-unverified gap between commanded-zero and the URDF's own zero pose — now checkable with `display.launch.py` below since `_JOINT_CENTER_RAD` is current (see Previously resolved), but not yet actually checked. |
| Camera Module 3 driver (`camera_ros` + Raspberry Pi libcamera fork) | ✅ Done — built from source in a **separate** workspace `~/camera_ws` on the RPi (not `ros2_ws`). Publishes image topics for a future `facade_vision` node. libcamera is the Raspberry Pi fork (imx708 / Module 3 support), built in-tree, not the apt package. Reproducible via `scripts/setup_camera.sh` + pins in `camera_ws.repos` (libcamera @ `26bfadc6`, camera_ros @ `03c9e03`); run instructions in `LAUNCH.md` step 13. Third-party source itself is NOT vendored into this repo |
| `/robot_description` + `/tf` (robot_state_publisher) | ✅ Done (2026-09-17) — `ros2_ws/src/facade_control/launch/display.launch.py` starts `robot_state_publisher` (reads the URDF XML named by `robot_model.yaml`'s `active_model` at launch time — the one place in the project a raw URDF is opened at runtime, deliberately kept out of every control node) and `rviz2` with a minimal saved view (`ros2_ws/src/facade_control/rviz/facadebot.rviz`, meant to be extended). Deliberately its own launch file, not part of `bringup.launch.py` (user's call, 2026-09-17): visualization only, assumes a desktop session, so it shouldn't start for headless bench work. Not yet run against an actual display (no GUI available in dev environment) — verify it renders before trusting it. The rendered pose won't match the physical arm's zero yet — see `urdf_zero_offset_deg` above. |
| `facade_vision` pipeline | ⬜ Not started — camera driver is up (see row above); the FacadeBot-side node that consumes its image topics isn't written yet, and will live in `ros2_ws/src/facade_vision/` |
| Kinematics for the rebuilt arm (v2) | ✅ Done, unverified on hardware — URDF_V2 changed the structure, not just the numbers: joint_1's axis flipped to `0 0 -1`, joint_2's origin rpy lost its roll (working plane rotated 90°), links changed (shoulder→elbow 125.11→**135.11 mm**, elbow→wrist 165.11→**140.11 mm**), and critically **joint_4's rpy became `-1.5708 0 0`, moving its axis out of the joint_2/joint_3 plane** — so the old planar-3R solver was invalid, not just mis-parameterised. `kinematics.py` was rewritten: no hardcoded geometry (everything from the loaded `RobotModel`), `configure()` validates the model actually has this arm's shape and rejects it otherwise, and the closed form now fixes joint_4 first (which collapses wrist+tool into one rigid vector off the elbow), solves joint_1 from the out-of-plane distance, then the shoulder/elbow as a planar 2R. **`tool_angle_deg` changed meaning**: it is now joint_4's own angle — the tool's tilt out of the arm plane — not the tool's pitch. Pass 0 to keep the tool in-plane. Verified: 4000 random poses per model round-trip to 3.5e-13 mm with the seed configuration recovered every time, 0 elbow flips across nearby targets, and v1 still solves (the solver degenerates to the old in-plane case). Fixed a ±180° wrap bug that was failing the bounds-check on physically reachable poses. Sub-mm CAD asymmetries in URDF_V2 (0.14–0.89 mm) are snapped to zero — below one LX-16A count (~1.25 mm at full reach) |
| `facade_msgs` custom message package | ✅ Done — `ReadJointPositions.srv`, `ReadToolPose.srv`, and `MoveToPose.srv` (now also returns `solved_angles_deg`) (used by `esp32_bridge`/`facade_control`); `msg/Waypoint.msg`, `action/FollowTrajectory.action` (used by `trajectory_node`), and `action/FollowTrajectoryContinuous.action` (used by `continuous_trajectory_node`) |
| Arm model config (`facadebot_description`) | ✅ Done — the arm is now selected by one line (`active_model`) in `ros2_ws/src/facadebot_description/config/robot_model.yaml`. Both URDFs live side by side (`urdf/URDF_Test.urdf` = v1 retired, `urdf/URDF_V2.urdf` = v2 current) with meshes split into `meshes/v1/` and `meshes/v2/`; the broken `package://URDF_Test/` mesh paths are fixed in both. Geometry (link lengths, joint origins, axes) is **baked out of the selected URDF once** by `scripts/generate_geometry.py` into a generated `config/geometry_<model>.yaml` — never transcribed by hand, and never re-parsed at runtime: **no node opens a URDF or an XML parser at startup.** `facadebot_description/urdf_geometry.py` holds the design-time parser (generator-only); `robot_model.py` is pure YAML. Only hardware-measured numbers live in `robot_model.yaml`: per-joint limits and the joint_4→tool-tip offset. The package was converted from `ament_cmake` to `ament_python` so `facade_control` and `esp32_bridge` can both import the loader without the hardware gate depending on the planner. A model's `limits_source` records whether its hand-measured numbers were actually measured on that arm; anything but `measured` makes `describe()` print a startup warning banner (a warning, not a refusal — bench bring-up has to move the arm before the arm can be measured). Loader is otherwise **fail-closed**: unmeasured limits/offset, an unknown `active_model`, a missing `geometry_file`, a geometry file naming a different model, a joint chain not ordered base-to-tip, a joint-name mismatch against the limits, or inverted limits all abort node startup. Verified: geometry loaded from YAML is float-for-float identical to the URDF parse for both models, all three nodes refuse to start and print why, 33 tests pass. See `LAUNCH.md` step 3a |
| Joint homing / zero calibration | ✅ Re-checked for v2 (2026-09-16) — `esp32_bridge_node.py`'s `_JOINT_CENTER_RAD` holds each joint's separately measured true center (radians), so commanded `0°` means that center rather than raw position 0. Re-measured on the rebuilt v2 arm and found **unchanged from v1** — values in code are current. Kept in code, not `robot_model.yaml`, because they are per-servo calibration rather than a property of the model |
| Bounds-checking for joint commands | ✅ Done — `esp32_bridge_node.py`'s `_check_joint_bounds` is still the single, mandatory, last-resort gate (it applies whether a command came from `facade_control`'s IK or straight from `ros2 topic pub`), but the limits it enforces now come from `robot_model.yaml` via `_load_joint_limits()` instead of a hardcoded tuple, so this gate and `kinematics.py` can no longer drift apart. `_check_joint_bounds` raises rather than passing anything through if limits were never loaded. v2's limits are now measured (`limits_source: measured`, 2026-09-17) — joints 1–3 unchanged from v1, joint_4 widened to ±110° |
| Servo bus diagnostic (`scripts/check_servo_bus.py`) | ✅ Done — read-only probe run **on the ESP32 over USB** via `mpremote` (never Wi-Fi: a `move` ack only proves the command reached the ESP32). Sweeps servo IDs 1-8, dumps the raw bytes each returned, and classifies the fault as dead-bus / echo-only / garbage / wrong-ID, with a likelihood-ordered checklist. `RUN_LOOPBACK_TEST = True` (BusLinker unplugged, GPIO17 jumpered to GPIO16) clears or convicts the ESP32's own UART2. See `LAUNCH.md` step 2a |
| Emergency-stop logic | ⬜ Not started |

### Current blocker
**The three hand measurements (joint centers, joint limits, tool-tip offset) are all done and in
place (see Previously resolved) — what's left is verification and one solver question:**
1. IK has not been re-verified against the physical arm with the final numbers — `read_tool_pose`
   vs. a ruler-measured tool-tip position, at a couple of non-singular poses (avoid full
   extension/retraction).
2. `kinematics.py`'s least-effort candidate selection is flagged (user, 2026-09-17) as **not
   actually minimizing joint velocity** — it currently picks the valid IK candidate with the least
   *total joint travel in degrees* (see `current_angles_deg` in `inverse_kinematics`), which is not
   the same thing as minimizing time/velocity across joints with different speeds or when only one
   joint needs to move far. Needs its own look — not yet diagnosed further, no fix attempted.
3. `joint_state_publisher_node`'s `urdf_zero_offset_deg` is still an unverified placeholder (all
   0.0) — now checkable with `display.launch.py` (see the `/robot_description` + `/tf` row) since
   joint centers are confirmed current, but the actual visual check (comparing the rendered pose to
   the physical arm) hasn't been done yet.

### Previously resolved
IK accuracy: ✅ resolved — user confirmed IK "works now" (2026-07-14) after the joint-homing recalibration and a `kinematics.py` normalization bug fix (the `%360` candidate-angle wrap-around broke bounds-checking once joint limits became center-relative).

Elbow configuration continuity: ✅ resolved (2026-07-15) — `inverse_kinematics` now collects every valid candidate instead of returning the first, and `facade_control_node` reads the arm's actual joint positions from `esp32_bridge` before each `move_to_pose` call and picks the candidate with the least total joint travel from there (see `facade_control/kinematics.py`'s `current_angles_deg` param and `facade_control_node.py`'s `_read_current_joint_positions`). This adds a UART round-trip to every move and a new failure mode (move rejected if the position read fails) — not yet verified on hardware for added latency.

Trajectory following (V1, stop-at-each-waypoint): ✅ built (2026-07-15) — see `trajectory_node` row above. Not yet run against the real arm.

Continuous constant-velocity trajectory (blending): ✅ built (2026-07-17) — `continuous_trajectory_node` + `trajectory_planning.py` + the ESP32 `servo` streaming command; see the `continuous_trajectory_node` row above. Constant Cartesian tool speed with corner rounding; whole path validated before moving; setpoints streamed via `/facade_bot/joint_stream` → `esp32_bridge` `servo` command. Unit tests pass (planning + node-against-stubs); **not yet run against the real arm.**

Servo bus dead after v2 rebuild: ✅ resolved (2026-09-12) — commanding `[0,0,0,0]` now moves the arm and every joint reads back within noise of target (confirmed via `read_joint_positions`, repeated `move_to_pose` calls, and multi-waypoint moves, all against the real ESP32 at `192.168.1.150`). The original total dead-bus symptom (20/20 reads timing out with zero bytes) was never isolated to a root cause — it simply cleared before this session's testing started, so the underlying cause (servo power rail / common ground / TX-RX orientation / BusLinker jumper) is still unconfirmed and could recur. Separately, a new finding: single-attempt reads (no retry) succeed on only ~50% of calls, missing one servo — not consistently the same one — the rest of the time. The bus works but isn't fully clean; nothing is currently blocked on this (`read_joint_positions_fast` below is designed to tolerate exactly this), but it's worth another pass with `scripts/check_servo_bus.py` at some point.

Cartesian and trajectory moves verified on real hardware: ✅ (2026-09-12) — with the bus back, ran `move_to_pose`, `trajectory_node`'s `follow_trajectory` (a 30 mm straight-line sweep in global X, 4 waypoints; a 100×50 mm rectangular perimeter in the X-Z plane at 50 mm standoff, 13 waypoints, closed loop within ~2 mm), and the CG-folded home pose (`[0, -30, 100, 0]` deg, computed from URDF_V2's actual link masses/inertias — CG within ~1 mm of the base axis at ~119 mm height vs. ~148 mm for the all-zero vertical pose — see `LAUNCH.md` step 7a) all successfully. Every waypoint/corner was pre-validated against `kinematics.py` for reachability and joint-limit margin before anything was sent to hardware. `continuous_trajectory_node` was not touched and remains unverified on real hardware. All of the above still runs on v2's placeholder limits/tool-offset (see Current blocker) — internally consistent, not yet checked against a ruler.

Feedback speed for a `/joint_states` publisher: ✅ addressed (2026-09-12) — `read_joint_positions`'s full retry budget occasionally exceeded 600ms waiting out one slow joint, unsuitable for a polled visualization feed. Added `/facade_bot/read_joint_positions_fast` (`esp32_bridge_node`; single attempt, no retry, ~170ms typical measured on the arm, same `ReadJointPositions` message) alongside the existing service, which keeps its full retry behavior unchanged for the safety-critical move-fault check. See the `esp32_bridge` row above.

Single-command bringup + `/joint_states`: ✅ built (2026-09-12) — `bringup.launch.py` and `joint_state_publisher_node`; see their rows above. First real building block toward `robot_state_publisher`/RViz.

v2 hand measurements (joint centers, joint limits, tool-tip offset): ✅ resolved (2026-09-16/17) — joint centers re-checked on the rebuilt arm and found unchanged from v1 (no code edit needed); `joint_limits_deg` re-measured, joints 1–3 unchanged from v1, joint_4 widened to `[-110.0, 110.0]`; `tool_tip_offset_m` re-measured as `[0.06, 0.0, 0.0]` in link_4's frame, confirmed still along local x with no y/z offset despite joint_4 now being an out-of-plane axis. `robot_model.yaml`'s v2 `limits_source` is now `measured`, clearing the startup warning banner.

`/robot_description` + `/tf` (robot_state_publisher) + RViz: ✅ built (2026-09-17) — see the row above and `display.launch.py`. Kept as its own launch file, separate from `bringup.launch.py`, per the user's call.

Next task: verify the two open items in Current blocker above — IK-vs-ruler accuracy with the final measured numbers, and look at `kinematics.py`'s candidate-selection metric for the joint-velocity-minimization gap the user flagged (2026-09-17). Then check `display.launch.py` actually renders correctly (not yet run against a real display) and, once it does, use it to resolve `urdf_zero_offset_deg`. `trajectory_node` is verified on hardware; `continuous_trajectory_node` still needs its real-hardware pass: run a multi-waypoint sweep at low `tool_speed_mmps`, confirm the arm flows through interior waypoints without stopping and rounds corners, check that `stream_period_sec`/`servo_move_duration_ms` are generous enough against real Wi-Fi/servo-bus timing (the ~12 Hz bus cap), measure actual tool speed vs. commanded, and test cancel mid-sweep. Then, in the user's stated order (2026-09-10): (1) a raster/pattern generator for sweeping a rectangular area (which can now feed waypoints straight into `continuous_trajectory_node`), (2) the `facade_vision` pipeline (now unblocked — `/robot_description` + `/tf` exist).

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
