# FacadeBot — Project Status

This file is the single source of truth for what is done, what is verified on
hardware, and what is blocked. The root `CLAUDE.md` imports it, so it is in
context every session. Update it at the end of any session that changes
hardware state, installs something, or completes a milestone (the
`end-session` skill walks through this). Keep entries tight: every line here
costs context on every task.

Two review files sit alongside it and are worth reading before planning work:
`CODE_REVIEW_2026-09-18.md` (bug-level) and
`CODE_REVIEW_STRUCTURE_2026-09-18.md` (structural, with the open robotics
decisions and a suggested order of work).


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
| Minimum-jerk trajectory shaping for moves | ✅ Done — ESP32-side. Since the non-blocking rework (below) the shaping is stepped by the main loop (`start_motion` / `advance_motion_step`) on an absolute `ticks_ms` schedule; the blocking `move_joints_min_jerk` remains as the rollback path only |
| Non-blocking `servo` streaming command (firmware) | ✅ Done — `esp32_firmware/main.py` `handle_client` gains a `servo` command that moves straight toward a setpoint via the existing plain `move_joints` and acks immediately (no min-jerk, no readback). This is the primitive continuous motion streams onto (see `continuous_trajectory_node` row) |
| Non-blocking ESP32 firmware loop (`DECISIONS.md` D4) | ✅ **Staging fixed and verified on the arm 2026-09-26 (D13).** Cause measured with new step-timing counters (`status` → `timing`, bridge logs `move timing:` per move, `LAUNCH.md` 9d): mid-move `/joint_states` reads took ~93 ms of servo-bus time against ~70 ms step slack. Read gap cut 20 → 5 ms; under `bringup.launch.py` 1 s and 3 s moves now 0 late steps (one 25 ms wait in one move), smooth by eye. Earlier history: ⚠️ **On the board since 2026-09-26, and it made moves stage.** First flash needed two MicroPython fixes (`del bytearray[...]` unsupported; blocking `readinto` waited for a full buffer), so it had never actually run before. On the arm, `move_async` moves happen in ~3 visible stages at 1 s and 3 s, even with `/joint_states` polling stopped; the same moves on the blocking `move` path (`use_blocking_move:=true`, `esp32_timeout_sec:=7.0`) are smooth, as confirmed by the user by eye. Suspected cause, not measured: a status reply stalling on Wi-Fi (ping spikes ~1.1 s) delays the next trajectory step. Fix is open decision D13. Original entry (2026-09-20, sim-verified) — `handle_client` in `esp32_firmware/main.py` now polls its socket with `select.poll()` and advances the shaped trajectory one step per iteration, so `read_positions` and the new `status` command are answered **during** a move. New commands: `move_async` (acks on accept, the bridge's default) and `status` (`{"moving": bool}`); a second shaped move mid-motion is refused `{"status": "busy"}` and dropped (D3) — though on the normal ROS2 path the bridge's mutually-exclusive command group now serialises commands instead, so `busy` is a backstop rather than the usual outcome (see the D3 amendment in `DECISIONS.md`); `servo` preempts a shaped move. The blocking `move` is untouched and reachable via the bridge's new `use_blocking_move` parameter, so the previously hardware-verified path can be restored with no reflash. This fixes the reported symptom that the arm "jumps" between two poses in RViz: `/joint_states` used to go silent for the whole of every move. Measured in the simulator at a steady 5.0 Hz with 16 distinct intermediate poses through a 3 s move (was 2). **Requires a reflash** (`LAUNCH.md` step 1). Software stop still deferred (D1/D2) |
| Transport abstraction layer (send_command / read_response) | ✅ Done (`ros2_ws/src/esp32_bridge/esp32_bridge/transport.py`) |
| `esp32_bridge` ROS2 node | ✅ Done — subscribes to `/facade_bot/joint_cmd`, converts commanded degrees to raw servo positions and forwards `move` commands over Wi-Fi, reads positions back after each move and logs a fault if a joint misses target; serves `/facade_bot/read_joint_positions` for on-request position checks; per-joint bounds-check now in place (see below); no stop control of any kind (see the Emergency-stop row). Since 2026-09-20 it is **multi-threaded** (`MultiThreadedExecutor` + one `ReentrantCallbackGroup` + a `threading.Lock` held across each transport send/read pair in `_transact`), sends `move_async` and polls `status` in place of the old blocking ack (`_wait_for_motion_complete`, which releases the lock between polls so reads get served mid-move), and serves `/facade_bot/is_moving`. Also serves `/facade_bot/read_joint_positions_fast` (same `ReadJointPositions` message, single attempt/no retry, ~170ms measured vs. the full-retry service's occasional 600ms+) for a caller that polls continuously and can tolerate an occasional missing joint (e.g. `joint_state_publisher_node`, below) — the safety-critical move-fault check keeps using the full-retry version unchanged |
| `facade_control` ROS2 node | ✅ Done, verified on real hardware (2026-09-12) — first capability is Cartesian move commands: `/facade_bot/move_to_pose` service (`facade_msgs/srv/MoveToPose`) takes a tool-tip `(x_m, y_m, z_m, tool_angle_deg)` target, reads the arm's current joint positions from `esp32_bridge` first, then solves the target with the inverse kinematics in `facade_control/kinematics.py` — picking whichever valid candidate (azimuth branch × elbow-up/down) needs the least total joint travel from that current position, so the elbow no longer flips between up/down configurations across nearby targets — and publishes the result to `/facade_bot/joint_cmd`. Rejects (no publish, error message) anything geometrically unreachable, that would need a joint outside its safe range, or where the current-position read itself fails (`esp32_bridge` unreachable/timeout/invalid reading). Also serves `/facade_bot/read_tool_pose` (`facade_msgs/srv/ReadToolPose`): reads the arm's actual joint angles from `esp32_bridge` and runs them through forward kinematics, for checking real-world accuracy against a physically measured tool-tip position (avoid full-extension/retraction poses — those are singularities). Caveats: the 50 mm wrist→tool-tip offset is a measured constant (straight-line along reach direction, not a full 3D offset), never sends anything to the ESP32 directly, `move_to_pose` now needs `esp32_bridge` active just to solve (not only to move) — see `ros2_ws/src/facade_control/README.md` |
| `trajectory_node` (second node in `facade_control`) | ✅ Done, **verified on real hardware** (2026-09-12) — `/facade_bot/follow_trajectory` action (`facade_msgs/action/FollowTrajectory`) moves through an ordered list of Cartesian waypoints (`facade_msgs/msg/Waypoint`), one at a time: calls `facade_control_node`'s own `move_to_pose` per waypoint (reusing its IK/least-effort logic as-is), then confirms physical arrival by polling `esp32_bridge`'s `read_joint_positions` within a tolerance before advancing (as of 2026-09-20 arrival also requires `esp32_bridge`'s `/facade_bot/is_moving` to report stopped — position alone now yields a false arrival, since reads are answered mid-move; an unanswerable query counts as still moving). Reports feedback (`current_waypoint_index`/`total_waypoints`) and supports cancellation (stops commanding further waypoints only — no e-stop primitive exists to abort an in-flight move). This node is for precise point-to-point positioning (stops fully at each waypoint); continuous constant-speed motion is now a separate node (`continuous_trajectory_node`, below). Ran a real 30 mm straight-line sweep and a real 100×50 mm rectangular perimeter (13 waypoints) on the physical arm, both closing to within ~2 mm — see Previously resolved. |
| Continuous constant-velocity trajectory (`continuous_trajectory_node`) | ✅ Done — `/facade_bot/follow_trajectory_continuous` action (`facade_msgs/action/FollowTrajectoryContinuous`, adds `tool_speed_mmps` + `corner_blend_m` to the waypoint list) sweeps the tool tip through the waypoints at a constant Cartesian speed, rounding corners rather than stopping. Pure planning math lives in `facade_control/trajectory_planning.py` (corner-rounded Bézier path → constant-speed sampling with start/end ramps → chained IK); the whole path is validated for reachability **before** any motion, then joint setpoints are streamed to `/facade_bot/joint_stream`, which `esp32_bridge` forwards as the non-blocking `servo` command. Reuses `kinematics.py` and `trajectory_node`'s action-server/executor/helper-node patterns. Cancellation caveat same as `trajectory_node`. Unit-tested (planning + node against stubs); **not yet verified on real hardware** (Wi-Fi jitter, servo-bus rate cap ~12 Hz, corner accel) — see `ros2_ws/src/facade_control/README.md`'s "Following a trajectory continuously" section |
| Single-command bringup (`bringup.launch.py`) | ✅ Done (2026-09-12) — `ros2 launch facade_control bringup.launch.py` starts `esp32_bridge_node`, `facade_control_node`, `trajectory_node`, and `joint_state_publisher_node` together, and drives the bridge's lifecycle straight through `configure` → `activate` automatically (the standard ROS2 event-handler pattern for this), so the arm is live the moment the launch returns — no manual `ros2 lifecycle set` needed. `continuous_trajectory_node` is deliberately left out (run by hand — it's the one node not yet verified on real hardware). Lives in `ros2_ws/src/facade_control/launch/`; see `LAUNCH.md` step 5a. One trap hit and fixed while building this: giving a launch `Node` action an explicit `name=` injects a process-wide node-name remap, which also silently renamed the internal helper client node that `facade_control_node`/`trajectory_node`/`joint_state_publisher_node` each create for their own outgoing service calls — visible as a "Publisher already registered" warning and a duplicate entry in `ros2 node list`. Fixed by leaving `name=` unset for those three (they already name themselves in code); `esp32_bridge_node` has no such helper, so it keeps its explicit name (needed for the lifecycle event matching). |
| `/joint_states` publisher (`joint_state_publisher_node`, third node in `facade_control`) | ✅ Done (2026-09-12) — polls `esp32_bridge`'s `/facade_bot/read_joint_positions_fast` on a timer (`poll_rate_hz` param, default 5 Hz) and publishes `sensor_msgs/JointState` on `/joint_states` — deliberately the one topic in the project that does *not* get the `/facade_bot/...` namespace, since `robot_state_publisher` subscribes to this exact name by default. Holds each joint's last known angle across a miss rather than blocking or leaving a gap (see the fast-read note on the `esp32_bridge` row above); refuses to publish at all until every joint has had at least one real reading. Joint names come from the loaded model config, not hardcoded. Ships a placeholder `urdf_zero_offset_deg` parameter (all 0.0, startup warning) for the still-unverified gap between commanded-zero and the URDF's own zero pose — now checkable with `display.launch.py` below since `_JOINT_CENTER_RAD` is current (see Previously resolved), but not yet actually checked. |
| Camera Module 3 driver (`camera_ros` + Raspberry Pi libcamera fork) | ✅ Done — built from source in a **separate** workspace `~/camera_ws` on the RPi (not `ros2_ws`). Publishes image topics for a future `facade_vision` node. libcamera is the Raspberry Pi fork (imx708 / Module 3 support), built in-tree, not the apt package. Reproducible via `scripts/setup_camera.sh` + pins in `camera_ws.repos` (libcamera @ `26bfadc6`, camera_ros @ `03c9e03`); run instructions in `LAUNCH.md` step 13. Third-party source itself is NOT vendored into this repo |
| `/robot_description` + `/tf` (robot_state_publisher) | ✅ Done (2026-09-17) — `ros2_ws/src/facade_control/launch/display.launch.py` starts `robot_state_publisher` (reads the URDF XML named by `robot_model.yaml`'s `active_model` at launch time — the one place in the project a raw URDF is opened at runtime, deliberately kept out of every control node) and `rviz2` with a minimal saved view (`ros2_ws/src/facade_control/rviz/facadebot.rviz`, meant to be extended). Deliberately its own launch file, not part of `bringup.launch.py` (user's call, 2026-09-17): visualization only, assumes a desktop session, so it shouldn't start for headless bench work. Not yet run against an actual display (no GUI available in dev environment) — verify it renders before trusting it. The rendered pose won't match the physical arm's zero yet — see `urdf_zero_offset_deg` above. |
| `facade_vision` pipeline | ⬜ Not started — camera driver is up (see row above); the FacadeBot-side node that consumes its image topics isn't written yet, and will live in `ros2_ws/src/facade_vision/` |
| Kinematics for the rebuilt arm (v2) | ✅ Done, unverified on hardware — URDF_V2 changed the structure, not just the numbers: joint_1's axis flipped to `0 0 -1`, joint_2's origin rpy lost its roll (working plane rotated 90°), links changed (shoulder→elbow 125.11→**135.11 mm**, elbow→wrist 165.11→**140.11 mm**), and critically **joint_4's rpy became `-1.5708 0 0`, moving its axis out of the joint_2/joint_3 plane** — so the old planar-3R solver was invalid, not just mis-parameterised. `kinematics.py` was rewritten: no hardcoded geometry (everything from the loaded `RobotModel`), `configure()` validates the model actually has this arm's shape and rejects it otherwise, and the closed form now fixes joint_4 first (which collapses wrist+tool into one rigid vector off the elbow), solves joint_1 from the out-of-plane distance, then the shoulder/elbow as a planar 2R. **`tool_angle_deg` changed meaning**: it is now joint_4's own angle — the tool's tilt out of the arm plane — not the tool's pitch. Pass 0 to keep the tool in-plane. Verified: 4000 random poses per model round-trip to 3.5e-13 mm with the seed configuration recovered every time, 0 elbow flips across nearby targets, and v1 still solves (the solver degenerates to the old in-plane case). Fixed a ±180° wrap bug that was failing the bounds-check on physically reachable poses. Sub-mm CAD asymmetries in URDF_V2 (0.14–0.89 mm) are snapped to zero — below one LX-16A count (~1.25 mm at full reach) |
| `facade_msgs` custom message package | ✅ Done — `ReadJointPositions.srv`, `ReadToolPose.srv`, and `MoveToPose.srv` (now also returns `solved_angles_deg`) (used by `esp32_bridge`/`facade_control`); `msg/Waypoint.msg`, `action/FollowTrajectory.action` (used by `trajectory_node`), and `action/FollowTrajectoryContinuous.action` (used by `continuous_trajectory_node`) |
| Arm model config (`facadebot_description`) | ✅ Done — the arm is now selected by one line (`active_model`) in `ros2_ws/src/facadebot_description/config/robot_model.yaml`. Both URDFs live side by side (`urdf/URDF_Test.urdf` = v1 retired, `urdf/URDF_V2.urdf` = v2 current) with meshes split into `meshes/v1/` and `meshes/v2/`; the broken `package://URDF_Test/` mesh paths are fixed in both. Geometry (link lengths, joint origins, axes) is **baked out of the selected URDF once** by `scripts/generate_geometry.py` into a generated `config/geometry_<model>.yaml` — never transcribed by hand, and never re-parsed at runtime: **no node opens a URDF or an XML parser at startup.** `facadebot_description/urdf_geometry.py` holds the design-time parser (generator-only); `robot_model.py` is pure YAML. Only hardware-measured numbers live in `robot_model.yaml`: per-joint limits and the joint_4→tool-tip offset. The package was converted from `ament_cmake` to `ament_python` so `facade_control` and `esp32_bridge` can both import the loader without the hardware gate depending on the planner. A model's `limits_source` records whether its hand-measured numbers were actually measured on that arm; anything but `measured` makes `describe()` print a startup warning banner (a warning, not a refusal — bench bring-up has to move the arm before the arm can be measured). Loader is otherwise **fail-closed**: unmeasured limits/offset, an unknown `active_model`, a missing `geometry_file`, a geometry file naming a different model, a joint chain not ordered base-to-tip, a joint-name mismatch against the limits, or inverted limits all abort node startup. Verified: geometry loaded from YAML is float-for-float identical to the URDF parse for both models, all three nodes refuse to start and print why, 33 tests pass. See `LAUNCH.md` step 3a |
| Joint homing / zero calibration | ✅ Re-checked for v2 (2026-09-16) — `esp32_bridge_node.py`'s `_JOINT_CENTER_RAD` holds each joint's separately measured true center (radians), so commanded `0°` means that center rather than raw position 0. Re-measured on the rebuilt v2 arm; joint_1 moved from v1's 2.09 to **2.17** (commit `c751abc`) and joints 2-4 came back at their v1 values. Values in code confirmed current by the user 2026-09-21 (this row previously said the whole array was unchanged, which the diff contradicted — see `CODE_REVIEW_2026-09-18.md` finding 1, now closed). Kept in code, not `robot_model.yaml`, because they are per-servo calibration rather than a property of the model |
| Bounds-checking for joint commands | ✅ Done — `esp32_bridge_node.py`'s `_check_joint_bounds` is still the single, mandatory, last-resort gate (it applies whether a command came from `facade_control`'s IK or straight from `ros2 topic pub`), but the limits it enforces now come from `robot_model.yaml` via `_load_joint_limits()` instead of a hardcoded tuple, so this gate and `kinematics.py` can no longer drift apart. `_check_joint_bounds` raises rather than passing anything through if limits were never loaded. v2's limits are now measured (`limits_source: measured`, 2026-09-17) — joints 1–3 unchanged from v1, joint_4 widened to ±110° |
| Servo bus diagnostic (`scripts/check_servo_bus.py`) | ✅ Done — read-only probe run **on the ESP32 over USB** via `mpremote` (never Wi-Fi: a `move` ack only proves the command reached the ESP32). Sweeps servo IDs 1-8, dumps the raw bytes each returned, and classifies the fault as dead-bus / echo-only / garbage / wrong-ID, with a likelihood-ordered checklist. `RUN_LOOPBACK_TEST = True` (BusLinker unplugged, GPIO17 jumpered to GPIO16) clears or convicts the ESP32's own UART2. See `LAUNCH.md` step 2a |
| Emergency-stop logic | ❌ **None. Bench stop switch removed 2026-09-26 (user's decision)**, the same day it was built. Firmware, bridge (`status` polling, `/facade_bot/estop_active`, command dropping), trajectory-node aborts, simulator switch and 21 tests deleted. The only stop is the servo power plug. Root `CLAUDE.md` updated to match (user, 2026-09-26): the e-stop-must-exist rule was dropped. Kept from that work: a shaped move is refused if a joint's start position cannot be read. Flashed and verified on the arm 2026-09-26: bringup active, no `estop` topic, home ↔ `[0,0,0,0]` at 1 s with 0 late steps, all four joints read valid after each move. |
| `esp32_bridge` unit tests | ✅ Done — `test/test_esp32_bridge_node.py`, 33 tests against a fake transport (stop-switch tests removed 2026-09-26): bounds gate, degree↔raw conversion, move/stream paths, the `status`-poll wait, `busy` rejection, the `use_blocking_move` rollback, `is_moving`, fault and dead-bus reporting, readback merge. The fake answers by command name rather than from one reply queue. No hardware needed |
| Simulated-ESP32 bench tool (`scripts/fake_esp32_server.py` + `facade_control/launch/bringup_sim.launch.py`) | ✅ Done (2026-09-19) — a plain-Python TCP server that speaks `esp32_firmware/main.py`'s wire protocol on localhost with simulated joint positions (start at raw 500; speaks the full 2026-09-20 protocol — `move_async`/`status`/`busy`, `servo` preempts, legacy `move` still blocks), plus a dedicated launch file that starts it as a subprocess alongside `esp32_bridge_node` (hardcoded `esp32_host=127.0.0.1` — never the real IP, so it can't be pointed at hardware by mistake), `facade_control_node`, `trajectory_node`, and `joint_state_publisher_node`, auto-configuring/activating the bridge like `bringup.launch.py`. Deliberately a separate launch file rather than a parameter added to `bringup.launch.py`, so the real hardware bring-up path is untouched (user's call, 2026-09-19). Does not simulate the real bus's flaky single-attempt reads, so it verifies ROS2-level control-loop logic and timing, not hardware fault tolerance. See `LAUNCH.md` step 2b. **Verified end to end** (2026-09-19): `ros2 launch facade_control bringup_sim.launch.py` reached `active`, a `move` command through `/facade_bot/joint_cmd` was bounds-checked, forwarded, and read back correctly via `read_joint_positions`, and `/joint_states` reflected it — all against the simulator, no arm involved. |
| Project instruction files (root + per-package `CLAUDE.md`, skills, `safety-reviewer` agent, commit-blocking hook) | ✅ Done (2026-09-18) — awaiting the user's read-through; `STATUS.md` split out of `CLAUDE.md`, read on demand |
| `DECISIONS.md` (open robotics/architecture questions with options) | ✅ Created (2026-09-18) — ten questions, all unanswered |

### Next task (updated 2026-09-26, later session)
1. ~~D13 staged motion~~ and ~~servo 3 read misses~~: **fixed and verified on the arm
   2026-09-26** (see the D4 row).
2. **Pi is behind the dev box.** Today's bench runs used the bridge on the dev box.
   Copy the workspace to the Pi and rebuild (`LAUNCH.md` step 3) before running
   there.
3. Implement the decided-but-unbuilt items: D5 (IK L∞), D6 (per-joint speed),
   D7 (5 mm arrival), D11 (base-axis refusal), D12 (continuous-path joint speed).

### Current blocker
**The three hand measurements (joint centers, joint limits, tool-tip offset) are all done and in
place (see Previously resolved) — what's left is verification and one solver question:**
1. IK has not been re-verified against the physical arm with the final numbers — `read_tool_pose`
   vs. a ruler-measured tool-tip position, at a couple of non-singular poses (avoid full
   extension/retraction).
2. `kinematics.py`'s least-effort candidate selection was flagged (user, 2026-09-17) as **not
   actually minimizing joint velocity**. Diagnosed 2026-09-18: over 5000 random moves on v2,
   least-total-travel disagrees with least-peak-joint-travel ~30% of the time when there is a
   choice, and ties are broken by list order (worst case seen: a 184° single-joint swing chosen
   over 114°). **D5 answered 2026-09-21: least peak joint travel (L∞), tie-break by total.
   Decided but NOT yet implemented — `kinematics.py` still scores by the L1 sum.** The change
   and its test are specified in `DECISIONS.md` D5; nothing in the tree has moved yet.
3. `joint_state_publisher_node`'s `urdf_zero_offset_deg` is still an unverified placeholder (all
   0.0) — now checkable with `display.launch.py` (see the `/robot_description` + `/tf` row) since
   joint centers are confirmed current, but the actual visual check (comparing the rendered pose to
   the physical arm) hasn't been done yet.

### Previously resolved
Session 2026-09-26, verification ledger:
- Bench stop switch (firmware, bridge topic and command dropping, trajectory-node
  aborts, simulator switch): **unit tests** (bridge 43, facade_control 46) and
  **simulator**. **On the board:** switch read, 7 stops counted, releases on run.
  **Not verified on the arm during motion.**
- Start-pose refusal (no guessed start pose): unit tests on the bridge side; the
  firmware path is on the board, but no refusal has been observed yet.
- Firmware boot and protocol fixes (constant order, soft IRQ, bytearray slice
  assignment, `recv`, blocking `accept` restored): **verified on the board** (boot,
  ping, `status`, `read_positions`).
- Moves on the arm (bridge on the dev box): home ↔ `[0,0,0,0]` at 1 s and 3 s,
  both paths. Blocking path smooth; non-blocking path staged (by the user's eye).
- `LAUNCH.md` 9b rollback procedure corrected (needs cleanup + configure for the
  timeout): verified on the arm.

Decisions walk-through 2026-09-26 (`DECISIONS.md`): answered D6 (per-joint peak speed
60 deg/s in YAML, joints finish together, bridge splits moves over 5 s), D7 (arrive within
5 mm at the tool tip, FK in `trajectory_node`), D10 (stay custom, adopt standard
interfaces), D11 (refuse Cartesian targets within 10 mm of the base axis), new D12 (refuse
continuous paths needing any joint over its peak speed). D8 deferred; D9 still open.
**None of it is implemented** — the tree still behaves as before.

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

Tier-1 code fixes applied 2026-09-18 (unit-tested, **not yet run on the arm**): IK solves targets on the base axis (all-zero home pose) by keeping the current base angle; elbow cosine gets a 1e-9 slack so fully extended poses no longer fail on floating-point noise; `bringup.launch.py` only auto-activates after `configure` (manual `deactivate` now sticks); `urdf_zero_offset_deg` accepts integer overrides; startup refusals go through the rclpy logger instead of `print()`; the model loader refuses a geometry file whose URDF hash no longer matches and refuses duplicate joint names (4 new loader tests); `tool_angle_deg` comments/README corrected to "joint_4's angle"; elbow-flip regression test restored. Test counts: facade_control 38, esp32_bridge 29, facadebot_description 4 (as of 2026-09-20). `scripts/test_trajectory.py` still present (deletion awaiting the user's yes). Safety-reviewed 2026-09-18: nothing gates motion, "safe to bench test as is"; the reviewer's doc and log-level items were applied, and the near-axis base-swing behaviour it found (pre-existing) is logged as D11 in `DECISIONS.md`.

Docs corrected 2026-09-18: ESP32 address `.100` → `.150` in `LAUNCH.md` and the bridge README; bridge README no longer claims "no bounds-checking"; `LAUNCH.md` step 7 now warns against `scripts/test_trajectory.py` instead of suggesting it (script still present, slated for removal per structural review finding 4).

Non-blocking firmware loop (D4) + `/facade_bot/is_moving`: ✅ built 2026-09-20 — see the
"Non-blocking ESP32 firmware loop" row above. Fixes the reported RViz symptom (the arm
rendering only its start and end pose, because `/joint_states` went silent for the whole
of every move). Verified in unit tests (25 + 38 + 4 pass) and end to end against
`bringup_sim.launch.py`: 16 distinct intermediate poses at a steady 5.0 Hz through a 3 s
move, `is_moving` true mid-move and false either side, and a second move sent mid-move
logged as `busy` and dropped (before the safety-review fixes; after them the two moves
serialise instead — see the D3 amendment). Safety-reviewed 2026-09-20: seven findings,
six fixed (fail-open `status` reply, command/service callback-group split, transport
reconnect after a failed exchange, teardown under the lock, blocking socket for
`sendall`, clamped trajectory catch-up) and one stale (it read the docs mid-edit).
Two items were left for the user rather than decided here: how a min-jerk start pose
should be seeded when a joint's readback is missed, and whether a `busy`/queued command
should be surfaced back through `MoveToPose`. **Nothing has been run on the arm, and the ESP32 has not
been reflashed** — the board still has the blocking firmware, so the bridge on this tree
would get "unknown command" for `move_async` until it is. D3 and D4 are answered in
`DECISIONS.md`.

Superseded next-task list (written 2026-09-19 at session close): this session added a bench-only
tool (`scripts/fake_esp32_server.py` + `facade_control/launch/bringup_sim.launch.py`,
see the row above) — `ros2 launch facade_control bringup_sim.launch.py` runs the whole
control loop with no arm attached, verified working end to end. It's useful for
checking ROS2-level sequencing/timing (including re-checking the tier-1 fixes below
before spending arm time on step 4), but it cannot substitute for the arm: no e-stop
exists in either case, and the simulator doesn't reproduce the real bus's flaky reads.
Nothing from the 2026-09-18 review session has been run on the arm yet. In order:

1. **User reads** the new instruction set (root `CLAUDE.md`, the six package `CLAUDE.md` files,
   `TESTS.md`, `DECISIONS.md`) and marks up `TESTS.md`'s Status column if anything is wrong.
   Run `/sync-tests` after any markup.
2. **User answers `DECISIONS.md`**, D1 and D2 (e-stop meaning and signal path) first: they gate
   the firmware rework that most other findings depend on. D3–D11 as time allows.
3. **User says yes or no** to deleting `scripts/test_trajectory.py` (bypasses the bounds gate; the
   docs already warn against it).
4. **Bench check of the tier-1 fixes** (arm on): `ros2 launch facade_control bringup.launch.py`,
   confirm the bridge reaches `active` exactly once; home move (`LAUNCH.md` step 7a) then a
   `read_joint_positions` call; `ros2 lifecycle set /esp32_bridge_node deactivate` and confirm it
   stays inactive; optionally a `move_to_pose` to the home position from a turned base and see the
   base hold rather than swing. Then the IK-vs-ruler check (Current blocker item 1).
5. ~~**Tier 2**: the user states which joint-1 centre (2.09 or 2.17 rad) was measured on v2 and
   when; fix the `_JOINT_CENTER_RAD` comment accordingly.~~ **Done 2026-09-21**: 2.17 confirmed
   correct, STALE comment removed, provenance corrected in code, this file, and the package
   `CLAUDE.md`. Comment-only — no numeric or behavioural change.
6. **Tier 3**, each unblocked by its decision: bridge busy flag / motion owner (D3); non-blocking
   firmware loop + software stop + protocol version + keepalive + Wi-Fi credentials to a
   gitignored file, landed together with one reflash (D1, D2, D4); IK metric (D5); arrival
   tolerance in mm (D7); `JointTrajectory` command with duration (D6); wall-frame task definition
   (D8) before the raster generator. Extracting the four-copy service-call helper is unblocked but
   is its own reviewable change.
7. Then the previously planned order: `display.launch.py` on a real display and
   `urdf_zero_offset_deg`; the `continuous_trajectory_node` hardware pass (low `tool_speed_mmps`,
   check flow through interior waypoints, `stream_period_sec`/`servo_move_duration_ms` against the
   ~12 Hz bus cap, measured vs commanded speed, cancel mid-sweep); the raster generator; then
   `facade_vision`.
