# Launching FacadeBot

Steps and commands to get the arm from powered-off to accepting joint commands.
Run the RPi/ROS2 steps on the Raspberry Pi; the ESP32 flashing step can be run
from whichever machine has the ESP32 plugged in over USB.

## 0. Network addressing (reference)

Both the RPi and the ESP32 hold static IPs — nothing here is handed out by DHCP,
so these addresses are the same on every boot.

| Device | Address | Where it's set |
|--------|---------|----------------|
| RPi (wlan0) | `192.168.1.17` | NetworkManager profiles `TP-Link_08F3` and `Airtel_hart_5833`, both `ipv4.method manual` |
| ESP32 | `192.168.1.100` | `STATIC_IP` in `esp32_firmware/main.py` |
| Gateway | `192.168.1.1` | Both networks use `192.168.1.0/24` |

`TP-Link_08F3` is the network the RPi↔ESP32 link runs on — the ESP32 firmware
joins that SSID. The RPi keeps `192.168.1.17` on the Airtel network too, so SSH
is the same command either way:

```bash
ssh harthik@192.168.1.17
```

To put the RPi on the ESP32's network (do this from a monitor/keyboard on the Pi
— it drops any SSH session running over the other network):

```bash
sudo nmcli con up TP-Link_08F3
ip -4 -brief addr show wlan0   # expect 192.168.1.17/24
```

## 1. Flash the ESP32 (only needed after `esp32_firmware/main.py` changes)

The board needs to be connected over USB for this step — Wi-Fi isn't up yet
until the new code is running.

```bash
/home/harthik/.firmware/bin/mpremote connect /dev/ttyUSB0 fs cp esp32_firmware/main.py :main.py
/home/harthik/.firmware/bin/mpremote connect /dev/ttyUSB0 reset
```

The `reset` is not optional: `fs cp` only writes the file, it does not restart the `main.py`
already running on the board, so without it you are still talking to the old firmware.

Equivalently, activate the venv first and drop the absolute path — `mpremote` is not on PATH
otherwise:
```bash
source /home/harthik/.firmware/bin/activate
mpremote connect /dev/ttyUSB0 fs cp esp32_firmware/main.py :main.py
```

If `/dev/ttyUSB0` doesn't exist, find the actual port with:
```bash
ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
```

If it exists but `mpremote` reports permission denied, the port is `root:dialout` and your shell
session predates your `dialout` membership — log out and back in, or as a one-off:
```bash
sudo chown harthik /dev/ttyUSB0
```

Opening the REPL shows the connection banner and then nothing — no `>>>` prompt. That is the
healthy case: `main.py` is running and blocked waiting for a TCP client, and the firmware has no
`print()` calls, so there are no boot messages to read. Ctrl-] exits and leaves it running.

Silence does **not** confirm it joined Wi-Fi — `connect_wifi()` retries forever with no timeout
or message, so a hung join looks identical. Confirm over the network instead (step 2), not from
the REPL:
```bash
/home/harthik/.firmware/bin/mpremote connect /dev/ttyUSB0
# Ctrl-] to exit
```

## 2. Power on the hardware

- ESP32 + Hiwonder board powered, servos connected and powered.
- Confirm the ESP32 joined Wi-Fi and is reachable at its static IP:
  ```bash
  ping 192.168.1.100
  ```

## 2a. Check the servo bus (whenever the arm does not move)

Read-only — this probe only sends position reads, it cannot move the arm. It runs **on the
ESP32 over USB**, not over Wi-Fi: a `move` ack proves only that the command reached the ESP32,
never that a servo received it, so when the arm is silent the fault is below that layer and
Wi-Fi is the wrong place to look.

```bash
/home/harthik/.firmware/bin/mpremote connect /dev/ttyUSB0 run scripts/check_servo_bus.py
```

**No ROS2 needed** — do not start `esp32_bridge_node` for this, and stop it if it is running.
`mpremote` talks over the USB cable and interrupts `main.py` to run the script, so there is no TCP
server while it runs and any connected bridge would just drop. The probe opens UART2 itself. You
need only the USB cable and the **servo power rail on** (an unpowered bus cannot answer, which is
one of the things being tested). Because it is USB-only it also works from a machine with no route
to the ESP32's Wi-Fi network.

Restart the firmware afterwards:
```bash
/home/harthik/.firmware/bin/mpremote connect /dev/ttyUSB0 reset
```

If the command hangs instead of printing, `mpremote` could not break into `main.py` blocked in
`server.accept()`. Press the board's EN/reset button and re-run immediately, or open
`mpremote connect /dev/ttyUSB0 repl`, press Ctrl-C for a `>>>` prompt, Ctrl-] to exit, then re-run.

It probes IDs 1–8 (wider than the four the firmware drives, so a servo left on a factory-default
ID still shows up), prints the raw bytes each one returned, and ends with a verdict:

| Output | Meaning |
|--------|---------|
| `ALIVE` for IDs 1–4, nothing else | Bus is healthy |
| `NO BYTES` for every ID | Bus is dead — check servo power rail, common ground, TX/RX orientation, BusLinker mode jumper |
| `ECHO ONLY` | ESP32 reaches the BusLinker but no servo answers — servo power, or wrong servo IDs |
| `GARBAGE` | Baud mismatch or line noise |
| `ALIVE` on unexpected IDs | The rebuild left servos renumbered — renumber them or update `SERVO_IDS` in `esp32_firmware/main.py` |

To clear the ESP32 itself, unplug the BusLinker, jumper GPIO17 straight to GPIO16, set
`RUN_LOOPBACK_TEST = True` at the top of the script and re-run. A pass means the board's UART2
is fine and the fault is in the wiring or downstream.

## 3. Build the ROS2 workspace (only needed after code changes under `ros2_ws/`)

```bash
cd /home/harthik/FacadeBot_control/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select facade_msgs facadebot_description esp32_bridge facade_control
```

## 3a. Select the arm model (only after a URDF re-export or a recalibration)

Which arm the stack drives is set by one line in
`ros2_ws/src/facadebot_description/config/robot_model.yaml`:

```yaml
active_model: v2
```

Link lengths, joint origins and joint axes are **not** written in that file. They are baked out
of the SolidWorks URDF once by `scripts/generate_geometry.py` into a generated
`config/geometry_<model>.yaml`, which is what the nodes read. Nothing that runs on the arm ever
opens a URDF. Only the hand-measured numbers live in `robot_model.yaml`: the per-joint limits
and the joint_4-to-tool-tip offset.

Check what the stack will actually load before starting anything:

```bash
source /home/harthik/FacadeBot_control/ros2_ws/install/setup.bash
python3 -c "from facadebot_description.robot_model import load_robot_model, describe; print(describe(load_robot_model()))"
```

That prints every origin, axis and limit in force, or explains why it refuses to load. It
refuses — and so does every node — when the selected model has unmeasured limits or tool
offset, when `geometry_file` is missing or names another arm's geometry, or when the geometry
file's joint chain doesn't run base to tip.

### After re-exporting a URDF from SolidWorks

The re-export does nothing on its own — the stack keeps running the old geometry until you
regenerate. Re-run the generator, re-measure anything the new geometry invalidates, and commit
both files together:

```bash
cd /home/harthik/FacadeBot_control
python3 scripts/generate_geometry.py --model v2
```

It prints each joint it wrote so you can eyeball the link lengths against the CAD model. It
reads the URDF path from that model's `urdf_file` entry, touches nothing else in
`robot_model.yaml`, and needs no build or sourced workspace — it runs straight from a checkout.
Rebuild afterwards (step 3) so the new geometry reaches the install directory.

The generated file carries a `DO NOT EDIT` header and records the SHA-256 of the URDF it came
from. If you hand-edit it, the controller silently disagrees with both the CAD model and the
physical arm — change the URDF and regenerate instead.

## 4. Source the workspace (every new terminal)

```bash
source /opt/ros/jazzy/setup.bash
source /home/harthik/FacadeBot_control/ros2_ws/install/setup.bash
```

## 5. Start the bridge node (terminal A)

```bash
ros2 run esp32_bridge esp32_bridge_node
```

This does nothing yet — it's a lifecycle node, so it stays idle until you
explicitly configure and activate it (step 6). Leave this terminal running;
its log output is where you'll see `move fault` errors if a joint stalls.

## 5a. Or bring everything up at once with one launch file

Steps 5, 6, 8, and 11 (the bridge, its lifecycle transitions, `facade_control_node`,
and `trajectory_node`) can be started together instead of by hand, one terminal:

```bash
ros2 launch facade_control bringup.launch.py
```

This starts all three nodes and drives the bridge straight through `configure`
then `activate`, so **the arm is live and commandable the moment this returns**
— there is still no e-stop, so don't run it with anyone near the arm's reach
envelope. `continuous_trajectory_node` is deliberately left out (run it by hand,
`ros2 run facade_control continuous_trajectory_node`, only when you're
specifically testing continuous sweeps — it's the one node not yet verified on
real hardware). Skip steps 5, 6, 8, and 11 below if you use this; you still need
step 4 first.

## 6. Bring the node up (terminal B, after sourcing per step 4)

```bash
ros2 lifecycle set /esp32_bridge_node configure   # opens the TCP connection to the ESP32
ros2 lifecycle set /esp32_bridge_node activate     # starts listening on /facade_bot/joint_cmd
```

## 7. Send joint commands

Either a one-off test command:
```bash
ros2 topic pub --once /facade_bot/joint_cmd sensor_msgs/msg/JointState \
  "{position: [0.0, 1.5708, 3.14159, 0.7854]}"
```

or the demo trajectory script (talks to the ESP32 directly, bypassing ROS2 —
useful for a quick hardware sanity check without bringing up the node):
```bash
python3 scripts/test_trajectory.py
```

## 7a. Send the arm to the home position

The home (park) pose folds the arm into a compact triangle over its own base, so
the centre of gravity sits roughly over the base axis instead of hanging out to
one side. Use it between jobs, and before powering the servos down.

| Joint | Angle |
|-------|-------|
| joint_1 | 0 deg |
| joint_2 | -30 deg |
| joint_3 | +100 deg |
| joint_4 | 0 deg |

`/facade_bot/joint_cmd` takes **radians**, so those four angles on the wire are:
```bash
ros2 topic pub --once /facade_bot/joint_cmd sensor_msgs/msg/JointState \
  "{position: [0.0, -0.5235987756, 1.7453292519, 0.0]}"
```

Confirm it arrived (step 9 covers this service in full):
```bash
ros2 service call /facade_bot/read_joint_positions facade_msgs/srv/ReadJointPositions "{}"
```
Expect roughly `[0.00, -29.76, 100.08, 0.48]` deg with `all_valid=True` — within
about half a degree of target, which is servo quantisation, not an error.

Two things to know about these numbers:

- They are relative to `esp32_bridge_node.py`'s `_JOINT_CENTER_RAD`, like every
  other angle on `/facade_bot/joint_cmd`. That calibration is still the stale v1
  measurement, so **re-check this pose by eye after `_JOINT_CENTER_RAD` is
  re-measured on the v2 arm** — the same commanded angles will land elsewhere.
- joint_3 is held at +100 deg rather than its +110 deg limit because that limit is
  a v1 placeholder (see step 3a). Do not raise it until v2's real travel is measured.

## 8. Move to a Cartesian pose with IK (optional, terminal C)

`facade_control` converts a tool-tip `(x, y, z, angle)` target into joint
angles and publishes them to `/facade_bot/joint_cmd` — same topic as step 7,
so `esp32_bridge` must already be activated (steps 5–6) to actually move.

```bash
ros2 run facade_control facade_control_node
```

Then, in another sourced terminal:
```bash
ros2 service call /facade_bot/move_to_pose facade_msgs/srv/MoveToPose \
  "{x_m: 0.2, y_m: 0.0, z_m: 0.2, tool_angle_deg: 0.0}"
```

`move_to_pose` now reads the arm's current joint positions (step 9's service)
before solving, so it picks the valid IK candidate closest to where the arm
already is instead of always the same one — this needs `esp32_bridge` to be
activated (steps 5–6) even just to solve, not only to actually move.

`success: false` means either the target was geometrically unreachable /
needed a joint outside its safe range, or the current-position read failed
(`esp32_bridge` not activated, unreachable, or a joint had no reading) —
either way nothing was sent to the arm. See
`ros2_ws/src/facade_control/README.md` for what the fields mean and the
known IK limitations (currently flagged as inaccurate — verify against a
known position before trusting it).

## 9. Check the arm's current position

```bash
ros2 service call /facade_bot/read_joint_positions facade_msgs/srv/ReadJointPositions {}
```

Don't try to read position with a separate script/`nc` while the node is running —
the ESP32 only accepts one TCP client at a time, and the node holds its connection
open the whole time it's active. This service exists precisely so you don't have to
fight that limitation; use it instead.

## 10. Check the arm's actual tool-tip pose (needs `facade_control_node` from step 8)

Reads real joint angles back (via step 9's service) and runs them through
forward kinematics — reports where the tool tip actually is, as opposed to
step 8's `move_to_pose`, which only reports where a commanded target was
solved to go.

```bash
ros2 service call /facade_bot/read_tool_pose facade_msgs/srv/ReadToolPose {}
```

Useful for checking real-world accuracy against a physically measured
tool-tip position. Pick a pose away from full extension/retraction first —
those are kinematic singularities and won't reveal an elbow-branch or
per-joint offset error even if one's there.

## 11. Move through a sequence of waypoints (optional, needs steps 5–8 already running)

```bash
ros2 run facade_control trajectory_node
```

Then, in another sourced terminal:
```bash
ros2 action send_goal /facade_bot/follow_trajectory facade_msgs/action/FollowTrajectory   "{waypoints: [{x_m: -0.17, y_m: 0.0, z_m: 0.25, tool_angle_deg: 0.0}, {x_m: -0.17, y_m: 0.15, z_m: 0.22, tool_angle_deg: 0.0},{x_m: -0.17, y_m: 0.15, z_m: 0.3, tool_angle_deg: 0.0},{x_m: -0.17, y_m: 0.0, z_m: 0.35, tool_angle_deg: 0.0},{x_m: -0.17, y_m: 0.0, z_m: 0.22, tool_angle_deg: 0.0}]}"   --feedback

```

Moves through each waypoint in order, stopping fully at each one before the
next (no blending yet). Ctrl-C the `send_goal` call to cancel — this stops
further waypoints from being commanded, but does **not** stop the arm
mid-move (no e-stop primitive exists yet). See
`ros2_ws/src/facade_control/README.md`'s "Following a trajectory" section
for the feedback/result fields and known limitations.

## 12. Sweep through waypoints continuously at constant speed (optional, needs steps 5–8 already running)

```bash
ros2 run facade_control continuous_trajectory_node
```

Then, in another sourced terminal:
```bash
ros2 action send_goal /facade_bot/follow_trajectory_continuous facade_msgs/action/FollowTrajectoryContinuous \
  "{waypoints: [{x_m: 0.20, y_m: -0.05, z_m: 0.20, tool_angle_deg: 0.0}, {x_m: 0.20, y_m: 0.0, z_m: 0.20, tool_angle_deg: 0.0}, {x_m: 0.20, y_m: 0.05, z_m: 0.20, tool_angle_deg: 0.0}], tool_speed_mmps: 30.0, corner_blend_m: 0.02}" \
  --feedback
```

Unlike step 11, the tool tip flows through the waypoints at a constant speed
(`tool_speed_mmps`) without stopping, rounding each corner within
`corner_blend_m` instead of passing exactly through it. The whole path is
planned and checked for reachability **before** any motion, then streamed to
the arm. Start with a low speed on hardware. Cancellation and the no-e-stop
caveat are the same as step 11. See
`ros2_ws/src/facade_control/README.md`'s "Following a trajectory continuously"
section for the goal fields, tuning, and constraints.

## 12a. Visualize the arm in RViz (optional, needs `/joint_states` already publishing — step 5a's `joint_state_publisher_node`, or step 5-6 plus running it by hand)

```bash
ros2 launch facade_control display.launch.py
```

Starts `robot_state_publisher` (publishes `/robot_description` and `/tf` from
the URDF named by `robot_model.yaml`'s `active_model` — same single source of
truth every control node uses, so this always matches the arm actually
selected) and `rviz2` with a minimal saved view
(`ros2_ws/src/facade_control/rviz/facadebot.rviz`). Needs a desktop session —
won't do anything useful over a headless SSH terminal. Deliberately kept out
of `bringup.launch.py`: it's visualization only, reads the raw URDF at launch
time (fine here, not allowed in any control node), and assumes a display is
attached.

The rendered pose will not exactly match the physical arm's zero position yet —
`joint_state_publisher_node`'s `urdf_zero_offset_deg` parameter (see its row in
CLAUDE.md's Project Status) is still an unverified placeholder (all 0.0), so
treat this as topology/proportions-correct but not yet zero-calibrated against
the real arm.

## 13. Start the camera (optional, separate `~/camera_ws` — RPi only)

The Camera Module 3 driver (`camera_ros` built against the Raspberry Pi fork of
libcamera) lives in its own workspace, `~/camera_ws`, **not** the main
`ros2_ws`. To rebuild it from scratch on a fresh Pi, run
`scripts/setup_camera.sh` (pins are in `camera_ws.repos`).

Source it *in addition to* the main workspace (order matters — overlay the
camera ws, then the main ws):
```bash
source /opt/ros/jazzy/setup.bash
source /home/harthik/camera_ws/install/setup.bash
source /home/harthik/FacadeBot_control/ros2_ws/install/setup.bash
```

Then start the camera node:
```bash
ros2 run camera_ros camera_node --ros-args -p orientation:=180 -p width:=800 -p height:=600
```

It publishes image topics that a future `facade_vision` node will subscribe to.
Confirm it's alive with `ros2 topic list | grep -i image` in another sourced
terminal. `orientation:=180` flips the image for the arm's mounting; drop it if
the camera is mounted upright.

### 13a. Viewing the stream

The Pi is normally headless, so watch the stream from the laptop
(`harthikxps`, 192.168.1.12) — it has ROS2 Jazzy and `rqt_image_view` already.
Both machines must be on the same network **and** the same `ROS_DOMAIN_ID`
(both unset = domain 0 = fine). Nothing from `camera_ws` needs sourcing on the
laptop: the topics are plain `sensor_msgs/Image`.

On the laptop:
```bash
source /opt/ros/jazzy/setup.bash
ros2 topic list | grep -i image     # confirm the Pi's topics are discovered
ros2 run rqt_image_view rqt_image_view
```
Pick `/camera/image_raw` from the dropdown at the top left.

**Use the compressed transport over Wi-Fi.** Raw 800×600 RGB at 30 fps is
~43 MB/s, which Wi-Fi will not carry — the window will stutter or stay black.

Do **not** type `/camera/image_raw/compressed` into the topic box. That box is
editable, and a typed name subscribes to the literal topic instead of using the
compressed transport; `image_transport` then warns "you are trying to subscribe
directly to a transport-specific image topic". Pick the entry from the dropdown
list instead (hit the refresh button left of the box if it isn't listed) — the
list entries carry the transport hidden alongside the topic name.

The dependable way, which also keeps the decompression off the Wi-Fi hop, is to
run a republisher on the laptop and point `rqt_image_view` at its local output:
```bash
ros2 run image_transport republish compressed raw \
  --ros-args -r in/compressed:=/camera/image_raw/compressed \
             -r out:=/camera/image_raw_local
```
Then select `/camera/image_raw_local` (raw) in `rqt_image_view`. Only the
compressed stream crosses the network.

If `ros2 topic list` shows no `/camera/image_raw/compressed`, the plugins are
missing on the Pi: `sudo apt install ros-jazzy-image-transport-plugins`, then
restart the camera node. Confirm the laptop side with
`ros2 run image_transport list_transports` (it should list
`image_transport/compressed` — it does on `harthikxps`).

`Wayland does not support QWindow::requestActivate()` is harmless — it just
means the window doesn't auto-focus.

If you do have a monitor or VNC on the Pi, the same `rqt_image_view` command
works there directly and skips the network entirely.

## Shutting down

```bash
ros2 lifecycle set /esp32_bridge_node deactivate   # stop listening, keep ESP32 connection open
ros2 lifecycle set /esp32_bridge_node cleanup       # close the ESP32 connection
```
or just Ctrl-C the node in terminal A — `on_shutdown` tears down the subscription and the
ESP32 connection either way.

## Troubleshooting

- **`ros2 run` says package not found**: you skipped step 4 (source the workspace) in that terminal.
- **Node builds/runs old behavior after editing code**: you skipped step 3 (rebuild) — the
  installed copy under `ros2_ws/install/` is a separate copy from `ros2_ws/src/`, it doesn't
  auto-update.
- **`configure` fails / can't connect to ESP32**: confirm `ping 192.168.1.100` works first: it
  isolates the problem to Wi-Fi/wiring vs. ROS2. If the ping fails, check the RPi is actually on
  the ESP32's network (`nmcli -t -f NAME,DEVICE con show --active` should show `TP-Link_08F3`) —
  it can't reach the ESP32 from the Airtel network. See step 0.
- **Can't SSH to the RPi / don't know its IP**: it's static at `192.168.1.17` on both networks
  (step 0), but you must be on the same network as it. To find it after a network change, sweep
  the subnet from another machine — the RPi's MAC starts `2c:cf:67` (Raspberry Pi Ltd):
  ```bash
  nmap -sn 192.168.1.0/24 && ip neigh show
  ```
  `harthikpi.local` is unreliable — mDNS resolution isn't installed on every machine.
- **A node exits at startup with `RobotModelError`**: it refused to drive the arm because it
  can't prove which arm it is. Read the message — it names the cause: unmeasured joint limits or
  tool offset in `robot_model.yaml`, an `active_model` with no matching entry, a missing or
  mismatched `geometry_file`, or a geometry file whose joint chain is out of order. See step 3a.
  This is fail-closed on purpose. If you must get past it for bench bring-up, set that model's
  `limits_source` to something other than `measured` so every node prints a warning banner at
  startup — never let copied-down numbers pass as measured ones.
- **`servo bus fault - no joint answered a position read`**: every joint came back silent, which
  is a dead bus rather than four simultaneous stalls — a stalled servo still answers a position
  read. The ESP32's `ack ok` does not contradict this; it only means the command reached the
  ESP32. Run step 2a.
- **Moves stopped working after a firmware change**: the `move`/`servo` wire format must match on
  both sides — if you edited `esp32_bridge_node.py`'s protocol without reflashing the ESP32 (or
  vice versa), reflash per step 1.
- **Continuous sweep is jerky or lags the plan**: `continuous_trajectory_node`'s
  `stream_period_sec` and `esp32_bridge_node`'s `servo_move_duration_ms` should match, and neither
  should be faster than the servo bus can keep up with (~12 Hz). Raise both (slower stream) if the
  arm can't follow.
