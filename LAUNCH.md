# Launching FacadeBot

Steps and commands to get the arm from powered-off to accepting joint commands.
Run the RPi/ROS2 steps on the Raspberry Pi; the ESP32 flashing step can be run
from whichever machine has the ESP32 plugged in over USB.

## 1. Flash the ESP32 (only needed after `esp32_firmware/main.py` changes)

The board needs to be connected over USB for this step — Wi-Fi isn't up yet
until the new code is running.

```bash
/home/harthik/.firmware/bin/mpremote connect /dev/ttyUSB0 fs cp esp32_firmware/main.py :main.py
/home/harthik/.firmware/bin/mpremote connect /dev/ttyUSB0 reset
```

If `/dev/ttyUSB0` doesn't exist, find the actual port with:
```bash
ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
```

Optional: watch it boot and confirm it joins Wi-Fi with no errors:
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

## 3. Build the ROS2 workspace (only needed after code changes under `ros2_ws/`)

```bash
cd /home/harthik/FacadeBot_control/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select facade_msgs esp32_bridge facade_control
```

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
ros2 action send_goal /facade_bot/follow_trajectory facade_msgs/action/FollowTrajectory \
  "{waypoints: [{x_m: 0.2, y_m: 0.0, z_m: 0.2, tool_angle_deg: 0.0}, {x_m: 0.2, y_m: 0.05, z_m: 0.2, tool_angle_deg: 0.0}]}" \
  --feedback
```

Moves through each waypoint in order, stopping fully at each one before the
next (no blending yet). Ctrl-C the `send_goal` call to cancel — this stops
further waypoints from being commanded, but does **not** stop the arm
mid-move (no e-stop primitive exists yet). See
`ros2_ws/src/facade_control/README.md`'s "Following a trajectory" section
for the feedback/result fields and known limitations.

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
  isolates the problem to Wi-Fi/wiring vs. ROS2.
- **Moves stopped working after a firmware change**: the `move` wire format must match on both
  sides — if you edited `esp32_bridge_node.py`'s protocol without reflashing the ESP32 (or vice
  versa), reflash per step 1.
