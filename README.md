# FacadeBot Control

Control stack for FacadeBot, a 4-joint servo arm for building facade work
(painting, cleaning, inspection). A Raspberry Pi runs ROS2 Jazzy; an ESP32 drives
the LX-16A servo bus over a Hiwonder BusLinker; the two talk over Wi-Fi/TCP.

| Read this | For |
|-----------|-----|
| `LAUNCH.md` | Every step from powered-off to each capability, plus troubleshooting |
| `STATUS.md` | What is done, what is verified on the physical arm, what is blocked |
| `DECISIONS.md` | Open robotics and architecture decisions, with options, waiting on an answer |
| `TESTS.md` | Every automated test in plain language, plus what is not tested |
| `CLAUDE.md` | Conventions, safety rules and the map of the repo (also read by Claude Code) |
| `CODE_REVIEW_STRUCTURE_2026-09-18.md` | Where the current design will fail or fall behind, ranked |

Layout:

```
esp32_firmware/main.py          MicroPython on the ESP32 - the only code that touches servos
ros2_ws/src/esp32_bridge/       ROS2 <-> ESP32 transport and the mandatory joint bounds gate
ros2_ws/src/facade_control/     Kinematics, Cartesian moves, trajectory following, /joint_states
ros2_ws/src/facade_msgs/        Custom messages, services, actions
ros2_ws/src/facadebot_description/  URDFs, generated geometry, measured limits
scripts/                        Bench and design-time tools
```

There is no emergency stop yet. Starting the bridge makes the arm live.
