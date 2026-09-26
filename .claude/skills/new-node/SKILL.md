---
name: new-node
description: The house recipe for adding a ROS2 node, service, action, or topic to FacadeBot. Use whenever a new node or interface is proposed, before any code is written.
---

# New ROS2 node or interface

## 1. Ask first

Before writing anything, put to the user in plain terms:
- What the node will do, in one paragraph.
- Which package it belongs in and why (`esp32_bridge` only if it must touch the
  transport; `facade_control` for planning and motion; a new package only if
  neither fits, and say so).
- Its interfaces: topics, services, actions, parameters, with names under
  `/facade_bot/` and units in every field name.
- Who commands the arm as a result. If the answer adds a second path into
  `esp32_bridge`, stop and raise it: the project already has an unarbitrated
  `joint_cmd` / `joint_stream` split (structural finding 2).
- Any robotics decision embedded in it (tolerance, timing, what "done" means).
  Offer options; do not pick.

Wait for the answer. Explain any new concept in one sentence the first time.

## 2. Write it

- One file, one node class, `rclpy.node.Node` or the lifecycle node if it touches
  hardware directly.
- Constants at the top with sources. No magic numbers.
- Parameters in `__init__` via `declare_parameter` with defaults and descriptions.
- Logging through `self.get_logger()`. Startup refusals before the node exists use
  `rclpy.logging.get_logger(...)`.
- If it must block on another node's service, note that four nodes already do this
  with a helper node and executor. Prefer extracting a shared helper over a fifth
  copy.
- If it needs the arm's current state, query `esp32_bridge`'s read service each
  time. Do not cache.
- Loads the arm model through `facadebot_description.robot_model` if it needs any
  geometry or limits, and fails closed on `RobotModelError`.
- Message types come from `facade_msgs`; new ones go there, rebuilt first.

## 3. Wire it in

- `setup.py` `console_scripts` entry: `name = package.module:main`.
- `package.xml` dependencies if a new one appeared.
- `bringup.launch.py` only if the user wants it started by default. Do **not** pass
  `name=` for a node that creates a helper node (it renames the helper too).
- Rebuild: `colcon build --packages-select <package>`.

## 4. Test it

- Pure logic in its own module with pytest tests, following `test_kinematics.py`.
- The node against stubbed services, following `test_trajectory_node.py`.
- Run the package's tests on their own, skipping the lint tests.

## 5. Document it

- A numbered `LAUNCH.md` step: command block, prerequisites, one line on when to
  use it. Same turn, unprompted.
- The package README's interface table.
- A row in `STATUS.md`, marked verified in tests only until the user runs it on
  the arm.
