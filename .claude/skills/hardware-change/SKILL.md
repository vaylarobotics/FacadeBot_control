---
name: hardware-change
description: Checklist for any edit to esp32_bridge, robot_model.yaml, joint centres, or kinematics limits. Use before, during, and after such a change so nothing reaches the arm unchecked.
---

# Hardware-path change

Use this whenever a change touches `ros2_ws/src/esp32_bridge/`,
`facadebot_description/config/robot_model.yaml`, `_JOINT_CENTER_RAD`, or the
limit handling in `facade_control/kinematics.py`. For firmware edits use
`esp32-firmware-change` instead; it includes this list.

## Before writing

1. Read `STATUS.md` for the current hardware state and open blockers.
2. Say in plain terms what will change and why, and wait for the user if it is a
   new node, a new interface, or a safety-behaviour change.
3. State which of these the change affects: the bounds gate, the degree↔raw
   conversion, the joint centres, the wire protocol, a timeout. Each has a package
   `CLAUDE.md` invariant; quote the one that applies.

## While writing

- Limits stay defined once, in `robot_model.yaml`. No copies.
- Any constant that must match the firmware or another node is changed on both
  sides in the same edit, with the "must match" comment kept.
- New numbers get a named constant with the source (datasheet, measurement with a
  date, empirical test).

## After writing

1. Rebuild and run the package tests (each package separately):
   ```bash
   cd ros2_ws && source /opt/ros/jazzy/setup.bash
   colcon build --packages-select facade_msgs facadebot_description esp32_bridge facade_control
   source install/setup.bash
   python3 -m pytest src/facade_control/test -q -k 'not flake8 and not pep257 and not copyright' -p no:cacheprovider
   python3 -m pytest src/esp32_bridge/test -q -k 'not flake8 and not pep257 and not copyright' -p no:cacheprovider
   ```
   The flake8/pep257 tests fail at HEAD by design; do not report them.
2. Run the `safety-reviewer` agent on the diff.
3. Update `esp32_bridge/README.md` if the protocol or an interface changed, and
   `LAUNCH.md` if how anything is run changed.
4. Tell the user which bench check is still required. The minimum is `LAUNCH.md`
   step 7a (the `[0,0,0,0]` home move) followed by a `read_joint_positions` call
   showing all four joints. Do not describe the change as working until they
   report that.
5. Note in `STATUS.md` what changed and that it is unverified on hardware until the
   user says otherwise.
