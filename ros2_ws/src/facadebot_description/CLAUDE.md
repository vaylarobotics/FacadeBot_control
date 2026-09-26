# facadebot_description — CLAUDE.md

Which arm the stack drives, and the numbers that describe it. An `ament_python`
package so both `esp32_bridge` and `facade_control` can import the loader.

## The two kinds of numbers

| Kind | Where | How it changes |
|------|-------|----------------|
| Geometry: link offsets, joint origins, joint axes | `config/geometry_<model>.yaml`, **generated** from the URDF by `scripts/generate_geometry.py` | Re-export the URDF from SolidWorks, rerun the generator, commit both together. Never hand-edit |
| Hand-measured: joint limits, tool-tip offset, `limits_source` | `config/robot_model.yaml` | Measure on the physical arm, edit by hand, record the date |

Joint centres are the third kind and live in `esp32_bridge_node.py`
(`_JOINT_CENTER_RAD`), because they are per-servo calibration, not a property of
the model.

## Invariants

- **No node opens a URDF at runtime.** `robot_model.py` is pure YAML.
  `urdf_geometry.py` is generator-only. The one runtime URDF read is
  `display.launch.py`, for RViz, and it stays out of every control node.
- The loader is fail-closed: unknown `active_model`, missing geometry file, geometry
  file naming a different model, joint chain not base-to-tip, joint names not
  matching the limits, inverted limits, or null limits/offset all raise
  `RobotModelError` and the node exits. Do not add fallbacks.
- `limits_source: measured` is the only trusted value. Anything else prints a
  warning banner at startup; that is a warning, not a refusal, so bench bring-up can
  move an unmeasured arm. Never set `measured` on numbers that were not.
- `active_model` is the single switch. `v1` is retired and must not be selected;
  `v2` is the current arm.
- The loader hashes the source URDF and refuses a geometry file whose recorded
  `source_urdf_sha256` does not match (added 2026-09-18). Hashing is not parsing,
  so the no-URDF-at-runtime rule holds. A re-exported URDF therefore fails every
  node at startup until the generator is rerun. Duplicate joint names are refused
  too.
- Consequence for tests: any temporary package root handed to `load_robot_model`
  must contain `urdf/` as well as `config/` (see `facade_control/test/conftest.py`).

## Verify after a change

```bash
python3 scripts/generate_geometry.py --model v2      # regenerates config/geometry_v2.yaml
git diff ros2_ws/src/facadebot_description/config/   # an empty diff means geometry is current
cd ros2_ws && source /opt/ros/jazzy/setup.bash
colcon build --packages-select facadebot_description && source install/setup.bash
```
The generator has one flag, `--model`, and checks its own output round-trips
against the URDF before writing.
`test/test_robot_model.py` (4 tests) covers the hash check, a missing URDF, and
duplicate joint names:
```bash
python3 -m pytest src/facadebot_description/test -q -p no:cacheprovider
```
Then start any control node and read the banner it prints: every joint's origin,
axis and limits, plus the tool offset. That banner is the numbers in force.
