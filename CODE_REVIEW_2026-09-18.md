# Code Review — 2026-09-18

Scope: full repository at HEAD (commit `c751abc`, "Migrate control stack to rebuilt
v2 arm; add robot_state_publisher/RViz"). Review only — no code was modified.
Model used for the initial pass is unconfirmed (see note at bottom); all findings
below were independently re-verified against the actual files by the reviewing
session before being reported.

10 findings, ranked most severe first.

---

## 1. `_JOINT_CENTER_RAD` calibration status is unclear — contradicts CLAUDE.md

**File:** `ros2_ws/src/esp32_bridge/esp32_bridge/esp32_bridge_node.py:53`

In the `c751abc` commit, joint_1's value in `_JOINT_CENTER_RAD` was changed from
`2.09` (v1's value) to `2.17` rad, while joints 2-4 stayed at v1's values
(`2.25, 2.05, 2.25`). At the same time, a comment was added marking the **whole
array** stale:

> STALE: measured on the v1 arm. The arm has since been rebuilt (URDF_V2), so
> every one of these has to be re-read off the physical joints before the arm
> is commanded.

This directly contradicts CLAUDE.md's claim that joint centers were "re-checked
on the rebuilt v2 arm and found unchanged from v1 (no code edit needed)" — the
diff itself shows joint_1 *did* change. It's unclear whether joint_1 actually
was re-measured on the physical v2 arm and the blanket STALE comment just
wasn't reconciled, or something else is going on.

**Why it matters:** `_JOINT_CENTER_RAD` is the calibration anchor — commanded
0° means this value, for every joint, on every move and every bounds-check
margin. If this is wrong, every commanded angle is silently offset from the
true joint center, with nothing in the code path flagging it beyond a
comment.

**Action needed:** re-verify all four joint centers against the physical arm
before trusting any move. Do not trust CLAUDE.md's current claim or the STALE
comment blindly — check the hardware.

---

## 2. `tool_angle_deg` documentation is stale in two message files

**Files:** `ros2_ws/src/facade_msgs/srv/MoveToPose.srv:4`,
`ros2_ws/src/facade_msgs/msg/Waypoint.msg:4`

Both still document v1 semantics: "tool pitch, degrees from horizontal... in
the arm's vertical reach-plane." The v2 `kinematics.py` rewrite redefines
`tool_angle_deg` as joint_4's own angle — tilt *out of* the arm plane, not
pitch *within* it.

**Failure scenario:** a human calling the service via `ros2 interface show`,
or the raster/pattern generator planned as next work, sets `tool_angle_deg`
under the documented pitch model. It actually solves under the new
out-of-plane model, tilting the tool sideways instead of pitching it —
moving the tool tip to an unintended physical position.

---

## 3. joint_4 angle skips normalization before the bounds-check

**File:** `ros2_ws/src/facade_control/facade_control/kinematics.py:404`

`inverse_kinematics()` normalizes joints 1-3 into `(-180, 180]` before the
bounds-check, but appends the caller-supplied `tool_angle_deg` (joint_4) into
the candidate tuple completely unnormalized.

**Failure scenario:** a caller passes `tool_angle_deg=250.0` (physically
equivalent to `-110.0`, exactly at joint_4's measured limit). `_first_limit_violation`
does a plain `angle_deg < lower or angle_deg > upper` with no mod-360 folding,
so a geometrically reachable pose is spuriously rejected whenever the caller's
value happens to land outside ±180.

---

## 4. Generated geometry's source hash is written but never verified

**File:** `ros2_ws/src/facadebot_description/facadebot_description/robot_model.py:128`

`scripts/generate_geometry.py` computes and writes `source_urdf_sha256` into
every `geometry_<model>.yaml`, but `robot_model.py`'s `_load_geometry()` only
reads the `source_urdf` path string back — it never reads or checks the
sha256 field. `kinematics.py`'s `_validate_structure` only checks kinematic
topology (axis directions, coplanarity), not actual numeric link
lengths/origins.

**Failure scenario:** `URDF_V2.urdf` is re-exported from SolidWorks with a
dimension-only change (same topology) but `scripts/generate_geometry.py`
isn't re-run first. `load_robot_model()` silently loads the stale
`geometry_v2.yaml`; every downstream IK/FK computation runs against wrong
link lengths with no exception raised anywhere — despite `robot_model.py`'s
own stated design goal that "a node that cannot prove which arm it is
driving must not drive it."

---

## 5. Bounds-check logic is duplicated across two files

**Files:** `ros2_ws/src/facade_control/facade_control/kinematics.py:294`,
`ros2_ws/src/esp32_bridge/esp32_bridge/esp32_bridge_node.py:113-124`

`_first_limit_violation` (kinematics.py) re-implements the exact same
comparison (`angle_deg < lower or angle_deg > upper`) as `_check_joint_bounds`
(esp32_bridge_node.py), both reading the same `robot_model.yaml`-derived
limits but with the comparison written twice — against CLAUDE.md's
non-negotiable rule that "the bounds-check function must be the single place
where limits are defined."

**Failure scenario:** semantics match today (both inclusive), but if bounds
semantics ever change (e.g. adding a margin/deadband, or
inclusive→exclusive), a fix applied to the hardware-facing gate could be
missed in the reachability check, letting IK report a pose valid that the
hardware gate would actually reject, or vice versa.

---

## 6. Duplicate joint names would slip past validation

**File:** `ros2_ws/src/facadebot_description/facadebot_description/robot_model.py:158`

`_require_limits` validates joint-name/limits correspondence via
`set(raw_limits) != set(joint_names)`, which is blind to duplicate joint
names since `_load_geometry` never checks joint-name uniqueness.
`esp32_bridge_node.py` and `joint_state_publisher_node.py` both call
`load_robot_model()` directly, bypassing `kinematics.py`'s stricter
`_validate_structure`.

**Failure scenario:** a malformed/mis-generated `geometry_v2.yaml` with a
duplicated joint name (e.g. joint_2 listed where joint_3 should be, chain
still unbroken) passes if the resulting *set* of names still matches
`robot_model.yaml`'s limit keys — assigning joint_2's limits to both slots
with no joint_3 in the resulting model, and the hardware-facing bounds gate
never invokes the one validator that would catch it.

---

## 7. Startup-refusal messages use `print()` instead of the ROS2 logger

**Files:** `ros2_ws/src/esp32_bridge/esp32_bridge/esp32_bridge_node.py:416`,
`ros2_ws/src/facade_control/facade_control/facade_control_node.py:183`,
`ros2_ws/src/facade_control/facade_control/continuous_trajectory_node.py:232`,
`ros2_ws/src/facade_control/facade_control/joint_state_publisher_node.py:155`

All four `main()` functions use `print()` for the `RobotModelError` "refusing
to start" message, violating CLAUDE.md's explicit rule: "Use
`self.get_logger().info/warn/error()` — never `print()`." `rclpy.init()` has
already run at that point in every case, so `rclpy.logging.get_logger(<name>).error(...)`
is available even before a `Node` object exists.

**Failure scenario:** under `ros2 launch` (used by `bringup.launch.py` for 3
of these 4 nodes), stdout from `print()` isn't routed through ROS2's logging
system, so a startup refusal — the fail-closed safety gate this exception
handler exists for — can be missing from `ros2 log`/launch's captured output
exactly when it matters most: first bringup after a model/config change.

---

## 8. Rotation structural check has no CAD-noise tolerance

**File:** `ros2_ws/src/facade_control/facade_control/kinematics.py:215`

`_validate_structure` compares `geometry.rotations[2]` to `_IDENTITY_MAT3`
with exact float equality, while joint origin *positions* get snapped to a
tolerance (`_snap_vector`/`_snap_small`) to absorb documented SolidWorks CAD
export noise (0.14-0.89 mm). Rotations get no equivalent tolerance.

**Failure scenario:** `URDF_V2` currently exports joint_3's rpy as exactly
`[0,0,0]`, so this passes by luck. On the next SolidWorks re-export (or a
hand-edit) producing e.g. `1e-6` rad instead of exactly `0`, `configure()`/
`load_active_model()` raises `ModelStructureError` and refuses to start every
control node on a physically-correct model — a false-positive fail-closed
from a missing tolerance, and untested (no test injects noise-level rpy
values).

---

## 9. Elbow-flip regression test replaced with a weaker one

**File:** `ros2_ws/src/facade_control/test/test_kinematics.py:71`

The `c751abc` commit deleted
`test_inverse_kinematics_prefers_candidate_closest_to_current_position`
(which drove two different `current_angles_deg` values to two different
elbow-configuration candidates and explicitly asserted they differ) and
replaced it with `test_least_travel_solution_is_chosen`, which only checks
that a target solved with `current_angles_deg` equal to the target
round-trips to itself (zero travel).

**Failure scenario:** the new test passes even if least-travel candidate
selection silently degenerated to always returning the first candidate,
since zero-travel-to-self doesn't discriminate between candidates. The
specific regression this used to guard against — the elbow silently flipping
between configurations on nearby targets, a real hardware defect fixed
2026-07-15 — no longer has a regression test covering it.

---

## Note on model used

The review was launched via `/code-review high . --model fable`, but
`--model` is not a documented flag for this skill (only `--comment` and
`--fix` are). It's unconfirmed whether the Fable model was actually used for
the analysis pass; there's no tool available to inspect after the fact which
model a completed background agent ran on. All 10 findings above were
independently re-verified against the actual repository files (Sonnet 5, this
session) before being reported, regardless of what ran the first pass.
