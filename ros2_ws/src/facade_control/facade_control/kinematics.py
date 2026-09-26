"""Forward and inverse kinematics for the arm named by robot_model.yaml.

No geometry is hardcoded here. Call configure() (or load_active_model()) with a
RobotModel before using this module; every length, origin and axis then comes
from the geometry file that model selects (generated from its URDF), and the
joint limits from its config entry.

The solver is closed-form, and it only works for this arm's shape: a base
rotation, two rotations about parallel axes (shoulder and elbow, which sweep a
common plane), and a wrist whose axis leaves that plane. configure() checks the
model really has that shape and refuses it otherwise, so a regenerated model with
a different structure fails loudly instead of producing plausible-looking
nonsense.

The fourth task-space number, tool_angle_deg, is joint_4's own angle - how far
the wrist tips the tool out of the shoulder/elbow plane. On the previous arm
joint_4 lay in that plane and the same number meant the tool's pitch; the
solver below reduces to that older behaviour when the wrist axis happens to be
parallel to the elbow's, so v1 still solves correctly.
"""

import math

from facadebot_description.robot_model import RobotModel, RobotModelError

# SolidWorks writes sub-millimetre asymmetries into origins that are nominally
# zero (URDF_V2 has 0.14-0.89 mm of them). One LX-16A position count is 0.24
# deg, about 1.25 mm of tool travel at full reach, so anything under a
# millimetre is below what the arm can resolve or repeat. Such components are
# snapped to zero so the closed form stays exact; anything larger is real
# geometry this solver cannot represent, and configure() rejects it.
_STRUCTURAL_TOLERANCE_M = 0.001

# Axes are unit vectors in the URDF, so a dot product this close to +/-1 means
# parallel. Slack is for the 5-decimal rounding in the export.
_AXIS_PARALLEL_TOLERANCE = 1e-4

# How closely a candidate solution's forward kinematics must reproduce the
# requested target to be trusted. The closed form is exact, so this only ever
# catches a candidate that landed on the wrong branch.
_REACHABILITY_TOLERANCE_M = 1e-4
_REACHABILITY_TOLERANCE_DEG = 0.05

# A fully extended (or fully folded) arm puts the elbow's cos at exactly +/-1.
# Forward kinematics computed in floating point hands that back as 1 + ~1e-16,
# which a strict > 1.0 test rejects as unreachable. Anything inside this slack
# is treated as exactly on the boundary; anything beyond it is really out of reach.
_COS_INTERIOR_EPSILON = 1e-9

_JOINT_COUNT = 4

_FULL_TURN_DEG = 360.0
_HALF_TURN_DEG = 180.0


class NotReachableError(Exception):
    """No joint-angle solution reaches the target within the arm's safe range."""


class ModelStructureError(RobotModelError):
    """The model's kinematic shape is not the one this closed-form solver handles."""


_IDENTITY_MAT3 = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))


def _mat_mul(a, b):
    return tuple(
        tuple(sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3))
        for i in range(3)
    )


def _mat_vec(a, v):
    return tuple(sum(a[i][k] * v[k] for k in range(3)) for i in range(3))


def _transpose(a):
    return tuple(tuple(a[j][i] for j in range(3)) for i in range(3))


def _vec_add(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _vec_sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _rotx(theta_rad):
    c, s = math.cos(theta_rad), math.sin(theta_rad)
    return ((1.0, 0.0, 0.0), (0.0, c, -s), (0.0, s, c))


def _roty(theta_rad):
    c, s = math.cos(theta_rad), math.sin(theta_rad)
    return ((c, 0.0, s), (0.0, 1.0, 0.0), (-s, 0.0, c))


def _rotz(theta_rad):
    c, s = math.cos(theta_rad), math.sin(theta_rad)
    return ((c, -s, 0.0), (s, c, 0.0), (0.0, 0.0, 1.0))


def _rpy_to_matrix(roll_rad, pitch_rad, yaw_rad):
    # URDF convention: fixed-axis roll-pitch-yaw, R = Rz(yaw) @ Ry(pitch) @ Rx(roll).
    return _mat_mul(_rotz(yaw_rad), _mat_mul(_roty(pitch_rad), _rotx(roll_rad)))


def _rotation_about_axis(axis, theta_rad):
    x, y, z = axis
    c, s = math.cos(theta_rad), math.sin(theta_rad)
    one_minus_c = 1.0 - c
    return (
        (c + x * x * one_minus_c, x * y * one_minus_c - z * s, x * z * one_minus_c + y * s),
        (y * x * one_minus_c + z * s, c + y * y * one_minus_c, y * z * one_minus_c - x * s),
        (z * x * one_minus_c - y * s, z * y * one_minus_c + x * s, c + z * z * one_minus_c),
    )


def _normalize_deg(angle_deg: float) -> float:
    """Fold an angle into (-180, 180].

    atan2 and acos can hand back a candidate a full turn away from the
    physically identical angle - -265 deg instead of +95 deg. Both drive the
    servo to the same place, but only one survives a bounds-check against
    limits measured within a half turn of centre, so this has to happen before
    _first_limit_violation sees the candidate.
    """
    folded_deg = math.fmod(angle_deg, _FULL_TURN_DEG)
    if folded_deg > _HALF_TURN_DEG:
        folded_deg -= _FULL_TURN_DEG
    elif folded_deg <= -_HALF_TURN_DEG:
        folded_deg += _FULL_TURN_DEG
    return folded_deg


def _snap_small(value_m: float) -> float:
    return 0.0 if abs(value_m) < _STRUCTURAL_TOLERANCE_M else value_m


def _snap_vector(vector, context: str) -> tuple[float, float, float]:
    return tuple(_snap_small(component) for component in vector)


class _Geometry:
    """The model's numbers, pre-arranged into what the closed form needs."""

    def __init__(self, model: RobotModel) -> None:
        self.model = model
        joints = model.joints

        origins = [_snap_vector(joint.origin_xyz_m, joint.name) for joint in joints]
        rotations = [_rpy_to_matrix(*joint.origin_rpy_rad) for joint in joints]
        axes = [joint.axis for joint in joints]

        self.origins = origins
        self.rotations = rotations
        self.axes = axes
        self.tool_tip_offset_m = _snap_vector(model.tool_tip_offset_m, "tool tip offset")

        # Sign of each of the first three joints' rotation about its own frame's
        # z. The closed form is written in terms of z-rotations; a joint declared
        # axis="0 0 -1" (URDF_V2's joint_1) just runs the solution backwards.
        self.spin = [1.0 if axis[2] > 0.0 else -1.0 for axis in axes[:3]]

        self.base_rotation = rotations[0]
        self.shoulder_rotation = rotations[1]

        # Shoulder point. Independent of joint_1 because joint_2's origin lies on
        # joint_1's rotation axis (checked in _validate_structure).
        self.shoulder_m = _vec_add(origins[0], _mat_vec(rotations[0], origins[1]))

        # Upper arm: shoulder to elbow, as a length and a phase inside the
        # shoulder/elbow plane.
        self.upper_arm_m = math.hypot(origins[2][0], origins[2][1])
        self.upper_arm_phase_rad = math.atan2(origins[2][1], origins[2][0])

    def tool_in_elbow_frame(self, wrist_angle_rad: float) -> tuple[float, float, float]:
        """Tool tip in link_3's frame - a fixed point once joint_4's angle is set.

        This is what makes the closed form possible: with joint_4 chosen, the
        wrist and tool collapse into one rigid vector hanging off the elbow.
        """
        wrist_rotation = _mat_mul(
            self.rotations[3], _rotation_about_axis(self.axes[3], wrist_angle_rad))
        return _vec_add(self.origins[3], _mat_vec(wrist_rotation, self.tool_tip_offset_m))


def _validate_structure(geometry: _Geometry) -> None:
    """Refuse any model whose shape breaks an assumption in inverse_kinematics()."""
    model = geometry.model
    if len(model.joints) != _JOINT_COUNT:
        raise ModelStructureError(
            f"model '{model.name}': this solver handles {_JOINT_COUNT} joints, "
            f"the URDF declares {len(model.joints)}")

    for index in range(3):
        axis = geometry.axes[index]
        if abs(abs(axis[2]) - 1.0) > _AXIS_PARALLEL_TOLERANCE:
            raise ModelStructureError(
                f"model '{model.name}': {model.joints[index].name} must rotate about "
                f"its own frame's z axis, but declares axis {axis}")

    # joint_2's origin has to sit on joint_1's rotation axis, or the shoulder
    # would swing as the base turns and the arm would no longer be a simple
    # yaw-then-planar chain.
    shoulder_offset = geometry.origins[1]
    if math.hypot(shoulder_offset[0], shoulder_offset[1]) > 0.0:
        raise ModelStructureError(
            f"model '{model.name}': {model.joints[1].name} origin {shoulder_offset} is "
            f"{math.hypot(shoulder_offset[0], shoulder_offset[1]) * 1000:.2f} mm off "
            f"{model.joints[0].name}'s rotation axis; this solver needs it on the axis")

    # The elbow must stay in the plane the shoulder sweeps: no z offset, and no
    # rotation between link_2 and link_3.
    if geometry.origins[2][2] != 0.0:
        raise ModelStructureError(
            f"model '{model.name}': {model.joints[2].name} origin has a z component "
            f"({geometry.origins[2][2] * 1000:.2f} mm); shoulder and elbow must be coplanar")
    if geometry.rotations[2] != _IDENTITY_MAT3:
        raise ModelStructureError(
            f"model '{model.name}': {model.joints[2].name} declares a non-zero origin rpy "
            f"{model.joints[2].origin_rpy_rad}; shoulder and elbow must share a plane")

    if geometry.upper_arm_m <= 0.0:
        raise ModelStructureError(
            f"model '{model.name}': {model.joints[2].name} sits on top of "
            f"{model.joints[1].name}; the upper arm has no length")

    # The tool tip must be off the wrist axis, otherwise joint_4 spins the tool
    # in place and the fourth task-space number controls nothing.
    tool_in_elbow = geometry.tool_in_elbow_frame(0.0)
    if math.hypot(tool_in_elbow[0], tool_in_elbow[1]) <= 0.0:
        raise ModelStructureError(
            f"model '{model.name}': the tool tip lies on {model.joints[3].name}'s "
            "rotation axis, so that joint cannot move it")


_geometry: _Geometry | None = None


def configure(model: RobotModel) -> None:
    """Point this module at an arm model. Raises ModelStructureError if unsuitable."""
    global _geometry
    geometry = _Geometry(model)
    _validate_structure(geometry)
    _geometry = geometry


def load_active_model() -> RobotModel:
    """Load the model named by robot_model.yaml and configure this module with it."""
    from facadebot_description.robot_model import load_robot_model

    model = load_robot_model()
    configure(model)
    return model


def _require_geometry() -> _Geometry:
    if _geometry is None:
        raise RobotModelError(
            "kinematics used before configure()/load_active_model(); no arm model is "
            "loaded, so there is no geometry to solve against")
    return _geometry


def active_model() -> RobotModel:
    return _require_geometry().model


def joint_limits_deg() -> tuple[tuple[float, float], ...]:
    return _require_geometry().model.joint_limits_deg


def forward_kinematics(theta1_deg: float, theta2_deg: float, theta3_deg: float,
                       theta4_deg: float) -> tuple[float, float, float, float]:
    """Joint angles (degrees) -> tool tip (x_m, y_m, z_m, tool_angle_deg).

    tool_angle_deg is joint_4's angle: the tool's tilt out of the shoulder/elbow
    plane. It comes back unchanged, which is what makes this exactly invertible
    by inverse_kinematics().
    """
    geometry = _require_geometry()
    angles_rad = [math.radians(angle_deg)
                  for angle_deg in (theta1_deg, theta2_deg, theta3_deg, theta4_deg)]

    rotation = _IDENTITY_MAT3
    position_m = (0.0, 0.0, 0.0)
    for index, angle_rad in enumerate(angles_rad):
        position_m = _vec_add(position_m, _mat_vec(rotation, geometry.origins[index]))
        rotation = _mat_mul(rotation, _mat_mul(
            geometry.rotations[index],
            _rotation_about_axis(geometry.axes[index], angle_rad)))

    tip_m = _vec_add(position_m, _mat_vec(rotation, geometry.tool_tip_offset_m))
    return (tip_m[0], tip_m[1], tip_m[2], theta4_deg)


def _first_limit_violation(angles_deg) -> str | None:
    for index, (angle_deg, (lower_deg, upper_deg)) in enumerate(
            zip(angles_deg, joint_limits_deg())):
        if angle_deg < lower_deg or angle_deg > upper_deg:
            name = _require_geometry().model.joints[index].name
            return (f"{name} would need {angle_deg:.1f} deg, outside its safe range "
                    f"({lower_deg:.1f} to {upper_deg:.1f} deg)")
    return None


def _forward_kinematics_matches(angles_deg, target) -> bool:
    x_m, y_m, z_m, tool_angle_deg = forward_kinematics(*angles_deg)
    target_x_m, target_y_m, target_z_m, target_tool_angle_deg = target
    return (
        abs(x_m - target_x_m) < _REACHABILITY_TOLERANCE_M
        and abs(y_m - target_y_m) < _REACHABILITY_TOLERANCE_M
        and abs(z_m - target_z_m) < _REACHABILITY_TOLERANCE_M
        and abs(tool_angle_deg - target_tool_angle_deg) < _REACHABILITY_TOLERANCE_DEG
    )


def _base_angle_candidates_rad(geometry: _Geometry, target_m,
                               tool_in_elbow, fallback_base_rad: float) -> list[float]:
    """Angles for joint_1 that put the target the right distance out of the arm plane.

    Turning the base cannot change how far the tool tip sits off the
    shoulder/elbow plane - only joint_4 can - so matching that one distance
    pins joint_1 down to at most two choices.

    A target sitting on joint_1's own axis (the straight-up home pose is one) is
    reached at every base angle, so there is nothing to solve for: the base
    simply stays where it is, which is what fallback_base_rad carries in.
    """
    to_target = _vec_sub(target_m, geometry.shoulder_m)
    in_base = _mat_vec(_transpose(geometry.base_rotation), to_target)
    plane_normal = tuple(geometry.shoulder_rotation[row][2] for row in range(3))

    cos_coefficient = plane_normal[0] * in_base[0] + plane_normal[1] * in_base[1]
    sin_coefficient = plane_normal[0] * in_base[1] - plane_normal[1] * in_base[0]
    required = tool_in_elbow[2] - plane_normal[2] * in_base[2]

    magnitude = math.hypot(cos_coefficient, sin_coefficient)
    if magnitude < _REACHABILITY_TOLERANCE_M:
        if abs(required) < _REACHABILITY_TOLERANCE_M:
            return [fallback_base_rad]
        return []
    if abs(required) > magnitude:
        return []

    phase_rad = math.atan2(sin_coefficient, cos_coefficient)
    offset_rad = math.acos(max(-1.0, min(1.0, required / magnitude)))
    return [phase_rad + offset_rad, phase_rad - offset_rad]


def _elbow_solutions_rad(geometry: _Geometry, in_plane, tool_in_elbow):
    """Shoulder/elbow angles placing the tool at a point in their shared plane."""
    forearm_m = math.hypot(tool_in_elbow[0], tool_in_elbow[1])
    forearm_phase_rad = math.atan2(tool_in_elbow[1], tool_in_elbow[0])
    reach_m = math.hypot(in_plane[0], in_plane[1])

    cos_interior = ((reach_m * reach_m - geometry.upper_arm_m ** 2 - forearm_m ** 2)
                    / (2.0 * geometry.upper_arm_m * forearm_m))
    if abs(cos_interior) > 1.0 + _COS_INTERIOR_EPSILON:
        return []

    interior_rad = math.acos(max(-1.0, min(1.0, cos_interior)))
    direction_rad = math.atan2(in_plane[1], in_plane[0])

    solutions = []
    for elbow_rad in (interior_rad, -interior_rad):
        droop_rad = math.atan2(forearm_m * math.sin(elbow_rad),
                               geometry.upper_arm_m + forearm_m * math.cos(elbow_rad))
        shoulder_rad = direction_rad - geometry.upper_arm_phase_rad - droop_rad
        solutions.append((
            shoulder_rad / geometry.spin[1],
            (elbow_rad - forearm_phase_rad + geometry.upper_arm_phase_rad) / geometry.spin[2],
        ))
    return solutions


def inverse_kinematics(
    x_m: float,
    y_m: float,
    z_m: float,
    tool_angle_deg: float,
    current_angles_deg: tuple[float, float, float, float] | None = None,
) -> tuple[float, float, float, float]:
    """Tool-tip target -> joint angles (degrees).

    tool_angle_deg is joint_4's angle: how far the wrist tips the tool out of the
    shoulder/elbow plane. Pass 0 to keep the tool in that plane.

    Several joint configurations can reach the same target (the base can face the
    target from either side, and the elbow can bend either way). When
    current_angles_deg is given, the configuration needing the least total joint
    travel from there is returned, so the arm doesn't flip between configurations
    on nearby targets.
    """
    geometry = _require_geometry()
    wrist_angle_rad = math.radians(tool_angle_deg)
    tool_in_elbow = geometry.tool_in_elbow_frame(wrist_angle_rad)

    target_m = (x_m, y_m, z_m)
    target = (x_m, y_m, z_m, tool_angle_deg)

    # Only consulted when the target is on the base axis (see
    # _base_angle_candidates_rad). Joint angles are stored relative to the
    # joint's own spin direction, so undo that to get the solver's internal angle.
    if current_angles_deg is None:
        fallback_base_rad = 0.0
    else:
        fallback_base_rad = math.radians(current_angles_deg[0]) * geometry.spin[0]

    candidates = []
    for base_rad in _base_angle_candidates_rad(
            geometry, target_m, tool_in_elbow, fallback_base_rad):
        to_target = _vec_sub(target_m, geometry.shoulder_m)
        in_base = _mat_vec(_transpose(geometry.base_rotation), to_target)
        in_plane = _mat_vec(_transpose(geometry.shoulder_rotation),
                            _mat_vec(_rotz(-base_rad), in_base))

        for shoulder_rad, elbow_rad in _elbow_solutions_rad(
                geometry, in_plane, tool_in_elbow):
            candidates.append((
                _normalize_deg(math.degrees(base_rad / geometry.spin[0])),
                _normalize_deg(math.degrees(shoulder_rad)),
                _normalize_deg(math.degrees(elbow_rad)),
                tool_angle_deg,
            ))

    if not candidates:
        raise NotReachableError(
            f"target ({x_m:.3f}, {y_m:.3f}, {z_m:.3f}) m at tool angle "
            f"{tool_angle_deg:.1f} deg is outside the arm's reach")

    valid = [angles for angles in candidates if _forward_kinematics_matches(angles, target)]
    if not valid:
        raise NotReachableError(
            f"no solution reproduces target ({x_m:.3f}, {y_m:.3f}, {z_m:.3f}) m at tool "
            f"angle {tool_angle_deg:.1f} deg")

    in_range = [angles for angles in valid if _first_limit_violation(angles) is None]
    if not in_range:
        raise NotReachableError(
            f"target ({x_m:.3f}, {y_m:.3f}, {z_m:.3f}) m at tool angle "
            f"{tool_angle_deg:.1f} deg is geometrically reachable but needs a joint "
            f"outside its safe range: {_first_limit_violation(valid[0])}")

    if current_angles_deg is None:
        return in_range[0]

    def travel_deg(angles) -> float:
        return sum(abs(angle_deg - current_deg)
                   for angle_deg, current_deg in zip(angles, current_angles_deg))

    return min(in_range, key=travel_deg)
