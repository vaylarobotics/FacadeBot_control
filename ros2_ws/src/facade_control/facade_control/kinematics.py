import math

# ── Arm geometry ───────────────────────────────────────────────────────────
# Transcribed from ros2_ws/src/facadebot_description/urdf/URDF_Test.urdf.
# If that URDF changes, update these and esp32_bridge_node.py's
# _JOINT_LIMITS_DEG to match.
_BASE_TO_SHOULDER_M = 0.10061    # joint_1 origin xyz z
_SHOULDER_TO_ELBOW_M = 0.12511   # joint_2->joint_3 origin xyz x
_ELBOW_TO_WRIST_M = 0.16511      # joint_3->joint_4 origin xyz x
_WRIST_TO_TOOL_TIP_M = 0.05      # measured on hardware - not in the URDF

# joint_2's <origin rpy>. This re-orients joint_2/3/4's rotation axis from
# vertical (URDF axis="0 0 1" in the joint's own local frame) to the
# physical horizontal pitch axis, and makes "all joint angles = 0" point
# the arm straight up rather than straight out - see facade_control/README.md.
# joint_1/3/4 origins have no rotation.
_JOINT_2_ORIGIN_RPY_RAD = (1.5708, -1.5708, 0.0)

# Per-joint safe range, degrees, base -> end-effector order, relative to each
# joint's own measured true center (no longer the URDF's absolute 0-240°
# range - see esp32_bridge_node.py's _JOINT_CENTER_RAD for the measured
# centers themselves). Must match esp32_bridge_node.py's _JOINT_LIMITS_DEG -
# that copy is the last-resort hardware gate; this one decides IK
# reachability before we get that far.
_JOINT_LIMITS_DEG = (
    (-110.0, 110.0),  # joint_1
    (-110.0, 110.0),  # joint_2
    (-110.0, 110.0),  # joint_3
    (-100.0, 100.0),  # joint_4: narrower, measured range
)

# How closely a candidate IK solution's forward kinematics must reproduce
# the requested target to be trusted (see _forward_kinematics_matches).
_REACHABILITY_TOLERANCE_M = 1e-4
_REACHABILITY_TOLERANCE_DEG = 0.05


class NotReachableError(Exception):
    """No joint-angle solution reaches the target within the arm's safe range."""


_IDENTITY_MAT3 = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))


def _mat_mul(a, b):
    return tuple(
        tuple(sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3))
        for i in range(3)
    )


def _mat_vec(a, v):
    return tuple(sum(a[i][k] * v[k] for k in range(3)) for i in range(3))


def _vec_add(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


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


_JOINT_2_ORIGIN_ROTATION = _rpy_to_matrix(*_JOINT_2_ORIGIN_RPY_RAD)


class _Frame:
    """A rigid transform (rotation + position) relative to base_link."""

    __slots__ = ("rot", "pos")

    def __init__(self, rot, pos):
        self.rot = rot
        self.pos = pos

    def then(self, rel_rot, rel_trans):
        return _Frame(_mat_mul(self.rot, rel_rot), _vec_add(self.pos, _mat_vec(self.rot, rel_trans)))


def _chain_frames(angles_deg):
    t1, t2, t3, t4 = (math.radians(a) for a in angles_deg)

    base = _Frame(_IDENTITY_MAT3, (0.0, 0.0, 0.0))
    f1 = base.then(_rotz(t1), (0.0, 0.0, _BASE_TO_SHOULDER_M))
    f2 = f1.then(_mat_mul(_JOINT_2_ORIGIN_ROTATION, _rotz(t2)), (0.0, 0.0, 0.0))
    f3 = f2.then(_rotz(t3), (_SHOULDER_TO_ELBOW_M, 0.0, 0.0))
    f4 = f3.then(_rotz(t4), (_ELBOW_TO_WRIST_M, 0.0, 0.0))
    tool = f4.then(_IDENTITY_MAT3, (_WRIST_TO_TOOL_TIP_M, 0.0, 0.0))
    return tool


def forward_kinematics(theta1_deg: float, theta2_deg: float, theta3_deg: float, theta4_deg: float) -> tuple[float, float, float, float]:
    """Joint angles (degrees, URDF convention) -> tool-tip (x_m, y_m, z_m, tool_angle_deg).

    tool_angle_deg is measured against the resulting point's own azimuth
    (atan2(y_m, x_m)), not against theta1 directly - the same tool tip can
    be reached either by facing it directly or by turning the base 180 deg
    and folding the arm back over the top, and those two joint solutions
    disagree on theta1 for the same physical pointing direction. Anchoring
    to the point's own azimuth keeps this return value unambiguous.
    """
    tool = _chain_frames((theta1_deg, theta2_deg, theta3_deg, theta4_deg))
    x_m, y_m, z_m = tool.pos

    facing_rad = math.atan2(y_m, x_m)
    radial_dir = (math.cos(facing_rad), math.sin(facing_rad), 0.0)
    vertical_dir = (0.0, 0.0, 1.0)
    pointing_dir = _mat_vec(tool.rot, (1.0, 0.0, 0.0))

    radial_component = sum(a * b for a, b in zip(pointing_dir, radial_dir))
    vertical_component = sum(a * b for a, b in zip(pointing_dir, vertical_dir))
    tool_angle_deg = math.degrees(math.atan2(vertical_component, radial_component))

    return x_m, y_m, z_m, tool_angle_deg


def _first_limit_violation(angles_deg: tuple[float, float, float, float]) -> str | None:
    for i, (angle_deg, (lower, upper)) in enumerate(zip(angles_deg, _JOINT_LIMITS_DEG)):
        if angle_deg < lower or angle_deg > upper:
            return (
                f"IK failure: joint_{i + 1} would need {angle_deg:.1f} deg, "
                f"outside its safe range ({lower:.1f}-{upper:.1f} deg)"
            )
    return None


def _forward_kinematics_matches(angles_deg, target) -> bool:
    x_m, y_m, z_m, tool_angle_deg = forward_kinematics(*angles_deg)
    tx, ty, tz, t_angle = target
    position_ok = (
        abs(x_m - tx) < _REACHABILITY_TOLERANCE_M
        and abs(y_m - ty) < _REACHABILITY_TOLERANCE_M
        and abs(z_m - tz) < _REACHABILITY_TOLERANCE_M
    )
    angle_ok = abs((tool_angle_deg - t_angle + 180.0) % 360.0 - 180.0) < _REACHABILITY_TOLERANCE_DEG
    return position_ok and angle_ok


def inverse_kinematics(
    x_m: float,
    y_m: float,
    z_m: float,
    tool_angle_deg: float,
    current_angles_deg: tuple[float, float, float, float] | None = None,
) -> tuple[float, float, float, float]:
    """Cartesian tool-tip target -> joint angles (degrees).

    Up to 4 candidates can reach the same target (2 azimuth branches x 2
    elbow directions). If current_angles_deg is given, the candidate with
    the least total joint travel from it is returned - this keeps the arm's
    elbow configuration from flipping between nearby targets. If it's None,
    the first valid candidate found is returned (facing-branch before
    folded-back, elbow-config-A before elbow-config-B).

    Raises NotReachableError if there's no geometric solution, or if every
    candidate solution needs a joint outside its safe range.
    """
    l1, l2, l3 = _SHOULDER_TO_ELBOW_M, _ELBOW_TO_WRIST_M, _WRIST_TO_TOOL_TIP_M

    facing_theta1_deg = math.degrees(math.atan2(y_m, x_m)) % 360.0
    r_mag = math.hypot(x_m, y_m)
    z_rel = z_m - _BASE_TO_SHOULDER_M

    target = (x_m, y_m, z_m, tool_angle_deg)
    last_error = None
    valid_candidates = []

    # A target can be reached two ways: turn the base to face it and reach
    # outward (r positive in that frame), or turn 180 deg away and fold the
    # shoulder/elbow/wrist back over the top (r negative in that frame).
    # tool_angle_deg is defined relative to the *facing* frame's horizontal,
    # so the folded-back frame needs it mirrored (180 - angle) to describe
    # the same physical pointing direction.
    azimuth_branches = (
        (facing_theta1_deg, r_mag, tool_angle_deg),
        ((facing_theta1_deg + 180.0) % 360.0, -r_mag, 180.0 - tool_angle_deg),
    )

    for theta1_deg, r_target, branch_tool_angle_deg in azimuth_branches:
        phi3_deg = branch_tool_angle_deg - 90.0
        phi3_rad = math.radians(phi3_deg)

        # Subtract the fixed tool segment to get the target for the shoulder/elbow pair.
        a_val = -r_target - l3 * math.sin(phi3_rad)
        b_val = z_rel - l3 * math.cos(phi3_rad)

        d_sq = a_val ** 2 + b_val ** 2
        d = math.sqrt(d_sq)
        max_reach = l1 + l2
        min_reach = abs(l1 - l2)
        if d > max_reach or d < min_reach:
            last_error = (
                f"IK failure: position not reachable (required reach {d * 1000:.1f} mm, "
                f"arm spans {min_reach * 1000:.1f}-{max_reach * 1000:.1f} mm)"
            )
            continue

        cos_elbow = max(-1.0, min(1.0, (d_sq - l1 ** 2 - l2 ** 2) / (2 * l1 * l2)))
        for elbow_rad in (math.acos(cos_elbow), -math.acos(cos_elbow)):
            psi1_rad = math.atan2(b_val, a_val) - math.atan2(
                l2 * math.sin(elbow_rad), l1 + l2 * math.cos(elbow_rad)
            )
            psi2_rad = psi1_rad + elbow_rad

            theta2_deg = 90.0 - math.degrees(psi1_rad)
            phi2_deg = 90.0 - math.degrees(psi2_rad)
            theta3_deg = phi2_deg - theta2_deg
            theta4_deg = phi3_deg - phi2_deg

            # Normalize into [-180, 180) - the closed form can produce
            # physically-equivalent angles many turns apart (e.g. 212.6 deg
            # vs -147.4 deg), and _JOINT_LIMITS_DEG is now centered on 0
            # (e.g. -110 to 110), so the canonical representative must be
            # the one in [-180, 180), not [0, 360) (a negative in-range
            # angle like -50 would otherwise normalize to 310 and wrongly
            # fail the bounds check below).
            candidate = tuple(
                ((a + 180.0) % 360.0) - 180.0 for a in (theta1_deg, theta2_deg, theta3_deg, theta4_deg)
            )

            violation = _first_limit_violation(candidate)
            if violation is not None:
                last_error = violation
                continue

            if not _forward_kinematics_matches(candidate, target):
                last_error = "IK failure: candidate solution failed the forward-kinematics self-check"
                continue

            valid_candidates.append(candidate)

    if not valid_candidates:
        raise NotReachableError(last_error or "IK failure: position not reachable")

    if current_angles_deg is None:
        return valid_candidates[0]

    return min(
        valid_candidates,
        key=lambda c: sum(abs(a - b) for a, b in zip(c, current_angles_deg)),
    )
