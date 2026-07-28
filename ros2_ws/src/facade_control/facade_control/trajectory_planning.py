"""Continuous-motion trajectory planning (pure geometry + timing, no ROS).

Turns a list of Cartesian waypoints into a dense stream of joint-angle setpoints
that the arm can flow through at a constant tool-tip speed without stopping at the
in-between points. Three stages, each a plain function so they can be unit-tested
without a running ROS graph (same style as kinematics.py):

  1. build_blended_path   - waypoints -> a smooth Cartesian path that rounds each
                            interior corner instead of passing exactly through it.
  2. sample_at_constant_speed - path -> evenly-timed Cartesian points spaced so the
                            tool tip moves at the requested speed (with a smooth
                            speed-up at the start and slow-down at the end).
  3. solve_path_to_setpoints - Cartesian points -> joint angles, chaining the IK so
                            the elbow configuration stays continuous along the path.

Distances are metres, speed is millimetres per second (the action's unit), angles
are degrees. Waypoints are plain (x_m, y_m, z_m, tool_angle_deg) tuples so this
module never imports ROS message types - the node converts msgs to tuples.
"""
import bisect
import math

from facade_control import kinematics

# Resolution the blended path is sampled to before we measure arc length along it.
# Linear interpolation between these points is what point_at() returns, so this must
# be fine enough that the interpolation error is far below the servos' own ~0.24 deg
# resolution. 2 mm is comfortably below that for this arm's link lengths.
_PATH_ARC_STEP_M = 0.002

# Acceleration used only for the smooth speed-up at the very start and slow-down at
# the very end of the whole path (the middle cruises at constant speed). Empirical
# starting point - the arm is small and the servos are position-controlled, so a
# gentle ramp is enough; tune on hardware if the start/stop looks abrupt.
_RAMP_ACCEL_MPS2 = 0.05

Point3 = tuple[float, float, float]
CartesianSample = tuple[float, float, float, float]  # x_m, y_m, z_m, tool_angle_deg


def _sub(a: Point3, b: Point3) -> Point3:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _add(a: Point3, b: Point3) -> Point3:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _scale(a: Point3, k: float) -> Point3:
    return (a[0] * k, a[1] * k, a[2] * k)


def _norm(a: Point3) -> float:
    return math.sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2])


def _dist(a: Point3, b: Point3) -> float:
    return _norm(_sub(a, b))


def _unit(a: Point3) -> Point3:
    length = _norm(a)
    return _scale(a, 1.0 / length) if length > 0.0 else (0.0, 0.0, 0.0)


def _lerp(a: Point3, b: Point3, t: float) -> Point3:
    return (a[0] + (b[0] - a[0]) * t,
            a[1] + (b[1] - a[1]) * t,
            a[2] + (b[2] - a[2]) * t)


def _quad_bezier(a: Point3, control: Point3, b: Point3, t: float) -> Point3:
    # Quadratic Bezier: starts at a, is pulled toward the corner vertex `control`,
    # ends at b. Never actually reaches `control`, which is exactly why it rounds
    # the corner rather than driving through it.
    mt = 1.0 - t
    w_a, w_c, w_b = mt * mt, 2.0 * mt * t, t * t
    return (w_a * a[0] + w_c * control[0] + w_b * b[0],
            w_a * a[1] + w_c * control[1] + w_b * b[1],
            w_a * a[2] + w_c * control[2] + w_b * b[2])


class _Path:
    """A Cartesian path stored as a fine polyline with cumulative arc length and a
    per-point tool angle. point_at(s) interpolates position and tool angle at arc
    length s (metres) measured from the start."""

    def __init__(self, points: list[Point3], tool_angles_deg: list[float]) -> None:
        cum = [0.0]
        for i in range(1, len(points)):
            cum.append(cum[-1] + _dist(points[i - 1], points[i]))
        self._points = points
        self._tool_angles_deg = tool_angles_deg
        self._cum_len_m = cum
        self.total_length_m = cum[-1]

    def point_at(self, s_m: float) -> CartesianSample:
        s_m = max(0.0, min(self.total_length_m, s_m))
        # Find the segment [i-1, i] whose cumulative length brackets s_m.
        i = bisect.bisect_left(self._cum_len_m, s_m)
        if i == 0:
            x, y, z = self._points[0]
            return (x, y, z, self._tool_angles_deg[0])
        seg_len = self._cum_len_m[i] - self._cum_len_m[i - 1]
        t = 0.0 if seg_len == 0.0 else (s_m - self._cum_len_m[i - 1]) / seg_len
        x, y, z = _lerp(self._points[i - 1], self._points[i], t)
        tool = (self._tool_angles_deg[i - 1]
                + (self._tool_angles_deg[i] - self._tool_angles_deg[i - 1]) * t)
        return (x, y, z, tool)


def _subdivide_line(start: Point3, end: Point3) -> list[Point3]:
    # Points strictly after `start`, up to and including `end`, spaced ~_PATH_ARC_STEP_M.
    length = _dist(start, end)
    steps = max(1, math.ceil(length / _PATH_ARC_STEP_M))
    return [_lerp(start, end, j / steps) for j in range(1, steps + 1)]


def _subdivide_bezier(a: Point3, control: Point3, b: Point3) -> list[Point3]:
    # Points strictly after `a`, up to and including `b`. The control-polygon length
    # (a->control->b) upper-bounds the arc length, so it's a safe step-count estimate.
    approx_len = _dist(a, control) + _dist(control, b)
    steps = max(2, math.ceil(approx_len / _PATH_ARC_STEP_M))
    return [_quad_bezier(a, control, b, j / steps) for j in range(1, steps + 1)]


def build_blended_path(waypoints: list[CartesianSample], corner_blend_m: float) -> _Path:
    """Build a smooth Cartesian path through `waypoints`, rounding each interior
    corner within `corner_blend_m` of the vertex (0 = sharp corners, no rounding).

    Straight or collinear runs stay straight, so e.g. a raster's parallel passes are
    unaffected and only the U-turns get rounded. Needs at least 2 waypoints.
    """
    if len(waypoints) < 2:
        raise ValueError("a continuous path needs at least 2 waypoints")

    positions: list[Point3] = [(w[0], w[1], w[2]) for w in waypoints]
    raw_tool_angles_deg = [w[3] for w in waypoints]

    # Cumulative length along the raw (un-rounded) polyline, used only to map tool
    # angle onto the path so each waypoint's tool angle is honoured in order.
    raw_cum = [0.0]
    for i in range(1, len(positions)):
        raw_cum.append(raw_cum[-1] + _dist(positions[i - 1], positions[i]))
    raw_total = raw_cum[-1]

    points: list[Point3] = [positions[0]]
    cursor = positions[0]
    n = len(positions)
    for i in range(1, n - 1):
        prev_pt, vertex, next_pt = positions[i - 1], positions[i], positions[i + 1]
        # Trim back from the corner along each adjacent segment, but never past its
        # midpoint (so two nearby corners can't overlap and cross each other).
        trim = min(corner_blend_m,
                   _dist(vertex, prev_pt) / 2.0,
                   _dist(vertex, next_pt) / 2.0)
        if trim <= 0.0:
            points.extend(_subdivide_line(cursor, vertex))
            cursor = vertex
            continue
        entry = _add(vertex, _scale(_unit(_sub(prev_pt, vertex)), trim))
        exit_ = _add(vertex, _scale(_unit(_sub(next_pt, vertex)), trim))
        points.extend(_subdivide_line(cursor, entry))
        points.extend(_subdivide_bezier(entry, vertex, exit_))
        cursor = exit_
    points.extend(_subdivide_line(cursor, positions[-1]))

    # Assign each blended point a tool angle by mapping its fractional progress along
    # the blended path onto the raw polyline, then interpolating between waypoints.
    blended_cum = [0.0]
    for i in range(1, len(points)):
        blended_cum.append(blended_cum[-1] + _dist(points[i - 1], points[i]))
    blended_total = blended_cum[-1]
    tool_angles_deg = [
        _tool_angle_at_raw_length(
            (blended_cum[k] / blended_total * raw_total) if blended_total > 0.0 else 0.0,
            raw_cum, raw_tool_angles_deg)
        for k in range(len(points))
    ]
    return _Path(points, tool_angles_deg)


def _tool_angle_at_raw_length(
    s_raw_m: float, raw_cum: list[float], tool_angles_deg: list[float]
) -> float:
    if s_raw_m <= 0.0:
        return tool_angles_deg[0]
    if s_raw_m >= raw_cum[-1]:
        return tool_angles_deg[-1]
    i = bisect.bisect_left(raw_cum, s_raw_m)
    seg_len = raw_cum[i] - raw_cum[i - 1]
    t = 0.0 if seg_len == 0.0 else (s_raw_m - raw_cum[i - 1]) / seg_len
    return tool_angles_deg[i - 1] + (tool_angles_deg[i] - tool_angles_deg[i - 1]) * t


def _arc_length_at_time(t_sec: float, cruise_speed_mps: float, ramp_dist_m: float,
                        cruise_dist_m: float, t_accel_sec: float,
                        t_cruise_sec: float) -> float:
    # Trapezoidal speed profile: accelerate 0->cruise, hold, decelerate cruise->0.
    if t_sec <= t_accel_sec:
        return 0.5 * _RAMP_ACCEL_MPS2 * t_sec * t_sec
    if t_sec <= t_accel_sec + t_cruise_sec:
        return ramp_dist_m + cruise_speed_mps * (t_sec - t_accel_sec)
    td = t_sec - t_accel_sec - t_cruise_sec
    return ramp_dist_m + cruise_dist_m + cruise_speed_mps * td - 0.5 * _RAMP_ACCEL_MPS2 * td * td


def sample_at_constant_speed(
    path: _Path, tool_speed_mmps: float, stream_period_sec: float
) -> list[CartesianSample]:
    """Sample `path` into evenly-timed Cartesian points so the tool tip moves at
    tool_speed_mmps, ramping smoothly up at the start and down at the end.

    One point per stream_period_sec; the final point always lands exactly on the
    path end so the arm finishes at the last waypoint.
    """
    if tool_speed_mmps <= 0.0:
        raise ValueError("tool_speed_mmps must be positive")
    if stream_period_sec <= 0.0:
        raise ValueError("stream_period_sec must be positive")

    total_m = path.total_length_m
    if total_m <= 0.0:
        return [path.point_at(0.0)]

    cruise_speed_mps = tool_speed_mmps / 1000.0
    ramp_dist_m = cruise_speed_mps * cruise_speed_mps / (2.0 * _RAMP_ACCEL_MPS2)

    if 2.0 * ramp_dist_m <= total_m:
        # Long enough to reach and hold cruise speed (trapezoid).
        peak_speed_mps = cruise_speed_mps
        cruise_dist_m = total_m - 2.0 * ramp_dist_m
    else:
        # Too short to reach cruise speed - accelerate to a lower peak, then decelerate
        # (triangle). ramp_dist becomes half the path.
        peak_speed_mps = math.sqrt(_RAMP_ACCEL_MPS2 * total_m)
        ramp_dist_m = total_m / 2.0
        cruise_dist_m = 0.0

    t_accel_sec = peak_speed_mps / _RAMP_ACCEL_MPS2
    t_cruise_sec = cruise_dist_m / peak_speed_mps if peak_speed_mps > 0.0 else 0.0
    total_time_sec = 2.0 * t_accel_sec + t_cruise_sec

    samples: list[CartesianSample] = []
    steps = max(1, math.ceil(total_time_sec / stream_period_sec))
    for k in range(steps):
        t_sec = k * stream_period_sec
        s_m = _arc_length_at_time(t_sec, peak_speed_mps, ramp_dist_m,
                                  cruise_dist_m, t_accel_sec, t_cruise_sec)
        samples.append(path.point_at(s_m))
    samples.append(path.point_at(total_m))  # exact endpoint
    return samples


def solve_path_to_setpoints(
    cartesian_samples: list[CartesianSample],
    current_angles_deg: tuple[float, float, float, float],
) -> list[tuple[float, float, float, float]]:
    """Run IK on each Cartesian sample, seeding each solve from the previous joint
    solution so the elbow configuration stays continuous along the path (no flips).
    The first sample is seeded from the arm's actual current angles.

    Raises kinematics.NotReachableError if any sample is unreachable or would need a
    joint outside its safe range - the caller validates the whole path before moving.
    """
    setpoints: list[tuple[float, float, float, float]] = []
    previous_angles_deg = current_angles_deg
    for x_m, y_m, z_m, tool_angle_deg in cartesian_samples:
        angles_deg = kinematics.inverse_kinematics(
            x_m, y_m, z_m, tool_angle_deg, current_angles_deg=previous_angles_deg
        )
        setpoints.append(angles_deg)
        previous_angles_deg = angles_deg
    return setpoints


def plan_joint_setpoints(
    waypoints: list[CartesianSample],
    tool_speed_mmps: float,
    corner_blend_m: float,
    stream_period_sec: float,
    current_angles_deg: tuple[float, float, float, float],
) -> list[tuple[float, float, float, float]]:
    """Convenience wrapper: waypoints -> blended path -> constant-speed samples ->
    chained-IK joint setpoints. Raises ValueError for bad inputs and
    kinematics.NotReachableError if any point on the path can't be reached."""
    path = build_blended_path(waypoints, corner_blend_m)
    samples = sample_at_constant_speed(path, tool_speed_mmps, stream_period_sec)
    return solve_path_to_setpoints(samples, current_angles_deg)
