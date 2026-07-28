import math

import pytest

from facade_control import kinematics
from facade_control import trajectory_planning as tp


def _dist(a, b) -> float:
    return math.sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))


def test_straight_path_length_matches_endpoint_distance():
    waypoints = [(0.0, 0.0, 0.0, 0.0), (0.3, 0.0, 0.0, 0.0)]
    path = tp.build_blended_path(waypoints, corner_blend_m=0.0)
    assert path.total_length_m == pytest.approx(0.3, abs=1e-3)
    assert path.point_at(0.0)[:3] == pytest.approx((0.0, 0.0, 0.0), abs=1e-6)
    assert path.point_at(path.total_length_m)[:3] == pytest.approx((0.3, 0.0, 0.0), abs=1e-3)


def test_constant_speed_samples_are_evenly_spaced_in_cruise():
    path = tp.build_blended_path([(0.0, 0.0, 0.0, 0.0), (0.5, 0.0, 0.0, 0.0)], corner_blend_m=0.0)
    speed_mmps, period_sec = 50.0, 0.1
    samples = tp.sample_at_constant_speed(path, speed_mmps, period_sec)

    expected_step_m = speed_mmps / 1000.0 * period_sec  # 0.005 m per period at cruise
    spacings = [_dist(samples[i], samples[i + 1]) for i in range(len(samples) - 1)]
    # Constant speed means no spacing exceeds the cruise step, and the middle of the
    # path actually reaches that cruise step (the ends are the smaller ramp steps).
    assert max(spacings) == pytest.approx(expected_step_m, abs=1e-4)
    assert any(s == pytest.approx(expected_step_m, abs=1e-5) for s in spacings[1:-1])


def test_corner_is_rounded_not_passed_through():
    waypoints = [(0.2, 0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 0.0), (0.0, 0.2, 0.0, 0.0)]
    blend_m = 0.05
    path = tp.build_blended_path(waypoints, corner_blend_m=blend_m)

    vertex = (0.0, 0.0, 0.0)
    samples = 400
    min_dist_m = min(
        _dist(path.point_at(path.total_length_m * k / samples), vertex)
        for k in range(samples + 1)
    )
    # The path curves near the corner but never reaches the sharp vertex, and stays
    # within the blend radius of it.
    assert min_dist_m > 1e-3
    assert min_dist_m < blend_m


def test_collinear_waypoints_stay_straight():
    waypoints = [(0.0, 0.0, 0.0, 0.0), (0.1, 0.0, 0.0, 0.0), (0.3, 0.0, 0.0, 0.0)]
    path = tp.build_blended_path(waypoints, corner_blend_m=0.05)

    samples = 200
    for k in range(samples + 1):
        _, y_m, z_m, _ = path.point_at(path.total_length_m * k / samples)
        assert y_m == pytest.approx(0.0, abs=1e-9)
        assert z_m == pytest.approx(0.0, abs=1e-9)
    assert path.total_length_m == pytest.approx(0.3, abs=1e-3)


def test_tool_angle_interpolates_along_path():
    path = tp.build_blended_path([(0.0, 0.0, 0.0, 0.0), (0.4, 0.0, 0.0, 30.0)], corner_blend_m=0.0)
    assert path.point_at(0.0)[3] == pytest.approx(0.0, abs=1e-6)
    assert path.point_at(path.total_length_m)[3] == pytest.approx(30.0, abs=1e-6)
    assert path.point_at(path.total_length_m * 0.5)[3] == pytest.approx(15.0, abs=0.5)


def test_final_sample_lands_exactly_on_path_end():
    path = tp.build_blended_path([(0.0, 0.0, 0.0, 0.0), (0.25, 0.0, 0.0, 0.0)], corner_blend_m=0.0)
    samples = tp.sample_at_constant_speed(path, tool_speed_mmps=40.0, stream_period_sec=0.1)
    assert samples[-1][:3] == pytest.approx((0.25, 0.0, 0.0), abs=1e-4)


def test_plan_joint_setpoints_on_reachable_waypoints_stay_in_limits():
    seed = (10.0, -30.0, 20.0, 10.0)
    waypoint_a = kinematics.forward_kinematics(*seed)
    waypoint_b = kinematics.forward_kinematics(20.0, -40.0, 25.0, 15.0)

    setpoints = tp.plan_joint_setpoints(
        [waypoint_a, waypoint_b],
        tool_speed_mmps=30.0,
        corner_blend_m=0.0,
        stream_period_sec=0.1,
        current_angles_deg=seed,
    )

    assert len(setpoints) >= 2
    for setpoint in setpoints:
        assert len(setpoint) == 4
        for angle_deg, (lower, upper) in zip(setpoint, kinematics._JOINT_LIMITS_DEG):
            assert lower - 1e-6 <= angle_deg <= upper + 1e-6
    # First setpoint should recover the seed angles it was solved from.
    assert setpoints[0] == pytest.approx(seed, abs=1.0)


def test_plan_raises_when_a_point_on_the_path_is_unreachable():
    seed = (10.0, -30.0, 20.0, 10.0)
    reachable = kinematics.forward_kinematics(*seed)
    unreachable = (10.0, 0.0, 0.2, 0.0)  # well beyond the arm's max reach
    with pytest.raises(kinematics.NotReachableError):
        tp.plan_joint_setpoints(
            [reachable, unreachable], 30.0, 0.0, 0.1, current_angles_deg=seed
        )


def test_build_blended_path_needs_two_waypoints():
    with pytest.raises(ValueError):
        tp.build_blended_path([(0.0, 0.0, 0.0, 0.0)], corner_blend_m=0.0)


def test_sample_rejects_non_positive_speed():
    path = tp.build_blended_path([(0.0, 0.0, 0.0, 0.0), (0.3, 0.0, 0.0, 0.0)], corner_blend_m=0.0)
    with pytest.raises(ValueError):
        tp.sample_at_constant_speed(path, tool_speed_mmps=0.0, stream_period_sec=0.1)
