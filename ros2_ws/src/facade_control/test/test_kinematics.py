import pytest

from facade_control import kinematics


def test_zero_pose_points_straight_up():
    x_m, y_m, z_m, tool_angle_deg = kinematics.forward_kinematics(0.0, 0.0, 0.0, 0.0)
    expected_z_m = (
        kinematics._BASE_TO_SHOULDER_M
        + kinematics._SHOULDER_TO_ELBOW_M
        + kinematics._ELBOW_TO_WRIST_M
        + kinematics._WRIST_TO_TOOL_TIP_M
    )
    assert x_m == pytest.approx(0.0, abs=1e-4)
    assert y_m == pytest.approx(0.0, abs=1e-4)
    assert z_m == pytest.approx(expected_z_m, abs=1e-4)
    assert tool_angle_deg == pytest.approx(90.0, abs=1e-3)


@pytest.mark.parametrize("theta1,theta2,theta3,theta4", [
    (30.0, 60.0, 90.0, 100.0),
    (10.0, -80.0, 20.0, -70.0),
    (95.0, -90.0, 70.0, 33.1),
    (5.0, 5.0, 5.0, 90.0),
])
def test_inverse_kinematics_round_trips_reachable_targets(theta1, theta2, theta3, theta4):
    target = kinematics.forward_kinematics(theta1, theta2, theta3, theta4)
    solved = kinematics.inverse_kinematics(*target)
    check = kinematics.forward_kinematics(*solved)

    for t, c in zip(target[:3], check[:3]):
        assert t == pytest.approx(c, abs=1e-3)
    angle_err = abs((target[3] - check[3] + 180.0) % 360.0 - 180.0)
    assert angle_err < 0.1


def test_target_beyond_max_reach_is_rejected():
    with pytest.raises(kinematics.NotReachableError):
        kinematics.inverse_kinematics(10.0, 0.0, 0.2, 0.0)


def test_target_requiring_out_of_range_joint_is_rejected_with_named_joint():
    with pytest.raises(kinematics.NotReachableError) as exc_info:
        kinematics.inverse_kinematics(-0.1, 0.0, 0.2, 0.0)
    assert "joint_" in str(exc_info.value)


def test_inverse_kinematics_defaults_to_first_valid_candidate():
    # This target has two valid elbow candidates within joint limits: one
    # near (20, -90, -10, 30) (found first: facing branch, elbow config A)
    # and one near (20, -101.4, 10, 21.4) (elbow config B). With no current
    # position given, the first one found should win, same as before this
    # least-effort selection was added.
    target = kinematics.forward_kinematics(20.0, -90.0, -10.0, 30.0)
    solved = kinematics.inverse_kinematics(*target)
    assert solved == pytest.approx((20.0, -90.0, -10.0, 30.0), abs=1e-2)


def test_inverse_kinematics_prefers_candidate_closest_to_current_position():
    target = kinematics.forward_kinematics(20.0, -90.0, -10.0, 30.0)
    elbow_config_b = (20.0, -101.4, 10.0, 21.4)

    solved_near_a = kinematics.inverse_kinematics(*target, current_angles_deg=(20.0, -90.0, -10.0, 30.0))
    assert solved_near_a == pytest.approx((20.0, -90.0, -10.0, 30.0), abs=1e-2)

    solved_near_b = kinematics.inverse_kinematics(*target, current_angles_deg=elbow_config_b)
    assert solved_near_b == pytest.approx(elbow_config_b, abs=1e-1)
    assert solved_near_b != pytest.approx((20.0, -90.0, -10.0, 30.0), abs=1e-2)


def test_random_reachable_targets_round_trip():
    import random

    random.seed(1234)
    for _ in range(200):
        angles = tuple(
            random.uniform(lo + 2.0, hi - 2.0) for lo, hi in kinematics._JOINT_LIMITS_DEG
        )
        target = kinematics.forward_kinematics(*angles)
        solved = kinematics.inverse_kinematics(*target)
        check = kinematics.forward_kinematics(*solved)
        for t, c in zip(target[:3], check[:3]):
            assert t == pytest.approx(c, abs=1e-3)
