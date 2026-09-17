import math
import random

import pytest

from facade_control import kinematics


def test_zero_pose_points_straight_up(active_arm_model):
    x_m, y_m, z_m, tool_angle_deg = kinematics.forward_kinematics(0.0, 0.0, 0.0, 0.0)

    # Every joint origin stacks along the arm when all angles are zero, so the
    # tip height is just the chain summed nose-to-tail.
    expected_z_m = (
        active_arm_model.joints[0].origin_xyz_m[2]
        + active_arm_model.joints[1].origin_xyz_m[2]
        + active_arm_model.joints[2].origin_xyz_m[0]
        + active_arm_model.joints[3].origin_xyz_m[0]
        + active_arm_model.tool_tip_offset_m[0]
    )
    assert x_m == pytest.approx(0.0, abs=1e-3)
    assert y_m == pytest.approx(0.0, abs=1e-3)
    assert z_m == pytest.approx(expected_z_m, abs=1e-3)
    assert tool_angle_deg == pytest.approx(0.0, abs=1e-6)


def test_joint_4_tilts_the_tool_out_of_the_shoulder_elbow_plane():
    in_plane = kinematics.forward_kinematics(0.0, 30.0, -20.0, 0.0)
    tilted = kinematics.forward_kinematics(0.0, 30.0, -20.0, 40.0)

    # Shoulder and elbow sweep the y-z plane on this arm, so a joint_4 tilt is
    # the only thing that can move the tip in x.
    assert in_plane[0] == pytest.approx(0.0, abs=1e-3)
    assert abs(tilted[0]) > 0.02
    assert tilted[3] == pytest.approx(40.0)


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
    for expected, actual in zip(target[:3], check[:3]):
        assert expected == pytest.approx(actual, abs=1e-4)


def test_unreachable_target_raises():
    with pytest.raises(kinematics.NotReachableError):
        kinematics.inverse_kinematics(10.0, 0.0, 0.2, 0.0)


def test_target_needing_an_out_of_range_joint_raises(active_arm_model):
    reachable = kinematics.forward_kinematics(0.0, 100.0, 60.0, 0.0)
    kinematics.configure(_with_tight_limits(active_arm_model))
    with pytest.raises(kinematics.NotReachableError) as exc_info:
        kinematics.inverse_kinematics(*reachable)
    assert "safe range" in str(exc_info.value)


def _with_tight_limits(model):
    from dataclasses import replace
    return replace(model, joints=tuple(
        replace(joint, limit_deg=(-5.0, 5.0)) for joint in model.joints))


def test_least_travel_solution_is_chosen():
    target = kinematics.forward_kinematics(20.0, -60.0, 40.0, 30.0)
    solved = kinematics.inverse_kinematics(*target, current_angles_deg=(20.0, -60.0, 40.0, 30.0))
    for expected, actual in zip((20.0, -60.0, 40.0, 30.0), solved):
        assert expected == pytest.approx(actual, abs=1e-6)


def test_random_reachable_targets_round_trip(test_joint_limits_deg):
    random.seed(1234)
    for _ in range(200):
        angles = tuple(
            random.uniform(lower + 2.0, upper - 2.0) for lower, upper in test_joint_limits_deg
        )
        target = kinematics.forward_kinematics(*angles)
        try:
            solved = kinematics.inverse_kinematics(*target, current_angles_deg=angles)
        except kinematics.NotReachableError:
            # Straight-up poses put the tip on joint_1's axis, where the base
            # angle is undefined. Genuinely unsolvable, not a solver bug.
            assert math.hypot(target[0], target[1]) < 1e-3
            continue
        check = kinematics.forward_kinematics(*solved)
        for expected, actual in zip(target[:3], check[:3]):
            assert expected == pytest.approx(actual, abs=1e-3)


def test_previous_arm_model_still_solves(previous_arm_model):
    """v1's wrist lay in the arm plane; the same solver must still handle it."""
    kinematics.configure(previous_arm_model)
    target = kinematics.forward_kinematics(25.0, -50.0, 35.0, 20.0)
    solved = kinematics.inverse_kinematics(*target, current_angles_deg=(25.0, -50.0, 35.0, 20.0))
    for expected, actual in zip((25.0, -50.0, 35.0, 20.0), solved):
        assert expected == pytest.approx(actual, abs=1e-4)


def test_model_with_a_wrong_shape_is_rejected(active_arm_model):
    from dataclasses import replace
    broken = replace(active_arm_model, joints=tuple(
        replace(joint, origin_rpy_rad=(0.3, 0.0, 0.0)) if joint.name == "joint_3" else joint
        for joint in active_arm_model.joints))
    with pytest.raises(kinematics.ModelStructureError):
        kinematics.configure(broken)
