"""Configures the kinematics module for tests.

The shipped robot_model.yaml leaves v2's limits and tool offset unmeasured on
purpose, so load_robot_model() refuses it - correct for the real arm, useless
for testing the maths. These fixtures build a model from the same generated
geometry with stand-in limits and tool offset, so the tests exercise the solver
rather than the calibration.
"""

import os
import tempfile
from glob import glob

import pytest
import yaml

from ament_index_python.packages import get_package_share_directory

from facade_control import kinematics
from facadebot_description.robot_model import load_robot_model

_TEST_LIMITS_DEG = [-110.0, 110.0]
_TEST_TOOL_TIP_OFFSET_M = [0.05, 0.0, 0.0]


def _build_model(model_name: str):
    share_dir = get_package_share_directory("facadebot_description")
    config_path = os.path.join(share_dir, "config", "robot_model.yaml")
    with open(config_path, "r") as config_file:
        config = yaml.safe_load(config_file)

    config["active_model"] = model_name
    model_config = config["models"][model_name]
    model_config["joint_limits_deg"] = {
        f"joint_{index}": list(_TEST_LIMITS_DEG) for index in range(1, 5)
    }
    model_config["tool_tip_offset_m"] = list(_TEST_TOOL_TIP_OFFSET_M)

    temp_dir = tempfile.mkdtemp()
    temp_config_dir = os.path.join(temp_dir, "config")
    os.makedirs(temp_config_dir)
    # geometry_file paths are resolved relative to the package root, which is the
    # temp dir here, so the real generated geometry has to be visible under it.
    for geometry_file in glob(os.path.join(share_dir, "config", "geometry_*.yaml")):
        os.symlink(geometry_file,
                   os.path.join(temp_config_dir, os.path.basename(geometry_file)))
    # The loader hashes each geometry file's source URDF to prove they match, so
    # the URDFs have to be reachable from the temp package root as well.
    os.symlink(os.path.join(share_dir, "urdf"), os.path.join(temp_dir, "urdf"))
    temp_config_path = os.path.join(temp_config_dir, "robot_model.yaml")
    with open(temp_config_path, "w") as temp_config:
        yaml.safe_dump(config, temp_config)

    return load_robot_model(temp_config_path)


@pytest.fixture(autouse=True)
def active_arm_model():
    """Every test runs against v2, the current arm."""
    model = _build_model("v2")
    kinematics.configure(model)
    return model


@pytest.fixture
def previous_arm_model():
    """v1, for checking the solver still handles the retired in-plane wrist."""
    return _build_model("v1")


@pytest.fixture
def test_joint_limits_deg():
    """The stand-in limits the fixtures above install, for tests that assert on them."""
    return tuple(tuple(_TEST_LIMITS_DEG) for _ in range(4))
