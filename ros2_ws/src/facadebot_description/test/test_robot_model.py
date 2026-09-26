# Loader behaviour that has to stay fail-closed. Each test builds a throwaway
# package root (config/ + urdf/) so it can corrupt one thing at a time without
# touching the installed model.

import os
import shutil
import tempfile

import pytest
import yaml

from ament_index_python.packages import get_package_share_directory

from facadebot_description import robot_model


@pytest.fixture
def package_root():
    share_dir = get_package_share_directory("facadebot_description")
    temp_dir = tempfile.mkdtemp()
    shutil.copytree(os.path.join(share_dir, "config"), os.path.join(temp_dir, "config"))
    shutil.copytree(os.path.join(share_dir, "urdf"), os.path.join(temp_dir, "urdf"))
    yield temp_dir
    shutil.rmtree(temp_dir)


def _config_path(package_root: str) -> str:
    return os.path.join(package_root, "config", "robot_model.yaml")


def _active_geometry_path(package_root: str) -> str:
    with open(_config_path(package_root)) as config_file:
        config = yaml.safe_load(config_file)
    model = config["models"][config["active_model"]]
    return os.path.join(package_root, model["geometry_file"])


def _active_urdf_path(package_root: str) -> str:
    with open(_active_geometry_path(package_root)) as geometry_file:
        return os.path.join(package_root, yaml.safe_load(geometry_file)["source_urdf"])


def test_installed_model_loads(package_root):
    model = robot_model.load_robot_model(_config_path(package_root))
    assert len(model.joints) == 4
    assert model.joint_names == tuple(f"joint_{i}" for i in range(1, 5))


def test_edited_urdf_without_regeneration_is_refused(package_root):
    with open(_active_urdf_path(package_root), "a") as urdf_file:
        urdf_file.write("\n<!-- re-exported -->\n")
    with pytest.raises(robot_model.RobotModelError) as exc_info:
        robot_model.load_robot_model(_config_path(package_root))
    assert "regenerated" in str(exc_info.value)


def test_missing_urdf_is_refused(package_root):
    os.remove(_active_urdf_path(package_root))
    with pytest.raises(robot_model.RobotModelError):
        robot_model.load_robot_model(_config_path(package_root))


def test_duplicate_joint_names_are_refused(package_root):
    geometry_path = _active_geometry_path(package_root)
    with open(geometry_path) as geometry_file:
        geometry = yaml.safe_load(geometry_file)
    geometry["joints"][1]["name"] = geometry["joints"][0]["name"]
    with open(geometry_path, "w") as geometry_file:
        yaml.safe_dump(geometry, geometry_file)
    with pytest.raises(robot_model.RobotModelError) as exc_info:
        robot_model.load_robot_model(_config_path(package_root))
    assert "more than once" in str(exc_info.value)
