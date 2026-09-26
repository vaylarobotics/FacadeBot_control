"""Loads the active arm model: its geometry, its joint limits and its tool offset.

Everything comes from YAML. Geometry (link offsets, joint origins, joint axes) is
read from the generated config/geometry_<model>.yaml, which scripts/
generate_geometry.py writes once from the SolidWorks URDF; limits and the
tool-tip offset are hand-measured and live in config/robot_model.yaml. A running
arm therefore never opens a URDF - the CAD export is a design-time input, not a
runtime dependency.

Every loader failure is fatal by design: a missing config, an unknown model, a
geometry file describing a different model, a broken joint chain or an unmeasured
limit all raise RobotModelError rather than falling back to a default. A node
that cannot prove which arm it is driving must not drive it.
"""

import hashlib
import os
from dataclasses import dataclass

import yaml

_CONFIG_RELATIVE_PATH = "config/robot_model.yaml"

# SolidWorks writes joint origins to 5 decimal places, so this is the most
# precision worth showing when logging the geometry in force.
_ORIGIN_DECIMALS = 5

# A model whose limits_source is anything but this was not measured on the arm it
# claims to describe. The loader still accepts it - bench bring-up has to move the
# arm before the arm can be measured - but describe() says so on every startup, so
# placeholder numbers cannot quietly become the numbers everyone trusts.
_LIMITS_SOURCE_MEASURED = "measured"


class RobotModelError(Exception):
    """The active arm model could not be loaded or could not be trusted."""


@dataclass(frozen=True)
class Joint:
    name: str
    parent_link: str
    child_link: str
    origin_xyz_m: tuple[float, float, float]
    origin_rpy_rad: tuple[float, float, float]
    axis: tuple[float, float, float]
    limit_deg: tuple[float, float]


@dataclass(frozen=True)
class RobotModel:
    name: str
    source_urdf: str
    root_link: str
    joints: tuple[Joint, ...]
    tool_tip_offset_m: tuple[float, float, float]
    limits_source: str | None = None

    @property
    def joint_names(self) -> tuple[str, ...]:
        return tuple(joint.name for joint in self.joints)

    @property
    def joint_limits_deg(self) -> tuple[tuple[float, float], ...]:
        return tuple(joint.limit_deg for joint in self.joints)


def _read_yaml_mapping(path: str, description: str) -> dict:
    try:
        with open(path, "r") as source_file:
            loaded = yaml.safe_load(source_file)
    except OSError as error:
        raise RobotModelError(f"cannot read {description} {path}: {error}") from error
    if not isinstance(loaded, dict):
        raise RobotModelError(f"{path} is not a YAML mapping")
    return loaded


def _require_triple(value: object, context: str) -> tuple[float, float, float]:
    if not isinstance(value, list) or len(value) != 3:
        raise RobotModelError(f"{context}: expected 3 numbers, got {value!r}")
    try:
        return (float(value[0]), float(value[1]), float(value[2]))
    except (TypeError, ValueError) as error:
        raise RobotModelError(f"{context}: not a number: {value!r}") from error


def _parse_geometry_joint(entry: object, index: int,
                          geometry_path: str) -> Joint:
    """Build a Joint with a placeholder limit; the real limits are merged later."""
    context = f"{geometry_path}: joints[{index}]"
    if not isinstance(entry, dict):
        raise RobotModelError(f"{context} is not a mapping")

    fields = {}
    for key in ("name", "parent_link", "child_link"):
        value = entry.get(key)
        if not isinstance(value, str) or not value:
            raise RobotModelError(f"{context}: {key} missing or not a string")
        fields[key] = value

    return Joint(
        name=fields["name"],
        parent_link=fields["parent_link"],
        child_link=fields["child_link"],
        origin_xyz_m=_require_triple(
            entry.get("origin_xyz_m"), f"{context} origin_xyz_m"),
        origin_rpy_rad=_require_triple(
            entry.get("origin_rpy_rad"), f"{context} origin_rpy_rad"),
        axis=_require_triple(entry.get("axis"), f"{context} axis"),
        limit_deg=(0.0, 0.0),
    )


def _sha256_of_file(path: str) -> str:
    digest = hashlib.sha256()
    with open(path, "rb") as source:
        for chunk in iter(lambda: source.read(65536), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _require_geometry_matches_urdf(geometry: dict, geometry_path: str,
                                   package_root: str, source_urdf: str) -> None:
    """Refuse a geometry file that was generated from a different URDF than the
    one on disk. Hashing the file is not parsing it, so the no-URDF-at-runtime
    rule holds; this only catches a re-export that was never regenerated."""
    declared_sha256 = geometry.get("source_urdf_sha256")
    if not isinstance(declared_sha256, str) or not declared_sha256:
        raise RobotModelError(
            f"{geometry_path}: source_urdf_sha256 missing. Regenerate it with "
            "scripts/generate_geometry.py.")
    urdf_path = os.path.join(package_root, source_urdf)
    if not os.path.isfile(urdf_path):
        raise RobotModelError(
            f"{geometry_path} was generated from {source_urdf}, but there is no such "
            f"file at {urdf_path}, so the geometry cannot be checked against it.")
    actual_sha256 = _sha256_of_file(urdf_path)
    if actual_sha256 != declared_sha256:
        raise RobotModelError(
            f"{geometry_path} was generated from an older {source_urdf} (sha256 "
            f"{declared_sha256[:12]}..., on disk {actual_sha256[:12]}...). The URDF "
            "changed without the geometry being regenerated; run "
            "scripts/generate_geometry.py and commit both files together.")


def _load_geometry(geometry_path: str, model_name: str,
                   package_root: str) -> tuple[str, str, list[Joint]]:
    """Read a generated geometry file. Returns (source_urdf, root_link, joints)."""
    geometry = _read_yaml_mapping(geometry_path, "geometry file")

    # Guards against a model entry pointing at another arm's geometry file, which
    # would otherwise load cleanly and silently drive the wrong shape.
    declared_model = geometry.get("model")
    if declared_model != model_name:
        raise RobotModelError(
            f"{geometry_path} describes model {declared_model!r}, but it is "
            f"configured as the geometry for model '{model_name}'. Regenerate it "
            "with scripts/generate_geometry.py.")

    source_urdf = geometry.get("source_urdf")
    root_link = geometry.get("root_link")
    if not isinstance(source_urdf, str) or not isinstance(root_link, str):
        raise RobotModelError(
            f"{geometry_path}: source_urdf and root_link are both required")

    _require_geometry_matches_urdf(geometry, geometry_path, package_root, source_urdf)

    raw_joints = geometry.get("joints")
    if not isinstance(raw_joints, list) or not raw_joints:
        raise RobotModelError(f"{geometry_path}: joints missing or empty")

    joints = [
        _parse_geometry_joint(entry, index, geometry_path)
        for index, entry in enumerate(raw_joints)
    ]

    names = [joint.name for joint in joints]
    duplicates = sorted({name for name in names if names.count(name) > 1})
    if duplicates:
        raise RobotModelError(
            f"{geometry_path}: joint name(s) {duplicates} appear more than once; "
            "limits are looked up by name, so every joint needs a unique one.")

    # The generator writes the chain base-to-tip and the solver relies on that
    # order, so re-check it here rather than trusting a file that may have been
    # hand-edited.
    expected_parent = root_link
    for joint in joints:
        if joint.parent_link != expected_parent:
            raise RobotModelError(
                f"{geometry_path}: joint {joint.name} hangs off "
                f"{joint.parent_link!r}, but the chain reached "
                f"{expected_parent!r}. Joints must be listed base to tip.")
        expected_parent = joint.child_link

    return source_urdf, root_link, joints


def _require_limits(model_config: dict, model_name: str,
                    joint_names: tuple[str, ...]) -> dict[str, tuple[float, float]]:
    raw_limits = model_config.get("joint_limits_deg")
    if not isinstance(raw_limits, dict):
        raise RobotModelError(
            f"model '{model_name}': joint_limits_deg missing or not a mapping")

    configured = set(raw_limits)
    declared = set(joint_names)
    if configured != declared:
        raise RobotModelError(
            f"model '{model_name}': joint_limits_deg covers {sorted(configured)} "
            f"but the geometry declares {sorted(declared)}")

    limits: dict[str, tuple[float, float]] = {}
    for joint_name in joint_names:
        value = raw_limits[joint_name]
        if value is None:
            raise RobotModelError(
                f"model '{model_name}': joint {joint_name} has no measured limits "
                "(joint_limits_deg is null). Measure them on the arm before "
                "commanding motion.")
        if not isinstance(value, list) or len(value) != 2:
            raise RobotModelError(
                f"model '{model_name}': joint {joint_name} limits must be "
                f"[lower_deg, upper_deg], got {value!r}")
        lower_deg, upper_deg = float(value[0]), float(value[1])
        if lower_deg >= upper_deg:
            raise RobotModelError(
                f"model '{model_name}': joint {joint_name} lower limit "
                f"{lower_deg} is not below upper limit {upper_deg}")
        limits[joint_name] = (lower_deg, upper_deg)
    return limits


def _require_tool_offset(model_config: dict,
                         model_name: str) -> tuple[float, float, float]:
    value = model_config.get("tool_tip_offset_m")
    if value is None:
        raise RobotModelError(
            f"model '{model_name}': tool_tip_offset_m is unmeasured (null). "
            "Measure the joint_4-to-tool-tip offset in link_4's frame before "
            "commanding motion.")
    return _require_triple(value, f"model '{model_name}': tool_tip_offset_m")


def default_config_path() -> str:
    """Path to the installed robot_model.yaml. Requires a sourced ROS2 workspace."""
    from ament_index_python.packages import get_package_share_directory

    return os.path.join(
        get_package_share_directory("facadebot_description"), _CONFIG_RELATIVE_PATH)


def load_robot_model(config_path: str | None = None) -> RobotModel:
    """Load the model named by active_model, or raise RobotModelError."""
    if config_path is None:
        config_path = default_config_path()

    config = _read_yaml_mapping(config_path, "config")

    model_name = config.get("active_model")
    if not model_name:
        raise RobotModelError(f"{config_path}: active_model is not set")

    models = config.get("models")
    if not isinstance(models, dict) or model_name not in models:
        raise RobotModelError(
            f"{config_path}: active_model '{model_name}' has no entry under models")
    model_config = models[model_name]

    geometry_file = model_config.get("geometry_file")
    if not geometry_file:
        raise RobotModelError(
            f"model '{model_name}': geometry_file is not set. Generate it with "
            "scripts/generate_geometry.py.")

    # geometry_file is written relative to the package root so the same config
    # works from a source tree and from an installed share directory.
    package_root = os.path.dirname(os.path.dirname(config_path))
    geometry_path = os.path.join(package_root, geometry_file)
    if not os.path.isfile(geometry_path):
        raise RobotModelError(
            f"model '{model_name}': no geometry file at {geometry_path}. "
            "Generate it with scripts/generate_geometry.py.")

    source_urdf, root_link, joints = _load_geometry(geometry_path, model_name, package_root)
    joint_names = tuple(joint.name for joint in joints)

    limits = _require_limits(model_config, model_name, joint_names)
    tool_tip_offset_m = _require_tool_offset(model_config, model_name)

    return RobotModel(
        name=model_name,
        source_urdf=source_urdf,
        root_link=root_link,
        joints=tuple(
            Joint(
                name=joint.name,
                parent_link=joint.parent_link,
                child_link=joint.child_link,
                origin_xyz_m=joint.origin_xyz_m,
                origin_rpy_rad=joint.origin_rpy_rad,
                axis=joint.axis,
                limit_deg=limits[joint.name],
            )
            for joint in joints
        ),
        tool_tip_offset_m=tool_tip_offset_m,
        limits_source=model_config.get("limits_source"),
    )


def describe(model: RobotModel) -> str:
    """One line per joint, for logging at startup so the numbers in force are visible."""
    lines = [f"arm model '{model.name}' from {os.path.basename(model.source_urdf)} "
             f"(root link {model.root_link})"]
    for joint in model.joints:
        xyz = ", ".join(f"{v:.{_ORIGIN_DECIMALS}f}" for v in joint.origin_xyz_m)
        rpy = ", ".join(f"{v:.{_ORIGIN_DECIMALS}f}" for v in joint.origin_rpy_rad)
        axis = ", ".join(f"{v:g}" for v in joint.axis)
        lines.append(
            f"  {joint.name}: origin xyz ({xyz}) rpy ({rpy}) axis ({axis}) "
            f"limits {joint.limit_deg[0]:.1f} to {joint.limit_deg[1]:.1f} deg")
    tool = ", ".join(f"{v:.{_ORIGIN_DECIMALS}f}" for v in model.tool_tip_offset_m)
    lines.append(f"  tool tip offset in {model.joints[-1].child_link} frame: ({tool}) m")

    if model.limits_source != _LIMITS_SOURCE_MEASURED:
        source = model.limits_source or "unrecorded"
        lines.append(
            f"  *** WARNING: limits and tool offset above are {source}, NOT measured on "
            f"model '{model.name}'. Bench use only - do not trust a reachability check, "
            f"a bounds-check or a Cartesian pose until they are re-measured. ***")
    return "\n".join(lines)
