"""Parses an arm URDF into the joint geometry the control stack needs.

Design-time only. Nothing that runs on the arm imports this module: the nodes
read their geometry from the generated config/geometry_<model>.yaml files, so a
running arm never opens a URDF or an XML parser. scripts/generate_geometry.py is
the only caller, and it is run by hand after a SolidWorks re-export.
"""

import hashlib
import xml.etree.ElementTree as ElementTree
from dataclasses import dataclass


class UrdfGeometryError(Exception):
    """A URDF could not be read as a single base-to-tip chain of joints."""


@dataclass(frozen=True)
class JointGeometry:
    """One URDF <joint>, with the fixed geometry that positions its child link."""

    name: str
    parent_link: str
    child_link: str
    origin_xyz_m: tuple[float, float, float]
    origin_rpy_rad: tuple[float, float, float]
    axis: tuple[float, float, float]


def sha256_of_file(path: str) -> str:
    """Digest of the exact bytes on disk, recorded as provenance in the output."""
    with open(path, "rb") as source_file:
        return hashlib.sha256(source_file.read()).hexdigest()


def _read_triple(text: str | None, default: tuple[float, float, float],
                 context: str) -> tuple[float, float, float]:
    if text is None:
        return default
    parts = text.split()
    if len(parts) != 3:
        raise UrdfGeometryError(f"{context}: expected 3 numbers, got {text!r}")
    try:
        return (float(parts[0]), float(parts[1]), float(parts[2]))
    except ValueError as error:
        raise UrdfGeometryError(f"{context}: not a number: {text!r}") from error


def _parse_joint_element(element: ElementTree.Element) -> JointGeometry:
    name = element.get("name")
    if name is None:
        raise UrdfGeometryError("URDF contains a <joint> with no name")

    parent = element.find("parent")
    child = element.find("child")
    if parent is None or child is None:
        raise UrdfGeometryError(f"joint {name}: missing <parent> or <child>")
    parent_link = parent.get("link")
    child_link = child.get("link")
    if parent_link is None or child_link is None:
        raise UrdfGeometryError(
            f"joint {name}: <parent>/<child> missing link attribute")

    origin = element.find("origin")
    origin_xyz_m = _read_triple(
        None if origin is None else origin.get("xyz"), (0.0, 0.0, 0.0),
        f"joint {name} origin xyz")
    origin_rpy_rad = _read_triple(
        None if origin is None else origin.get("rpy"), (0.0, 0.0, 0.0),
        f"joint {name} origin rpy")

    axis_element = element.find("axis")
    # A URDF joint with no <axis> defaults to (1, 0, 0) per the spec. Both of
    # this arm's exports state it explicitly; the default is here so a hand-
    # edited URDF fails on geometry rather than on a missing tag.
    axis = _read_triple(
        None if axis_element is None else axis_element.get("xyz"), (1.0, 0.0, 0.0),
        f"joint {name} axis")

    return JointGeometry(
        name=name,
        parent_link=parent_link,
        child_link=child_link,
        origin_xyz_m=origin_xyz_m,
        origin_rpy_rad=origin_rpy_rad,
        axis=axis,
    )


def _order_joints_from_root(
    parsed: list[JointGeometry],
) -> tuple[str, list[JointGeometry]]:
    """Walk parent -> child from the root link so joints come out base-to-tip."""
    child_links = {joint.child_link for joint in parsed}
    root_candidates = {joint.parent_link for joint in parsed} - child_links
    if len(root_candidates) != 1:
        raise UrdfGeometryError(
            f"URDF must have exactly one root link, found {sorted(root_candidates)}")
    root_link = root_candidates.pop()

    by_parent: dict[str, list[JointGeometry]] = {}
    for joint in parsed:
        by_parent.setdefault(joint.parent_link, []).append(joint)

    ordered: list[JointGeometry] = []
    link = root_link
    while link in by_parent:
        branches = by_parent[link]
        if len(branches) != 1:
            raise UrdfGeometryError(
                f"link {link} has {len(branches)} child joints; this arm is a "
                "single unbranched chain")
        joint = branches[0]
        ordered.append(joint)
        link = joint.child_link

    if len(ordered) != len(parsed):
        raise UrdfGeometryError(
            f"URDF chain from {root_link} covers {len(ordered)} of {len(parsed)} "
            "joints; the remainder are not reachable from the root")
    return root_link, ordered


def parse_urdf(urdf_path: str) -> tuple[str, list[JointGeometry]]:
    """Return (root_link, joints ordered base to tip) for a single-chain URDF."""
    try:
        urdf_root = ElementTree.parse(urdf_path).getroot()
    except ElementTree.ParseError as error:
        raise UrdfGeometryError(f"{urdf_path}: malformed XML: {error}") from error
    except OSError as error:
        raise UrdfGeometryError(f"cannot read {urdf_path}: {error}") from error

    parsed = [_parse_joint_element(element) for element in urdf_root.findall("joint")]
    if not parsed:
        raise UrdfGeometryError(f"{urdf_path}: declares no joints")

    return _order_joints_from_root(parsed)
