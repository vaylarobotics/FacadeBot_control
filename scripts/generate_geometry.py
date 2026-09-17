#!/usr/bin/env python3
"""Bakes an arm's geometry out of its URDF into config/geometry_<model>.yaml.

Run this by hand after re-exporting a URDF from SolidWorks. The nodes never read
the URDF - they read the file this writes - so a re-export does nothing until
this script is run and the result committed.

    python3 scripts/generate_geometry.py --model v2

The URDF to read comes from that model's urdf_file entry in robot_model.yaml.
Joint limits and the tool-tip offset are hand-measured and are NOT touched here;
they stay in robot_model.yaml.
"""

import argparse
import os
import sys

import yaml

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_PACKAGE_ROOT = os.path.join(
    _REPO_ROOT, "ros2_ws", "src", "facadebot_description")

# Import the parser straight from the source tree so this runs on a fresh clone
# without building or sourcing the ROS2 workspace.
sys.path.insert(0, _PACKAGE_ROOT)

from facadebot_description.urdf_geometry import (  # noqa: E402,I100
    UrdfGeometryError, parse_urdf, sha256_of_file)

_CONFIG_PATH = os.path.join(_PACKAGE_ROOT, "config", "robot_model.yaml")

_HEADER = """\
# GENERATED FILE - DO NOT EDIT BY HAND.
#
# Written by scripts/generate_geometry.py from the URDF named below. Every number
# here came out of the SolidWorks export, so hand-editing this file makes the
# controller disagree with the CAD model and with the physical arm.
#
# To change it: re-export the URDF, then re-run
#     python3 scripts/generate_geometry.py --model {model}
# and commit both files together.
#
# Measured values (joint limits, tool-tip offset) are NOT here - they live in
# robot_model.yaml, which is hand-written.
"""


def _format_triple(values: tuple[float, float, float]) -> str:
    """repr() keeps full round-trip precision; yaml's float dump does not always."""
    return "[" + ", ".join(repr(float(value)) for value in values) + "]"


def _render(model_name: str, urdf_file: str, urdf_sha256: str,
            root_link: str, joints: list) -> str:
    lines = [_HEADER.format(model=model_name), ""]
    lines.append(f"model: {model_name}")
    lines.append(f"source_urdf: {urdf_file}")
    lines.append(f'source_urdf_sha256: "{urdf_sha256}"')
    lines.append(f"root_link: {root_link}")
    lines.append("")
    lines.append("# Ordered base to tip. The solver relies on this order.")
    lines.append("joints:")
    for joint in joints:
        lines.append(f"  - name: {joint.name}")
        lines.append(f"    parent_link: {joint.parent_link}")
        lines.append(f"    child_link: {joint.child_link}")
        lines.append(f"    origin_xyz_m: {_format_triple(joint.origin_xyz_m)}")
        lines.append(f"    origin_rpy_rad: {_format_triple(joint.origin_rpy_rad)}")
        lines.append(f"    axis: {_format_triple(joint.axis)}")
    return "\n".join(lines) + "\n"


def _verify_round_trip(text: str, root_link: str, joints: list) -> None:
    """Re-read what we are about to write, so a formatting slip cannot ship."""
    reloaded = yaml.safe_load(text)
    if reloaded.get("root_link") != root_link:
        raise SystemExit("internal error: root_link did not survive the round trip")
    reloaded_joints = reloaded.get("joints") or []
    if len(reloaded_joints) != len(joints):
        raise SystemExit("internal error: joint count did not survive the round trip")
    for written, joint in zip(reloaded_joints, joints):
        expected = {
            "name": joint.name,
            "parent_link": joint.parent_link,
            "child_link": joint.child_link,
            "origin_xyz_m": list(joint.origin_xyz_m),
            "origin_rpy_rad": list(joint.origin_rpy_rad),
            "axis": list(joint.axis),
        }
        if written != expected:
            raise SystemExit(
                f"internal error: joint {joint.name} did not survive the round "
                f"trip:\n  wrote {written}\n  meant {expected}")


def main() -> int:
    """Regenerate one model's geometry file. Returns a shell exit code."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--model", required=True,
        help="model name to regenerate, as it appears under models: in "
             "robot_model.yaml (e.g. v2)")
    args = parser.parse_args()

    with open(_CONFIG_PATH, "r") as config_file:
        config = yaml.safe_load(config_file)

    models = config.get("models") or {}
    if args.model not in models:
        print(f"error: model '{args.model}' has no entry in {_CONFIG_PATH}",
              file=sys.stderr)
        print(f"       known models: {', '.join(sorted(models))}", file=sys.stderr)
        return 1

    urdf_file = models[args.model].get("urdf_file")
    if not urdf_file:
        print(f"error: model '{args.model}' has no urdf_file entry", file=sys.stderr)
        return 1

    urdf_path = os.path.join(_PACKAGE_ROOT, urdf_file)
    if not os.path.isfile(urdf_path):
        print(f"error: no URDF at {urdf_path}", file=sys.stderr)
        return 1

    try:
        root_link, joints = parse_urdf(urdf_path)
    except UrdfGeometryError as error:
        print(f"error: {error}", file=sys.stderr)
        return 1

    text = _render(args.model, urdf_file, sha256_of_file(urdf_path), root_link, joints)
    _verify_round_trip(text, root_link, joints)

    output_path = os.path.join(_PACKAGE_ROOT, "config", f"geometry_{args.model}.yaml")
    with open(output_path, "w") as output_file:
        output_file.write(text)

    print(f"wrote {output_path}")
    print(f"  from {urdf_file} (root link {root_link})")
    for joint in joints:
        xyz = ", ".join(f"{value:.5f}" for value in joint.origin_xyz_m)
        axis = ", ".join(f"{value:g}" for value in joint.axis)
        print(f"  {joint.name}: origin xyz ({xyz}) axis ({axis})")
    print(f"\nRemember: geometry_file: config/geometry_{args.model}.yaml must be "
          f"set for model '{args.model}' in robot_model.yaml.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
