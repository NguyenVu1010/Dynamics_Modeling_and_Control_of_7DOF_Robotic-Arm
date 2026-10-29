"""Apply the mass budget to the SolidWorks URDF export (design 5.8).

Sets every link mass from sim/urdf/mass_budget.yaml, scales its inertia tensor by the same
factor (uniform-density assumption, centre of mass unchanged), fixes the effort typos, keeps
only the standard <dynamics> attributes, writes the reflected rotor inertia J_in * N^2 of every
revolute joint as <cobot_armature value="..."/> (kg m^2, `armature` section), points meshes at
the cobot_sim package and drops the ROS 1 tags (<transmission>, <gazebo>). Running it on its own
output changes nothing.

With --gazebo-out it also writes the Gazebo plant variant of OUT (see gazebo_urdf): Gazebo
ignores <cobot_armature>, so the variant folds each joint's armature into the inertia of the
joint's child link about the joint axis instead.

Usage: python tools/fix_urdf_inertia.py --in IN.urdf --budget BUDGET.yaml --out OUT.urdf
       [--gazebo-out GAZEBO.urdf]
"""

from __future__ import annotations

import argparse
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import yaml

INERTIA_KEYS = ("ixx", "ixy", "ixz", "iyy", "iyz", "izz")
INERTIA_INDEX = {
    "ixx": (0, 0),
    "ixy": (0, 1),
    "ixz": (0, 2),
    "iyy": (1, 1),
    "iyz": (1, 2),
    "izz": (2, 2),
}
G_MM2_PER_KG_M2 = 1e9  # 1 g mm^2 = 1e-3 kg * 1e-6 m^2
ARMATURE_TAG = "cobot_armature"


def _fmt(value: float) -> str:
    return repr(float(value))


def armature_kg_m2(section: dict, joint: str) -> float:
    """Reflected rotor inertia J_in * N^2 of `joint`, kg m^2."""
    n = float(section["gear_ratio"])
    j_in = float(section["joints"][joint]["input_inertia_g_mm2"])
    # Divide by the exact 1e9: integer inputs then round once (0.22674, not 0.22674000000000002).
    return j_in * n * n / G_MM2_PER_KG_M2


def fix_urdf(original: Path, budget_path: Path, out: Path) -> None:
    budget = yaml.safe_load(Path(budget_path).read_text(encoding="utf-8"))
    tree = ET.parse(original)
    root = tree.getroot()

    for link in root.findall("link"):
        name = link.get("name")
        inertial = link.find("inertial")
        if inertial is None or name not in budget["links"]:
            continue
        mass_el = inertial.find("mass")
        mass_old = float(mass_el.get("value"))
        mass_new = float(budget["links"][name]["mass_kg"])
        ratio = mass_new / mass_old
        mass_el.set("value", _fmt(mass_new))
        inertia = inertial.find("inertia")
        for key in INERTIA_KEYS:
            inertia.set(key, _fmt(float(inertia.get(key)) * ratio))

    package = budget["mesh_package"]
    for mesh in root.iter("mesh"):
        filename = mesh.get("filename")
        mesh.set("filename", f"package://{package}/meshes/{Path(filename).name}")

    efforts = budget.get("effort_fix", {})
    damping = float(budget["damping_nm_s_rad"])
    armature = budget["armature"]
    for joint in root.findall("joint"):
        name = joint.get("name")
        limit = joint.find("limit")
        if name in efforts and limit is not None:
            limit.set("effort", _fmt(efforts[name]["effort_nm"]))
        dyn = joint.find("dynamics")
        if dyn is not None:
            friction = dyn.get("friction", "0.0")
            dyn.attrib.clear()
            dyn.set("damping", _fmt(damping))
            dyn.set("friction", _fmt(float(friction)))
        for old in joint.findall(ARMATURE_TAG):
            joint.remove(old)
        if joint.get("type") == "revolute":
            if name not in armature["joints"]:
                raise KeyError(f"{name}: no entry in the mass budget armature section")
            ET.SubElement(joint, ARMATURE_TAG, value=_fmt(armature_kg_m2(armature, name)))

    for tag in ("transmission", "gazebo"):
        for el in root.findall(tag):
            root.remove(el)

    _write(tree, out)


def _rpy_matrix(rpy: list[float]) -> np.ndarray:
    """URDF fixed-axis roll-pitch-yaw: R = Rz(yaw) * Ry(pitch) * Rx(roll)."""
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    return rz @ ry @ rx


def _write(tree: ET.ElementTree, out: Path) -> None:
    ET.indent(tree, space="  ")
    text = ET.tostring(tree.getroot(), encoding="unicode")
    Path(out).write_text('<?xml version="1.0"?>\n' + text + "\n", encoding="utf-8", newline="\n")


def gazebo_urdf(model: Path, out: Path) -> None:
    """Write the Gazebo plant variant of the corrected URDF `model`.

    Gazebo (URDF and SDF) has no joint armature, so the plant would miss the reflected rotor
    inertia the controller model carries. For every revolute joint with <cobot_armature value=a>,
    this adds a thin rotor disc spinning about the joint axis to the child link's inertia about
    its centre of mass, a * u u^T + (a / 2) * (I - u u^T) with u the joint axis in the link's
    inertial frame, and drops the <cobot_armature> element. The link's inertia about the joint
    axis (any line parallel to u) then grows by exactly a, which matches the a * qdd term of
    joint i itself. The a / 2 about the two perpendicular axes is the least that keeps the
    tensor physical (triangle inequality; SDFormat rejects the link otherwise).

    It is an approximation: in the plant the disc also moves with the joints above, so it enters
    their mass-matrix rows (the a / 2 terms most) and the Coriolis terms, while a real rotor's
    reflected inertia acts on joint i only. The controller model (`model`) is not changed.
    """
    tree = ET.parse(model)
    root = tree.getroot()
    links = {link.get("name"): link for link in root.findall("link")}
    for joint in root.findall("joint"):
        name = joint.get("name")
        arm = joint.find(ARMATURE_TAG)
        if arm is None:
            continue
        joint.remove(arm)
        a = float(arm.get("value"))
        if a == 0.0:
            continue
        child = joint.find("child").get("link")
        inertial = links[child].find("inertial")
        if inertial is None:
            raise ValueError(f"{name}: child link {child} has no <inertial> to carry the armature")
        axis_el = joint.find("axis")
        xyz = axis_el.get("xyz") if axis_el is not None else "1 0 0"  # URDF default axis
        axis = np.array([float(v) for v in xyz.split()])
        axis = axis / np.linalg.norm(axis)
        origin = inertial.find("origin")
        rpy = origin.get("rpy", "0 0 0") if origin is not None else "0 0 0"
        # Joint axis in the inertial frame; the child link frame is the joint frame.
        u = _rpy_matrix([float(v) for v in rpy.split()]).T @ axis
        uu = np.outer(u, u)
        add = a * uu + 0.5 * a * (np.eye(3) - uu)
        inertia = inertial.find("inertia")
        for key, (r, c) in INERTIA_INDEX.items():
            delta = float(add[r, c])
            if delta != 0.0:
                inertia.set(key, _fmt(float(inertia.get(key)) + delta))
    _write(tree, out)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--in", dest="inp", type=Path, required=True)
    parser.add_argument("--budget", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument(
        "--gazebo-out", type=Path, help="also write the Gazebo plant variant of OUT"
    )
    args = parser.parse_args()
    fix_urdf(args.inp, args.budget, args.out)
    if args.gazebo_out is not None:
        gazebo_urdf(args.out, args.gazebo_out)


if __name__ == "__main__":
    main()
