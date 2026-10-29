"""Apply the mass budget to the SolidWorks URDF export (design 5.8).

Sets every link mass from sim/urdf/mass_budget.yaml, scales its inertia tensor by the same
factor (uniform-density assumption, centre of mass unchanged), fixes the effort typos, keeps
only the standard <dynamics> attributes, writes the reflected rotor inertia J_in * N^2 of every
revolute joint as <cobot_armature value="..."/> (kg m^2, `armature` section), points meshes at
the cobot_sim package and drops the ROS 1 tags (<transmission>, <gazebo>). Running it on its own
output changes nothing.

Usage: python tools/fix_urdf_inertia.py --in IN.urdf --budget BUDGET.yaml --out OUT.urdf
"""

from __future__ import annotations

import argparse
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

INERTIA_KEYS = ("ixx", "ixy", "ixz", "iyy", "iyz", "izz")
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

    ET.indent(tree, space="  ")
    text = ET.tostring(root, encoding="unicode")
    Path(out).write_text('<?xml version="1.0"?>\n' + text + "\n", encoding="utf-8", newline="\n")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--in", dest="inp", type=Path, required=True)
    parser.add_argument("--budget", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    fix_urdf(args.inp, args.budget, args.out)


if __name__ == "__main__":
    main()
