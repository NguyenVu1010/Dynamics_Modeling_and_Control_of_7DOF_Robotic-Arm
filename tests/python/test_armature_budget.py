"""Feature model-armature: sim/urdf/mass_budget.yaml `armature` section -> <cobot_armature>.

Pinned layout of the section (every entry names its source):

    armature:
      status: "assumed until the eRob H-series datasheet values are confirmed ..."
      gear_ratio: 100
      gear_ratio_source: "..."
      joints:
        joint_1: {input_inertia_g_mm2: 308632, source: "..."}
        ...

tools/fix_urdf_inertia.py writes armature = J_in * N^2 in kg m^2 as
<cobot_armature value="..."/>, one child of every revolute joint.
"""

import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import yaml

from fix_urdf_inertia import fix_urdf

REPO = Path(__file__).resolve().parents[2]
ORIGINAL = REPO / "sim/urdf/cobot30_5_original.urdf"
BUDGET = REPO / "sim/urdf/mass_budget.yaml"
COMMITTED = REPO / "sim/urdf/cobot30_5.urdf"

JOINTS = [f"joint_{k}" for k in range(1, 8)]
G_MM2_TO_KG_M2 = 1e-9  # 1 g mm^2 = 1e-3 kg * 1e-6 m^2

# Input inertia J_in per joint, g mm^2 (brief model-armature): eRob110 x2, eRob90 x2, eRob70 x3.
EXPECTED_J_IN = [308632, 308632, 100000, 100000, 22674, 22674, 22674]
# J_in * 1e-9 * 100^2:
#   308632 g mm^2 = 3.08632e-4 kg m^2 -> 3.08632
#   100000 g mm^2 = 1.0e-4 kg m^2     -> 1.0
#    22674 g mm^2 = 2.2674e-5 kg m^2  -> 0.22674
EXPECTED_ARMATURE = [3.08632, 3.08632, 1.0, 1.0, 0.22674, 0.22674, 0.22674]


def load_budget(path=BUDGET):
    return yaml.safe_load(Path(path).read_text(encoding="utf-8"))


def armature_elements(path):
    """{joint name: (joint type, [<cobot_armature> elements])} for every joint of the URDF."""
    root = ET.parse(path).getroot()
    return {
        j.get("name"): (j.get("type"), j.findall("cobot_armature")) for j in root.findall("joint")
    }


def armature_values(path):
    out = {}
    for name, (jtype, elements) in armature_elements(path).items():
        if jtype != "revolute":
            continue
        assert len(elements) == 1, f"{name}: {len(elements)} <cobot_armature> elements"
        out[name] = float(elements[0].get("value"))
    return out


@pytest.fixture(scope="module")
def fixed(tmp_path_factory):
    out = tmp_path_factory.mktemp("urdf_armature") / "fixed.urdf"
    fix_urdf(ORIGINAL, BUDGET, out)
    return out


def test_section_marked_assumed():
    section = load_budget()["armature"]
    status = section["status"]
    assert isinstance(status, str)
    assert "assumed until the eRob H-series datasheet values are confirmed" in status


def test_gear_ratio_and_source():
    section = load_budget()["armature"]
    assert section["gear_ratio"] == 100
    source = section["gear_ratio_source"]
    assert isinstance(source, str) and source.strip()


def test_every_joint_entry_has_inertia_and_source():
    joints = load_budget()["armature"]["joints"]
    assert list(joints) == JOINTS
    for name, j_in in zip(JOINTS, EXPECTED_J_IN, strict=True):
        entry = joints[name]
        assert set(entry) == {"input_inertia_g_mm2", "source"}, name
        assert entry["input_inertia_g_mm2"] == j_in, name
        assert isinstance(entry["source"], str) and entry["source"].strip(), name


def test_fixed_urdf_armature_is_j_in_times_n_squared(fixed):
    section = load_budget()["armature"]
    n = float(section["gear_ratio"])
    values = armature_values(fixed)
    assert sorted(values) == JOINTS
    for name, expected in zip(JOINTS, EXPECTED_ARMATURE, strict=True):
        j_in = float(section["joints"][name]["input_inertia_g_mm2"])
        assert values[name] == pytest.approx(j_in * G_MM2_TO_KG_M2 * n * n, rel=1e-12), name
        assert values[name] == pytest.approx(expected, rel=1e-12), name


def test_only_revolute_joints_carry_armature(fixed):
    elements = armature_elements(fixed)
    assert any(jtype == "fixed" for jtype, _ in elements.values()), "fixture has no fixed joint"
    for name, (jtype, found) in elements.items():
        if jtype != "revolute":
            assert not found, f"{name} ({jtype}) must not carry <cobot_armature>"


def test_committed_urdf_carries_armature():
    values = armature_values(COMMITTED)
    for name, expected in zip(JOINTS, EXPECTED_ARMATURE, strict=True):
        assert values[name] == pytest.approx(expected, rel=1e-12), name


def test_rerun_keeps_one_element_per_joint(fixed, tmp_path):
    again = tmp_path / "again.urdf"
    fix_urdf(fixed, BUDGET, again)
    values = armature_values(again)  # asserts exactly one element per revolute joint
    for name, expected in zip(JOINTS, EXPECTED_ARMATURE, strict=True):
        assert values[name] == pytest.approx(expected, rel=1e-12), name


def test_values_follow_the_budget(tmp_path):
    # The tool computes from the YAML, not from constants: gear ratio 50 and a new joint_7 inertia.
    budget = load_budget()
    budget["armature"]["gear_ratio"] = 50
    budget["armature"]["joints"]["joint_7"]["input_inertia_g_mm2"] = 40000
    alt = tmp_path / "budget.yaml"
    alt.write_text(yaml.safe_dump(budget, sort_keys=False), encoding="utf-8", newline="\n")
    out = tmp_path / "alt.urdf"
    fix_urdf(ORIGINAL, alt, out)
    values = armature_values(out)
    # 308632e-9 * 50^2 = 3.08632e-4 * 2500 = 0.77158
    assert values["joint_1"] == pytest.approx(0.77158, rel=1e-12)
    # 100000e-9 * 2500 = 0.25
    assert values["joint_3"] == pytest.approx(0.25, rel=1e-12)
    # 40000e-9 * 2500 = 4.0e-5 * 2500 = 0.1
    assert values["joint_7"] == pytest.approx(0.1, rel=1e-12)
