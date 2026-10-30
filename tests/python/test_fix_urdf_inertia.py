import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest
import yaml

from fix_urdf_inertia import fix_urdf

REPO = Path(__file__).resolve().parents[2]
ORIGINAL = REPO / "src/sim/urdf/cobot30_5_original.urdf"
BUDGET = REPO / "src/sim/urdf/mass_budget.yaml"
COMMITTED = REPO / "src/sim/urdf/cobot30_5.urdf"


def inertials(path):
    root = ET.parse(path).getroot()
    out = {}
    for link in root.findall("link"):
        inertial = link.find("inertial")
        if inertial is None:
            continue
        mass = float(inertial.find("mass").get("value"))
        com = [float(v) for v in inertial.find("origin").get("xyz").split()]
        i = inertial.find("inertia")
        tensor = np.array([float(i.get(k)) for k in ("ixx", "ixy", "ixz", "iyy", "iyz", "izz")])
        out[link.get("name")] = (mass, np.array(com), tensor)
    return out


@pytest.fixture(scope="module")
def fixed(tmp_path_factory):
    out = tmp_path_factory.mktemp("urdf") / "fixed.urdf"
    fix_urdf(ORIGINAL, BUDGET, out)
    return out


def test_masses_match_budget(fixed):
    budget = yaml.safe_load(BUDGET.read_text())["links"]
    new = inertials(fixed)
    for name, entry in budget.items():
        assert new[name][0] == pytest.approx(entry["mass_kg"], abs=1e-9)
    moving = sum(m for name, (m, _, _) in new.items() if name != "base_link")
    assert moving == pytest.approx(17.053, abs=1e-3)


def test_inertia_scaled_by_mass_ratio_and_com_unchanged(fixed):
    old = inertials(ORIGINAL)
    new = inertials(fixed)
    for name, (m_old, com_old, i_old) in old.items():
        m_new, com_new, i_new = new[name]
        np.testing.assert_allclose(i_new, i_old * m_new / m_old, rtol=1e-9, atol=1e-15)
        np.testing.assert_allclose(com_new, com_old, atol=0.0)


def test_effort_typo_fixed(fixed):
    root = ET.parse(fixed).getroot()
    efforts = {
        j.get("name"): float(j.find("limit").get("effort"))
        for j in root.findall("joint")
        if j.get("type") == "revolute"
    }
    assert efforts["joint_2"] == 87.0
    assert efforts["joint_4"] == 87.0
    assert efforts["joint_1"] == 87.0
    assert efforts["joint_7"] == 12.0


def test_dynamics_has_only_standard_attributes(fixed):
    root = ET.parse(fixed).getroot()
    for dyn in root.iter("dynamics"):
        assert set(dyn.attrib) == {"damping", "friction"}
        assert float(dyn.get("damping")) == 0.003


def test_mesh_uris_point_to_cobot_sim(fixed):
    root = ET.parse(fixed).getroot()
    meshes = [m.get("filename") for m in root.iter("mesh")]
    assert meshes, "no meshes found"
    for uri in meshes:
        assert uri.startswith("package://cobot_sim/meshes/"), uri
        assert (REPO / "src/sim/meshes" / Path(uri).name).is_file(), uri


def test_ros1_tags_removed(fixed):
    root = ET.parse(fixed).getroot()
    assert root.find("transmission") is None
    assert root.find("gazebo") is None
    assert not list(root.iter("hardwareInterface"))


def test_idempotent(fixed, tmp_path):
    again = tmp_path / "again.urdf"
    fix_urdf(fixed, BUDGET, again)
    assert again.read_bytes() == fixed.read_bytes()


def test_committed_urdf_is_up_to_date(fixed):
    assert COMMITTED.read_bytes() == fixed.read_bytes()
