"""Gazebo plant variant carries each joint's armature (review gap of `model-armature`).

tools/fix_urdf_inertia.py:gazebo_urdf adds a rotor disc to the child link of every revolute
joint: + a about the joint axis, + a / 2 about the two perpendicular axes. Checked here
independently of the generator: trace grows by 2 a, u^T dI u = a with u the joint axis in the
inertial frame, and the committed sim/urdf/cobot30_5_gazebo.urdf matches a fresh generation.
"""

import math
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest

from fix_urdf_inertia import gazebo_urdf

REPO = Path(__file__).resolve().parents[2]
MODEL = REPO / "sim/urdf/cobot30_5.urdf"
GAZEBO = REPO / "sim/urdf/cobot30_5_gazebo.urdf"
JOINTS = [f"joint_{k}" for k in range(1, 8)]


def rpy_matrix(roll, pitch, yaw):
    cr, sr, cp, sp, cy, sy = (f(a) for a in (roll, pitch, yaw) for f in (math.cos, math.sin))
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def inertia(link):
    el = link.find("inertial/inertia")
    v = {k: float(el.get(k)) for k in ("ixx", "ixy", "ixz", "iyy", "iyz", "izz")}
    return np.array(
        [
            [v["ixx"], v["ixy"], v["ixz"]],
            [v["ixy"], v["iyy"], v["iyz"]],
            [v["ixz"], v["iyz"], v["izz"]],
        ]
    )


def joints_and_links(path):
    root = ET.parse(path).getroot()
    return (
        {j.get("name"): j for j in root.findall("joint")},
        {lk.get("name"): lk for lk in root.findall("link")},
    )


@pytest.fixture(scope="module")
def generated(tmp_path_factory):
    out = tmp_path_factory.mktemp("gz") / "gazebo.urdf"
    gazebo_urdf(MODEL, out)
    return out


@pytest.mark.parametrize("name", JOINTS)
def test_child_link_gains_the_armature_about_the_joint_axis(generated, name):
    joints, links = joints_and_links(MODEL)
    _, gz_links = joints_and_links(generated)
    joint = joints[name]
    a = float(joint.find("cobot_armature").get("value"))
    assert a > 0.0
    child = joint.find("child").get("link")
    delta = inertia(gz_links[child]) - inertia(links[child])
    axis = np.array([float(v) for v in joint.find("axis").get("xyz").split()])
    origin = links[child].find("inertial/origin")
    rpy_text = origin.get("rpy", "0 0 0") if origin is not None else "0 0 0"
    rpy = [float(v) for v in rpy_text.split()]
    u = rpy_matrix(*rpy).T @ (axis / np.linalg.norm(axis))
    assert np.trace(delta) == pytest.approx(2.0 * a, rel=1e-6)
    assert u @ delta @ u == pytest.approx(a, rel=1e-6)
    assert np.allclose(delta, delta.T)
    assert np.all(np.linalg.eigvalsh(delta) >= 0.5 * a * (1 - 1e-6))


def test_no_armature_element_left_for_gazebo(generated):
    joints, _ = joints_and_links(generated)
    assert all(j.find("cobot_armature") is None for j in joints.values())


def test_committed_gazebo_urdf_inertias_match_a_fresh_generation(generated):
    _, fresh = joints_and_links(generated)
    _, committed = joints_and_links(GAZEBO)
    assert fresh.keys() == committed.keys()
    for name, link in fresh.items():
        if link.find("inertial") is None:
            continue
        np.testing.assert_allclose(inertia(committed[name]), inertia(link), rtol=1e-9, err_msg=name)
