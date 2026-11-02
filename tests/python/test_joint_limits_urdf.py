"""eRob joint limits in the URDF files and config/default.yaml (feature joint-limits).

Design docs/design/2026-11-02-realism-2-design.md G1 and section 5, eRob ratio 100:

| Joints | rated R (N m) | peak P (N m) | momentary M (N m) | velocity (rad/s) |
| 1-2    | 87            | 204          | 369               | 2.1              |
| 3-4    | 52            | 107          | 191               | 2.1              |
| 5-7    | 10            | 36           | 70                | 3.1              |

URDF `<limit effort>` = M and `velocity` = the table in cobot30_5.urdf and cobot30_5_gazebo.urdf
(the xacro includes the latter); cobot30_5_original.urdf keeps the raw export. The config
carries R and P under `cobot.limits`.
"""

import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import yaml

from fix_urdf_inertia import fix_urdf

REPO = Path(__file__).resolve().parents[2]
URDF_DIR = REPO / "src/sim/urdf"
EFFORT_M = [369.0, 369.0, 191.0, 191.0, 70.0, 70.0, 70.0]
VELOCITY = [2.1, 2.1, 2.1, 2.1, 3.1, 3.1, 3.1]
RATED = [87.0, 87.0, 52.0, 52.0, 10.0, 10.0, 10.0]
PEAK = [204.0, 204.0, 107.0, 107.0, 36.0, 36.0, 36.0]


def limits(path: Path) -> dict[str, tuple[float, float]]:
    """joint name -> (effort, velocity) of every revolute joint."""
    root = ET.parse(path).getroot()
    out = {}
    for joint in root.iter("joint"):
        if joint.get("type") != "revolute":
            continue
        limit = joint.find("limit")
        assert limit is not None, f"{path.name}: {joint.get('name')} has no <limit>"
        out[joint.get("name")] = (float(limit.get("effort")), float(limit.get("velocity")))
    return out


def assert_table(path: Path) -> None:
    got = limits(path)
    assert sorted(got) == [f"joint_{j}" for j in range(1, 8)], path.name
    for j in range(7):
        effort, velocity = got[f"joint_{j + 1}"]
        assert effort == EFFORT_M[j], f"{path.name} joint_{j + 1} effort"
        assert velocity == VELOCITY[j], f"{path.name} joint_{j + 1} velocity"


@pytest.mark.parametrize("name", ["cobot30_5.urdf", "cobot30_5_gazebo.urdf"])
def test_urdf_carries_momentary_torque_and_erob_velocity(name):
    assert_table(URDF_DIR / name)


def test_xacro_takes_its_limits_from_the_gazebo_urdf():
    # The xacro has no joint <limit> of its own: it includes cobot30_5_gazebo.urdf. Any <limit>
    # it would declare must carry the table too.
    text = (URDF_DIR / "cobot30_5.urdf.xacro").read_text(encoding="utf-8")
    root = ET.fromstring(text)
    includes = [
        el.get("filename")
        for el in root.iter()
        if el.tag.endswith("include") and el.get("filename") is not None
    ]
    own = limits(URDF_DIR / "cobot30_5.urdf.xacro")
    assert "cobot30_5_gazebo.urdf" in includes or len(own) == 7
    for j in range(7):
        name = f"joint_{j + 1}"
        if name in own:
            assert own[name] == (EFFORT_M[j], VELOCITY[j]), name


def test_original_export_is_untouched():
    got = limits(URDF_DIR / "cobot30_5_original.urdf")
    efforts = [got[f"joint_{j}"][0] for j in range(1, 8)]
    velocities = [got[f"joint_{j}"][1] for j in range(1, 8)]
    assert efforts == [87.0, 870.0, 87.0, 870.0, 12.0, 12.0, 12.0]
    assert velocities == [2.5, 2.5, 2.5, 2.5, 3.0, 3.0, 3.0]


def test_fix_urdf_writes_the_table(tmp_path):
    # cobot30_5.urdf is generated from the original by tools/fix_urdf_inertia.py with
    # mass_budget.yaml (test_fix_urdf_inertia checks the committed file equals a fresh run).
    out = tmp_path / "fixed.urdf"
    fix_urdf(URDF_DIR / "cobot30_5_original.urdf", URDF_DIR / "mass_budget.yaml", out)
    assert_table(out)


def test_default_config_limits_section():
    data = yaml.safe_load((REPO / "config/default.yaml").read_text(encoding="utf-8"))
    section = data["cobot"]["limits"]
    assert set(section) == {"rated_torque_nm", "peak_torque_nm"}
    assert [float(v) for v in section["rated_torque_nm"]] == RATED
    assert [float(v) for v in section["peak_torque_nm"]] == PEAK


def test_table_is_ordered_rated_peak_momentary():
    for j in range(7):
        assert 0.0 < RATED[j] < PEAK[j] < EFFORT_M[j], f"joint_{j + 1}"
